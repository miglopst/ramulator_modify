#include "Processor.h"
#include "Config.h"
#include "Controller.h"
#include "SpeedyController.h"
#include "Memory.h"
#include "DRAM.h"
#include "Statistics.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdlib.h>
#include <functional>
#include <map>
#include <queue>

/* Standards */
#include "Gem5Wrapper.h"
#include "DDR3.h"
#include "DDR4.h"
#include "DSARP.h"
#include "GDDR5.h"
#include "LPDDR3.h"
#include "LPDDR4.h"
#include "WideIO.h"
#include "WideIO2.h"
#include "HBM.h"
#include "HBM2.h"
#include "NVM.h"
#include "SALP.h"
#include "ALDRAM.h"
#include "TLDRAM.h"

using namespace std;
using namespace ramulator;

bool ramulator::warmup_complete = false;


typedef struct {
	// inputs to ramulator
	float request_start = 0; //ns
	bool read = true;
	long address = 0;
	long length = 0;
	// output from ramulator
	float respond_begin = 0; //ns, ramulator result
	float respond_end = 0;		//ns, ramulator result
	// optional
	double energy_activation = 0;
	double energy_refresh = 0;
	double energy_io = 0;
} REQUEST_RAMULATOR;


class Rundram_env{
    public:
    bool stall;
    bool end;
    int reads;
    int writes;
    int clks;
    long addr;
    Request::Type type;
    map<int, int> latencies;
    Rundram_env(){
        stall = false;
        end = false;
	reads = 0;
	writes = 0;
	clks = 0;
	addr = 0;
	type = Request::Type::READ;
    }
};

Request setup_dramrun(Rundram_env** dram_env){
    /* initialize runtime variable class*/
    Rundram_env* temp_dram_env = new Rundram_env();
    *dram_env = temp_dram_env;
    /* define callback function when read request is completed*/
    auto read_complete = [temp_dram_env](Request& r){temp_dram_env->latencies[r.depart - r.arrive]++;};
    Request req(temp_dram_env->addr, temp_dram_env->type, read_complete);
    return req;
}

template<typename T>
void ramulator_batch_request(const Config& configs, Memory<T, Controller>& memory, std::vector<REQUEST_RAMULATOR> req_vec, Rundram_env* dram_env, Request req){
    std::vector<REQUEST_RAMULATOR>::iterator vec_it = req_vec.begin();
    while (!dram_env->end || memory.pending_requests()){
        if (!dram_env->end && !dram_env->stall){
            //dram_env->end = !trace.get_dramtrace_request(dram_env->addr, dram_env->type);
            dram_env->end = (vec_it == req_vec.end());
            if (!dram_env->end){
                dram_env->addr = vec_it->address;
                if (vec_it->read){
                    dram_env->type = Request::Type::READ;
                } else {
                    dram_env->type = Request::Type::WRITE;
                }
                ++vec_it;
            }
        }
        if (!dram_env->end){
            req.addr = dram_env->addr;
            req.type = dram_env->type;
            dram_env->stall = !memory.send(req);
            if (!dram_env->stall){
                if (dram_env->type == Request::Type::READ) dram_env->reads++;
                else if (dram_env->type == Request::Type::WRITE) dram_env->writes++;
            }
        }
        else {
            memory.set_high_writeq_watermark(0.0f); // make sure that all write requests in the
                                                    // write queue are drained
        }

        memory.tick();
        dram_env->clks ++;
        Stats::curTick++; // memory clock, global, for Statistics
    }
}


template<typename T>
void run_dramtrace(const Config& configs, Memory<T, Controller>& memory, std::vector<REQUEST_RAMULATOR> req_vec) {

    /* initialize runtime variable class*/
    Rundram_env* dram_env = new Rundram_env();
    /* define callback function when read request is completed*/
    auto read_complete = [dram_env](Request& r){dram_env->latencies[r.depart - r.arrive]++;};
    Request req(dram_env->addr, dram_env->type, read_complete);


    std::vector<REQUEST_RAMULATOR>::iterator vec_it = req_vec.begin();
    while (!dram_env->end || memory.pending_requests()){
        if (!dram_env->end && !dram_env->stall){
            //dram_env->end = !trace.get_dramtrace_request(dram_env->addr, dram_env->type);
	    dram_env->end = (vec_it == req_vec.end());
	    if (!dram_env->end){
	        dram_env->addr = vec_it->address;
		if (vec_it->read){
		    dram_env->type = Request::Type::READ;
		} else {
		    dram_env->type = Request::Type::WRITE;
		}
		++vec_it;
	    }
        }
        if (!dram_env->end){
            req.addr = dram_env->addr;
            req.type = dram_env->type;
            dram_env->stall = !memory.send(req);
            if (!dram_env->stall){
                if (dram_env->type == Request::Type::READ) dram_env->reads++;
                else if (dram_env->type == Request::Type::WRITE) dram_env->writes++;
            }
        }
        else {
            memory.set_high_writeq_watermark(0.0f); // make sure that all write requests in the 
                                                    // write queue are drained
        }

        memory.tick();
        dram_env->clks ++;
        Stats::curTick++; // memory clock, global, for Statistics
    }
    // This a workaround for statistics set only initially lost in the end
    memory.finish();
    Stats::statlist.printall();
}

template<typename T>
Memory<T, Controller> config_memory(const Config& configs, T* spec) {
  // initiate controller and memory
  int C = configs.get_channels(), R = configs.get_ranks();
  // Check and Set channel, rank number
  spec->set_channel_number(C);
  spec->set_rank_number(R);
  std::vector<Controller<T>*> ctrls;
  for (int c = 0 ; c < C ; c++) {
    DRAM<T>* channel = new DRAM<T>(spec, T::Level::Channel);
    channel->id = c;
    channel->regStats("");
    Controller<T>* ctrl = new Controller<T>(configs, channel);
    ctrls.push_back(ctrl);
  }
  Memory<T, Controller> memory(configs, ctrls);
  return memory;
}


Config config_setup(){
    //memory configuration parameters; need to be specified by the user
    //the path of the memory configuration file
    std::string config_file = "configs/HBM-config.cfg";
    Config configs(config_file);
    //the path of the output stat file
    std::string stat_file = "my_output.txt";
    Stats::statlist.output(stat_file);
    //default number of cores
    configs.set_core_num(1);
    return configs;
}


std::vector<REQUEST_RAMULATOR> gen_req_vec(){
    std::vector<REQUEST_RAMULATOR> req_vec;
    REQUEST_RAMULATOR req;
    //we have verified this req can be used repeatedly
    req.request_start = 0;
    req.read = true;
    req.address = 305419904;
    req.length = 64;
    req_vec.push_back(req);
    req.request_start = 1;
    req.read = false;
    req.address = 1287476928;
    req.length = 64;
    req_vec.push_back(req);//this is a deep copy
    return req_vec;
}


int main(int argc, const char *argv[])
{
    //Part1: setup memory configuration
    //make sure you can access memory later
    //the path of the trace file
    Config configs = config_setup();
    //specify memory standard; requires user modification
    HBM* hbm = new HBM(configs["org"], configs["speed"]);
    Memory<HBM, Controller> memory = config_memory(configs, hbm);

    //Part2: start memory simulation
    //generate request vector
    std::vector<REQUEST_RAMULATOR> req_vec = gen_req_vec();
    //setup dram runtime enviroment
    Rundram_env* dram_env;
    Request req = setup_dramrun(&dram_env);

    //run batched request
    ramulator_batch_request(configs, memory, req_vec, dram_env, req);
    
    memory.finish();
    Stats::statlist.printall();
    return 0;
}
