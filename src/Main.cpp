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
#include <math.h>
#include <time.h>

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
        //added by peng
	//global transaction id
	int tran_id = 0;
        // don't touch
	//EVARGS evArgs;
} REQUEST_RAMULATOR;


class Rundram_env{
    public:
    bool stall;
    bool end;
    int reads;
    int writes;
    int clks;
    long addr;
    int req_id;
    int tran_id;
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
    void reset_next_rnd(){
        stall = false;
        end = false;
    }
};

void setup_dramrun(Rundram_env** dram_env){
    /* initialize runtime variable class*/
    Rundram_env* temp_dram_env = new Rundram_env();
    *dram_env = temp_dram_env;
}


typedef struct {
    //HBM2 standard
    //assume this atom size = 256 bits = 32 bytes
    float request_start = 0;
    long address = 0;
    int tran_id = 0;
    int req_id = 0;
    bool read = true;
    float respond_begin = 0;
    float respond_end = 0;
} ATOM_REQ;

/* HBM2 address format */
/*
    |--------------| |--|     |--|      |-|       |-----|    |---|     |-----|
    R=14bits         B=2bits  BG=2bits  Pch=1bit  Col=5bits  Ch=3bits  BO=5bits
    __________________________________________________________________________
    R:Row
    B:Bank
    BG:Bank Group
    PCh:Pseudo Channel
    Col:Column
    Ch:Channel
    BO:Byte Offset
*/

/*
layout order (BO is not counted):
Ch->Pch->BG->B->Col->R
*/
//assumes total address space to be 4Gb*8=32Gb

std::vector<ATOM_REQ> address_translate(REQUEST_RAMULATOR new_tran){
    //make sure the range is not exceeded
    assert(new_tran.address < pow(2,32));
    std::vector<ATOM_REQ> atom_req_queue;
    int R=0, B=0, BG=0, Pch=0, Col=0, Ch=0, BO=0;
    //cout<<"addr="<<new_tran.address<<endl;
    BO = new_tran.address%(1<<5); new_tran.address=new_tran.address>>5;
    Ch = new_tran.address%(1<<3); new_tran.address=new_tran.address>>3;
    Col = new_tran.address%(1<<5); new_tran.address=new_tran.address>>5;
    Pch = new_tran.address%(1<<1); new_tran.address=new_tran.address>>1;
    BG = new_tran.address%(1<<2); new_tran.address=new_tran.address>>2;
    B = new_tran.address%(1<<2); new_tran.address=new_tran.address>>2;
    R = new_tran.address%(1<<14); new_tran.address=new_tran.address>>14;
    int num_req = ceil(float(new_tran.length+BO)/32);
    //cout<<"BO="<<BO<<" Ch="<<Ch<<" Col="<<Col<<" Pch="<<Pch<<" BG="<<BG<<" B="<<B<<" R="<<R<<endl;
    //cout<<"[ADDR_TRANSLATE]: tran id = "<<new_tran.tran_id<<"; BO = "<<BO<<"; req num = "<< num_req <<endl;
    int i;
    ATOM_REQ req;
    for (i=0;i<num_req;i++){
        req.read = new_tran.read;
	req.tran_id = new_tran.tran_id;
	req.req_id = i;
	req.request_start = new_tran.request_start;
	if (Ch==(1<<3)){
	    Ch=0;
	    Pch++;
	    if (Pch==(1<<1)){
	        Pch=0;
		BG++;
	        if (BG==(1<<2)){
		    BG=0;
		    B++;
	            if (B==(1<<2)){
		        B=0;
			Col++;
		        if (Col==(1<<5)){
			    Col=0;
			    R++;
		            if (R==(1<<14)){
			        cout<<"[Error]: Address Space Out-of-bound!"<<endl;
				exit(0);
			    } else {
			        R++;
			    }
		        } else {
			    Col++;
			}
		    } else {
		        B++;
		    }
	        } else {
		    BG++;
		}
	    } else {
	        Pch++;
	    }
	} else {
	    Ch++;
	}
	req.address = 0 + (Ch<<5) + (Col<<8) + (Pch<<13) + (BG<<14) + (B<<16) + (R<<18);
	atom_req_queue.push_back(req);
	//cout<<"[AFTER_ADDR_TRANSLATE]: "<<req.address<<endl;
    }
    return atom_req_queue;
}


template<typename T>
void ramulator_batch_request(const Config& configs, Memory<T, Controller>& memory, std::vector<REQUEST_RAMULATOR>& req_vec, Rundram_env* dram_env, int last_batch_tran_id){
    std::vector<std::vector<ATOM_REQ>> tran_vec;
    std::vector<REQUEST_RAMULATOR>::iterator vec_it = req_vec.begin();
    std::vector<ATOM_REQ> atom_req_vec;
    while (vec_it != req_vec.end()){
        atom_req_vec = address_translate(*vec_it);
	tran_vec.push_back(atom_req_vec);
	vec_it++;
    }
    /* define callback function when read request is completed*/
    auto read_complete = [&tran_vec](Request& r){
	tran_vec[r.tran_id-r.last_batch_tran_id][r.req_id].respond_begin = r.arrive;
	tran_vec[r.tran_id-r.last_batch_tran_id][r.req_id].respond_end = r.depart;
	cout<<"[callback] transaction ID = "<<r.tran_id<<"; local read_req_id = "<<r.req_id<<"; arrive at = "<<r.arrive<<"; depart at = "<<r.depart<<endl;
    };
    Request req(dram_env->addr, dram_env->type, read_complete);

    //Here we should issue multiple requests per transaction
    //outer iterator
    std::vector<std::vector<ATOM_REQ>>::iterator tran_vec_it = tran_vec.begin();
    //inner iterator
    std::vector<ATOM_REQ>::iterator req_vec_it = tran_vec_it->begin();
    //drain the current transaction
    //reset the "end" flag
    dram_env->end = false;
    while (!dram_env->end || memory.pending_requests()){
        if (!dram_env->end && !dram_env->stall){
	    //if all transactions has been drained
	    dram_env->end = ((req_vec_it == tran_vec_it->end()) && (tran_vec_it == tran_vec.end()));
            if (!dram_env->end){
                dram_env->addr = req_vec_it->address;
	        dram_env->tran_id = req_vec_it->tran_id;
		dram_env->req_id = req_vec_it->req_id;
		if (req_vec_it->read){
                    dram_env->type = Request::Type::READ;
	    	} else {
                    dram_env->type = Request::Type::WRITE;
                }
		//advance the inner iterator
		++req_vec_it;
		//judge if the pointer reaches the end
		if (req_vec_it == tran_vec_it->end()){
		    //advance the outer iterator
		    ++tran_vec_it;
		    //judge if the pointer reaches the end
		    if (tran_vec_it != tran_vec.end()){
			//reinitialize the inner iterator
			req_vec_it = tran_vec_it->begin();
		    } else {
		        dram_env->end = true;
		    }
		}
            }
        }
        if (!dram_env->end){
	    if (dram_env->type == Request::Type::READ){
	        cout<<"Issue READ request, transaction ID = "<< dram_env->tran_id <<"; req_id = "<< dram_env->req_id <<endl;
	    } else {
	        cout<<"Issue WRITE request transaction ID = "<< dram_env->tran_id <<"; req_id = "<< dram_env->req_id <<endl;
	    }
            req.last_batch_tran_id = last_batch_tran_id;
	    req.addr = dram_env->addr;
            req.type = dram_env->type;
	    req.req_id = dram_env->req_id;
	    req.tran_id = dram_env->tran_id;
            dram_env->stall = !memory.send(req);
            if (!dram_env->stall){
                if (dram_env->type == Request::Type::READ) dram_env->reads++;
                else if (dram_env->type == Request::Type::WRITE) dram_env->writes++;
            }
        } else {
	    memory.set_high_writeq_watermark(0.0f); // make sure that all write requests in the write queue are drained
	}
	memory.tick();
        dram_env->clks ++;
        Stats::curTick++; // memory clock, global, for Statistics
    }
    vec_it = req_vec.begin();
    tran_vec_it = tran_vec.begin();
    int begin_t, end_t;
    while (vec_it!=req_vec.end() || tran_vec_it!=tran_vec.end()){
        //calculate batch begin and end time
        req_vec_it = tran_vec_it->begin();
	begin_t = req_vec_it->respond_begin;
	end_t = req_vec_it->respond_end;
	while (req_vec_it != tran_vec_it->end()){
	    if (begin_t > req_vec_it->respond_begin){
	        begin_t = req_vec_it->respond_begin;
	    }
	    if (end_t < req_vec_it->respond_end){
	        end_t = req_vec_it->respond_end;
	    }
	    req_vec_it++;
	}
	vec_it->respond_begin = begin_t;
	vec_it->respond_end = end_t;
	vec_it++;
	tran_vec_it++;
    }
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


std::vector<REQUEST_RAMULATOR> gen_req_vec(int* tran_id){
    std::vector<REQUEST_RAMULATOR> req_vec;
    REQUEST_RAMULATOR req;
    //we have verified this req can be used repeatedly
    req.request_start = 0;
    req.read = true;
    req.address = 305419904;
    req.length = 32*5;
    req.tran_id = *tran_id;
    req_vec.push_back(req);
    (*tran_id)++;
    req.request_start = 1;
    req.read = false;
    req.address = 1287476928;
    req.length = 32*3;
    req.tran_id = *tran_id;
    req_vec.push_back(req);//this is a deep copy
    (*tran_id)++;
    req.request_start = 2;
    req.read = true;
    req.address = 903114496;
    req.length = 32*30;
    req.tran_id = *tran_id;
    req_vec.push_back(req);
    (*tran_id)++;
    return req_vec;
}

std::vector<REQUEST_RAMULATOR> gen_random_req_vec(int* tran_id){
    std::vector<REQUEST_RAMULATOR> req_vec;
    REQUEST_RAMULATOR req;
    int req_num = rand() % 20 + 10; // the request number will be between 10 - 29
    int start = 0;
    int round;
    while (req_num != 0){
	req.request_start = start;
	//random read / write
	if (rand() % 2==0){
	    req.read = true;
	} else {
	    req.read = false;
	};
	//random address between 0 and 2^20-1
	round = 20;
	req.address = rand()%2;
	while (round>0){
	    req.address = (req.address*2) + rand()%2;
	    round--;
	}
	//random length between 10 and 19
	req.length = (rand() % 10 + 10)*32;
	req.tran_id = *tran_id;
	//cout<<"[GEN_REQ]: tran id = "<<*tran_id<<"; addr = "<< req.address <<"; req num = "<< req.length/32 <<endl;
	req_vec.push_back(req);
	(*tran_id)++;
	start++;
        req_num--;
    }
    return req_vec;
}



void print_req_vec(std::vector<REQUEST_RAMULATOR>& req_vec){
    std::vector<REQUEST_RAMULATOR>::iterator req_vec_it = req_vec.begin();
    while (req_vec_it != req_vec.end()){
        cout<<"[PRINT] tran id = "<<req_vec_it->tran_id<<"; begin = "<<req_vec_it->respond_begin<<"; end = "<<req_vec_it->respond_end<<endl;
        req_vec_it++;
    }
}

int main(int argc, const char *argv[])
{
    /* initialize random seed: */
    //this is used in test only
    srand (time(NULL));

    //Part1: setup memory configuration
    //make sure you can access "memory" object and "configs" object later
    //the path of the trace file
    Config configs = config_setup();
    //specify memory standard; requires user modification
    HBM2* hbm = new HBM2(configs["org"], configs["speed"]);
    Memory<HBM2, Controller> memory = config_memory(configs, hbm);

    //Part2: start memory simulation
    //setup dram runtime enviroment
    Rundram_env* dram_env;
    setup_dramrun(&dram_env);
    //generate request vector
    int tran_id = 0;
    int last_tran_id = tran_id;
    //std::vector<REQUEST_RAMULATOR> req_vec = gen_req_vec(&tran_id); //simple test
    std::vector<REQUEST_RAMULATOR> req_vec = gen_random_req_vec(&tran_id); //random test
    //run batched request transaction
    ramulator_batch_request(configs, memory, req_vec, dram_env, last_tran_id);
    //verify results
    print_req_vec(req_vec);
    //start a new round of request transaction
    
    dram_env->reset_next_rnd();
    last_tran_id = tran_id;
    //req_vec = gen_req_vec(&tran_id); //simple test
    req_vec = gen_random_req_vec(&tran_id); //random test
    ramulator_batch_request(configs, memory, req_vec, dram_env, last_tran_id);
    print_req_vec(req_vec);
    

    memory.finish();
    Stats::statlist.printall();
    return 0;
}
