#ifndef __NVM_H
#define __NVM_H

#include "DRAM.h"
#include "Request.h"
#include <vector>
#include <functional>

using namespace std;

namespace ramulator
{

class NVM
{
public:
    static string standard_name;
    enum class Org;
    enum class Speed;
    NVM(Org org, Speed speed);
    NVM(const string& org_str, const string& speed_str);

    static map<string, enum Org> org_map;
    static map<string, enum Speed> speed_map;

    /* Level */
	/* we treat Rank as Pseudo Channel */
    enum class Level : int
    {
        Channel, Rank, BankGroup, Bank, Row, Column, MAX
    };

    /* Command */
    //we do not change these commands for compatible reasons
    //we only modify the command timing in .cpp file
    enum class Command : int
    {
        ACT, PRE,   PREA,
        RD,  WR,    RDA, WRA,
        REF, REFSB, PDE, PDX,  SRE, SRX,
        MAX
    };

    // REFSB and REF is not compatible, choose one or the other.
    // REFSB can be issued to banks in any order, as long as REFI1B
    // is satisfied for all banks

    string command_name[int(Command::MAX)] = {
        "ACT", "PRE",   "PREA",
        "RD",  "WR",    "RDA",  "WRA",
        "REF", "REFSB", "PDE",  "PDX",  "SRE", "SRX"
    };

    Level scope[int(Command::MAX)] = {
        Level::Row,    Level::Bank,   Level::Rank,
        Level::Column, Level::Column, Level::Column, Level::Column,
        Level::Rank,   Level::Bank,   Level::Channel,   Level::Channel,   Level::Channel,   Level::Channel
    };

    bool is_opening(Command cmd)
    {
        switch(int(cmd)) {
            case int(Command::ACT):
                return true;
            default:
                return false;
        }
    }

    bool is_accessing(Command cmd)
    {
        switch(int(cmd)) {
            case int(Command::RD):
            case int(Command::WR):
            case int(Command::RDA):
            case int(Command::WRA):
                return true;
            default:
                return false;
        }
    }

    bool is_closing(Command cmd)
    {
        switch(int(cmd)) {
            case int(Command::RDA):
            case int(Command::WRA):
            case int(Command::PRE):
            case int(Command::PREA):
                return true;
            default:
                return false;
        }
    }

    bool is_refreshing(Command cmd)
    //modify here. we always assume the banks are not refreshed
    {
        return false;
    }

    /* State */
    //we should change here
    enum class State : int
    {
        Opened, Closed, PowerUp, ActPowerDown, PrePowerDown, SelfRefresh, MAX
    } start[int(Level::MAX)] = {
        State::MAX, State::PowerUp, State::MAX, State::Closed, State::Closed, State::MAX
    };

    /* Translate */
    Command translate[int(Request::Type::MAX)] = {
        Command::RD,  Command::WR,
        Command::REF, Command::PDE, Command::SRE
    };

    /* Prereq */
    //we should change here
    function<Command(DRAM<NVM>*, Command cmd, int)> prereq[int(Level::MAX)][int(Command::MAX)];

    // SAUGATA: added function object container for row hit status
    /* Row hit */
    function<bool(DRAM<NVM>*, Command cmd, int)> rowhit[int(Level::MAX)][int(Command::MAX)];
    function<bool(DRAM<NVM>*, Command cmd, int)> rowopen[int(Level::MAX)][int(Command::MAX)];

    /* Timing */
    struct TimingEntry
    {
        Command cmd;
        int dist;
        int val;
        bool sibling;
    };
    vector<TimingEntry> timing[int(Level::MAX)][int(Command::MAX)];

    /* Lambda */
    //we should change here
    function<void(DRAM<NVM>*, int)> lambda[int(Level::MAX)][int(Command::MAX)];

    /* Organization */
    enum class Org : int
    { // per channel density here. Each stack comes with 8 channels
        NVM_16Gb,
        MAX
    };

    //modify here, add 1 bit row address and 1 bit column address
    struct OrgEntry {
        int size;
        int dq;
        int count[int(Level::MAX)];
    } org_table[int(Org::MAX)] = {
        {16<<10, 128, {0, 0, 4, 4, 1<<15, 1<<(7+1)}},
    }, org_entry;

    void set_channel_number(int channel);
    void set_rank_number(int rank);

    /* Speed */
    enum class Speed : int
    {
        NVM_1Gbps,
        MAX
    };

    int prefetch_size = 4; // burst length could be 2 and 4 (choose 4 here), 2n prefetch
    int channel_width = 128;

    //modify here to reflext NVM changes
    struct SpeedEntry {
        int rate;
        double freq, tCK;
        int nBL, nCCD;
        int nCL, nRCD, nRP;
        //int nRAS, nRC;
        //RAS and RC are calculated according to RD/WR
        int nRTPL, nRTPS, nWTR, nWR;
        int nRRDS, nRRDL, nFAW;
        int nRFC, nREFI, nREFI1B;
        int nPD, nXP;
        int nCKESR, nXS;
        /* newly added parameters */
        int nCKE;
        int nRREFD, nRFCSB, nREFSBPDE;
        int nACTPDE, nREFPDE, nPRPDE;
        int nWL, nRL, nPL;
        int nRTW;
        int nRDSRE;
        int nWRAPDE, nWRPDE, nRDPDE;
    } speed_table[int(Speed::MAX)] = {
		{1000,
		500, 2.0,
		2, 4,
		7, 25, 1,
		3, 2, 4, 75,
		2, 3, 25,
		0, 0, 0,
		3, 4,
		4, 0,
		3,
		0, 0, 0,
		1, 1, 1,
		2, 2, 1,
		5,
		0,
		12, 12, 5}
    }, speed_entry;

    int read_latency;

private:
    void init_speed();
    void init_lambda();
    void init_prereq();
    void init_rowhit();  // SAUGATA: added function to check for row hits
    void init_rowopen();
    void init_timing();
};

} /*namespace ramulator*/

#endif /*__NVM_H*/
