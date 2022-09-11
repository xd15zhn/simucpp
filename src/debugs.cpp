#include "simulator.hpp"
#include "tracelog.h"
#ifdef SUPPORT_DEBUG
#include <iostream>
#endif  // SUPPORT_DEBUG
NAMESPACE_SIMUCPP_L

void Simulator::Print_Modules() {
#ifdef SUPPORT_DEBUG
    using namespace std;
    int childcnt;
    PUnitModule bm;  // pointer to child module
    for (PUnitModule m: _modules) {
        cout << "name:" << m->_name << "; id:" << m->_id << ". Child modules:" << endl;
        for (int i=0; i<m->Get_childCnt(); ++i) {
            bm = m->Get_child(i);
            cout << "    name:" << bm->_name << "; id:" << bm->_id << endl;
        }
    }
#else
    TraceLog(LOG_WARNING, "Simucpp: You didn't add debug functions.");
#endif  // SUPPORT_DEBUG
}

NAMESPACE_SIMUCPP_R
