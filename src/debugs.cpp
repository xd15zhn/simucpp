#include "simulator.hpp"
#ifdef SUPPORT_DEBUG
    #include <iostream>
#else
    #include "definitions.hpp"
#endif  // SUPPORT_DEBUG
NAMESPACE_SIMUCPP_L

void Simulator::Print_Modules() {
#if defined(SUPPORT_DEBUG)
    using namespace std;
    int childcnt;
    PUnitModule bm;  // pointer to child module
    for (PUnitModule m: _modules) {
        cout << "name:" << m->_name << "  id:" << m->_id << "  type:" << typeid(*m).name() << endl;
        for (int i=0; i<m->Get_childCnt(); ++i) {
            bm = m->Get_child(i);
            cout << "    name:" << bm->_name << "  id:" << bm->_id << endl;
        }
    }
#else
    TRACELOG(LOG_WARNING, "Simulator debug: You didn't add debug functions.");
#endif  // SUPPORT_DEBUG
}

NAMESPACE_SIMUCPP_R
