#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#include "tracelog.h"

/**********************
global
**********************/
#define SIMUCPP_LIMIT(x, min, max)           (((x)<=(min) ? (min) : ((x)>=(max) ? (max) : (x))))
#define SIMUCPP_MIN(a, b)                    ((a) < (b) ? (a) : (b))
#define SIMUCPP_MAX(a, b)                    ((a) > (b) ? (a) : (b))
// Required for: CHECK_CONVERGENCE
#define SIMUCPP_INFINITE1                    1e16
// Used to solve the problem of imprecision of floating-point numbers
#define SIMUCPP_DBL_EPSILON                  1e-6


/**********************
unitmodules.cpp
**********************/
enum SIMUCPP_ERROR_CODE{
    SIMUCPP_NULLPTR = 1,
    SIMUCPP_NO_ID = 2,
    SIMUCPP_NO_FUNCTION = 3,
    SIMUCPP_NO_CHILD = 4,
    SIMUCPP_NO_DATA = -1,
};

#define CHECK_CHILD(module) \
    if(_next==nullptr) { \
        TraceLog(LOG_WARNING, "Simucpp: "#module" module \"%s\" doesn't have a child module.", _name.c_str()); \
        return SIMUCPP_NO_ID; }
#define CHECK_FUNCTION(module) \
    if((_f==nullptr)&&(_fu==nullptr)) { \
        TraceLog(LOG_WARNING, "Simucpp: "#module" module \"%s\" doesn't have a function.", _name.c_str()); \
        return SIMUCPP_NO_FUNCTION; }

#define UNITMODULE_INIT() \
    if(!sim) return; \
    sim->Add_Module(this); \
    _enable = false; \
    _sim = sim

#define DISCRETE_INITIALIZE(x) \
    _T = x; \
    _ltn = -_T
#define DISCRETE_UPDATE() \
    if (time-_ltn<_T-SIMUCPP_DBL_EPSILON) return; \
    _ltn += _T


/**********************
simulator.cpp
**********************/
#define MODULE_INTEGRATOR_UPDATE() \
    for(int i=0; i<_cntI; ++i)  for (int j=_integIDs[i].size()-1; j>0; --j) \
        _modules[_integIDs[i][j]]->Module_Update(_t)
#define MODULE_OUTPUT_UPDATE() \
    for(int i=0; i<_cntO; ++i)  for (int j=_outIDs[i].size()-1; j>=0; --j) \
        _modules[_outIDs[i][j]]->Module_Update(_t)
#define MODULE_UNITDELAY_UPDATE() \
    for(int i=0; i<_cntD; ++i)  for (int j=_delayIDs[i].size()-1; j>=0; --j) \
        _modules[_delayIDs[i][j]]->Module_Update(_t)
#define MODULE_UNITDELAY_UPDATE_OUTPUT() \
    for(int i=0; i<_cntD; ++i) _unitdelays[i]->Output_Update(_t)
#define CHECK_NULLPTR(x) \
    if (x==nullptr) TraceLog(LOG_FATAL, "Simucpp: Module "#x" is a null pointer!")
#define CHECK_NULLID(x) \
    if (x->_id==-1) TraceLog(LOG_FATAL, "Simucpp: Module \"%s\" is not added to a simulator!", x->_name.c_str())
#define CHECK_SIMULATOR(x) \
    if (x->_sim!=this) TraceLog(LOG_FATAL, "Simucpp: Module \"%s\" is added to a wrong simulator!", x->_name.c_str())
#define SET_DISCRETE_ENABLE(x) \
    for (int i: _discIDs) \
        _modules[i]->Set_Enable(x)
#define CHECK_CONVERGENCE(x, y) \
    for (x m: y) { \
        if (m->_outvalue > SIMUCPP_INFINITE1) return 1; \
        if (m->_outvalue < -SIMUCPP_INFINITE1) return 2; \
        if (std::isnan(m->_outvalue)) return 3; \
    }
#define PRINT_CONVERGENCE(x) \
    TraceLog(LOG_WARNING, "Simulation diverged. Type: "); \
    switch (x) { \
    case 1: TraceLog(LOG_WARNING, "    Positive infinity."); break; \
    case 2: TraceLog(LOG_WARNING, "    Negative infinity."); break; \
    case 3: TraceLog(LOG_WARNING, "    Not a number."); break; \
    default: break; \
    }


/**********************
matmodules.cpp
**********************/
#define MATMODULE_INIT() \
    if(!sim) return; \
    sim->Add_Module(this); \
    _sim = sim; \
    _name = name

#define BUS_SIZED           0x01
#define BUS_GENERATED       0x03
#define BUS_INITIALIZED     0X07

#endif // DEFINITIONS_H
