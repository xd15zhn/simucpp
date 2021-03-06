#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#include "config.h"

/**********************
global
**********************/
#define SIMUCPP_ASSERT_ERROR(e, s)           if(!(e)){std::cout<<"Simucpp Error: "<<s<<std::endl;abort();}
#define SIMUCPP_ASSERT_WARNING(e, s)         if(!(e)){std::cout<<"Simucpp Warning: "<<s<<std::endl;}
#define SIMUCPP_PRINT(s)                     std::cout<<s<<std::endl;
#define SIMUCPP_INFINITE1                    1e16
#define SIMUCPP_INFINITE2                    9e15
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
        std::cout<<"Simucpp Warning: "<<#module " module \"" << _name << "\" doesn't have a child module."<<std::endl; \
        return SIMUCPP_NO_ID; }
#define CHECK_FUNCTION(module) \
    if((_f==nullptr)&&(_fu==nullptr)) { \
        std::cout<<"Simucpp Warning: "<<#module " module \"" << _name << "\" doesn't have a function."<<std::endl; \
        return SIMUCPP_NO_FUNCTION; }

#define ADD_SIMULATOR() \
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
    for(int i=0; i<_cntI; ++i) \
        for (int j=_integIDs[i].size()-1; j>0; --j) \
            _modules[_integIDs[i][j]]->Module_Update(_t)
#define MODULE_OUTPUT_UPDATE() \
    for(int i=0; i<_cntO; ++i) \
        for (int j=_outIDs[i].size()-1; j>=0; --j) \
            _modules[_outIDs[i][j]]->Module_Update(_t)
#define MODULE_UNITDELAY_UPDATE() \
    for(int i=0; i<_cntD; ++i) \
        for (int j=_delayIDs[i].size()-1; j>=0; --j) \
            _modules[_delayIDs[i][j]]->Module_Update(_t)
#define MODULE_UNITDELAY_UPDATE_OUTPUT() \
    for(int i=0; i<_cntD; ++i) \
        _unitdelays[i]->Output_Update(_t)
#define CHECK_NULLPTR(x) \
    SIMUCPP_ASSERT_ERROR(x!=nullptr, \
        "Module "<<#x<<" is a null pointer!")
#define CHECK_NULLID(x) \
    SIMUCPP_ASSERT_ERROR(x->_id!=-1, \
        "Module \"" << x->_name << "\" wasn't added to a simulator!")
#define CHECK_SIMULATOR(x) \
    SIMUCPP_ASSERT_ERROR(x->_sim==this, \
        "Module \"" << x->_name << "\" was added to a wrong simulator!")
#define PRINT_OUTPUT() \
    if ((_print) && (_t-_ltn>=_T-SIMUCPP_DBL_EPSILON)) { \
        _ltn += _T; \
        *_fp << _t; \
        for (int i=0; i<_cntO; ++i){ \
            if (!_outputs[i]->_print) continue; \
            *_fp << ',' << _outputs[i]->_outvalue; \
        } \
        *_fp << '\n'; \
    }
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
    std::cout << "Simulation diverged. Type: "; \
    switch (x) { \
    case 1: std::cout<< "Positive infinity." << std::endl; break; \
    case 2: std::cout<< "Negative infinity." << std::endl; break; \
    case 3: std::cout<< "Not a number." << std::endl; break; \
    default: break; \
    }

#define STRING_DIRECT(x)    #x
#define STRING(x)           STRING_DIRECT(x)
#define SIMUCPP_VERSION     "V" STRING(simucpp_VERSION_MAJOR) \
                            "." STRING(simucpp_VERSION_MINOR) \
                            "." STRING(simucpp_VERSION_PATCH)

#endif // DEFINITIONS_H
