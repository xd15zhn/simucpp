#include <iostream>
#include <fstream>
#include <stack>
#include <algorithm>
#include "simulator.hpp"
#include "definitions.hpp"
#ifdef USE_MPLT
#include "matplotlibcpp.h"
#endif

NAMESPACE_SIMUCPP_L

bool Find_vector(std::vector<int>& data, int x) {
    std::vector<int>::iterator iter = std::find(data.begin(), data.end(), x);
    return iter != data.end();
}

/**********************
**********************/
Simulator::Simulator(double endtime) {
    Set_SimStep();
    _endtime = endtime;
    _cntM = 0;
    _t = 0;
    _store = true;
    _print = false;
    _precision = 8;
    DISCRETE_INITIALIZE(-1);
    _fp = nullptr;
    for(int i=0; i<4; ++i) _ode4K[i] = nullptr;
    _errlevel = 0;
    _divmode = 0;
    _diverge = false;
}
Simulator::~Simulator() {
    for(int i=0; i<4; ++i) {
        if (!_ode4K[i]) continue;
        delete _ode4K[i]; _ode4K[i] = nullptr;
    }
    if (!_print) return;
    if (!_fp) return;
    if (!_fp->is_open()) return;
    _fp->close();
    delete _fp; _fp = nullptr;
}


/**********************
Add a unit module to this simulator, and divide it into Direct-Through modules
 and Endpoint modules.
**********************/
void Simulator::Add_Module(const PUnitModule m) {
    CHECK_NULLPTR(m);
    m->_id = _cntM;
    _modules.push_back(m);
    if (typeid(*m) == typeid(UIntegrator)){
        _integrators.push_back((PUIntegrator)m);
        _integIDs.push_back(std::vector<int>{_cntM});
        _discIDs.push_back(_cntM);
        _outref.push_back(m->Get_OutValue());
    }
    else if (typeid(*m) == typeid(UOutput)){
        _outputs.push_back((PUOutput)m);
        _outIDs.push_back(std::vector<int>{_cntM});
        _discIDs.push_back(_cntM);
    }
    else if (typeid(*m) == typeid(UUnitDelay)){
        _unitdelays.push_back((PUUnitDelay)m);
        _delayIDs.push_back(std::vector<int>{_cntM});
        _discIDs.push_back(_cntM);
    }
    _cntM++;
}
void Simulator::Add_Module(const PMatModule m) {
    CHECK_NULLPTR(m);
    _matmodules.push_back(m);
}


/**********************
Simulation initialization procedure, which includes the following steps:
 - Self check procedure of unit modules and simulators;
 - Delete redundant connection of SUM module.
 - Build sequence table.
 - Index for discrete modules.
 - Initialize a data file for storage.
**********************/
void Simulator::Initialize() {
    _cntI = _integIDs.size();
    _cntO = _outIDs.size();
    _cntD = _delayIDs.size();
    int errcode;
    PUnitModule bm, curm;

    /* Decompose and destroy every matrix modules */
    bool isInit;
    int cntdown;
    for (cntdown=_matmodules.size()+1; cntdown>0; --cntdown) {
        isInit = true;
        for (PMatModule m: _matmodules)
            isInit &= m->Initialize();
        if (isInit) break;
    }
    if (cntdown<0) TraceLog(LOG_FATAL, "Simucpp: Matrix modules initialization failed!");
    _matmodules.clear();
    TraceLog(LOG_DEBUG, "Simucpp: Matrix modules initialization completed.");

    /* Self check procedure of unit modules and simulators */
    for(int i=0; i<4; ++i) _ode4K[i] = new double[_cntI];
    if (_H<=0) TraceLog(LOG_FATAL, "Simucpp: Simulation step must be greator than zero!");
    for(int i=0; i<_cntM; ++i) {
        errcode = _modules[i]->Self_Check();
        if ((errcode!=0) && (errcode>_errlevel)) {
            TraceLog(LOG_ERROR, "Simucpp: Self check of module \"%s\" failed!", _modules[i]->_name);
        }
    }
    TraceLog(LOG_DEBUG, "Simucpp: Module self check completed.");

    /* Delete redundant connections of SUM module */
    for(int i=_cntM-1; i>=0; --i){
        if (_modules[i]==nullptr) continue;
        bm = _modules[i];
        if (typeid(*bm) != typeid(USum)) continue;
        USum *mdl = (USum*)bm;
        if (mdl->_rdnt) continue;
        for (int i=mdl->_next.size()-1; i>=0; --i) {
            if (mdl->_ingain[i]!=0) continue;
            mdl->_ingain.erase(mdl->_ingain.begin() + i);
            mdl->_next.erase(mdl->_next.begin() + i);
        }
    }
    TraceLog(LOG_DEBUG, "Simucpp: Delete redundant connections completed.");

    /* Build sequence table */
    for(int i=0; i<_cntI; ++i)
        Build_Connection(_integIDs[i]);
    for(int i=0; i<_cntD; ++i)
        Build_Connection(_delayIDs[i]);
    for(int i=0; i<_cntO; ++i)
        Build_Connection(_outIDs[i]);
    if (_cntO==0) TraceLog(LOG_WARNING, "Simucpp: You haven't add any OUTPUT modules.");
    TraceLog(LOG_DEBUG, "Simucpp: Build sequence table completed.");

    /* Index for discrete modules*/
    _discIDs.clear();
    for (PUnitModule m: _modules) {
        if (m==nullptr) continue;
        if (typeid(*m) == typeid(UZOH)) {
            _discIDs.push_back(m->_id);
        }
        else if (typeid(*m) == typeid(UInput)) {
            UInput* bm = (UInput*)m;
            if (bm->_isc) continue;
            _discIDs.push_back(m->_id);
        }
        else if (typeid(*m) == typeid(UNoise)) {
            UNoise* bm = (UNoise*)m;
            if (bm->_T < 0) continue;
            _discIDs.push_back(m->_id);
        }
    }
    TraceLog(LOG_DEBUG, "Simucpp: Discrete modules indexing completed.");

    /* Initialize a data file for storage.*/
    if (_print) {
        _fp = new std::fstream;
        _fp->open("data.csv", std::ios::out);
        if (!_fp->is_open())  TraceLog(LOG_FATAL, "Simucpp: Failed to open data file!");
        _fp->precision(_precision);
        TraceLog(LOG_INFO, "Simucpp: Data file initialization completed.");
    }
    TraceLog(LOG_INFO, "Simucpp: Simulator initialization completed.");
}


/**********************
Simulate();
Simulate_FirstStep();
Simulate_FinalStep();
Simulate_OneStep();
**********************/
int Simulator::Simulate() {
    int err = 0;
    while (_t < _endtime-SIMUCPP_DBL_EPSILON) {
        err = Simulate_OneStep();
        if (!err) continue;
        if (_divmode == 0) {
            PRINT_CONVERGENCE(err); abort(); }
        else if ((_divmode == 1) && !_diverge) {
            _diverge = true;
            PRINT_CONVERGENCE(err); }
        else if (_divmode == 3) {
            PRINT_CONVERGENCE(err);
            return err; }
        else if (_divmode == 4) {
            return err; }
    }
    err = Simulate_FinalStep();
    return err;
}
int Simulator::Simulate_FirstStep() {
    MODULE_INTEGRATOR_UPDATE();
    MODULE_UNITDELAY_UPDATE();
    MODULE_OUTPUT_UPDATE();
    PRINT_OUTPUT();
    _tvec.push_back(_t);
    return 0;
}
int Simulator::Simulate_FinalStep() {
    MODULE_UNITDELAY_UPDATE_OUTPUT();
    MODULE_INTEGRATOR_UPDATE();
    MODULE_UNITDELAY_UPDATE();
    MODULE_OUTPUT_UPDATE();
    PRINT_OUTPUT();
    _tvec.push_back(_t);
    return 0;
}
int Simulator::Simulate_OneStep() {
    if ((_store) && (_t-_ltn >= _T-SIMUCPP_DBL_EPSILON)) {
        _ltn += _T;
        _tvec.push_back(_t);
    }
    MODULE_UNITDELAY_UPDATE_OUTPUT();
    for(int i=0; i<_cntI; ++i){
        _outref[i] = _integrators[i]->_outvalue;
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[0][i] = _integrators[i]->Get_child()->Get_OutValue();
    }
    MODULE_UNITDELAY_UPDATE();
    MODULE_OUTPUT_UPDATE();
    PRINT_OUTPUT();

    _t += _H;
    SET_DISCRETE_ENABLE(false);
    for(int i=0; i<_cntI; ++i)
        _integrators[i]->_outvalue = _outref[i] + _H*_ode4K[0][i];
    for(int i=0; i<_cntI; ++i){
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[1][i] = _integrators[i]->Get_child()->Get_OutValue();
    }
    for(int i=0; i<_cntI; ++i)
        _integrators[i]->_outvalue = _outref[i] + _H*_ode4K[1][i];
    for(int i=0; i<_cntI; ++i){
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[2][i] = _integrators[i]->Get_child()->Get_OutValue();
    }

    _t += _H;
    for(int i=0; i<_cntI; ++i)
        _integrators[i]->_outvalue = _outref[i] + (_H+_H)*_ode4K[2][i];
    for(int i=0; i<_cntI; ++i){
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[3][i] = _integrators[i]->Get_child()->Get_OutValue();
    }
    for(int i=0; i<_cntI; ++i)
        _integrators[i]->_outvalue = _outref[i] +
            _H/3*(_ode4K[0][i] + _ode4K[1][i] + _ode4K[1][i] + _ode4K[2][i] + _ode4K[2][i] + _ode4K[3][i]);
    SET_DISCRETE_ENABLE(true);

    // Convergence and divergence check
    CHECK_CONVERGENCE(PUIntegrator, _integrators);
    CHECK_CONVERGENCE(PUUnitDelay, _unitdelays);
    CHECK_CONVERGENCE(PUOutput, _outputs);
    return 0;
}


/**********************
ids: IDs of every Endpoint modules.
The input is a sequence table and has only one element, and this function
 build the connection accroding to the modules' children modules and add
 them to that sequence table and return it.
**********************/
void Simulator::Build_Connection(std::vector<int> &ids) {
    std::stack<int> idq;  // ids in stack
    std::vector<int> idqv;  // compare in idq
    int id;  // id of child module
    int curid;  // id of module in top of stack
    bool equal = false;  // if repetitive id detected
    PUnitModule bm;  // pointer to child module
    idq.push(ids[0]); idqv.push_back(ids[0]);
    while (!idq.empty()){
        curid = idq.top(); idq.pop(); idqv.pop_back();
        for (int i=0; i<_modules[curid]->Get_childCnt(); ++i){
            bm = _modules[curid]->Get_child(i);
            if (bm==nullptr) continue;
            id = bm->_id;
            if (id<0) TraceLog(LOG_FATAL, "Simucpp: internal error: connection.");
            if (typeid(*bm) == typeid(UIntegrator)) continue;
            if (typeid(*bm) == typeid(UUnitDelay)) continue;
            for (int j=0; j<(int)_discIDs.size(); ++j){
                if (_discIDs[j] != id) continue;
                std::stack<int> agq;  // ids in another stack
                agq.push(id);
                while (!agq.empty()){
                    int agcurid = agq.top(); agq.pop();
                    for (int k=0; k<_modules[agcurid]->Get_childCnt(); ++k){
                        bm = _modules[agcurid]->Get_child(k);
                        if (bm==nullptr) continue;
                        if (typeid(*bm) == typeid(UIntegrator)) continue;
                        if (typeid(*bm) == typeid(UUnitDelay)) continue;
                        int agid = bm->_id;
                        if (agid==curid) TraceLog(LOG_FATAL, "Simucpp: Algebraic loop detected!");
                        agq.push(agid);
                    }
                }
                if (Find_vector(ids, id)) {
                    std::remove(std::begin(ids), std::end(ids), id);
                    ids.pop_back(); }
                else {
                    equal = true; }
                break;
            }
            if (equal) {
                equal=false; continue; }
            ids.push_back(id);
            _discIDs.push_back(id);
            if (!Find_vector(idqv, id)) {
                idq.push(id);idqv.push_back(id); }
        }
    }
}


/**********************
Reset all modules of this simulation to their initial state.
**********************/
void Simulator::Simulation_Reset() {
     _t = 0; _tvec.clear();
     _ltn = -_T;
    for(PUnitModule m:_modules) {
        if (m==nullptr) continue;
        m->Module_Reset();
    }
    _diverge = false;
}
/**********************
Use data stored in OUTPUT modules to draw a waveform.
**********************/
void Simulator::Plot() {
#ifdef USE_MPLT
    TraceLog(LOG_INFO, "Simucpp: Wait for ploting......");
    for (PUOutput m: _outputs) {
        if (!m->_store) continue;
        if (_tvec.size()!=m->_values.size())
            TraceLog(LOG_FATAL, "Simucpp: Module \"%s\" has a wrong data amount for plot!"
            "Time points num is %d; data points num is %d.", m->_name, _tvec.size(), m->_values.size());
        matplotlibcpp::named_plot(m->_name, _tvec, m->_values);
    }
    matplotlibcpp::legend();
    matplotlibcpp::show();
    TraceLog(LOG_INFO, "Simucpp: Plot completed.");
#else
    TraceLog(LOG_WARNING, "Simucpp: You didn't add library \"matplotlibcpp\" for plot.");
#endif
}
/**********************
Set the output values of every pass-through modules to NaN.
**********************/
void Simulator::Set_DivergenceCheckMode() {
    PUnitModule bm = nullptr;
    for(int i=_cntM-1; i>=0; --i){
        if (_modules[i]==nullptr) continue;
        bm = _modules[i];
        if (typeid(*bm) == typeid(UInput)) {
           UInput *mdl = (UInput*)bm;
           mdl->_outvalue = nan(""); }
        else if (typeid(*bm) == typeid(UFcn)) {
           UFcn *mdl = (UFcn*)bm;
           mdl->_outvalue = nan(""); }
        else if (typeid(*bm) == typeid(UFcnMISO)) {
           UFcnMISO *mdl = (UFcnMISO*)bm;
           mdl->_outvalue = nan(""); }
        else if (typeid(*bm) == typeid(UProduct)) {
           UProduct *mdl = (UProduct*)bm;
           mdl->_outvalue = nan(""); }
        else if (typeid(*bm) == typeid(USum)) {
           USum *mdl = (USum*)bm;
           mdl->_outvalue = nan(""); }
        else if (typeid(*bm) == typeid(UZOH)) {
           UZOH *mdl = (UZOH*)bm;
           mdl->_outvalue = nan(""); }
    }
}

/**********************
**********************/
void Simulator::Set_EnableStore(bool store) { _store=store;
    for (PUOutput m: _outputs) m->Set_EnableStore(store); }
void Simulator::Set_EnablePrint(bool print) { _print=print;
    for (PUOutput m: _outputs) m->Set_EnablePrint(print); }
void Simulator::Set_SampleTime(double time) { _T=time;_ltn=-_T;
    for (PUOutput m: _outputs) m->Set_SampleTime(time); }
void Simulator::Set_PrintPrecision(unsigned int n) {
    _precision = SIMUCPP_LIMIT(n, 2, 20); }
void Simulator::Set_t(double t) { _t = t; }
double Simulator::Get_t() { return _t; }
void Simulator::Set_Endtime(double t) { _endtime=t; }
double Simulator::Get_Endtime() { return _endtime; }
void Simulator::Set_SimStep(double step) { _H=0.5*step; }
double Simulator::Get_SimStep() { return _H+_H; }
void Simulator::Set_DivergenceCheckMode(int mode) { _divmode=mode; };

NAMESPACE_SIMUCPP_R
