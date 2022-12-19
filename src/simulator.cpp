#include <stack>
#include <cmath>
#include <algorithm>
#include "simulator.hpp"
#include "definitions.hpp"
#ifdef USE_MPLT
#include "matplotlibcpp.h"
#endif
NAMESPACE_SIMUCPP_L

enum {
    FLAG_INITIALIZED  = 0x01,   // Whether to store simulation data to memory
    FLAG_DIVERGED     = 0x02,   // Set to run program in fullscreen
    FLAG_STORE        = 0x04,   // Set to allow resizable window
    FLAG_REDUNDANT    = 0x08,   // Clear to delete redundant modules
} SimulatorFlags;

/**********************
**********************/
bool Find_vector(std::vector<uint>& data, uint x) {
    std::vector<uint>::iterator iter = std::find(data.begin(), data.end(), x);
    return iter != data.end();
}

/**********************
**********************/
void Print_Connection(std::vector<uint> &ids) {
#if defined(SUPPORT_DEBUG)
    std::cout << "Sequence table: ";
    for (uint i: ids) {
        std::cout << i << ", ";
    }
    std::cout << std::endl;
#endif  // SUPPORT_DEBUG
}

/**********************
**********************/
Simulator::Simulator(double endtime) {
    Set_SimStep();
    _endtime = endtime;
    _cntM = 0;
    _t = 0;
    _status = FLAG_STORE | FLAG_REDUNDANT;
    DISCRETE_INITIALIZE(-1);
    for(int i=0; i<4; ++i) _ode4K[i] = nullptr;
    _divmode = 0;
}
Simulator::~Simulator() {
    for(int i=0; i<4; ++i) {
        if (!_ode4K[i]) continue;
        delete _ode4K[i]; _ode4K[i] = nullptr;
    }
}


/**********************
Add a unit module to this simulator, and divide it into Direct-Through modules
 and Endpoint modules.
**********************/
void Simulator::Add_Module(const PUnitModule m) {
    CHECK_NULLPTR(m, UnitModule);
    m->_id = int(_cntM);
    _modules.push_back(m);
    if (typeid(*m) == typeid(UIntegrator))
        _integIDs.push_back(std::vector<uint>{_cntM});
    else if (typeid(*m) == typeid(UOutput))
        _outIDs.push_back(std::vector<uint>{_cntM});
    else if (typeid(*m) == typeid(UUnitDelay))
        _delayIDs.push_back(std::vector<uint>{_cntM});
    _cntM++;
}
void Simulator::Add_Module(const PMatModule m) {
    CHECK_NULLPTR(m, MatModule);
    _matmodules.push_back(m);
}


/**********************
Simulation initialization procedure, which includes the following steps:
 - Self check procedure of unit modules and simulators;
 - Delete redundant connections.
 - Build sequence table.
 - Index for discrete modules.
 - Initialize a data file for storage.
**********************/
void Simulator::Initialize(bool print) {
    if (_status & FLAG_INITIALIZED) {
        TRACELOG(LOG_WARNING, "Simulator: Initialize again. Review your code.");
        return;
    }
    TRACELOG(LOG_INFO, "Simulator: Initialization start.");
    int errcode;

    /* Decompose and destroy every matrix modules */
    bool isInit;
    int cntdown = int(_matmodules.size())+1;
    for (; cntdown>0; --cntdown) {
        isInit = true;
        for (PMatModule m: _matmodules)
            isInit &= m->Initialize();
        if (isInit) break;
    }
    if (cntdown<0) TRACELOG(LOG_FATAL, "Simucpp: Matrix modules initialization failed!");
    _matmodules.clear();
    _cntI = _integIDs.size();
    _cntO = _outIDs.size();
    _cntD = _delayIDs.size();
    TRACELOG(LOG_DEBUG, "Simucpp: Matrix modules initialization completed.");

    /* Self check procedure of unit modules and simulators */
    for(int i=0; i<4; ++i) _ode4K[i] = new double[_cntI];
    _outref = new double[_cntI];
    for (uint i=0; i<_cntI; i++) {
        _outref[i] = PUIntegrator(_modules[_integIDs[i][0]])->_outvalue;
    }
    if (_H<=0) TRACELOG(LOG_FATAL, "Simucpp: Simulation step must be greator than zero!");
    for(uint i=0; i<_cntM; ++i) {
        errcode = _modules[i]->Self_Check();
        if (errcode!=0) TRACELOG(LOG_ERROR, "Simucpp: Self check of module \"%s\" failed!"
            "Errcode: %d", _modules[i]->_name.c_str(), errcode);
    }
    TRACELOG(LOG_DEBUG, "Simucpp: Module self check completed.");
    if (print) Print_Modules();

    /* Build sequence table */
    for(uint i=0; i<_cntI; ++i)
        Build_Connection(_integIDs[i]);
    for(uint i=0; i<_cntD; ++i)
        Build_Connection(_delayIDs[i]);
    for(uint i=0; i<_cntO; ++i)
        Build_Connection(_outIDs[i]);
    if (_cntO==0) TRACELOG(LOG_WARNING, "Simucpp: You haven't add any OUTPUT modules.");
    TRACELOG(LOG_DEBUG, "Simucpp: Build sequence table completed.");

    /* Index for discrete modules*/
    _discIDs.clear();
    for (PUnitModule m: _modules) {
        if (m==nullptr) continue;
        if (typeid(*m) == typeid(UZOH)) {
            _discIDs.push_back(uint(m->_id));
        } else if (typeid(*m) == typeid(UInput)) {
            if (!(PUInput(m)->_isc))
                _discIDs.push_back(uint(m->_id));
        } else if (typeid(*m) == typeid(UNoise)) {
            if (PUNoise(m)->_T > 0)
                _discIDs.push_back(uint(m->_id));
        }
    }
    TRACELOG(LOG_DEBUG, "Simucpp: Discrete modules indexing completed.");
    TRACELOG(LOG_INFO, "Simulator: Initialization successfully completed.");
    _status |= FLAG_INITIALIZED;
}


/**********************
**********************/
int Simulator::Simulate() {
    int err = 0;
    while (_t < _endtime-SIMUCPP_DBL_EPSILON) {
        err = Simulate_OneStep();
        if (!err) continue;
        if (_divmode == 0) {
            PRINT_CONVERGENCE(err); exit(1); }
        else if ((_divmode == 1) && !(_status & FLAG_DIVERGED)) {
            _status |= FLAG_DIVERGED;
            PRINT_CONVERGENCE(err); }
        else if (_divmode == 3) {
            PRINT_CONVERGENCE(err);
            return err; }
        else if (_divmode == 4) {
            return err; }
    }
    return err;
}
/**********************
**********************/
void Simulator::Simulate_FirstStep() {
    for(int i=0; i<_cntI; ++i){
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[0][i] = PUIntegrator(_modules[_integIDs[i][0]])->_next->Get_OutValue();
    }
    for(uint i=0; i<_cntI; ++i)
        for (int j=int(_integIDs[i].size())-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
    for(uint i=0; i<_cntD; ++i)
        for (int j=int(_delayIDs[i].size())-1; j>=0; --j)
            _modules[_delayIDs[i][j]]->Module_Update(_t);
    for(uint i=0; i<_cntO; ++i)
        for (int j=int(_outIDs[i].size())-1; j>=0; --j)
            _modules[_outIDs[i][j]]->Module_Update(_t);
    if (_status & FLAG_STORE) _tvec.push_back(_t);
}
/**********************
**********************/
void Simulator::Simulate_FinalStep() {
    for(int i=0; i<_cntD; ++i)
        PUUnitDelay(_modules[_integIDs[i][0]])->Output_Update(_t);
    for(int i=0; i<_cntI; ++i)
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
    for(int i=0; i<_cntD; ++i)
        for (int j=_delayIDs[i].size()-1; j>=0; --j)
            _modules[_delayIDs[i][j]]->Module_Update(_t);
    for(int i=0; i<_cntO; ++i)
        for (int j=_outIDs[i].size()-1; j>=0; --j)
            _modules[_outIDs[i][j]]->Module_Update(_t);
    if (_status & FLAG_STORE) _tvec.push_back(_t);
}
/**********************
**********************/
int Simulator::Simulate_OneStep() {
    if ((_status & FLAG_STORE) && (_t-_ltn >= _T-SIMUCPP_DBL_EPSILON)) {
        _ltn += _T;
        _tvec.push_back(_t);
    }

    /*  t = t(n) */
    for(uint i=0; i<_cntD; ++i)
        PUUnitDelay(_modules[_delayIDs[i][0]])->Output_Update(_t);
    for(uint i=0; i<_cntI; ++i) {
        _outref[i] = PUIntegrator(_modules[_integIDs[i][0]])->_outvalue;
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[0][i] = PUIntegrator(_modules[_integIDs[i][0]])->_next->Get_OutValue();
    }
    for(uint i=0; i<_cntD; ++i)
        for (int j=_delayIDs[i].size()-1; j>=0; --j)
            _modules[_delayIDs[i][j]]->Module_Update(_t);
    for(uint i=0; i<_cntO; ++i)
        for (int j=_outIDs[i].size()-1; j>=0; --j)
            _modules[_outIDs[i][j]]->Module_Update(_t);

    /* t = t(n)+h/2 */
    _t += _H;
    for (uint i: _discIDs)
        _modules[i]->Set_Enable(false);
    for(uint i=0; i<_cntI; ++i)
        PUIntegrator(_modules[_integIDs[i][0]])->_outvalue = _outref[i] + _H*_ode4K[0][i];
    for(uint i=0; i<_cntI; ++i){
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[1][i] = PUIntegrator(_modules[_integIDs[i][0]])->_next->Get_OutValue();
    }
    for(uint i=0; i<_cntI; ++i)
        PUIntegrator(_modules[_integIDs[i][0]])->_outvalue = _outref[i] + _H*_ode4K[1][i];
    for(uint i=0; i<_cntI; ++i){
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[2][i] = PUIntegrator(_modules[_integIDs[i][0]])->_next->Get_OutValue();
    }

    /* t = t(n)+h */
    _t += _H;
    for(uint i=0; i<_cntI; ++i)
        PUIntegrator(_modules[_integIDs[i][0]])->_outvalue = _outref[i] + (_H+_H)*_ode4K[2][i];
    for(uint i=0; i<_cntI; ++i){
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[3][i] = PUIntegrator(_modules[_integIDs[i][0]])->_next->Get_OutValue();
    }
    for(uint i=0; i<_cntI; ++i)
        PUIntegrator(_modules[_integIDs[i][0]])->_outvalue = _outref[i] +
            _H/3.0*(_ode4K[0][i] + _ode4K[1][i]+_ode4K[1][i] + _ode4K[2][i]+_ode4K[2][i] + _ode4K[3][i]);
    for (uint i: _discIDs)
        _modules[i]->Set_Enable(true);

    /* Convergence and divergence check */
    CHECK_CONVERGENCE(PUIntegrator, _integIDs);
    CHECK_CONVERGENCE(PUUnitDelay, _delayIDs);
    CHECK_CONVERGENCE(PUOutput, _outIDs);
    return 0;
}


/**********************
ids: IDs of every Endpoint modules.
The input is a sequence table and has only one element, and this function
 build the connection accroding to the modules' children modules and add
 them to that sequence table and return it.
**********************/
void Simulator::Build_Connection(std::vector<uint> &ids) {
    if (Find_vector(_discIDs, ids[0]))
        TRACELOG(LOG_FATAL, "Simucpp: internal error: connection.");
    _discIDs.push_back(ids[0]);
    std::stack<uint> idq;  // ids in stack
    std::vector<uint> idqv;  // compare in idq
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
            if (id<0) TRACELOG(LOG_FATAL, "Simucpp: internal error: connection.");
            if (typeid(*bm) == typeid(UIntegrator)) continue;
            if (typeid(*bm) == typeid(UUnitDelay)) continue;
            for (uint j=0; j<_discIDs.size(); ++j){
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
                        if (agid==curid) {
                            Print_Connection(ids);
                            TRACELOG(LOG_FATAL, "Simucpp: Algebraic loop detected!");
                        }
                        agq.push(agid);
                    }
                }
                if (Find_vector(ids, (uint)id)) {
                    std::remove(std::begin(ids), std::end(ids), id);
                    ids.pop_back(); }
                else
                    equal = true;
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
**********************/
void Simulator::Simulation_Reset() {
     _t = 0; _tvec.clear();
     _ltn = -_T;
    for(PUnitModule m:_modules) {
        if (m==nullptr) continue;
        m->Module_Reset();
    }
    _status &=~ FLAG_DIVERGED;
}
/**********************
**********************/
void Simulator::Plot() {
#ifdef USE_MPLT
    if (!(_status & FLAG_STORE)) { TRACELOG(LOG_WARNING, "Simucpp: There is no data for plotting."); return; }
    TRACELOG(LOG_INFO, "Simucpp: Wait for ploting......");
    if (_outIDs.size() < 1)
        TRACELOG(LOG_FATAL, "Simucpp plot: No output data for plot!");
    PUOutput m;
    for (uint i=0; i<_outIDs.size(); i++) {
        m = PUOutput(_modules[_outIDs[i][0]]);
        if (!m->_store) continue;
        if (m->_values.size() < 3)
            TRACELOG(LOG_FATAL, "Simucpp plot: Module \"%s\" has too few data points to plot!"
            "data points: %d.", m->_name.c_str(), m->_values.size());
        if (_tvec.size()!=m->_values.size())
            TRACELOG(LOG_FATAL, "Simucpp plot: Module \"%s\" has a wrong data amount for plotting!"
            "Time points:%d; data points:%d.", m->_name.c_str(), _tvec.size(), m->_values.size());
        matplotlibcpp::named_plot(m->_name, _tvec, m->_values);
    }
    matplotlibcpp::legend();
    matplotlibcpp::show();
    TRACELOG(LOG_INFO, "Simucpp: Plot completed.");
#else
    TRACELOG(LOG_WARNING, "Simucpp: You didn't add library \"matplotlibcpp\" for plotting.");
#endif
}
/**********************
**********************/
void Simulator::Print_Modules() {
#if defined(SUPPORT_DEBUG)
    using namespace std;
    PUnitModule bm;  // pointer to child module
    cout << "Model structure print start." << endl;
    for (PUnitModule m: _modules) {
        cout << "name:" << m->_name << "  id:" << m->_id << "  type:" << typeid(*m).name() << endl;
        for (int i=0; i<m->Get_childCnt(); ++i) {
            bm = m->Get_child(i);
            cout << "    name:" << bm->_name << "  id:" << bm->_id;
            if (typeid(*m) == typeid(UOutput))
                std::cout << "  gain:" << PUOutput(m)->_ingain << std::endl;
            else if (typeid(*m) == typeid(UProduct))
                std::cout << "  gain:" << PUProduct(m)->_ingain[i] << std::endl;
            else if (typeid(*m) == typeid(USum))
                std::cout << "  gain:" << PUProduct(m)->_ingain[i] << std::endl;
            else
                std::cout << std::endl;
        }
    }
    cout << "Model structure print finished." << endl;
#else
    TRACELOG(LOG_WARNING, "Simulator debug: You didn't add debug functions.");
#endif  // SUPPORT_DEBUG
}


/**********************
**********************/
void Simulator::Set_EnableStore(bool store) {
    if (store) _status |= FLAG_STORE;
    else _status &=~ FLAG_STORE;
    for (uint i=0; i<_outIDs.size(); i++) PUOutput(_modules[_outIDs[i][0]])->Set_EnableStore(store);}
void Simulator::Set_SampleTime(double time) { _T=time;_ltn=-_T;
    for (uint i=0; i<_outIDs.size(); i++) PUOutput(_modules[_outIDs[i][0]])->Set_SampleTime(time); }
void Simulator::Set_t(double t) { _t = t; }
double Simulator::Get_t() { return _t; }
void Simulator::Set_Endtime(double t) { _endtime=t; }
double Simulator::Get_Endtime() { return _endtime; }
void Simulator::Set_SimStep(double step) { _H=0.5*step; }
double Simulator::Get_SimStep() { return _H+_H; }
void Simulator::Set_DivergenceCheckMode(int mode) { _divmode=mode; };

NAMESPACE_SIMUCPP_R
