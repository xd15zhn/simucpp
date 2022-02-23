#include <fstream>
#include <stack>
#include "unitmodules.hpp"
#include "definitions.hpp"
NAMESPACE_SIMUCPP_L

/**********************
**********************/
Simulator::Simulator(double duration)
{
    SIMUCPP_ASSERT_ERROR(duration!=0, "Simulation duration must not be equal to zero!");
    Set_SimStep();
    _duration = duration;
    _cntM = 0;
    _t = 0;
    _print = true;
    _store = false;
    _precision = 8;
    DISCRETE_INITIALIZE(-1);
    _errlevel = 0;
    _fp = nullptr;
    for(int i=0; i<4; ++i) _ode4K[i] = nullptr;
}
Simulator::~Simulator()
{
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
void Simulator::Add_Module(const PUnitModule m)
{
    CHECK_NULLPTR(m);
    m->_id = _cntM;
    _modules.push_back(m);
    if (typeid(*m) == typeid(MIntegrator)){
        _integrators.push_back((PMIntegrator)m);
        _integIDs.push_back(std::vector<int>{_cntM});
        _discIDs.push_back(_cntM);
        _outref.push_back(m->Get_OutValue());
    }
    else if (typeid(*m) == typeid(MOutput)){
        _outputs.push_back((PMOutput)m);
        _outIDs.push_back(std::vector<int>{_cntM});
        _discIDs.push_back(_cntM);
    }
    else if (typeid(*m) == typeid(MUnitDelay)){
        _unitdelays.push_back((PMUnitDelay)m);
        _delayIDs.push_back(std::vector<int>{_cntM});
        _discIDs.push_back(_cntM);
    }
    _cntM++;
}


/**********************
connect
**********************/
void Simulator::connect(PUnitModule m1, PUnitModule m2)
{
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_NULLID(m1); CHECK_NULLID(m2);
    CHECK_SIMULATOR(m1); CHECK_SIMULATOR(m2);
    m2->connect(m1);
}
void Simulator::connect(PPackModule m1, int n1, PUnitModule m2)
{
    CHECK_NULLPTR(m1);
    CHECK_NULLPTR(m2); CHECK_NULLID(m2); CHECK_SIMULATOR(m2);
    PUnitModule bm1 = m1->Get_OutputPort(n1);
    CHECK_NULLPTR(bm1); CHECK_NULLID(bm1); CHECK_SIMULATOR(bm1);
    m2->connect(bm1);
}
void Simulator::connect(PPackModule m1, PUnitModule m2)
{
    this->connect(m1, 0, m2);
}
void Simulator::connect(PUnitModule m1, PPackModule m2, int n2)
{
    CHECK_NULLPTR(m1); CHECK_NULLID(m1); CHECK_SIMULATOR(m1);
    CHECK_NULLPTR(m2);
    PUnitModule bm2 = m2->Get_InputPort(n2);
    CHECK_NULLPTR(bm2); CHECK_NULLID(bm2); CHECK_SIMULATOR(bm2);
    bm2->connect(m1);
}
void Simulator::connect(PPackModule m1, int n1, PPackModule m2, int n2)
{
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    PUnitModule bm1 = m1->Get_OutputPort(n1);
    PUnitModule bm2 = m2->Get_InputPort(n2);
    CHECK_NULLPTR(bm1); CHECK_NULLID(bm1); CHECK_SIMULATOR(bm1);
    CHECK_NULLPTR(bm2); CHECK_NULLID(bm2); CHECK_SIMULATOR(bm2);
    bm2->connect(bm1);
}
void Simulator::connect(PPackModule m1, PPackModule m2, int n2)
{
    this->connect(m1, 0, m2, n2);
}


/**********************
Simulation initialization procedure, which includes the following steps:
 - step 1: Let every unit modules run a self check procedure;
 - step 2: Delete CONNECTOR module;
 - step 3: Build sequence table.
 - step 4: Index for discrete modules.
 - step 5: Initialize a data file for storage.
**********************/
void Simulator::Initialize()
{
    _cntI = _integIDs.size();
    _cntO = _outIDs.size();
    _cntD = _delayIDs.size();
    double ingaintemp;
    int errcode;
    PUnitModule bm, curm;
    std::vector<int> connectorids;
    MSum *sum;
    MProduct *prod;
    MFcnMISO *miso;
    
    /* step 1: Let every unit modules run a self check procedure*/
    for(int i=0; i<4; ++i)
        _ode4K[i] = new double[_cntI];
    for(int i=0; i<_cntM; ++i) {
        errcode = _modules[i]->Self_Check();
        if ((errcode!=0) && (errcode>_errlevel)) {
            std::cout << "Simucpp Error: "
            << "Self check of module \"" << _modules[i]->_name << "\" failed!"
            << std::endl; abort();
        }
    }

    /* step 2: Delete CONNECTOR module*/
    for(int i=_cntM-1; i>=0; --i){
        if (_modules[i]==nullptr) continue;
        for (int j=_modules[i]->Get_childCnt()-1; j>=0; --j){
            bm = _modules[i]->Get_child(j);
            if (bm==nullptr) continue;
            if (typeid(*bm) == typeid(MConnector)){
                curm = _modules[i];
                if (typeid(*curm) == typeid(MSum)){
                    sum = (MSum*)curm;
                    ingaintemp = sum->disconnect(j);
                    curm = bm->Get_child();
                    if(curm) sum->connect(curm);
                    sum->Set_InputGain(ingaintemp);
                }
                else if (typeid(*curm) == typeid(MProduct)){
                    prod = (MProduct*)curm;
                    ingaintemp = prod->disconnect(j);
                    curm = bm->Get_child();
                    if(curm) prod->connect(bm->Get_child());
                    prod->Set_InputGain(ingaintemp);
                }
                else if (typeid(*curm) == typeid(MFcnMISO)){
                    miso = (MFcnMISO*)curm;
                    miso->disconnect(j);
                    curm = bm->Get_child();
                    if(curm) miso->connect(bm->Get_child());
                }
                else {
                    SIMUCPP_ASSERT_ERROR(false, "Multiple input modules undefined!");
                }
                _cntM--;
                connectorids.push_back(bm->_id);
            }
        }
    }
    for (int id: connectorids){
        delete _modules[id];
        _modules[id] = nullptr;
    }

    /*step 3: Build sequence table*/
    for(int i=0; i<_cntI; ++i)
        Build_Connection(_integIDs[i]);
    for(int i=0; i<_cntD; ++i)
        Build_Connection(_delayIDs[i]);
    for(int i=0; i<_cntO; ++i)
        Build_Connection(_outIDs[i]);

    /*step 4: Index for discrete modules*/
    _discIDs.clear();
    for (PUnitModule m: _modules) {
        if (m==nullptr) continue;
        if (typeid(*m) == typeid(MZOH)) {
            _discIDs.push_back(m->_id);
            m->Set_Enable(false);
        }
        else if (typeid(*m) == typeid(MInput)) {
            MInput* bm = (MInput*)m;
            if (bm->_isc) continue;
            _discIDs.push_back(m->_id);
            m->Set_Enable(false);
        }
        else if (typeid(*m) == typeid(MNoise)) {
            MNoise* bm = (MNoise*)m;
            if (bm->_T < 0) continue;
            _discIDs.push_back(m->_id);
            m->Set_Enable(false);
        }
    }
    SIMUCPP_ASSERT_WARNING(_cntO>0, "You haven't add any OUTPUT modules.");

    /*step 5: Initialize a data file for storage.*/
    if (!_print) return;
    _fp = new std::fstream;
    _fp->open("data.csv", std::ios::out);
    SIMUCPP_ASSERT_ERROR(_fp->is_open(), "Failed to open data file!");
    _fp->precision(_precision);
}


/**********************
Simulate();
Simulate_FinalStep();
Simulate_OneStep();
**********************/
int Simulator::Simulate()
{
    int err = 0;
    _t = 0;
    while (_t < _duration)
        err |= Simulate_OneStep();
    err |= Simulate_FinalStep();
    return err;
}
int Simulator::Simulate_FinalStep()
{
    MODULE_INTEGRATOR_UPDATE();
    MODULE_UNITDELAY_UPDATE();
    MODULE_OUTPUT_UPDATE();
    PRINT_OUTPUT();
    return 0;
}
int Simulator::Simulate_OneStep()
{
    MODULE_UNITDELAY_UPDATE_OUTPUT();
    SET_DISCRETE_ENABLE(true);
    for(int i=0; i<_cntI; ++i){
        _outref[i] = _integrators[i]->_outvalue;
        for (int j=_integIDs[i].size()-1; j>0; --j)
            _modules[_integIDs[i][j]]->Module_Update(_t);
        _ode4K[0][i] = _integrators[i]->Get_child()->Get_OutValue();
    }
    SET_DISCRETE_ENABLE(false);
    MODULE_UNITDELAY_UPDATE();
    MODULE_OUTPUT_UPDATE();
    PRINT_OUTPUT();

    _t += _H;
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

    // Convergence and divergence check
    for (PMIntegrator m: _integrators)
        if (m->_outvalue > SIMUCPP_INFINITE1)
            return 1;
    for (PMUnitDelay m: _unitdelays)
        if (m->_outvalue > SIMUCPP_INFINITE1)
            return 1;
    for (PMOutput m: _outputs)
        if (m->_outvalue > SIMUCPP_INFINITE1)
            return 1;
    return 0;
}


/**********************
ids: IDs of every Endpoint modules.
The input is a sequence table and has only one element, and this function
 build the connection accroding to the modules' children modules and add
 them to that sequence table and return it.
**********************/
void Simulator::Build_Connection(std::vector<int> &ids)
{
    std::stack<int> idq;  // ids in stack
    int id;  // id of child module
    int curid;  // id of module in top of stack
    bool equal = false;  // if repetitive id detected
    PUnitModule bm;  // pointer to child module
    idq.push(ids[0]);
    while (!idq.empty()){
        curid = idq.top(); idq.pop();
        for (int i=0; i<_modules[curid]->Get_childCnt(); ++i){
            bm = _modules[curid]->Get_child(i);
            if (bm==nullptr) continue;
            id = bm->_id;
            SIMUCPP_ASSERT_ERROR(id>=0, "internal error: connection.");
            if (typeid(*bm) == typeid(MIntegrator)) continue;
            if (typeid(*bm) == typeid(MUnitDelay)) continue;
            for (int j=0; j<(int)_discIDs.size(); ++j){
                if (_discIDs[j] != id) continue;
                std::stack<int> agq;  // ids in another stack
                agq.push(id);
                while (!agq.empty()){
                    int agcurid = agq.top(); agq.pop();
                    for (int k=0; k<_modules[agcurid]->Get_childCnt(); ++k){
                        bm = _modules[agcurid]->Get_child(k);
                        if (bm==nullptr) continue;
                        if (typeid(*bm) == typeid(MIntegrator)) continue;
                        if (typeid(*bm) == typeid(MUnitDelay)) continue;
                        int agid = bm->_id;
                        SIMUCPP_ASSERT_ERROR(agid!=curid, "Algebraic loop detected!");
                        agq.push(agid);
                    }
                }
                equal = true;
                break;
            }
            if (equal) {
                equal=false; continue;
            }
            ids.push_back(id);
            _discIDs.push_back(id);
            idq.push(id);
        }
    }
}


/**********************
Reset all modules of this simulation to their initial state.
**********************/
void Simulator::Simulation_Reset()
{
     _t = 0;
    for(PUnitModule m:_modules) {
        if (m==nullptr) continue;
        m->Module_Reset();
    }
}

/**********************
**********************/
void Simulator::Set_EnablePrint(bool print) { _print=print; }
void Simulator::Set_t(double t) { _t = t; }
double Simulator::Get_t() { return _t; }
void Simulator::Set_Duration(double t) { _duration=t; }
double Simulator::Get_Duration() { return _duration; }
void Simulator::Set_SimStep(double step) { _H=0.5*step; }
double Simulator::Get_SimStep() { return _H+_H; }
void Simulator::Set_SampleTime(double time) { _T=time;_ltn=-_T; }
void Simulator::Set_EnableStore(bool store) {
    for (int i=0; i<_cntO; ++i) _outputs[i]->Set_EnableStore(store); }
void Simulator::Set_PrintPrecision(unsigned int n) {
    _precision = SIMUCPP_LIMIT(n, 2, 20); }
void Simulator::Set_WarningLevel(uint8_t level) {
    _errlevel=(level>0)?SIMUCPP_INFINITE1:((level<0)?-SIMUCPP_INFINITE1:0); }

NAMESPACE_SIMUCPP_R
