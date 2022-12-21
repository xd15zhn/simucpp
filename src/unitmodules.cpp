#include <cmath>
#include "simulator.hpp"
#include "definitions.hpp"
NAMESPACE_SIMUCPP_L

UnitModule::UnitModule(Simulator *sim, std::string name)
    : _name(name),_id(-1) {}
UnitModule::~UnitModule() {}

/**********************
CONSTANT module.
**********************/
UConstant::~UConstant() {}
double UConstant::Get_OutValue() const { return _outvalue; }
int UConstant::Self_Check() const { return 0; }
void UConstant::Set_Enable(bool enable) {}
void UConstant::Module_Update(double time) {}
void UConstant::Module_Reset() {}
uint UConstant::Get_childCnt() const { return 0; }
PUnitModule UConstant::Get_child(uint n) const { return nullptr; }
void UConstant::connect(const PUnitModule m) { TRACELOG(LOG_WARNING, "UConstant: cannot add child modules."); }
void UConstant::Set_OutValue(double v) { _outvalue=v; };
UConstant::UConstant(Simulator *sim, std::string name): UnitModule(sim, name) {
    _outvalue = 1;
    UNITMODULE_INIT();
}


/**********************
FCN module.
**********************/
UFcn::~UFcn() { _f=nullptr;_next=nullptr; }
double UFcn::Get_OutValue() const { return _outvalue; }
void UFcn::Set_Enable(bool enable) {}
void UFcn::Set_Function(std::function<double(double)> function) { _f=function; }
void UFcn::Module_Reset() {}
uint UFcn::Get_childCnt() const { return 1; }
PUnitModule UFcn::Get_child(uint n) const { return n==0?_next:nullptr; }
void UFcn::connect(const PUnitModule m) { _next=m; }
UFcn::UFcn(Simulator *sim, std::string name): UnitModule(sim, name) {
    _outvalue = 0.0/0.0;
    UNITMODULE_INIT();
}
int UFcn::Self_Check() const {
    CHECK_CHILD(FCN);
    CHECK_FUNCTION(FCN);
    return 0;
}
void UFcn::Module_Update(double time) {
    _outvalue = _f(_next->Get_OutValue());
}


/**********************
FCNMISO module.
**********************/
UFcnMISO::~UFcnMISO() { _f=nullptr;_next.clear(); }
double UFcnMISO::Get_OutValue() const { return _outvalue; }
void UFcnMISO::Set_Enable(bool enable) {}
void UFcnMISO::Set_Function(std::function<double(double*)> function) { _f=function; }
void UFcnMISO::Module_Reset() {}
uint UFcnMISO::Get_childCnt() const { return _next.size(); }
UFcnMISO::UFcnMISO(Simulator *sim, std::string name): UnitModule(sim, name) {
    _outvalue = 0.0/0.0;
    UNITMODULE_INIT();
}
int UFcnMISO::Self_Check() const {
    if (_next.size() <= 0) TRACELOG(LOG_WARNING,
        "UFcnMISO: Module \"%s\" doesn't have enough child module.", _name.c_str());
    CHECK_FUNCTION(FCNMISO);
    if (_next.size()==0) return SIMUCPP_NO_CHILD;
    if (_f==nullptr) return SIMUCPP_NO_FUNCTION;
    for (int i=0; i<(int)_next.size(); ++i)
        if (_next[i]==nullptr) return SIMUCPP_NULLPTR;
    return 0;
}
void UFcnMISO::Module_Update(double time) {
    uint n = _next.size();
    double* param = new double[n];
    for (int i=0; i<n; ++i)
        param[i] = _next[i]->Get_OutValue();
    _outvalue = _f(param);
    delete[] param;
}
PUnitModule UFcnMISO::Get_child(uint n) const {
    if (n >= (uint)_next.size())
        return nullptr;
    return _next[n];
}
void UFcnMISO::connect(const PUnitModule m) {
    _next.push_back(m);
}
void UFcnMISO::connect2(const PUnitModule m, uint n) {
    if (n>=_next.size()) TRACELOG(LOG_FATAL, "Simucpp internal error: reconnect.");
    if (_next[n]!=nullptr) TRACELOG(LOG_FATAL, "Simucpp internal error: reconnect.");
    _next[n] = m;
}
void UFcnMISO::disconnect(uint n) {
    if (n>=(int)_next.size()) TRACELOG(LOG_FATAL, "Simucpp internal error: disconnect.");
    _next[n] = nullptr;
}


/**********************
GAIN module.
**********************/
UGain::~UGain() { _next=nullptr; }
double UGain::Get_OutValue() const { return _outvalue; }
void UGain::Set_Enable(bool enable) {}
void UGain::Set_Gain(double gain) { _gain=gain; }
void UGain::Module_Reset() {}
uint UGain::Get_childCnt() const { return 1; }
PUnitModule UGain::Get_child(uint n) const { return n==0?_next:nullptr; }
void UGain::connect(const PUnitModule m) { _next=m; }
UGain::UGain(Simulator *sim, std::string name): UnitModule(sim, name) {
    _gain = 1;
    _outvalue = 0.0/0.0;
    _next = nullptr;
    UNITMODULE_INIT();
}
int UGain::Self_Check() const {
    CHECK_CHILD(GAIN);
    return 0;
}
void UGain::Module_Update(double time) {
    _outvalue = _gain * _next->Get_OutValue();
}


/**********************
INPUT module.
**********************/
UInput::~UInput() { _data.clear();_f=nullptr; }
double UInput::Get_OutValue() const { return _outvalue; }
void UInput::Set_Enable(bool enable) { _enable=enable; }
uint UInput::Get_childCnt() const { return 0; }
PUnitModule UInput::Get_child(uint n) const { return nullptr; }
void UInput::connect(const PUnitModule m) { TRACELOG(LOG_WARNING, "UInput: cannot add child modules."); }
void UInput::Set_Function(std::function<double(double)> function) { _f=function; }
void UInput::Set_InputData(const vecdble &data) { _data=data; }
void UInput::Set_Continuous(bool isContinuous) { _isc=isContinuous; }
void UInput::Set_SampleTime(double time) { _T=time; }
UInput::UInput(Simulator *sim, std::string name): UnitModule(sim, name) {
    _outvalue = 0.0/0.0;
    _cnt = -1;
    _isc = true;
    UNITMODULE_INIT();
}
int UInput::Self_Check() const {
    if (_isc){
        if (_f!=nullptr) return 0;
        TRACELOG(LOG_WARNING,
            "UInput: Module \"%s\" is in continuous mode but doesn't have an input function.", _name.c_str());
        return SIMUCPP_NO_FUNCTION;
    }
    else{
        if (_data.size() <= 0) TRACELOG(LOG_WARNING, 
            "UInput: Module \"%s\" is in discrete mode but doesn't have input data.", _name.c_str());
        if (_T <= 0) TRACELOG(LOG_WARNING, 
            "UInput: Module \"%s\" is in discrete mode with a non-positive sample time.", _name.c_str());
        if (_data.size()==0) return SIMUCPP_NO_DATA;
    }
    return 0;
}
void UInput::Module_Update(double time) {
    if (_isc)
        _outvalue = _f(time);
    else {
        if (!_enable) return;
        if (time - _cnt*_T < _T-SIMUCPP_DBL_EPSILON) return;
        _cnt++;
        if (_cnt >= (int)_data.size()) return;
        _outvalue = _data[_cnt];
    }
}
void UInput::Module_Reset() {
    _cnt = -1;
    _outvalue = 0.0/0.0;
}


/**********************
INTEGRATOR module.
**********************/
UIntegrator::~UIntegrator() { _next=nullptr; }
double UIntegrator::Get_OutValue() const { return _outvalue; }
void UIntegrator::Set_Enable(bool enable) {}
void UIntegrator::Set_InitialValue(double value) { _outvalue=_iv=value; }
void UIntegrator::Module_Update(double time) {}
void UIntegrator::Module_Reset() { _outvalue=_iv; }
uint UIntegrator::Get_childCnt() const { return 1; }
PUnitModule UIntegrator::Get_child(uint n) const { return n==0?_next:nullptr; }
void UIntegrator::connect(const PUnitModule m) { _next=m; }
UIntegrator::UIntegrator(Simulator *sim, std::string name): UnitModule(sim, name) {
    _outvalue = 0;
    _iv = 0;
    _next = nullptr;
    UNITMODULE_INIT();
}
int UIntegrator::Self_Check() const {
    CHECK_CHILD(INTEGRATOR);
    return 0;
}


/**********************
NOISE module.
**********************/
UNoise::~UNoise() {}
double UNoise::Get_OutValue() const { return _outvalue; }
int UNoise::Self_Check() const { return 0; }
void UNoise::Set_Enable(bool enable) { _enable=enable; }
void UNoise::Module_Reset() { _outvalue=0;_ltn=-_T; }
uint UNoise::Get_childCnt() const { return 0; }
PUnitModule UNoise::Get_child(uint n) const { return nullptr; }
void UNoise::connect(const PUnitModule m) { TRACELOG(LOG_WARNING, "UNoise: cannot add child modules."); }
void UNoise::Set_Mean(double mean) { _mean=mean; }
void UNoise::Set_Variance(double var) { _var=var; }
void UNoise::Set_SampleTime(double time) { _T=time;_ltn=-_T; }
UNoise::UNoise(Simulator *sim, std::string name): UnitModule(sim, name) {
    DISCRETE_INITIALIZE(-1);
    _outvalue = 0.0/0.0;
    UNITMODULE_INIT();
    _mean = 0;
    _var = 1;
}
void UNoise::Module_Update(double time) {
    if (!_enable) return;
    DISCRETE_UPDATE();
    double U = (rand()+1.0) / (RAND_MAX+1.0);
    double V = (rand()+1.0) / (RAND_MAX+1.0);
    double ans = sqrt(-2.0 * log(U))* cos(6.283185307179586477 * V);
    _outvalue = _var * ans + _mean;
}


/**********************
OUTPUT module.
**********************/
UOutput::~UOutput() { _values.clear(); }
double UOutput::Get_OutValue() const { return _outvalue; }
void UOutput::Set_Enable(bool enable) {}
void UOutput::Module_Reset() { _values.clear();_outvalue=0;_ltn=-_T; }
uint UOutput::Get_childCnt() const { return 1; }
PUnitModule UOutput::Get_child(uint n) const { return n==0?_next:nullptr; }
void UOutput::connect(const PUnitModule m) { _next=m; }
vecdble& UOutput::Get_StoredData() { return _values; }
void UOutput::Set_SampleTime(double time) { _T=time;_ltn=-_T; }
void UOutput::Set_EnableStore(bool store) { _store=store; }
void UOutput::Set_InputGain(double inputgain) { _ingain=inputgain; }
void UOutput::Set_MaxDataStorage(int n) { _maxstorage=n; }
UOutput::UOutput(Simulator *sim, std::string name): UnitModule(sim, name) {
    DISCRETE_INITIALIZE(-1);
    _outvalue = 0;
    _ingain = 1;
    _maxstorage = -1;
    _store = true;
    _next = nullptr;
    UNITMODULE_INIT();
}
int UOutput::Self_Check() const {
    CHECK_CHILD(OUTPUT);
    return 0;
}
void UOutput::Module_Update(double time) {
    DISCRETE_UPDATE();
    _outvalue = _ingain * _next->Get_OutValue();
    if (!_store) return;
    _values.push_back(_outvalue);
    if (_maxstorage>0 && (int)_values.size()>_maxstorage)
        _values.erase(_values.begin());
}


/**********************
PRODUCT module.
**********************/
UProduct::~UProduct() { _next.clear();_ingain.clear(); }
double UProduct::Get_OutValue() const { return _outvalue; }
void UProduct::Set_Enable(bool enable) {}
void UProduct::Module_Reset() {}
uint UProduct::Get_childCnt() const { return _next.size(); }
UProduct::UProduct(Simulator *sim, std::string name): UnitModule(sim, name) {
    _outvalue = 0.0/0.0;
    UNITMODULE_INIT();
}
void UProduct::Set_InputGain(double inputgain, int port) {
    if (port==-1){
        if (_next.size() <= 0) TRACELOG(LOG_WARNING, 
            "UProduct: Module \"%s\" doesn't have input port!", _name.c_str());
        _ingain[_next.size()-1] = inputgain;
    }
    else{
        if(port<0 || port>=(int)_next.size()) TRACELOG(LOG_WARNING,
            "UProduct: Module \"%s\" doesn't have input port!", _name.c_str());
        _ingain[port] = inputgain;
    }
}
int UProduct::Self_Check() const {
    if (_next.size() <= 1) TRACELOG(LOG_WARNING,
        "UProduct: Module \"%s\" doesn't have enough child module.", _name.c_str());
    if (_next.size()==0) return SIMUCPP_NO_CHILD;
    for (int i=0; i<(int)_next.size(); ++i)
        if (_next[i]==nullptr) return SIMUCPP_NULLPTR;
    return 0;
}
void UProduct::Module_Update(double time) {
    double ans = 1;
    for (int i=_next.size()-1; i>=0; --i)
        ans *= _ingain[i] * _next[i]->Get_OutValue();
    _outvalue = ans;
}
PUnitModule UProduct::Get_child(uint n) const {
    if (n >= (uint)_next.size())
        return nullptr;
    return _next[n];
}
void UProduct::connect(const PUnitModule m) {
    _next.push_back(m);
    _ingain.push_back(1);
}


/**********************
SUM module.
**********************/
USum::~USum() { _next.clear();_ingain.clear(); }
double USum::Get_OutValue() const { return _outvalue; }
void USum::Set_Enable(bool enable) {}
void USum::Module_Reset() {}
uint USum::Get_childCnt() const { return _next.size(); }
void USum::Set_Redundant(bool rdnt) { _rdnt=rdnt; };
USum::USum(Simulator *sim, std::string name): UnitModule(sim, name) {
    _outvalue = 0.0/0.0;
    _rdnt = true;
    UNITMODULE_INIT();
}
void USum::Set_InputGain(double inputgain, int port) {
    if (port==-1){
        if (_next.size()<=0)
            TRACELOG(LOG_WARNING, "SUM module \"%s\" doesn't have enough child module.", _name.c_str());
        _ingain[_next.size()-1] = inputgain;
    }
    else{
        if (port<0 || port>=(int)_next.size())
            TRACELOG(LOG_WARNING, "SUM module \"%s\" doesn't have enough child module.", _name.c_str());
        _ingain[port] = inputgain;
    }
}
int USum::Self_Check() const {
    if (_next.size()==0) {
        TRACELOG(LOG_WARNING, "SUM module \"%s\" doesn't have enough child module.", _name.c_str());
        return SIMUCPP_NO_CHILD;
    }
    for (int i=0; i<(int)_next.size(); ++i)
        if (_next[i]==nullptr) return SIMUCPP_NULLPTR;
    return 0;
}
void USum::Module_Update(double time) {
    double ans = 0;
    for (int i=_next.size()-1; i>=0; --i)
        ans += _ingain[i] * _next[i]->Get_OutValue();
    _outvalue = ans;
}
PUnitModule USum::Get_child(uint n) const {
    if (n >= (uint)_next.size())
        return nullptr;
    return _next[n];
}
void USum::connect(const PUnitModule m) {
    _next.push_back(m);
    _ingain.push_back(1);
}


/**********************
TRANSPORTDELAY module.
**********************/
UTransportDelay::~UTransportDelay() { _next=nullptr; }
double UTransportDelay::Get_OutValue() const { return _outvalue; }
void UTransportDelay::Set_Enable(bool enable) {}
void UTransportDelay::Set_InitialValue(double value) { _outvalue=_iv=value; }
uint UTransportDelay::Get_childCnt() const { return 1; }
PUnitModule UTransportDelay::Get_child(uint n) const { return n==0?_next:nullptr; }
void UTransportDelay::connect(const PUnitModule m) { _next=m; }
UTransportDelay::UTransportDelay(Simulator *sim, std::string name): UnitModule(sim, name) {
    _outvalue = 0.0/0.0;
    _iv = 0;
    _lv.push_back(0);
    _next = nullptr;
    UNITMODULE_INIT();
    _nexttime = _simstep = sim->Get_SimStep();
}
void UTransportDelay::Set_DelayTime(double time) {
    int delayPoints = time/_simstep + 0.5;
    if (delayPoints<=0) TRACELOG(LOG_WARNING, "TRANSPORTDELAY module was given an improper delay time.");        
    if (delayPoints>0){
        _lv = vecdble(delayPoints);
        for (int i=_lv.size()-1; i>=0; --i)
            _lv[i] = _iv;
    }
    else {
        _lv = vecdble{};
    }
}
int UTransportDelay::Self_Check() const {
    CHECK_CHILD(TRANSPORTDELAY);
    return 0;
}
void UTransportDelay::Module_Update(double time) {
    if (time < _nexttime) return;
    _nexttime += _simstep;
    if (_lv.size()>0){
        _outvalue = _lv[0];
        _lv.erase(_lv.begin());
        _lv.push_back(_next->Get_OutValue());
    }
    else{
        _outvalue = _next->Get_OutValue();
    }
}
void UTransportDelay::Module_Reset() {
    for (int i=_lv.size()-1; i>=0; --i)
        _lv[i] = _iv;
}


/**********************
UNITDELAY module.
**********************/
UUnitDelay::~UUnitDelay() { _next=nullptr; }
double UUnitDelay::Get_OutValue() const { return _outvalue; }
void UUnitDelay::Set_Enable(bool enable) {}
void UUnitDelay::Set_InitialValue(double value) { _outvalue=_lv=_iv=value; }
void UUnitDelay::Module_Reset() { _outvalue=_lv=_iv; }
uint UUnitDelay::Get_childCnt() const { return 1; }
PUnitModule UUnitDelay::Get_child(uint n) const { return n==0?_next:nullptr; }
void UUnitDelay::connect(const PUnitModule m) { _next=m; }
void UUnitDelay::Set_SampleTime(double time) { _T=time;_ltn=-_T; }
UUnitDelay::UUnitDelay(Simulator *sim, std::string name): UnitModule(sim, name) {
    DISCRETE_INITIALIZE(1);
    _iv = _lv = _outvalue = 0;
    _next = nullptr;
    UNITMODULE_INIT();
}
int UUnitDelay::Self_Check() const {
    CHECK_CHILD(UNITDELAY);
    return 0;
}
void UUnitDelay::Module_Update(double time) {
    DISCRETE_UPDATE();
    _lv = _next->Get_OutValue();
}
void UUnitDelay::Output_Update(double time) {
    if (time - _ltn < _T-SIMUCPP_DBL_EPSILON) return;
    _outvalue = _lv;
}


/**********************
ZOH module.
**********************/
UZOH::~UZOH() { _next=nullptr; }
double UZOH::Get_OutValue() const { return _outvalue; }
void UZOH::Set_Enable(bool enable) { _enable=enable; }
void UZOH::Module_Reset() { _outvalue=0;_ltn=-_T; }
uint UZOH::Get_childCnt() const { return 1; }
PUnitModule UZOH::Get_child(uint n) const { return n==0?_next:nullptr; }
void UZOH::connect(const PUnitModule m) { _next=m; }
void UZOH::Set_SampleTime(double time) { _T=time;_ltn=-_T; }
UZOH::UZOH(Simulator *sim, std::string name): UnitModule(sim, name) {
    DISCRETE_INITIALIZE(1);
    _outvalue = 0.0/0.0;
    _next = nullptr;
    UNITMODULE_INIT();
}
int UZOH::Self_Check() const {
    CHECK_CHILD(ZOH);
    return 0;
}
void UZOH::Module_Update(double time) {
    if (!_enable) return;
    DISCRETE_UPDATE();
    _outvalue = _next->Get_OutValue();
}

NAMESPACE_SIMUCPP_R
