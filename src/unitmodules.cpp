#include <iostream>
#include <ctime>
#include "unitmodules.hpp"
#include "definitions.hpp"
NAMESPACE_SIMUCPP_L


/**********************
CONNECTOR module.
**********************/
MConnector::~MConnector() { _next=nullptr; }
double MConnector::Get_OutValue() const{
    SIMUCPP_ASSERT_ERROR(false, "internal error: CONNECTOR");}
void MConnector::Set_Enable(bool enable) { _enable=enable; }
void MConnector::Module_Update(double time) {
    SIMUCPP_ASSERT_ERROR(false, "internal error: CONNECTOR");}
void MConnector::Module_Reset() {}
int MConnector::Get_childCnt() const { return 1; }
PUnitModule MConnector::Get_child(unsigned int n) const { return n==0?_next:nullptr; }
void MConnector::connect(const PUnitModule m) { _next=m;_enable=true; }
MConnector::MConnector(Simulator *sim, std::string name): UnitModule(name)
{
    _next = nullptr;
    ADD_SIMULATOR();
}
int MConnector::Self_Check() const
{
    CHECK_CHILD(CONNECTOR);
    return 0;
}


/**********************
CONSTANT module.
**********************/
MConstant::~MConstant() {}
double MConstant::Get_OutValue() const { return _outvalue; }
void MConstant::Set_Enable(bool enable) { _enable=enable; }
int MConstant::Self_Check() const { return 0; }
void MConstant::Module_Update(double time) {}
void MConstant::Module_Reset() {}
int MConstant::Get_childCnt() const { return 0; }
PUnitModule MConstant::Get_child(unsigned int n) const { return 0; }
void MConstant::connect(const PUnitModule m) {}
void MConstant::Set_OutValue(double v) { _outvalue=v; };
MConstant::MConstant(Simulator *sim, std::string name): UnitModule(name)
{
    _outvalue = 1;
    ADD_SIMULATOR();
}


/**********************
FCN module.
**********************/
MFcn::~MFcn() { _f=nullptr;_next=nullptr; }
double MFcn::Get_OutValue() const { return _outvalue; }
void MFcn::Set_Enable(bool enable) { _enable=enable; }
void MFcn::Set_Function(double (*function)(double)) { _f=function; }
void MFcn::Set_Function(UserFunc *function) { _fu=function; }
void MFcn::Module_Reset() {}
int MFcn::Get_childCnt() const { return 1; }
PUnitModule MFcn::Get_child(unsigned int n) const { return n==0?_next:nullptr; }
void MFcn::connect(const PUnitModule m) { _next=m;_enable=true; }
MFcn::MFcn(Simulator *sim, std::string name): UnitModule(name)
{
    _outvalue = 0;
    _f = [](double u){return u;};
    _fu = nullptr;
    _next = nullptr;
    ADD_SIMULATOR();
}
int MFcn::Self_Check() const
{
    CHECK_CHILD(FCN);
    CHECK_FUNCTION(FCN);
    return 0;
}
void MFcn::Module_Update(double time)
{
    if (!_enable) return;
    if (_fu) _outvalue = _fu->Function(_next->Get_OutValue());
    else _outvalue = _f(_next->Get_OutValue());
}


/**********************
FCNMISO module.
**********************/
MFcnMISO::~MFcnMISO() { _f=nullptr;_next.clear(); }
double MFcnMISO::Get_OutValue() const { return _outvalue; }
void MFcnMISO::Set_Enable(bool enable) { _enable=enable; }
void MFcnMISO::Set_Function(double (*function)(double *)) { _f=function; }
void MFcnMISO::Set_Function(UserFunc *function) { _fu=function; }
void MFcnMISO::Module_Reset() {}
int MFcnMISO::Get_childCnt() const { return _next.size(); }
MFcnMISO::MFcnMISO(Simulator *sim, std::string name): UnitModule(name)
{
    _outvalue = 0;
    _f = [](double *u){return u[0];};
    _fu = nullptr;
    ADD_SIMULATOR();
}
int MFcnMISO::Self_Check() const
{
    SIMUCPP_ASSERT_WARNING(_next.size()>0,
        "FCNMISO module \"" << _name << "\" doesn't have enough child module.");
    CHECK_FUNCTION(FCNMISO);
    if (_next.size()==0) return SIMUCPP_NO_CHILD;
    if ((_f==nullptr)&&(_fu==nullptr)) return SIMUCPP_NO_FUNCTION;
    for (int i=0; i<(int)_next.size(); ++i)
        if (_next[i]==nullptr) return SIMUCPP_NULLPTR;
    return 0;
}
void MFcnMISO::Module_Update(double time)
{
    if (!_enable) return;
    int n = _next.size();
    double* param = new double[n];
    for (int i=0; i<n; ++i)
        param[i] = _next[i]->Get_OutValue();
    if (_fu) _outvalue = _fu->Function(param);
    else _outvalue = _f(param);
}
PUnitModule MFcnMISO::Get_child(unsigned int n) const
{
    if (n >= (unsigned int)_next.size())
        return nullptr;
    return _next[n];
}
void MFcnMISO::connect(const PUnitModule m)
{
    _next.push_back(m);
    _enable = true;
}
void MFcnMISO::disconnect(int n)
{
    SIMUCPP_ASSERT_ERROR(n<(int)_next.size(), "internal error: disconnect");
    _next.erase(_next.begin()+n);
}


/**********************
GAIN module.
**********************/
MGain::~MGain() { _next=nullptr; }
double MGain::Get_OutValue() const { return _outvalue; }
void MGain::Set_Enable(bool enable) { _enable=enable; }
void MGain::Set_Gain(double gain) { _gain=gain; }
void MGain::Module_Reset() {}
int MGain::Get_childCnt() const { return 1; }
PUnitModule MGain::Get_child(unsigned int n) const { return n==0?_next:nullptr; }
void MGain::connect(const PUnitModule m) { _next=m;_enable=true; }
MGain::MGain(Simulator *sim, std::string name): UnitModule(name)
{
    _gain = 1;
    _outvalue = 0;
    _next = nullptr;
    ADD_SIMULATOR();
}
int MGain::Self_Check() const
{
    CHECK_CHILD(GAIN);
    return 0;
}
void MGain::Module_Update(double time)
{
    if (!_enable) return;
    _outvalue = _gain * _next->Get_OutValue();
}


/**********************
INPUT module.
**********************/
MInput::~MInput() { _data.clear();_f=nullptr; }
double MInput::Get_OutValue() const { return _outvalue; }
void MInput::Set_Enable(bool enable) { _enable=enable; }
void MInput::Set_Function(double (*function)(double u)) { _f=function; }
void MInput::Set_Function(UserFunc *function) { _fu=function; }
void MInput::Set_InputData(const vecdble &data) { _data=data; }
void MInput::Set_Continuous(bool isContinuous) { _isc=isContinuous; }
void MInput::Set_SampleTime(double time) { _T=time; }
int MInput::Get_childCnt() const { return 0; }
PUnitModule MInput::Get_child(unsigned int n) const { return nullptr; }
void MInput::connect(const PUnitModule m) {}
MInput::MInput(Simulator *sim, std::string name): UnitModule(name)
{
    _outvalue = 0;
    _cnt = -1;
    _isc = true;
    _f = [](double t){return 1.0;};
    _fu = nullptr;
    ADD_SIMULATOR();
    _enable = true;
}
int MInput::Self_Check() const
{
    if (_isc){
        SIMUCPP_ASSERT_WARNING(_f!=nullptr,
            "INPUT module \"" << _name << "\" is in continuous mode but doesn't have an input function.");
        if ((_f==nullptr)&&(_fu==nullptr)) return SIMUCPP_NO_FUNCTION;
    }
    else{
        SIMUCPP_ASSERT_WARNING(_data.size()>0,
            "INPUT module \"" << _name << "\" is in discrete mode but doesn't have input data.");
        SIMUCPP_ASSERT_WARNING(_T>0,
            "INPUT module \"" << _name << "\" is in discrete mode with a non-positive sample time.");
        if (_data.size()==0) return SIMUCPP_NO_DATA;
    }
    return 0;
}
void MInput::Module_Update(double time)
{
    if (_isc) {
        if (_fu) _outvalue = _fu->Function(time);
        else _outvalue = _f(time);
    }
    else {
        if (time - _cnt*_T < _T-SIMUCPP_DBL_EPSILON) return;
        _cnt++;
        if (_cnt >= (int)_data.size()) return;
        _outvalue = _data[_cnt];
    }
}
void MInput::Module_Reset()
{
    _cnt = -1;
    _outvalue = 0;
}


/**********************
INTEGRATOR module.
**********************/
MIntegrator::~MIntegrator() { _next=nullptr; }
double MIntegrator::Get_OutValue() const { return _outvalue; }
void MIntegrator::Set_Enable(bool enable) { _enable=enable; }
void MIntegrator::Set_InitialValue(double value) { _outvalue=_iv=value; }
void MIntegrator::Module_Update(double time) {}
void MIntegrator::Module_Reset() { _outvalue=_iv; }
int MIntegrator::Get_childCnt() const { return 1; }
PUnitModule MIntegrator::Get_child(unsigned int n) const { return n==0?_next:nullptr; }
void MIntegrator::connect(const PUnitModule m) { _next=m;_enable=true; }
MIntegrator::MIntegrator(Simulator *sim, std::string name): UnitModule(name)
{
    _outvalue = 0;
    _iv = 0;
    _next = nullptr;
    ADD_SIMULATOR();
}
int MIntegrator::Self_Check() const
{
    CHECK_CHILD(INTEGRATOR);
    return 0;
}


/**********************
NOISE module.
**********************/
MNoise::~MNoise() {}
double MNoise::Get_OutValue() const { return _outvalue; }
void MNoise::Set_Enable(bool enable) { _enable=enable; }
int MNoise::Self_Check() const { return 0; }
void MNoise::Set_Mean(double mean) { _mean=mean; }
void MNoise::Set_Variance(double var) { _var=var; }
void MNoise::Set_SampleTime(double time) { _T=time;_ltn=-_T; }
void MNoise::Module_Reset() {}
int MNoise::Get_childCnt() const { return 0; }
PUnitModule MNoise::Get_child(unsigned int n) const { return nullptr; }
void MNoise::connect(const PUnitModule m) {}
MNoise::MNoise(Simulator *sim, std::string name): UnitModule(name)
{
    DISCRETE_INITIALIZE(-1);
    _outvalue = 0;
    ADD_SIMULATOR();
    _enable = true;
    gen = std::default_random_engine((unsigned int)time(0));
    NormDis = std::normal_distribution<double>(0, 1);
}
void MNoise::Module_Update(double time)
{
    if (!_enable) return;
    DISCRETE_UPDATE();
    _outvalue = _var * NormDis(gen) + _mean;
}


/**********************
OUTPUT module.
**********************/
MOutput::~MOutput() { _values.clear(); }
double MOutput::Get_OutValue() const { return _outvalue; }
void MOutput::Set_Enable(bool enable) { _enable=enable; }
void MOutput::Module_Reset() { _values.clear(); }
int MOutput::Get_childCnt() const { return 1; }
PUnitModule MOutput::Get_child(unsigned int n) const { return n==0?_next:nullptr; }
void MOutput::connect(const PUnitModule m) { _next=m;_enable=true; }
vecdble& MOutput::Get_StoredData() { return _values; }
void MOutput::Set_SampleTime(double time) { _T=time;_ltn=-_T; }
void MOutput::Set_EnableStore(bool store) { _store=store; }
void MOutput::Set_EnablePrint(bool print) { _print=print; }
void MOutput::Set_InputGain(double inputgain) { _ingain=inputgain; }
void MOutput::Set_MaxDataStorage(int n) { _maxstorage=n; }
MOutput::MOutput(Simulator *sim, std::string name): UnitModule(name)
{
    DISCRETE_INITIALIZE(-1);
    _outvalue = 0;
    _ingain = 1;
    _maxstorage = -1;
    _store = true;
    _print = false;
    _next = nullptr;
    ADD_SIMULATOR();
}
int MOutput::Self_Check() const
{
    CHECK_CHILD(OUTPUT);
    return 0;
}
void MOutput::Module_Update(double time)
{
    if (!_enable) return;
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
MProduct::~MProduct() { _next.clear();_ingain.clear(); }
double MProduct::Get_OutValue() const { return _outvalue; }
void MProduct::Set_Enable(bool enable) { _enable=enable; }
void MProduct::Module_Reset() {}
int MProduct::Get_childCnt() const { return _next.size(); }
MProduct::MProduct(Simulator *sim, std::string name): UnitModule(name)
{
    _outvalue = 0;
    ADD_SIMULATOR();
}
void MProduct::Set_InputGain(double inputgain, int port)
{
    if (port==-1){
        SIMUCPP_ASSERT_ERROR(_next.size()>0,
            "PRODUCT module \"" << _name << "\" doesn't have input port!");
        _ingain[_next.size()-1] = inputgain;
    }
    else{
        SIMUCPP_ASSERT_ERROR(port>=0 && port<(int)_next.size(),
            "PRODUCT module \"" << _name << "\" doesn't have input port!");
        _ingain[port] = inputgain;
    }
}
int MProduct::Self_Check() const
{
    SIMUCPP_ASSERT_WARNING(_next.size()>1,
        "PRODUCT module \"" << _name << "\" doesn't have enough child module.");
    if (_next.size()==0) return SIMUCPP_NO_CHILD;
    for (int i=0; i<(int)_next.size(); ++i)
        if (_next[i]==nullptr) return SIMUCPP_NULLPTR;
    return 0;
}
void MProduct::Module_Update(double time)
{
    if (!_enable) return;
    double ans = 1;
    for (int i=_next.size()-1; i>=0; --i)
        ans *= _ingain[i] * _next[i]->Get_OutValue();
    _outvalue = ans;
}
PUnitModule MProduct::Get_child(unsigned int n) const
{
    if (n >= (unsigned int)_next.size())
        return nullptr;
    return _next[n];
}
void MProduct::connect(const PUnitModule m)
{
    _next.push_back(m);
    _ingain.push_back(1);
    _enable = true;
}
double MProduct::disconnect(int n)
{
    SIMUCPP_ASSERT_ERROR(n<(int)_next.size(), "internal error: disconnect");
    double ingaintemp = _ingain[n];
    _next.erase(_next.begin()+n);
    _ingain.erase(_ingain.begin()+n);
    return ingaintemp;
}


/**********************
SUM module.
**********************/
MSum::~MSum() { _next.clear();_ingain.clear(); }
double MSum::Get_OutValue() const { return _outvalue; }
void MSum::Set_Enable(bool enable) { _enable=enable; }
void MSum::Module_Reset() {}
int MSum::Get_childCnt() const { return _next.size(); }
MSum::MSum(Simulator *sim, std::string name): UnitModule(name)
{
    _outvalue = 0;
    ADD_SIMULATOR();
}
void MSum::Set_InputGain(double inputgain, int port)
{
    if (port==-1){
        SIMUCPP_ASSERT_ERROR(_next.size()>0,
            "SUM module \"" << _name << "\" doesn't have enough child module.");
        _ingain[_next.size()-1] = inputgain;
    }
    else{
        SIMUCPP_ASSERT_ERROR(port>=0 && port<(int)_next.size(),
            "SUM module \"" << _name << "\" doesn't have enough child module.");
        _ingain[port] = inputgain;
    }
}
int MSum::Self_Check() const
{
    SIMUCPP_ASSERT_WARNING(_next.size()>0,
        "SUM module \"" << _name << "\" doesn't have enough child module.");
    if (_next.size()==0) return SIMUCPP_NO_CHILD;
    for (int i=0; i<(int)_next.size(); ++i)
        if (_next[i]==nullptr) return SIMUCPP_NULLPTR;
    return 0;
}
void MSum::Module_Update(double time)
{
    if (!_enable) return;
    double ans = 0;
    for (int i=_next.size()-1; i>=0; --i)
        ans += _ingain[i] * _next[i]->Get_OutValue();
    _outvalue = ans;
}
PUnitModule MSum::Get_child(unsigned int n) const
{
    if (n >= (unsigned int)_next.size())
        return nullptr;
    return _next[n];
}
void MSum::connect(const PUnitModule m)
{
    _next.push_back(m);
    _ingain.push_back(1);
    _enable = true;
}
double MSum::disconnect(int n)
{
    SIMUCPP_ASSERT_ERROR(n<(int)_next.size(), "internal error: disconnect");
    double ingaintemp = _ingain[n];
    _next.erase(_next.begin()+n);
    _ingain.erase(_ingain.begin()+n);
    return ingaintemp;
}


/**********************
TRANSPORTDELAY module.
**********************/
MTransportDelay::~MTransportDelay() { _next=nullptr; }
double MTransportDelay::Get_OutValue() const { return _outvalue; }
void MTransportDelay::Set_Enable(bool enable) { _enable=enable; }
void MTransportDelay::Set_InitialValue(double value) { _outvalue=_iv=value; }
int MTransportDelay::Get_childCnt() const { return 1; }
PUnitModule MTransportDelay::Get_child(unsigned int n) const { return n==0?_next:nullptr; }
void MTransportDelay::connect(const PUnitModule m) { _next=m;_enable=true; }
MTransportDelay::MTransportDelay(Simulator *sim, std::string name): UnitModule(name)
{
    _iv = _outvalue = 0;
    _lv.push_back(0);
    _next = nullptr;
    ADD_SIMULATOR();
    _nexttime = _simstep = sim->Get_SimStep();
}
void MTransportDelay::Set_DelayTime(double time)
{
    int delayPoints = time/_simstep + 0.5;
    SIMUCPP_ASSERT_WARNING(delayPoints>0,
        "TRANSPORTDELAY module was given an improper delay time.");
    if (delayPoints>0){
        _lv = vecdble(delayPoints);
        for (int i=_lv.size()-1; i>=0; --i)
            _lv[i] = _iv;
    }
    else {
        _lv = vecdble{};
    }
}
int MTransportDelay::Self_Check() const
{
    CHECK_CHILD(TRANSPORTDELAY);
    return 0;
}
void MTransportDelay::Module_Update(double time)
{
    if (!_enable) return;
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
void MTransportDelay::Module_Reset()
{
    for (int i=_lv.size()-1; i>=0; --i)
        _lv[i] = _iv;
}


/**********************
UNITDELAY module.
**********************/
MUnitDelay::~MUnitDelay() { _next=nullptr; }
double MUnitDelay::Get_OutValue() const { return _outvalue; }
void MUnitDelay::Set_Enable(bool enable) { _enable=enable; }
void MUnitDelay::Set_InitialValue(double value) { _outvalue=_lv=_iv=value; }
void MUnitDelay::Set_SampleTime(double time) { _T=time;_ltn=-_T; }
void MUnitDelay::Module_Reset() { _outvalue=_lv=_iv; }
int MUnitDelay::Get_childCnt() const { return 1; }
PUnitModule MUnitDelay::Get_child(unsigned int n) const { return n==0?_next:nullptr; }
void MUnitDelay::connect(const PUnitModule m) { _next=m;_enable=true; }
MUnitDelay::MUnitDelay(Simulator *sim, std::string name): UnitModule(name)
{
    DISCRETE_INITIALIZE(1);
    _iv = _lv = _outvalue = 0;
    _next = nullptr;
    ADD_SIMULATOR();
}
int MUnitDelay::Self_Check() const
{
    CHECK_CHILD(UNITDELAY);
    return 0;
}
void MUnitDelay::Module_Update(double time)
{
    if (!_enable) return;
    DISCRETE_UPDATE();
    _lv = _next->Get_OutValue();
}
void MUnitDelay::Output_Update(double time)
{
    if (time - _ltn < _T-SIMUCPP_DBL_EPSILON) return;
    _outvalue = _lv;
}


/**********************
ZOH module.
**********************/
MZOH::~MZOH() { _next=nullptr; }
double MZOH::Get_OutValue() const { return _outvalue; }
void MZOH::Set_Enable(bool enable) { _enable=enable; }
void MZOH::Set_SampleTime(double time) { _T=time;_ltn=-_T; }
void MZOH::Module_Reset() { _outvalue=0;_ltn=-_T; }
int MZOH::Get_childCnt() const { return 1; }
PUnitModule MZOH::Get_child(unsigned int n) const { return n==0?_next:nullptr; }
void MZOH::connect(const PUnitModule m) { _next=m;_enable=true; }
MZOH::MZOH(Simulator *sim, std::string name): UnitModule(name)
{
    DISCRETE_INITIALIZE(1);
    _outvalue = 0;
    _next = nullptr;
    ADD_SIMULATOR();
}
int MZOH::Self_Check() const
{
    CHECK_CHILD(ZOH);
    return 0;
}
void MZOH::Module_Update(double time)
{
    if (!_enable) return;
    DISCRETE_UPDATE();
    _outvalue = _next->Get_OutValue();
}

NAMESPACE_SIMUCPP_R
