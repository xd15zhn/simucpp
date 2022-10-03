#include "packmodules.hpp"
#include "definitions.hpp"
NAMESPACE_SIMUCPP_L

PackModule::PackModule(Simulator *sim, std::string name): _name(name) {};
PackModule::~PackModule() {}
PUnitModule PackModule::Get_InputPort(int n) const { return nullptr; }
PUnitModule PackModule::Get_OutputPort(int n) const { return nullptr; }
PMatModule PackModule::Get_InputBus(int n) const { return nullptr; }
PMatModule PackModule::Get_OutputBus(int n) const { return nullptr; }

/**********************
Continuous transfer function module.
**********************/
TransferFcn::TransferFcn(Simulator *sim, const vecdble numerator, const vecdble denominator, std::string name) {
    if (denominator.size()<2) TRACELOG(LOG_FATAL, "Length of the denominator must be equal to or higher than 2!");
    if (numerator.size()<1) TRACELOG(LOG_FATAL, "Length of the numerator must be equal to or higher than 1!");
    if (denominator.size()<numerator.size())
        TRACELOG(LOG_FATAL, "The order of the denominator must be equal to or higher than the order of the numerator!");
    if (denominator[0]==0) TRACELOG(LOG_FATAL, "The highest order of the denominator must not be 0!");
    vecdble num = numerator, den = denominator;
    _order = den.size()-1;
    if (den[0]!=1){
        for (int i=num.size()-1; i>=0; --i)
            num[i] /= den[0];
        for (int i=den.size()-1; i>=0; --i)
            den[i] /= den[0];
    }
    for (int i=0; i<(int)(denominator.size()-numerator.size()); ++i){
        num.insert(num.begin(), 0);
    }
    sum1 = new USum(sim, _name+"_sumi");
    sum2 = new USum(sim, _name+"_sumo");
    integrators = new UIntegrator*[_order];
    for (int i=0; i<_order; ++i) {
        integrators[i] = new UIntegrator(sim, _name+"_int"+std::to_string(i));
        sim->connectU(integrators[i], sum1);
        sum1->Set_InputGain(-den[i+1]);
        if (i==0) sim->connectU(sum1, integrators[i]);
        else      sim->connectU(integrators[i-1], integrators[i]);
    }
    for (int i=0; i<=_order; ++i) {
        if (num[i] == 0) continue;
        if (i == 0) sim->connectU(sum1, sum2);
        else        sim->connectU(integrators[i-1], sum2);
        sum2->Set_InputGain(num[i]);
    }
}
TransferFcn::~TransferFcn() {
    delete sum1, sum2;
    for (int i=0; i<_order; ++i) {
        delete integrators[i];
    }
    delete[] integrators;
}
PUnitModule TransferFcn::Get_InputPort(int n) const { return n==0?sum1:nullptr; }
PUnitModule TransferFcn::Get_OutputPort(int n) const { return n==0?sum2:nullptr; }
void TransferFcn::Set_InitialValue(vecdble value) {
    if ((int)value.size()!=_order) TRACELOG(LOG_WARNING, "TransferFcn module \"%s\" accepted mismatched initial values.", _name);
    for (int i=SIMUCPP_MIN((int)value.size(), _order)-1; i>=0; --i)
        integrators[i]->Set_InitialValue(value[i]);
}
vecdble TransferFcn::Get_OutValue() {
    vecdble ans;
    for (int i=0; i<_order; ++i)
        ans.push_back(integrators[i]->Get_OutValue());
    return ans;
}


/**********************
Discrete transfer function module.
**********************/
DiscreteTransferFcn::DiscreteTransferFcn(Simulator *sim, const vecdble numerator, const vecdble denominator, std::string name) {
    if ((int)denominator.size()<1) TRACELOG(LOG_FATAL, "Length of the denominator must be equal to or higher than 1!");
    if ((int)numerator.size()<1) TRACELOG(LOG_FATAL, "Length of the numerator must be equal to or higher than 1!");
    _order = SIMUCPP_MAX(numerator.size()-1, denominator.size());
    sum1 = new USum(sim, _name+"_sumi");
    sum2 = new USum(sim, _name+"_sumo");
    unitdelays = new UUnitDelay*[_order];
    for (int i=0; i<_order; ++i){
        unitdelays[i] = new UUnitDelay(sim, _name+"_ud"+std::to_string(i));
        if (i<(int)denominator.size()){
            sim->connectU(unitdelays[i], sum1);
            sum1->Set_InputGain(denominator[i]);
        }
        if (i<(int)numerator.size()-1){
            sim->connectU(unitdelays[i], sum2);
            sum2->Set_InputGain(numerator[i+1]);
        }
        if (i==0)
            sim->connectU(sum1, unitdelays[i]);
        else
            sim->connectU(unitdelays[i-1], unitdelays[i]);
    }
    sim->connectU(sum1, sum2);
    sum2->Set_InputGain(numerator[0]);
}
DiscreteTransferFcn::~DiscreteTransferFcn() {
    delete sum1, sum2;
    for (int i=0; i<_order; ++i) {
        delete unitdelays[i];
    }
    delete[] unitdelays;
}
void DiscreteTransferFcn::Set_SampleTime(double time) {
    for (int i=0; i<_order; ++i)
        unitdelays[i]->Set_SampleTime(time);
}
PUnitModule DiscreteTransferFcn::Get_InputPort(int n) const { return n==0?sum1:nullptr; }
PUnitModule DiscreteTransferFcn::Get_OutputPort(int n) const { return n==0?sum2:nullptr; }
void DiscreteTransferFcn::Set_InitialValue(vecdble value) {
    if ((int)value.size()!=_order) TRACELOG(LOG_WARNING, "DiscreteTransferFcn module \"%s\" accepted mismatched initial values.", _name);
    for (int i=SIMUCPP_MIN((int)value.size(), _order)-1; i>=0; --i)
        unitdelays[i]->Set_InitialValue(value[i]);
}
vecdble DiscreteTransferFcn::Get_OutValue() {
    vecdble ans;
    for (int i=0; i<_order; ++i)
        ans.push_back(unitdelays[i]->Get_OutValue());
    return ans;
}


/**********************
Discrete time integrator module.
**********************/
DiscreteIntegrator::DiscreteIntegrator(Simulator *sim, std::string name) {
    _T = 1;
    delay1 = new UUnitDelay(sim, name+"_ud");
    zoh1 = new UZOH(sim, name+"_zoh");
    sum1 = new USum(sim, name+"_sum");
    in1 = new UGain(sim, name+"_in");
    sim->connectU(sum1, zoh1);
    sim->connectU(zoh1, delay1);
    sim->connectU(delay1, sum1);
    sim->connectU(in1, sum1);
    sum1->Set_InputGain(_T);
}
void DiscreteIntegrator::Set_SampleTime(double time) {
    _T = time;
    sum1->Set_InputGain(_T);
    delay1->Set_SampleTime(_T);
    zoh1->Set_SampleTime(_T);
}
PUnitModule DiscreteIntegrator::Get_InputPort(int n) const { return n==0?in1:nullptr; }
PUnitModule DiscreteIntegrator::Get_OutputPort(int n) const { return n==0?delay1:nullptr; }


#ifdef USE_ZHNMAT
/**********************
State transfer function module.
**********************/
StateTransFcn::StateTransFcn(Simulator *sim, const zhnmat::Mat& A, const zhnmat::Mat& B, const zhnmat::Mat& C,
    const zhnmat::Mat& D, bool isc, std::string name): _isc(isc) {
    _orderx = A.row(); _orderu = B.col(); _ordery = C.row();
    if (_orderx<2) TRACELOG(LOG_FATAL, "Order too few!");
    if (A.col()!=_orderx) TRACELOG(LOG_FATAL, "Shape of matrix A error!");
    if (B.row()!=_orderx) TRACELOG(LOG_FATAL, "Shape of matrix B error!");
    if (C.col()!=_orderx) TRACELOG(LOG_FATAL, "Shape of matrix C error!");
    zhnmat::Mat tD;
    if ((D.col()==0) && (D.row()==0))
        tD = zhnmat::Mat(_ordery, _orderu);
    else {
        if (D.row()!=_orderu) TRACELOG(LOG_FATAL, "Shape of matrix D error!");
        if (D.col()!=_ordery) TRACELOG(LOG_FATAL, "Shape of matrix D error!");
        tD = D;
    }
    _statex = new MStateSpace(sim, BusSize(_orderx, 1), true, name+"_msx");
    _gainU = new MGain(sim, zhnmat::eye(_orderu), true, name+"_gainU");
    _gainA = new MGain(sim, A, true, name+"_gainA");
    _gainB = new MGain(sim, B, true, name+"_gainB");
    _gainC = new MGain(sim, C, true, name+"_gainC");
    _gainD = new MGain(sim, tD, true, name+"_gainD");
    _sum1 = new MSum(sim, name+"_msum1");
    _sum2 = new MSum(sim, name+"_msum2");
    sim->connectM(_statex, _gainA);  // Ax
    sim->connectM(_gainU, _gainB);  // Bu
    sim->connectM(_gainA, _sum1);  // Ax+Bu
    sim->connectM(_gainB, _sum1);  // Ax+Bu
    sim->connectM(_sum1, _statex);  // x'=Ax+Bu
    sim->connectM(_statex, _gainC);  // Cx
    sim->connectM(_gainU, _gainD);  // Du
    sim->connectM(_gainC, _sum2);  // Cx+Du
    sim->connectM(_gainD, _sum2);  // Cx+Du
}
PMatModule StateTransFcn::Get_InputBus(int n) const { return n==0?_gainU:nullptr; }
PMatModule StateTransFcn::Get_OutputBus(int n) const { return n==0?_sum2:nullptr; }
void StateTransFcn::Set_SampleTime(double time) {
    _statex->Set_SampleTime(time);
}
void StateTransFcn::Set_InitialValue(const zhnmat::Mat& value) {
    _statex->Set_InitialValue(value);
}
zhnmat::Mat StateTransFcn::Get_OutValue() const {
    return _statex->Get_OutValue();
}


/**********************
Number multiplication of matrix.
**********************/
PUnitModule ProductScalarMatrix::Get_InputPort(int n) const { return n==0?_ugainin:nullptr; }
PMatModule ProductScalarMatrix::Get_InputBus(int n) const { return n==0?_dmxin:nullptr; }
PMatModule ProductScalarMatrix::Get_OutputBus(int n) const { return n==0?_mxout:nullptr; }
ProductScalarMatrix::ProductScalarMatrix(Simulator *sim, BusSize size, std::string name)
    : PackModule(sim, name) {
    if (size<BusSize(1, 1)) return;
    _size = size;
    _ugainin = new UGain(sim, "_dmxin");
    _dmxin = new DeMux(sim, _size, "dmxin");
    _mxout = new Mux(sim, _size, "mxout");
    _prd = new PUProduct[_size.r*_size.c];
    for (uint i = 0; i < _size.r; i++) {
        for (uint j = 0; j < _size.c; j++) {
            _prd[i*_size.c+j] = new UProduct(sim, _name+"_prd"+std::to_string(i)+"_"+std::to_string(j));
            sim->connectU(_dmxin, BusSize(i, j), _prd[i*_size.c+j]);
            sim->connectU(_ugainin, _prd[i*_size.c+j]);
            sim->connectU(_prd[i*_size.c+j], _mxout, BusSize(i, j));
        }
    }
}
zhnmat::Mat ProductScalarMatrix::Get_OutValue() {
    zhnmat::Mat ans(_size.r, _size.c);
    for (uint i = 0; i < _size.r; i++)
        for (uint j = 0; j < _size.c; j++)
            ans.set(i, j, _prd[i*_size.c+j]->Get_OutValue());
    return ans;
}
#endif

NAMESPACE_SIMUCPP_R
