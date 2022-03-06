#include <iostream>
#include "packmodules.hpp"
#include "definitions.hpp"
NAMESPACE_SIMUCPP_L


simucpp::UnitModule::UnitModule(std::string name): _name(name),_id(-1){}
simucpp::UnitModule::~UnitModule() {}
std::string simucpp::UnitModule::Get_Name() { return _name; }

/**********************
Continuous transfer function module.
**********************/
TransferFcn::TransferFcn(Simulator *sim, const vecdble numerator,
    const vecdble denominator, std::string name) :PackModule(name)
{
    SIMUCPP_ASSERT_ERROR(denominator.size()>=2,
        "Length of the denominator must be equal to or higher than 2!");
    SIMUCPP_ASSERT_ERROR(numerator.size()>=1,
        "Length of the numerator must be equal to or higher than 1!");
    SIMUCPP_ASSERT_ERROR(denominator.size()>=numerator.size(),
        "The order of the denominator must be equal to or higher than the order of the numerator!");
    SIMUCPP_ASSERT_ERROR(denominator[0]!=0,
        "The highest order of the denominator must not be 0!");
    vecdble num = numerator;
    vecdble den = denominator;
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
    sum1 = new MSum(sim, _name+"_sumi");
    sum2 = new MSum(sim, _name+"_sumo");
    integrators = new MIntegrator*[_order];
    for (int i=0; i<_order; ++i){
        integrators[i] = new MIntegrator(sim, _name+"_int"+std::to_string(i));
        sim->connect(integrators[i], sum1);
        sum1->Set_InputGain(-den[i+1]);
        if (i==0) sim->connect(sum1, integrators[i]);
        else      sim->connect(integrators[i-1], integrators[i]);
    }
    for (int i=0; i<=_order; ++i){
        if (num[i] == 0) continue;
        if (i == 0) sim->connect(sum1, sum2);
        else        sim->connect(integrators[i-1], sum2);
        sum2->Set_InputGain(num[i]);
    }
}
PUnitModule TransferFcn::Get_InputPort(int n) const { return n==0?sum1:nullptr; }
PUnitModule TransferFcn::Get_OutputPort(int n) const { return n==0?sum2:nullptr; }
void TransferFcn::Set_InitialValue(vecdble value)
{
    SIMUCPP_ASSERT_WARNING((int)value.size()==_order,
        "State space module \""<<_name<<"\" accepted mismatched initial values.");
    for (int i=SIMUCPP_MIN((int)value.size(), _order)-1; i>=0; --i)
        integrators[i]->Set_InitialValue(value[i]);
}
vecdble TransferFcn::Get_OutValue()
{
    vecdble ans;
    for (int i=0; i<_order; ++i)
        ans.push_back(integrators[i]->Get_OutValue());
    return ans;
}


/**********************
Discrete transfer function module.
**********************/
DiscreteTransferFcn::DiscreteTransferFcn(Simulator *sim, const vecdble numerator,
    const vecdble denominator, std::string name) :PackModule(name)
{
    SIMUCPP_ASSERT_ERROR(denominator.size()>=1,
        "Length of the denominator must be equal to or higher than 1!");
    SIMUCPP_ASSERT_ERROR(numerator.size()>=1,
        "Length of the numerator must be equal to or higher than 1!");
    _order = SIMUCPP_MAX(numerator.size()-1, denominator.size());
    sum1 = new MSum(sim, _name+"_sumi");
    sum2 = new MSum(sim, _name+"_sumi");
    unitdelays = new MUnitDelay*[_order];
    for (int i=0; i<_order; ++i){
        unitdelays[i] = new MUnitDelay(sim, _name+"_ud"+std::to_string(i));
        if (i<(int)denominator.size()){
            sim->connect(unitdelays[i], sum1);
            sum1->Set_InputGain(denominator[i]);
        }
        if (i<(int)numerator.size()-1){
            sim->connect(unitdelays[i], sum2);
            sum2->Set_InputGain(numerator[i+1]);
        }
        if (i==0)
            sim->connect(sum1, unitdelays[i]);
        else
            sim->connect(unitdelays[i-1], unitdelays[i]);
    }
    sim->connect(sum1, sum2);
    sum2->Set_InputGain(numerator[0]);
}
void DiscreteTransferFcn::Set_SampleTime(double time)
{
    for (int i=0; i<_order; ++i)
        unitdelays[i]->Set_SampleTime(time);
}
PUnitModule DiscreteTransferFcn::Get_InputPort(int n) const { return n==0?sum1:nullptr; }
PUnitModule DiscreteTransferFcn::Get_OutputPort(int n) const { return n==0?sum2:nullptr; }
void DiscreteTransferFcn::Set_InitialValue(vecdble value)
{
    SIMUCPP_ASSERT_WARNING((int)value.size()==_order,
        "State space module \""<<_name<<"\" accepted mismatched initial values.");
    for (int i=SIMUCPP_MIN((int)value.size(), _order)-1; i>=0; --i)
        unitdelays[i]->Set_InitialValue(value[i]);
}
vecdble DiscreteTransferFcn::Get_OutValue()
{
    vecdble ans;
    for (int i=0; i<_order; ++i)
        ans.push_back(unitdelays[i]->Get_OutValue());
    return ans;
}


/**********************
Discrete time integrator module.
**********************/
DiscreteIntegrator::DiscreteIntegrator(Simulator *sim, std::string name)
    :PackModule(name)
{
    _T = 1;
    delay1 = new MUnitDelay(sim, name+"_ud");
    zoh1 = new MZOH(sim, name+"_zoh");
    sum1 = new MSum(sim, name+"_sum");
    in1 = new MConnector(sim, name+"_int");
    sim->connect(sum1, zoh1);
    sim->connect(zoh1, delay1);
    sim->connect(delay1, sum1);
    sim->connect(in1, sum1);
    sum1->Set_InputGain(_T);
}
void DiscreteIntegrator::Set_SampleTime(double time)
{
    _T = time;
    sum1->Set_InputGain(_T);
    delay1->Set_SampleTime(_T);
    zoh1->Set_SampleTime(_T);
}
PUnitModule DiscreteIntegrator::Get_InputPort(int n) const { return n==0?in1:nullptr; }
PUnitModule DiscreteIntegrator::Get_OutputPort(int n) const { return n==0?delay1:nullptr; }


#ifdef USE_ZHNMAT
/**********************
State space module.
**********************/
StateSpace::StateSpace(Simulator *sim, const zhnmat::Mat& A, const zhnmat::Mat& B,
    const zhnmat::Mat& C, const zhnmat::Mat& D, bool isc, std::string name) :PackModule(name)
{
    _orderx = A.row(); _orderu = B.col(); _ordery = C.row();
    SIMUCPP_ASSERT_ERROR(_orderx>=2, "Order too few!");
    SIMUCPP_ASSERT_ERROR(A.col()==_orderx, "Shape of matrix A error!");
    SIMUCPP_ASSERT_ERROR(B.row()==_orderx, "Shape of matrix B error!");
    SIMUCPP_ASSERT_ERROR(C.col()==_orderx, "Shape of matrix C error!");
    zhnmat::Mat tD;
    if ((D.col()==0) && (D.row()==0))
        tD = zhnmat::Mat(_ordery, _orderu);
    else {
        SIMUCPP_ASSERT_ERROR(D.row()==_orderu, "Shape of matrix D error!");
        SIMUCPP_ASSERT_ERROR(D.col()==_ordery, "Shape of matrix D error!");
        tD = D;
    }
    _isc = isc;
    if (_isc) integrators = new MIntegrator*[_orderx];
    else      unitdelays = new MUnitDelay*[_orderx];
    sumx = new MSum*[_orderx];
    sumy = new MSum*[_ordery];
    inus = new MConnector*[_orderu];
    for (int i=0; i<_orderx; ++i) {
        if (_isc) integrators[i] = new MIntegrator(sim, name+"_int"+std::to_string(i));
        else      unitdelays[i] = new MUnitDelay(sim, name+"_int"+std::to_string(i));
    }
    for (int i=0; i<_orderu; ++i)
        inus[i] = new MConnector(sim, name+"_inu"+std::to_string(i));
    for (int i=0; i<_orderx; ++i) {
        sumx[i] = new MSum(sim, name+"_sumx"+std::to_string(i));
        for (int j=0; j<_orderx; ++j) {
            if (_isc) sim->connect(integrators[j], sumx[i]);
            else      sim->connect(unitdelays[j], sumx[i]);
            sumx[i]->Set_InputGain(A.at(i, j));
        }
        for (int j=0; j<_orderu; ++j) {
            sim->connect(inus[j], sumx[i]);
            sumx[i]->Set_InputGain(B.at(i, j));
        }
            if (_isc) sim->connect(sumx[i], integrators[i]);
            else      sim->connect(sumx[i], unitdelays[i]);
    }
    for (int i=0; i<_ordery; ++i) {
        sumy[i] = new MSum(sim, name+"_sumy"+std::to_string(i));
        for (int j=0; j<_orderx; ++j) {
            if (_isc) sim->connect(integrators[j], sumy[i]);
            else      sim->connect(unitdelays[j], sumy[i]);
            sumy[i]->Set_InputGain(C.at(i, j));
        }
        for (int j=0; j<_orderu; ++j) {
            sim->connect(inus[j], sumy[i]);
            sumy[i]->Set_InputGain(tD.at(i, j));
        }
    }
}
PUnitModule StateSpace::Get_InputPort(int n) const
{
        if (n<0) return nullptr;
        if (n<_orderu) return inus[n];
        return nullptr;
}
PUnitModule StateSpace::Get_OutputPort(int n) const
{
        if (n<0) return nullptr;
        if (n<_ordery) return sumy[n];
        return nullptr;
}
void StateSpace::Set_SampleTime(double time)
{
    if (_isc) return;
    _T = time;
    for (int i=0; i<_orderx; ++i)
        unitdelays[i]->Set_SampleTime(_T);
}
void StateSpace::Set_InitialValue(vecdble value)
{
    SIMUCPP_ASSERT_WARNING((int)value.size()==_orderx,
        "State space module \""<<_name<<"\" accepted mismatched initial values.");
    for (int i=SIMUCPP_MIN((int)value.size(), _orderx)-1; i>=0; --i)
        integrators[i]->Set_InitialValue(value[i]);
}
vecdble StateSpace::Get_OutValue()
{
    vecdble ans;
    for (int i=0; i<_orderx; ++i)
        ans.push_back(integrators[i]->Get_OutValue());
    return ans;
}

#endif
NAMESPACE_SIMUCPP_R
