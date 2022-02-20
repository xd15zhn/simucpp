#include "simucpp.hpp"
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
    std::vector<double> num = numerator;
    std::vector<double> den = denominator;
    int order = den.size()-1;
    if (den[0]!=1){
        for (int i=num.size()-1; i>=0; --i)
            num[i] /= den[0];
        for (int i=den.size()-1; i>=0; --i)
            den[i] /= den[0];
    }
    //std::vector<double> numtemp(den.size());
    for (int i=0; i<(int)(denominator.size()-numerator.size()); ++i){
        num.insert(num.begin(), 0);
    }
    sum1 = new MSum(sim, _name+"_sumi");
    sum2 = new MSum(sim, _name+"_sumo");
    integrators = new MIntegrator*[order];
    for (int i=0; i<order; ++i){
        integrators[i] = new MIntegrator(sim, _name+"_int"+std::to_string(i));
        sim->connect(integrators[i], sum1);
        sum1->Set_InputGain(-den[i+1]);
        if (i==0) sim->connect(sum1, integrators[i]);
        else      sim->connect(integrators[i-1], integrators[i]);
    }
    for (int i=0; i<=order; ++i){
        if (num[i] == 0) continue;
        if (i == 0) sim->connect(sum1, sum2);
        else        sim->connect(integrators[i-1], sum2);
        sum2->Set_InputGain(num[i]);
    }
}
PUnitModule TransferFcn::Get_InputPort(int n) const { return n==0?sum1:nullptr; }
PUnitModule TransferFcn::Get_OutputPort(int n) const { return n==0?sum2:nullptr; }


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
    delays = new MUnitDelay*[_order];
    for (int i=0; i<_order; ++i){
        delays[i] = new MUnitDelay(sim, _name+"_ud"+std::to_string(i));
        if (i<(int)denominator.size()){
            sim->connect(delays[i], sum1);
            sum1->Set_InputGain(denominator[i]);
        }
        if (i<(int)numerator.size()-1){
            sim->connect(delays[i], sum2);
            sum2->Set_InputGain(numerator[i+1]);
        }
        if (i==0)
            sim->connect(sum1, delays[i]);
        else
            sim->connect(delays[i-1], delays[i]);
    }
    sim->connect(sum1, sum2);
    sum2->Set_InputGain(numerator[0]);
}
void DiscreteTransferFcn::Set_SampleTime(double time)
{
    for (int i=0; i<_order; ++i)
        delays[i]->Set_SampleTime(time);
}
PUnitModule DiscreteTransferFcn::Get_InputPort(int n) const { return n==0?sum1:nullptr; }
PUnitModule DiscreteTransferFcn::Get_OutputPort(int n) const { return n==0?sum2:nullptr; }


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
SISO state space module.
**********************/
StateSpaceSISO::StateSpaceSISO(Simulator *sim, const zhnmat::Mat& A, const zhnmat::Mat& B,
    const zhnmat::Mat& C, double D, std::string name) :PackModule(name)
{
    int order = A.row();
    SIMUCPP_ASSERT_ERROR(A.col()==order, "Shape of matrix A error!");
    SIMUCPP_ASSERT_ERROR(B.col()==1, "Shape of matrix B error!");
    SIMUCPP_ASSERT_ERROR(C.row()==1, "Shape of matrix C error!");
    SIMUCPP_ASSERT_ERROR(B.row()==order, "Shape of matrix B error!");
    SIMUCPP_ASSERT_ERROR(C.col()==order, "Shape of matrix C error!");
    integrators = new MIntegrator*[order];
    sumx = new MSum*[order];
    sumy = new MSum(sim, name+"_sumy");
    in1 = new MConnector(sim, name+"_in1");
    for (int i=0; i<order; ++i)
        integrators[i] = new MIntegrator(sim, name+"_int"+std::to_string(i));
    for (int i=0; i<order; ++i) {
        sumx[i] = new MSum(sim, name+"_sumx"+std::to_string(i));
        for (int j=0; j<order; ++j) {
            sim->connect(integrators[j], sumx[i]);
            sumx[i]->Set_InputGain(A.at(i, j));
        }
        sim->connect(in1, sumx[i]);
        sumx[i]->Set_InputGain(B.at(i, 0));
        sim->connect(sumx[i], integrators[i]);
    }
    for (int i=0; i<order; ++i) {
        sim->connect(integrators[i], sumy);
        sumy->Set_InputGain(C.at(0, i));
    }
    sim->connect(in1, sumy);
    sumy->Set_InputGain(D);
}
PUnitModule StateSpaceSISO::Get_InputPort(int n) const { return n==0?in1:nullptr; }
PUnitModule StateSpaceSISO::Get_OutputPort(int n) const { return n==0?sumy:nullptr; }
#endif
NAMESPACE_SIMUCPP_R
