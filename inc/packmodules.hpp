#ifndef PACKMODULES_H
#define PACKMODULES_H
#include "unitmodules.hpp"
#ifdef USE_ZHNMAT
#include "zhnmat.hpp"
#endif
NAMESPACE_SIMUCPP_L


/**********************
Continuous transfer function module.
**********************/
class TransferFcn: public PackModule
{
public:
    TransferFcn(Simulator *sim, const vecdble numerator, const vecdble denominator, std::string name="tf");
    virtual PUnitModule Get_InputPort(int n=0) const;
    virtual PUnitModule Get_OutputPort(int n=0) const;
    void Set_InitialValue(vecdble value);
    vecdble Get_OutValue();
private:
    MIntegrator **integrators = nullptr;
    MSum *sum1 = nullptr;
    MSum *sum2 = nullptr;
    int _order;
};


/**********************
Discrete transfer function module.
**********************/
class DiscreteTransferFcn: public PackModule
{
public:
    DiscreteTransferFcn(Simulator *sim, const vecdble numerator, const vecdble denominator, std::string name="dtf");
    virtual PUnitModule Get_InputPort(int n=0) const;
    virtual PUnitModule Get_OutputPort(int n=0) const;
    void Set_SampleTime(double time);
    void Set_InitialValue(vecdble value);
    vecdble Get_OutValue();
private:
    MUnitDelay **unitdelays = nullptr;
    MSum *sum1 = nullptr;
    MSum *sum2 = nullptr;
    int _order;
};


/*********************
Discrete time integrator module.
**********************/
class DiscreteIntegrator: public PackModule
{
public:
    DiscreteIntegrator(Simulator *sim, std::string name="disint");
    virtual PUnitModule Get_InputPort(int n=0) const;
    virtual PUnitModule Get_OutputPort(int n=0) const;
    void Set_SampleTime(double time);
private:
    MUnitDelay *delay1 = nullptr;
    MZOH *zoh1 = nullptr;
    MSum *sum1 = nullptr;
    MConnector *in1 = nullptr;
    double _T;
};


#ifdef USE_ZHNMAT
/*********************
STATESPACE module.
Generate a state space module with matrix A,B,C,D as below:
x'=Ax+Bu
y=Cx+Du
You should specify that this module is working in continuous mode
 or discrete mode, default continuous.
Matrix D has an empty value by default.
**********************/
class StateSpace: public PackModule
{
public:
    StateSpace(Simulator *sim, const zhnmat::Mat& A, const zhnmat::Mat& B, const zhnmat::Mat& C,
        const zhnmat::Mat& D=zhnmat::Mat(), bool isc=true, std::string name="sts");
    virtual PUnitModule Get_InputPort(int n=0) const;
    virtual PUnitModule Get_OutputPort(int n=0) const;
    void Set_SampleTime(double time);
    void Set_InitialValue(vecdble value);
    vecdble Get_OutValue();
private:
    MIntegrator **integrators = nullptr;
    MUnitDelay **unitdelays = nullptr;
    MSum **sumx = nullptr;
    MSum **sumy = nullptr;
    MConnector **inus = nullptr;
    int _orderx, _orderu, _ordery;  // Order of states, inputs, outputs.
    bool _isc;  // True if in continuous mode.
    double _T;
};


/*********************
STATEGAIN module.
**********************/
class StateGain: public PackModule
{
public:
    StateGain(Simulator *sim, const zhnmat::Mat& G, std::string name="sgn");
    virtual PUnitModule Get_InputPort(int n=0) const;
    virtual PUnitModule Get_OutputPort(int n=0) const;
private:
    MSum **sumy = nullptr;
    MConnector **inu = nullptr;
    int _orderu, _ordery;  // Order of inputs, outputs.
};

#endif
NAMESPACE_SIMUCPP_R
#endif  // PACKMODULES_H
