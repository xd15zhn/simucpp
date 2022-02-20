#ifndef MISCELLANEOUS_H
#define MISCELLANEOUS_H
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
private:
    MIntegrator **integrators = nullptr;
    MSum *sum1 = nullptr;
    MSum *sum2 = nullptr;
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
private:
    MUnitDelay **delays = nullptr;
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
SISO state space module.
**********************/
class StateSpaceSISO: public PackModule
{
public:
    StateSpaceSISO(Simulator *sim, const zhnmat::Mat& A, const zhnmat::Mat& B,
        const zhnmat::Mat& C, double D, std::string name="space");
    virtual PUnitModule Get_InputPort(int n=0) const;
    virtual PUnitModule Get_OutputPort(int n=0) const;
private:
    MIntegrator **integrators = nullptr;
    MSum **sumx = nullptr;
    MSum *sumy = nullptr;
    MConnector *in1 = nullptr;
};
#endif

NAMESPACE_SIMUCPP_R
#endif
