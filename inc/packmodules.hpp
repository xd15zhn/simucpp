#ifndef PACKMODULES_H
#define PACKMODULES_H
#include "simulator.hpp"
NAMESPACE_SIMUCPP_L


/**********************
Continuous transfer function module.
**********************/
class TransferFcn: public PackModule {
public:
    TransferFcn(Simulator *sim, const vecdble numerator, const vecdble denominator, std::string name="tf");
    virtual ~TransferFcn();
    virtual PUnitModule Get_InputPort(int n=0) const override;
    virtual PUnitModule Get_OutputPort(int n=0) const override;
    void Set_InitialValue(vecdble value);
    vecdble Get_OutValue();
private:
    UIntegrator **integrators = nullptr;
    USum *sum1 = nullptr;
    USum *sum2 = nullptr;
    int _order;
};


/**********************
Discrete transfer function module.
**********************/
class DiscreteTransferFcn: public PackModule {
public:
    DiscreteTransferFcn(Simulator *sim, const vecdble numerator, const vecdble denominator, std::string name="dtf");
    virtual ~DiscreteTransferFcn();
    virtual PUnitModule Get_InputPort(int n=0) const override;
    virtual PUnitModule Get_OutputPort(int n=0) const override;
    void Set_SampleTime(double time);
    void Set_InitialValue(vecdble value);
    vecdble Get_OutValue();
private:
    UUnitDelay **unitdelays = nullptr;
    USum *sum1 = nullptr;
    USum *sum2 = nullptr;
    int _order;
};


/*********************
Discrete time integrator module.
**********************/
class DiscreteIntegrator: public PackModule {
public:
    DiscreteIntegrator(Simulator *sim, std::string name="disint");
    virtual ~DiscreteIntegrator() {}
    virtual PUnitModule Get_InputPort(int n=0) const override;
    virtual PUnitModule Get_OutputPort(int n=0) const override;
    void Set_SampleTime(double time);
private:
    UUnitDelay *delay1 = nullptr;
    UZOH *zoh1 = nullptr;
    USum *sum1 = nullptr;
    UGain *in1 = nullptr;
    double _T;
};


#ifdef USE_ZHNMAT
/**********************
State transfer function module.
**********************/
class StateTransFcn: public PackModule {
public:
    StateTransFcn(Simulator *sim, const zhnmat::Mat& A, const zhnmat::Mat& B, const zhnmat::Mat& C,
        bool isc=true, std::string name="stf");
    virtual ~StateTransFcn() {}
    virtual PMatModule Get_InputBus(int n=0) const override;
    virtual PMatModule Get_OutputBus(int n=0) const override;
    void Set_InitialValue(const zhnmat::Mat& value);
    zhnmat::Mat Get_OutValue() const;
    void Set_SampleTime(double time);
private:
    MStateSpace *_statex=nullptr;
    MGain *_gainA=nullptr;
    MGain *_gainB=nullptr;
    MGain *_gainC=nullptr;
    MSum *_sum1=nullptr;
};


/**********************
Number multiplication of matrix.
Y=kX
**********************/
class ProductScalarMatrix: public PackModule
{
public:
    ProductScalarMatrix(Simulator *sim, BusSize size, std::string name="psv");
    virtual ~ProductScalarMatrix() {}
    virtual PUnitModule Get_InputPort(int n=0) const override;
    virtual PMatModule Get_InputBus(int n=0) const override;
    virtual PMatModule Get_OutputBus(int n=0) const override;
    zhnmat::Mat Get_OutValue();
private:
    BusSize _size;
    PUnitModule _ugainin=nullptr;
    PDeMux _dmxin=nullptr;
    PMux _mxout=nullptr;
    PUProduct *_prd=nullptr;
};
#endif


NAMESPACE_SIMUCPP_R
#endif  // PACKMODULES_H
