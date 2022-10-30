#ifndef MATMODULES_H
#define MATMODULES_H
#include "unitmodules.hpp"
#ifdef USE_ZHNMAT
#include "zhnmat.hpp"
NAMESPACE_SIMUCPP_L
#define MATMODULE_VIRTUAL(classname) \
        friend class Simulator; \
    public: \
        virtual ~classname() override; \
        virtual bool Initialize() override; \
        virtual u8 Get_State() const override; \
        virtual PUnitModule Get_OutputPort(BusSize size) const override; \
        virtual BusSize Get_OutputBusSize() const override; \
        virtual void connect(const PMatModule m) override; \
    private: \
        BusSize _size


/**********************
Used to provide another method to replace the pointer to a function.
pure virture base class.
**********************/
class UserFuncM
{
public:
    UserFuncM();
    virtual ~UserFuncM() = 0;
    // Used for modules which need a MISO function.
    virtual zhnmat::Mat Function(zhnmat::Mat *u) const;
};


/*********************
MUX and DEMUX module.
Used to multiplex and demultiplex a bus port.
Their implementation are in file `multiplex.cpp`.
**********************/
class Mux: public MatModule {
    MATMODULE_VIRTUAL(Mux);
public:
    Mux(Simulator *sim=nullptr, BusSize size=BusSize(), std::string name="mx");
    void connect(PUnitModule m, BusSize n2=BusSize());
private:
    PUnitModule *_next=nullptr;
};

class DeMux: public MatModule {
    MATMODULE_VIRTUAL(DeMux);
public:
    DeMux(Simulator *sim=nullptr, BusSize size=BusSize(), std::string name="dmx");
    void connect(PUnitModule m, BusSize n2=BusSize());
private:
    PMatModule _next;
    PUGain *_gains=nullptr;
};


/*********************
StateSpace module.
It generates a 2-D state variables, for example a 2x3 matrix:
|x1 x2 x3|
|x4 x5 x6|
**********************/
class MStateSpace: public MatModule {
    MATMODULE_VIRTUAL(MStateSpace);
public:
    MStateSpace(Simulator *sim, BusSize size=BusSize(), bool isc=true, std::string name="mss");
    void Set_SampleTime(double time);
    void Set_InitialValue(const zhnmat::Mat& value);
    zhnmat::Mat Get_OutValue();
private:
    bool _isc;
    PMatModule _next;
    // Only one of the following two member variables can be a non null pointer.
    PUIntegrator *_intx = nullptr;
    PUUnitDelay *_udx = nullptr;
};


/*********************
matrix Constant module.
**********************/
class MConstant: public MatModule {
    MATMODULE_VIRTUAL(MConstant);
public:
    MConstant(Simulator *sim, const zhnmat::Mat& A, std::string name="mcst");
    void Set_OutValue(zhnmat::Mat A);
private:
    PUConstant *_ucst=nullptr;
};


/*********************
matrix FcnMISO module.
**********************/
class MFcnMISO: public MatModule {
    MATMODULE_VIRTUAL(MFcnMISO);
public:
    MFcnMISO(Simulator *sim, BusSize size, std::string name="mfcn");
    void Set_Function(zhnmat::Mat(*function)(zhnmat::Mat *u));
    void Set_Function(UserFuncM *function);
    zhnmat::Mat Get_OutValue();
private:
    PUFcnMISO *_misoy=nullptr;
    std::vector<PMatModule> _nexts;
    std::vector<BusSize> _buses;
    zhnmat::Mat(*_f)(zhnmat::Mat *m);
    UserFuncM *_fu=nullptr;
};


/*********************
matrix Gain module.
"isleft" refers to the side of matrix multiplication.
If "isleft=true" then y=Gx, else y=xG
**********************/
class MGain: public MatModule {
    MATMODULE_VIRTUAL(MGain);
public:
    MGain(Simulator *sim, const zhnmat::Mat& G, bool isleft=true, std::string name="mgn");
private:
    PUSum *_sumy = nullptr;  // Output port
    zhnmat::Mat _G;
    bool _isleft;
    PMatModule _next;
};


/*********************
matrix Product module.
**********************/
class MProduct: public MatModule {
    MATMODULE_VIRTUAL(MProduct);
public:
    MProduct(Simulator *sim, std::string name="mprd");
private:
    PUFcnMISO *_misoy = nullptr;
    PMatModule _nextL = nullptr;
    PMatModule _nextR = nullptr;
    u8 _portcnt;
};


/*********************
matrix Sum module.
**********************/
class MSum: public MatModule {
    MATMODULE_VIRTUAL(MSum);
public:
    MSum(Simulator *sim, std::string name="msum");
    void Set_InputGain(double inputgain, int port=-1);
private:
    PUSum *_sumy = nullptr;
    std::vector<double> _ingain;
    std::vector<PMatModule> _nexts;
};


/*********************
matrix Transpose module.
**********************/
class MTranspose: public MatModule {
    MATMODULE_VIRTUAL(MTranspose);
public:
    MTranspose(Simulator *sim, std::string name="mtsp");
private:
    PMatModule _next = nullptr;
};


NAMESPACE_SIMUCPP_R
#endif
#endif  // MATMODULES_H
