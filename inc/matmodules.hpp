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
        virtual void connect(const PMatModule m) override


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
    BusSize _size;
};

class DeMux: public MatModule {
    MATMODULE_VIRTUAL(DeMux);
public:
    DeMux(Simulator *sim=nullptr, BusSize size=BusSize(), std::string name="dmx");
    void connect(PUnitModule m, BusSize n2=BusSize());
private:
    PMatModule _next;
    PUGain *_gains=nullptr;
    BusSize _size;
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
    BusSize _size;
    bool _isc;
    PMatModule _next;
    // Only one of the following two member variables can be a non null pointer.
    PUIntegrator *_intx = nullptr;
    PUUnitDelay *_udx = nullptr;
};


/*********************
matrix Gain module.
"isleft" refers to the side of matrix multiplication.
Iif "isleft=true" then y=Gx, else y=xG
**********************/
class MGain: public MatModule {
    MATMODULE_VIRTUAL(MGain);
public:
    MGain(Simulator *sim, const zhnmat::Mat& G, bool isleft=true, std::string name="mgn");
private:
    PUSum *_sumy = nullptr;  // Output port
    BusSize _sizein, _sizeout;  // Bus size of input and output
    zhnmat::Mat _G;
    bool _isleft;
    PMatModule _next;
};


/*********************
matrix Sum module.
**********************/
class MSum: public MatModule {
    MATMODULE_VIRTUAL(MSum);
public:
    MSum(Simulator *sim, std::string name="msum");
private:
    BusSize _size;
    PUSum *_sumy = nullptr;
    std::vector<PMatModule> _nexts;
};


NAMESPACE_SIMUCPP_R
#endif
#endif  // MATMODULES_H
