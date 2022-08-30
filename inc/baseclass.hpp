/**********************
FILE DESCRIPTIONS
This file contains some base class definitions.
**********************/
#ifndef BASECLASS_H
#define BASECLASS_H
#include <string>
#define NAMESPACE_SIMUCPP_L                  namespace simucpp {
#define NAMESPACE_SIMUCPP_R                  }
#define SIMUCPP_LIMIT(x, min, max)           (((x)<=(min) ? (min) : ((x)>=(max) ? (max) : (x))))
#define SIMUCPP_MIN(a, b)                    ((a) < (b) ? (a) : (b))
#define SIMUCPP_MAX(a, b)                    ((a) > (b) ? (a) : (b))
#define DISCRETE_VARIABLES() \
    double _T, _ltn
NAMESPACE_SIMUCPP_L


class Simulator;
class UnitModule;
class MatModule;
class PackModule;
class Mux;
class DeMux;
class UIntegrator;
class UOutput;
class UUnitDelay;
typedef Simulator*           PSimulator;
typedef UnitModule*          PUnitModule;
typedef MatModule*           PMatModule;
typedef PackModule*          PPackModule;
typedef Mux*                 PMux;
typedef DeMux*               PDeMux;
typedef UIntegrator*         PUIntegrator;
typedef UOutput*             PUOutput;
typedef UUnitDelay*          PUUnitDelay;
typedef unsigned int         uint;
typedef unsigned char        u8;

enum DIVERGENCE_MODE {
    DIVERGENCE_ABORT,
    DIVERGENCE_PRINT,
    DIVERGENCE_NONE,
};

/**********************
Used to provide another method to replace the pointer to a function.
pure virture base class.
**********************/
class UserFunc
{
public:
    // Used for modules which need a SISO function.
    virtual double Function(double u) const = 0;
    // Used for modules which need a MISO function.
    virtual double Function(double *u) const = 0;
};


/**********************
The bus between two matrix modules has "row" and "column" properties.
It should be noticed that the cases between using BusSize as "size" and "point" is different.
For example, a bus with 3 rows and 4 columns has points from (0,0) to (2,3).
Its implementation is in file `matmodules.cpp`.
**********************/
class BusSize {
public:
    BusSize(uint row=0, uint col=0);
    BusSize(const BusSize& size);
    BusSize& operator=(const BusSize& size);
    bool operator==(const BusSize &size) const;
    // Check if a point is in a rectangle.
    // Here are some examples:
    //  (2,3)>=(1,1),(2,3)>=(1,3): true
    //  (2,3)>=(3,1),(2,3)>=(2,4): false
    bool operator<(const BusSize &size) const;
    uint r, c;
};


/**********************
Base class of every unit modules.
**********************/
class UnitModule
{
    friend class Simulator;
    friend class DeMux;
public:
    UnitModule(Simulator *sim=nullptr, std::string name="unitmodule");
    virtual ~UnitModule();
    virtual double Get_OutValue() const = 0;

protected:
    // Name of this unit module.
    std::string _name;
    // Which simulator does this module belongs to.
    PSimulator _sim = nullptr;
    // See private member function "Set_Enable".
    bool _enable;

private:
    // Enable or disable this module. Mainly used for discrete modules.
    virtual void Set_Enable(bool enable) = 0;

    // Self check. return 0 for everything is ok.
    virtual int Self_Check() const = 0;

    // Calculate output value for every simulation step.
    virtual void Module_Update(double time) = 0;

    // Reset inner parameters to prepare for next simulation.
    // It will be called by Simulator::Simulation_Reset().
    virtual void Module_Reset() = 0;

    // Return how many child modules does this module have.
    virtual int Get_childCnt() const = 0;

    // Get the pointer of this module's nth child module.
    virtual PUnitModule Get_child(unsigned int n=0) const = 0;

    // Connect the output port of "m" to the input port of this module.
    virtual void connect(PUnitModule m) = 0;

    // ID of this module, which will be used as modules' subscript index,
    //  so it should start at 0, and its default value is -1,
    //  which implies that it does not belong to any simulator temporarily.
    int _id;
};


/**********************
Base class of every matrix(state) modules.
**********************/
class MatModule
{
    friend class Simulator;
public:
    MatModule(Simulator *sim=nullptr, std::string name="matmodule");
    virtual ~MatModule();

    // Initialize this matrix module. Return false when failed to initialize.
    // Function 'Initialize()' will be called after module connections, in another word,
    //  it will be called in 'Simulator::Initialize()' function.
    virtual bool Initialize() = 0;
    virtual u8 Get_State() const = 0;

    // Return the output unit module given by bus point "size" of this matrix module.
    virtual PUnitModule Get_OutputPort(BusSize size) const = 0;
    virtual BusSize Get_OutputBusSize() const = 0;

    // Connect the output port of "m" to the input port of this module.
    virtual void connect(const PMatModule m) = 0;
protected:
    // Name of this matrix module.
    std::string _name;
    // Which simulator does this module belongs to.
    PSimulator _sim=nullptr;
    // bit0 indicates whether the size of this module is determined
    // bit1 indicates whether child module of is connected to this module
    u8 _state=0;
private:
};


/**********************
Base class of every pack(combination) modules.
**********************/
class PackModule
{
    friend class Simulator;
public:
    PackModule(Simulator *sim=nullptr, std::string name="packmodule");
    virtual ~PackModule() = 0;
protected:
    // Name of this pack module.
    std::string _name;
private:
    // Get nth input/output module of this PackModule.
    // It is used to build connections with other modules.
    virtual PUnitModule Get_InputPort(int n=0) const;
    virtual PUnitModule Get_OutputPort(int n=0) const;
    virtual PMatModule Get_InputBus(int n=0) const;
    virtual PMatModule Get_OutputBus(int n=0) const;
};


NAMESPACE_SIMUCPP_R
#endif // BASECLASS_H
