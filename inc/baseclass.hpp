/**********************
FILE DESCRIPTIONS
This file contains the following class definations:
class UserFunc
class UnitModule
class PackModule
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
class PackModule;
class MIntegrator;
class MOutput;
class MUnitDelay;
typedef Simulator* PSimulator;
typedef UnitModule* PUnitModule;
typedef PackModule* PPackModule;
typedef MIntegrator* PMIntegrator;
typedef MOutput* PMOutput;
typedef MUnitDelay* PMUnitDelay;
typedef std::vector<double> vecdble;

enum DIVERGENCE_MODE {
    DIVERGENCE_ABORT,
    DIVERGENCE_PRINT,
    DIVERGENCE_NONE,
};

/**********************
Used to provide another method to replace the pointer to a function.
**********************/
class UserFunc
{
public:
    // Used for MFCN module.
    virtual double Function(double u) const { return u; };
    // Used for MFCNMISO module.
    virtual double Function(double *u) const { return u[0]; };
};


/**********************
Base class of every unit modules.
**********************/
class UnitModule
{
    friend class Simulator;
public:
    UnitModule(std::string name="module");
    virtual ~UnitModule();
    virtual double Get_OutValue() const = 0;
    // Get the name of this module, which will be used in errors and warnings prompt.
    std::string Get_Name();

protected:
    // Name of this module.
    std::string _name;
    // Which simulator does this module belongs to.
    PSimulator _sim = nullptr;

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

    // Connect this module to its child module.
    virtual void connect(PUnitModule m) = 0;

    // ID of this module, which will be used as modules' subscript index,
    //  so it should start at 0, and its default value is -1,
    //  which implies that it does not belong to any simulator temporarily.
    int _id;
};


/**********************
Base class of every pack(combination) modules.
**********************/
class PackModule
{
    friend class Simulator;
public:
    PackModule(std::string name="module"): _name(name){};
protected:
    std::string _name;
private:
    // Get nth input/output module of this PackModule.
    // It is used to build connections with other modules.
    virtual PUnitModule Get_InputPort(int n=0) const = 0;
    virtual PUnitModule Get_OutputPort(int n=0) const = 0;
};

NAMESPACE_SIMUCPP_R
#endif // BASECLASS_H
