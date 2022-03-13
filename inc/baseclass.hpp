/**********************
FILE DESCRIPTIONS
This file contains the following class definations:
class UserFunc
class UnitModule
class PackModule
class Simulator
**********************/
#ifndef BASECLASS_H
#define BASECLASS_H
#include <vector>
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


/**********************
Simulator
**********************/
class Simulator
{
    // Here are all kinds of modules.
    friend class MConnector;
    friend class MConstant;
    friend class MFcn;
    friend class MFcnMISO;
    friend class MGain;
    friend class MInput;
    friend class MIntegrator;
    friend class MNoise;
    friend class MOutput;
    friend class MProduct;
    friend class MSum;
    friend class MTransportDelay;
    friend class MUnitDelay;
    friend class MZOH;
public:
    // Init a simulator with a duration.
    Simulator(double duration=10);
    ~Simulator();

    // Connect "n1"th output port of m1 to "n2"th input port of m2.
    // UnitModule doesn't need to specify which port should be connected.
    // As for PackModule, default zero if port is not specified.
    void connect(PUnitModule m1, PUnitModule m2);
    void connect(PUnitModule m1, PPackModule m2, int n2=0);
    void connect(PPackModule m1, int n1, PUnitModule m2);
    void connect(PPackModule m1, PUnitModule m2);
    void connect(PPackModule m1, int n1, PPackModule m2, int n2=0);
    void connect(PPackModule m1, PPackModule m2, int n2=0);

    // Update the connection between modules which belongs to THIS simulator.
    // It must be called before first simulation and has no need to be called afterward.
    void Initialize();

    // Reset the parameters of all modules to those before simulation,
    // especially those modules whose values will change during simulation.
    // This function doesn't need to be called for the first simulation,
    // and it should be called from the second and subsequent simulation.
    void Simulation_Reset();

    // Run a simulation once or step by step.
    // They return 0 for everything is normal and others for some errors.
    int Simulate();
    int Simulate_FirstStep();
    int Simulate_OneStep();
    int Simulate_FinalStep();

    // Is simulation data stored to memory.
    // It will also set all the OUTPUT modules.
    void Set_EnableStore(bool store=true);
    // Is simulation data printed to a file.
    // It will also set all the OUTPUT modules.
    void Set_EnablePrint(bool print=false);
    // Set the precision of floating point data printed to a file.
    void Set_PrintPrecision(unsigned int n=8);

    // Draw a waveform by using the stored data in every OUTPUT modules.
    void Plot();

    // Set storage duration of OUTPUT modules. It means how long do every OUTPUT
    //  modules store a data.
    void Set_SampleTime(double time=-1);

    // Get and set current simulation time.
    void Set_t(double t);
    double Get_t();
    // Get and set simulation duration.
    void Set_Duration(double t);
    double Get_Duration();
    // Get and set simulation step.
    void Set_SimStep(double step=0.001);
    double Get_SimStep();

    // Change the warning level in self-check procedure.
    // For example, if a module doesn't have a child module, simulator will give
    //  an error and stop the program by default, but you can ignore this kind
    //  of errors by setting the warning level to 1.
    // 0: Default;
    // 1: ignore all errors and only give warnings;
    // -1: any warning will be considered an error.
    void Set_WarningLevel(int level=0);

    // Set how the simulator works when the simulation diverged.
    // 0: Default, Print a message and stop the program.
    // 1: Print a message and keep going on, and return a none-zero value after simulation.
    // 2: Keep going on and return a none-zero value after simulation.
    // 3: Print a message and stop the simulation, return a none-zero value.
    // 4: Stop the simulation, return a none-zero value.
    // other: the same as 2.
    void Set_DivergenceCheckMode(int mode=0);

    // Print current Simucpp version.
    static void VERSION();

private:
    // Add a module to this simulator.
    void Add_Module(const PUnitModule m);
    // Build connection of Endpoint modules.
    void Build_Connection(std::vector<int> &ids);

    // Simulation step and duration.
    double _H, _duration;

    // Number of total modules, INTEGRATOR/UNITDELAY/OUTPUT modules.
    int _cntM, _cntI, _cntD, _cntO;

    // Parameters for 4-order runge-kutta algorithm.
    double *_ode4K[4];

    // Temporarily save output value of every integrator.
    std::vector<double> _outref;

    // Pointers to every modules which belongs to this simulator.
    std::vector<PUnitModule> _modules;
    std::vector<PMIntegrator> _integrators;
    std::vector<PMOutput> _outputs;
    std::vector<PMUnitDelay> _unitdelays;

    // IDs of every Endpoint modules according to the their updating orders.
    // First ID of every vector is an Endpoint module.
    // @_discIDs: Its name is "_allIDs" in previous version. Itis used to make
    //  sure that every modules will update only once in every simulation step,
    //  and it will be reused as discrete ids after building sequence table.
    std::vector<std::vector<int>> _integIDs, _delayIDs, _outIDs;
    std::vector<int> _discIDs;

    // See public member function "Set_EnableStore" and "Set_EnablePrint".
    bool _store, _print;
    int _precision;
    std::fstream *_fp;
    // See public member function and "Set_SampleTime".
    DISCRETE_VARIABLES();

    // See public member function "Set_t" and "Get_t".
    double _t;
    std::vector<double> _tvec;

    // See public member function "Set_WarningLevel" and "Set_DivergenceCheckMode".
    int _errlevel, _divmode;
    bool _diverge;
};

NAMESPACE_SIMUCPP_R
#endif // BASECLASS_H
