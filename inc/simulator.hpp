/**********************
FILE DESCRIPTIONS
This file contains the class defination of Simulator.
**********************/
#ifndef SIMUCPP_SIMULATOR_H
#define SIMUCPP_SIMULATOR_H
#include "matmodules.hpp"
NAMESPACE_SIMUCPP_L


class Simulator
{
    // All kinds of unit modules.
    friend class UConstant;
    friend class UFcn;
    friend class UFcnMISO;
    friend class UGain;
    friend class UInput;
    friend class UIntegrator;
    friend class UNoise;
    friend class UOutput;
    friend class UProduct;
    friend class USum;
    friend class UTransportDelay;
    friend class UUnitDelay;
    friend class UZOH;
    // All kinds of mat modules.
    friend class Mux;
    friend class DeMux;
    friend class MStateSpace;
    friend class MGain;
    friend class MSum;
public:
    // Init a simulator with a duration.
    Simulator(double duration=10);
    ~Simulator();

    /**********************
    The following 3 groups of functions are uesd to build connections
     between modules. Each function accepts 2 parameters of modules "m1"
     and "m2", and usually 2 parameters of orders "n1" and "n2", means to
     connect "n1"th output port of "m1" to "n2"th input port of "m2".
    Unit modules and matrix modules don't need to be specified which port
     of them should be connected.
    As for PackModule, default zero if port is not specified.
    **********************/
    // The following functions are used to build a single line connection between:
    //  unit module and unit module
    //  unit module and pack module
    //  pack module and pack module
    void connectU(PUnitModule m1, PUnitModule m2);
    void connectU(PUnitModule m1, PPackModule m2, int n2=0);
    void connectU(PPackModule m1, int n1, PUnitModule m2);
    void connectU(PPackModule m1, PUnitModule m2);
    void connectU(PPackModule m1, int n1, PPackModule m2, int n2=0);
    void connectU(PPackModule m1, PPackModule m2, int n2=0);

    // The following functions are used to build a bus connection between:
    //  matrix module and matrix module
    //  matrix module and pack module
    //  pack module and pack module
    void connectM(PMatModule m1, PMatModule m2);
    void connectM(PMatModule m1, PPackModule m2, int n2=0);
    void connectM(PPackModule m1, int n1, PMatModule m2);
    void connectM(PPackModule m1, PMatModule m2);
    void connectM(PPackModule m1, int n1, PPackModule m2, int n2=0);
    void connectM(PPackModule m1, PPackModule m2, int n2=0);

    // The following functions are used to build a single line connection between:
    //  unit module and multiplex module
    //  pack module and multiplex module
    //  demultiplex module and unit module
    //  demultiplex module and pack module
    //  demultiplex module and multiplex module
    void connectU(PUnitModule m1, PMux m2, BusSize n2=BusSize());  // unit to mux
    void connectU(PDeMux m1, BusSize n1, PUnitModule m2);  // demux to unit
    void connectU(PDeMux m1, PUnitModule m2);  // demux to unit
    void connectU(PDeMux m1, BusSize n1, PPackModule m2, int n2=0);  // demux to pack
    void connectU(PDeMux m1, PPackModule m2, int n2=0);  // demux to pack
    void connectU(PDeMux m1, BusSize n1, PMux m2, BusSize n2=BusSize());  // demux to mux
    void connectU(PDeMux m1, PMux m2, BusSize n2=BusSize());  // demux to mux
    void connectU(PPackModule m1, int n1, PMux m2, BusSize n2=BusSize());  // pack to nux
    void connectU(PPackModule m1, PMux m2, BusSize n2=BusSize());  // pack to mux
    // void connectM(PPackModule m1, int n1, PDeMux m2, BusSize n2=BusSize());  // pack to demux
    // void connectM(PPackModule m1, PDeMux m2, BusSize n2=BusSize());  // pack to demux
    /**********************
    Connect functions ended.
    **********************/

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

    // Set the output values of every pass-through modules to NaN, so that if there exists
    //  possible connection errors, they can be figured out immediately.
    void Set_DivergenceCheckMode();

    // Print current Simucpp version.
    static void VERSION();

private:
    // Add a module to this simulator.
    void Add_Module(const PUnitModule m);

    void Add_Module(const PMatModule m);
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
    std::vector<PMatModule> _matmodules;
    std::vector<PUIntegrator> _integrators;
    std::vector<PUOutput> _outputs;
    std::vector<PUUnitDelay> _unitdelays;

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
    // See public member function "Set_SampleTime".
    DISCRETE_VARIABLES();

    // See public member function "Set_t" and "Get_t".
    double _t;
    std::vector<double> _tvec;

    // See public member function "Set_WarningLevel" and "Set_DivergenceCheckMode".
    int _errlevel, _divmode;
    bool _diverge;
};

NAMESPACE_SIMUCPP_R
#endif // SIMULATOR_H
