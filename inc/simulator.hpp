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
    friend class MConstant;
    friend class MStateSpace;
    friend class MFcnMISO;
    friend class MGain;
    friend class MOutput;
    friend class MProduct;
    friend class MSum;
    friend class MTranspose;
public:
    // Init a simulator with end time.
    Simulator(double endtime=10);
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
#pragma region connect
    // The following functions are used to build a single line connection between:
    //  unit module and unit module
    //  unit module and pack module
    //  pack module and pack module
    void connectU(PUnitModule m1, PUnitModule m2);  // unit to unit
    void connectU(PUnitModule m1, PPackModule m2, int n2);  // unit to pack
    void connectU(PPackModule m1, int n1, PUnitModule m2);  // pack to unit
    void connectU(PPackModule m1, int n1, PPackModule m2, int n2);  // pack to pack

    // The following functions are used to build a bus connection between:
    //  matrix module and matrix module
    //  matrix module and pack module
    //  pack module and pack module
    void connectM(PMatModule m1, PMatModule m2);  // mat to mat
    void connectM(PMatModule m1, PPackModule m2, int n2);  // mat to pack
    void connectM(PPackModule m1, int n1, PMatModule m2);  // pack to mat
    void connectM(PPackModule m1, int n1, PPackModule m2, int n2);  // pack to pack

    // The following functions are used to build a single line connection between:
    //  unit module and multiplex module
    //  pack module and multiplex module
    //  demultiplex module and unit module
    //  demultiplex module and pack module
    //  demultiplex module and multiplex module
    void connectU(PUnitModule m1, PMux m2, BusSize n2);  // unit to mux
    void connectU(PDeMux m1, BusSize n1, PUnitModule m2);  // demux to unit
    void connectU(PPackModule m1, int n1, PMux m2, BusSize n2);  // pack to nux
    void connectU(PDeMux m1, BusSize n1, PPackModule m2, int n2);  // demux to pack
    void connectU(PDeMux m1, BusSize n1, PMux m2, BusSize n2);  // demux to mux
#pragma endregion connect

    // Update the connection between modules which belongs to THIS simulator.
    // It must be called before first simulation and has no need to be called afterward.
    // @print: Print all modules and their connections.
    void Initialize(bool print=false);

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

    // Whether to store simulation data to memory.
    // If set false, then function "Simulator::Plot()" won't be executed.
    // It will also set all the OUTPUT modules.
    void Set_EnableStore(bool store=true);
    // Set storage interval of all OUTPUT modules.
    // It means how long do every OUTPUT modules store a data.
    // Notice: It changes only OUTPUT modules, not others.
    void Set_SampleTime(double time=-1);

    // Draw a waveform by using the stored data in every OUTPUT modules.
    void Plot();

    // Get and set current simulation time.
    void Set_t(double t);
    double Get_t();
    // Get and set simulation end time.
    void Set_Endtime(double t);
    double Get_Endtime();
    // Get and set simulation step.
    void Set_SimStep(double step=0.001);
    double Get_SimStep();

    // Set how the simulator works when the simulation diverged.
    // 0: Default, Print a message and stop the program.
    // 1: Print a message and keep going on, and return a none-zero value after simulation.
    // 2: Keep going on and return a none-zero value after simulation.
    // 3: Print a message and stop the simulation, return a none-zero value.
    // 4: Stop the simulation, return a none-zero value.
    // other: the same as 2.
    void Set_DivergenceCheckMode(int mode=0);

private:
    // Add a module to this simulator.
    void Add_Module(const PUnitModule m);
    void Add_Module(const PMatModule m);

    // Build connection of Endpoint modules.
    void Build_Connection(std::vector<uint> &ids);
    // Print all modules and their connections.
    void Print_Modules();

    // Simulation step and end time.
    double _H, _endtime;

    // Number of total modules, INTEGRATOR/UNITDELAY/OUTPUT modules.
    uint _cntM, _cntI, _cntD, _cntO;

    // Parameters for 4-order runge-kutta algorithm.
    double *_ode4K[4];

    // Temporarily save output value of every integrator.
    std::vector<double> _outref;

    // Pointers to every unit modules which belongs to this simulator.
    std::vector<PUnitModule> _modules;
    // Pointers to every mat modules which belongs to this simulator.
    // Mat modules will be broke up during initialization.
    std::vector<PMatModule> _matmodules;

    // 3 kinds of pointers below are pointers to endpoint modules.
    // The reason why they should exist is they all have some private members or
    //  member functions that different to others and should be treated distunguishly.
    // "_integrators" has private member variations "_outvalue"
    //  which will be called in "Simulate_OneStep()".
    // "_outputs" has private member variations "_values"
    //  which will be called in "Plot()".
    // "_unitdelays" has private member functions "Output_Update()"
    //  which will be called in "Simulate_OneStep()".
    std::vector<PUIntegrator> _integrators;
    std::vector<PUOutput> _outputs;
    std::vector<PUUnitDelay> _unitdelays;

    // IDs of every Endpoint modules according to the their updating orders.
    // First ID of every vector is an Endpoint module.
    // @_discIDs: Its name is "_allIDs" in previous version. Itis used to make
    //  sure that every modules will update only once in every simulation step,
    //  and it will be reused as discrete ids after building sequence table.
    std::vector<std::vector<uint>> _integIDs, _delayIDs, _outIDs;
    std::vector<int> _discIDs;

    DISCRETE_VARIABLES;  // See public member function "Set_SampleTime".
    double _t;  // See public member function "Set_t" and "Get_t".
    std::vector<double> _tvec;
    int _divmode;  // See public member function "Set_DivergenceCheckMode".

    // BIT0: initialized
    // BIT1: diverged
    // BIT2: data store
    u8 _status;
};

NAMESPACE_SIMUCPP_R
#endif // SIMULATOR_H
