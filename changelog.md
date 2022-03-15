# V1.7.1
Add state space module and include "zhnmat" library.

# V1.7.2
Change continuous state space module from SISO to a general model.
Add 2 functions that user can set the initial value and get the output value
 of StateSpace, TransferFcn and DiscreteTransferFcn.
Add version.
Fix some bugs in destructor of Simulator.

# V1.7.3
Additional considerations are added to the convergence check
Add version() function to Simulator.

# V1.7.4
Add library matplotlib-cpp for waveform plot.
Delete functions of saving waveform data to files.

# V1.7.5
Add functions of saving waveform data to files again.
Change default state of ZOH module from disable to enable.
Change default output value of CONSTANT module from 0 to 1.
Fix a bug which has a slight effect on simulation accuracy.

# V1.7.6
Fix some bugs for DiscreteTransferFcn.
Add Simulate_FirstStep() function again.
Fix some little spell errors.
Add some default values.

# V1.7.7
Add pack module StateGain.

# V1.7.8
Fix some bugs of Simulation_Reset().
Fix a bug of deleting CONNECTOR module.
Add Set_DivergenceCheckMode() function.

# V1.7.9
Fix a bug of building Sequence Table.
Add some conditions for Set_DivergenceCheckMode() function.

# V1.7.10
Change the logic for disconnect() of MISO modules.
