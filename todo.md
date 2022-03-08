# bugs

# hidden danger
- Errors may occur when connecting two CONNECTOR modules. However, based on the fact
that all Unit modules have only one output port, CONNECTOR modules should only be used
as input ports of a Pack module.
- Unit modules which have multiple input ports will reconnect to another module when
ddeleting CONNECTOR modules, so it is not recommand for users to define a new Unit
module with multiple input ports.

# todo lists
- Add a statistic for all errors and warnings and give each of them a serial number.
- Add a discrete solver.
- Add a code generator for faster simulations.
