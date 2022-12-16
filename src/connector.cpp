#include "simulator.hpp"
#include "definitions.hpp"
NAMESPACE_SIMUCPP_L


/**********************
The following functions are used to build a single line connection between:
 unit module and unit module
 unit module and pack module
 pack module and pack module
**********************/
void Simulator::connectU(PUnitModule m1, PUnitModule m2) {  // unit to unit
    CHECK_NULLPTR(m1, UnitModule); CHECK_NULLPTR(m2, UnitModule);
    CHECK_NULLID(m1, UnitModule); CHECK_NULLID(m2, UnitModule);
    CHECK_SIMULATOR(m1, UnitModule); CHECK_SIMULATOR(m2, UnitModule);
    m2->connect(m1);
}
void Simulator::connectU(PUnitModule m1, PPackModule m2, uint n2) {  // unit to pack
    CHECK_NULLPTR(m1, UnitModule); CHECK_NULLPTR(m2, PackModule);
    CHECK_NULLID(m1, UnitModule); CHECK_SIMULATOR(m1, UnitModule);
    PUnitModule bm2 = m2->Get_InputPort(n2);
    CHECK_NULLPTR(bm2, UnitModule); CHECK_NULLID(bm2, UnitModule); CHECK_SIMULATOR(bm2, UnitModule);
    bm2->connect(m1);
}
void Simulator::connectU(PPackModule m1, uint n1, PUnitModule m2) {  // pack to unit
    CHECK_NULLPTR(m1, PackModule); CHECK_NULLPTR(m2, UnitModule);
    CHECK_NULLID(m2, UnitModule); CHECK_SIMULATOR(m2, UnitModule);
    PUnitModule bm1 = m1->Get_OutputPort(n1);
    CHECK_NULLPTR(bm1, UnitModule); CHECK_NULLID(bm1, UnitModule); CHECK_SIMULATOR(bm1, UnitModule);
    m2->connect(bm1);
}
void Simulator::connectU(PPackModule m1, uint n1, PPackModule m2, uint n2) {  // pack to pack
    CHECK_NULLPTR(m1, PackModule); CHECK_NULLPTR(m2, PackModule);
    PUnitModule bm1 = m1->Get_OutputPort(n1);
    PUnitModule bm2 = m2->Get_InputPort(n2);
    CHECK_NULLPTR(bm1, UnitModule); CHECK_NULLPTR(bm2, UnitModule);
    CHECK_NULLID(bm1, UnitModule); CHECK_NULLID(bm2, UnitModule);
    CHECK_SIMULATOR(bm1, UnitModule); CHECK_SIMULATOR(bm2, UnitModule);
    bm2->connect(bm1);
}

#ifdef USE_ZHNMAT
/**********************
The following functions are used to build a bus connection between:
 matrix module and matrix module
 matrix module and pack module
 pack module and pack module
**********************/
void Simulator::connectM(PMatModule m1, PMatModule m2) {  // mat to mat
    CHECK_NULLPTR(m1, MatModule); CHECK_NULLPTR(m2, MatModule);
    CHECK_SIMULATOR(m1, MatModule); CHECK_SIMULATOR(m2, MatModule);
    m2->connect(m1);
}
void Simulator::connectM(PMatModule m1, PPackModule m2, uint n2) {  // mat to pack
    CHECK_NULLPTR(m1, MatModule); CHECK_NULLPTR(m2, PackModule);
    CHECK_SIMULATOR(m1, MatModule);
    PMatModule bm2 = m2->Get_InputBus(n2);
    CHECK_NULLPTR(bm2, MatModule); CHECK_SIMULATOR(bm2, MatModule);
    bm2->connect(m1);
}
void Simulator::connectM(PPackModule m1, uint n1, PMatModule m2) {  // pack to mat
    CHECK_NULLPTR(m1, PackModule); CHECK_NULLPTR(m2, MatModule);
    CHECK_SIMULATOR(m2, MatModule);
    PMatModule bm1 = m1->Get_OutputBus(n1);
    CHECK_NULLPTR(bm1, MatModule); CHECK_SIMULATOR(bm1, MatModule);
    m2->connect(bm1);
}
void Simulator::connectM(PPackModule m1, uint n1, PPackModule m2, uint n2) {  // pack to pack
    CHECK_NULLPTR(m1, PackModule); CHECK_NULLPTR(m2, PackModule);
    PMatModule bm1 = m1->Get_InputBus(n1);
    PMatModule bm2 = m2->Get_OutputBus(n2);
    CHECK_NULLPTR(bm1, MatModule); CHECK_SIMULATOR(bm1, MatModule);
    CHECK_NULLPTR(bm2, MatModule); CHECK_SIMULATOR(bm2, MatModule);
    bm2->connect(bm1);
}

/**********************
The following functions are used to build a single line connection between:
 unit module and multiplex module
 pack module and multiplex module
 demultiplex module and unit module
 demultiplex module and pack module
 demultiplex module and multiplex module
**********************/
void Simulator::connectU(PUnitModule m1, PMux m2, BusSize n2) {  // unit to mux
    CHECK_NULLPTR(m1, UnitModule); CHECK_NULLPTR(m2, Mux);
    CHECK_SIMULATOR(m1, UnitModule); CHECK_SIMULATOR(m2, Mux);
    m2->connect(m1, n2);
}
void Simulator::connectU(PDeMux m1, BusSize n1, PUnitModule m2) {  // demux to unit
    CHECK_NULLPTR(m1, DeMux); CHECK_NULLPTR(m2, UnitModule);
    CHECK_SIMULATOR(m1, DeMux); CHECK_SIMULATOR(m2, UnitModule);
    m2->connect(m1->Get_OutputPort(n1));
}
void Simulator::connectU(PPackModule m1, uint n1, PMux m2, BusSize n2) {  // pack to nux
    CHECK_NULLPTR(m1, PackModule); CHECK_NULLPTR(m2, Mux);
    CHECK_SIMULATOR(m2, Mux);
    PUnitModule bm1 = m1->Get_InputPort(n1);
    CHECK_NULLPTR(bm1, UnitModule); CHECK_SIMULATOR(bm1, UnitModule);
    m2->connect(bm1, n2);
}
void Simulator::connectU(PDeMux m1, BusSize n1, PPackModule m2, uint n2) {  // demux to pack
    CHECK_NULLPTR(m1, DeMux); CHECK_NULLPTR(m2, PackModule);
    CHECK_SIMULATOR(m1, DeMux);
    PUnitModule bm2 = m2->Get_InputPort(n2);
    CHECK_NULLPTR(bm2, UnitModule); CHECK_SIMULATOR(bm2, UnitModule);
    bm2->connect(m1->Get_OutputPort(n1));
}
void Simulator::connectU(PDeMux m1, BusSize n1, PMux m2, BusSize n2) {  // demux to mux
    CHECK_NULLPTR(m1, DeMux); CHECK_NULLPTR(m2, Mux);
    CHECK_SIMULATOR(m1, DeMux); CHECK_SIMULATOR(m2, Mux);
    PUnitModule bm1 = m1->Get_OutputPort(n1);
    CHECK_NULLPTR(bm1, UnitModule); CHECK_SIMULATOR(bm1, UnitModule);
    m2->connect(bm1, n2);
}
#endif

NAMESPACE_SIMUCPP_R
