#include <iostream>
#include "simulator.hpp"
#include "definitions.hpp"
NAMESPACE_SIMUCPP_L


/**********************
The following functions are used to build a single line connection between:
 unit module and unit module
 unit module and pack module
 pack module and pack module
**********************/
void Simulator::connectU(PUnitModule m1, PUnitModule m2) {
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_NULLID(m1); CHECK_NULLID(m2);
    CHECK_SIMULATOR(m1); CHECK_SIMULATOR(m2);
    m2->connect(m1);
}
void Simulator::connectU(PUnitModule m1, PPackModule m2, int n2) {
    CHECK_NULLPTR(m1); CHECK_NULLID(m1); CHECK_SIMULATOR(m1);
    CHECK_NULLPTR(m2);
    PUnitModule bm2 = m2->Get_InputPort(n2);
    CHECK_NULLPTR(bm2); CHECK_NULLID(bm2); CHECK_SIMULATOR(bm2);
    bm2->connect(m1);
}
void Simulator::connectU(PPackModule m1, int n1, PUnitModule m2) {
    CHECK_NULLPTR(m1);
    CHECK_NULLPTR(m2); CHECK_NULLID(m2); CHECK_SIMULATOR(m2);
    PUnitModule bm1 = m1->Get_OutputPort(n1);
    CHECK_NULLPTR(bm1); CHECK_NULLID(bm1); CHECK_SIMULATOR(bm1);
    m2->connect(bm1);
}
void Simulator::connectU(PPackModule m1, PUnitModule m2) {
    this->connectU(m1, 0, m2);
}
void Simulator::connectU(PPackModule m1, int n1, PPackModule m2, int n2) {
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    PUnitModule bm1 = m1->Get_OutputPort(n1);
    PUnitModule bm2 = m2->Get_InputPort(n2);
    CHECK_NULLPTR(bm1); CHECK_NULLID(bm1); CHECK_SIMULATOR(bm1);
    CHECK_NULLPTR(bm2); CHECK_NULLID(bm2); CHECK_SIMULATOR(bm2);
    bm2->connect(bm1);
}
void Simulator::connectU(PPackModule m1, PPackModule m2, int n2) {
    this->connectU(m1, 0, m2, n2);
}

#ifdef USE_ZHNMAT
/**********************
The following functions are used to build a bus connection between:
 matrix module and matrix module
 matrix module and pack module
 pack module and pack module
**********************/
void Simulator::connectM(PMatModule m1, PMatModule m2) {
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_SIMULATOR(m1); CHECK_SIMULATOR(m2);
    m2->connect(m1);
}
void Simulator::connectM(PMatModule m1, PPackModule m2, int n2) {
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_SIMULATOR(m1);
    PMatModule bm2 = m2->Get_InputBus(n2);
    CHECK_NULLPTR(bm2); CHECK_SIMULATOR(bm2);
    bm2->connect(m1);
}
void Simulator::connectM(PPackModule m1, int n1, PMatModule m2) {
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_SIMULATOR(m2);
    PMatModule bm1 = m1->Get_OutputBus(n1);
    CHECK_NULLPTR(bm1); CHECK_SIMULATOR(bm1);
    m2->connect(bm1);
}
void Simulator::connectM(PPackModule m1, PMatModule m2) {
    this->connectM(m1, 0, m2);
}
void Simulator::connectM(PPackModule m1, int n1, PPackModule m2, int n2) {
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    PMatModule bm1 = m1->Get_InputBus(n1);
    PMatModule bm2 = m2->Get_OutputBus(n2);
    CHECK_NULLPTR(bm1); CHECK_SIMULATOR(bm1);
    CHECK_NULLPTR(bm2); CHECK_SIMULATOR(bm2);
    bm2->connect(bm1);
}
void Simulator::connectM(PPackModule m1, PPackModule m2, int n2) {
    this->connectM(m1, 0, m2, n2);
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
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_SIMULATOR(m1); CHECK_SIMULATOR(m2);
    m2->connect(m1, n2);
}
void Simulator::connectU(PDeMux m1, BusSize n1, PUnitModule m2) {  // demux to unit
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_SIMULATOR(m1); CHECK_SIMULATOR(m2);
    m2->connect(m1->Get_OutputPort(n1));
}
void Simulator::connectU(PDeMux m1, PUnitModule m2) {  // demux to unit
    this->connectU(m1, BusSize(0, 0), m2);
}
void Simulator::connectU(PDeMux m1, BusSize n1, PPackModule m2, int n2) {  // demux to pack
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_SIMULATOR(m1);
    PUnitModule bm2 = m2->Get_InputPort(n2);
    CHECK_NULLPTR(bm2); CHECK_SIMULATOR(bm2);
    bm2->connect(m1->Get_OutputPort(n1));
}
void Simulator::connectU(PDeMux m1, PPackModule m2, int n2) {  // demux to pack
    this->connectU(m1, BusSize(0, 0), m2, n2);
}
void Simulator::connectU(PDeMux m1, BusSize n1, PMux m2, BusSize n2) {  // demux to mux
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_SIMULATOR(m1); CHECK_SIMULATOR(m2);
    PUnitModule bm1 = m1->Get_OutputPort(n1);
    m2->connect(bm1, n2);
}
void Simulator::connectU(PDeMux m1, PMux m2, BusSize n2) {  // demux to mux
    this->connectU(m1, BusSize(0, 0), m2, n2);
}
void Simulator::connectU(PPackModule m1, int n1, PMux m2, BusSize n2) {  // pack to nux
    CHECK_NULLPTR(m1); CHECK_NULLPTR(m2);
    CHECK_SIMULATOR(m2);
    PUnitModule bm1 = m1->Get_InputPort(n1);
    CHECK_NULLPTR(bm1); CHECK_SIMULATOR(bm1);
    m2->connect(bm1, n2);
}
void Simulator::connectU(PPackModule m1, PMux m2, BusSize n2) {  // pack to mux
    this->connectU(m1, 0, m2, n2);
}
#endif

NAMESPACE_SIMUCPP_R
