#include <iostream>
#include "simulator.hpp"
#include "definitions.hpp"
#ifdef USE_ZHNMAT
NAMESPACE_SIMUCPP_L


MatModule::MatModule(Simulator *sim, std::string name): _sim(sim), _name(name) {};
MatModule::~MatModule() {}


/*********************
implementation of class BusSize.
**********************/
BusSize::BusSize(uint row, uint col): r(row),c(col) {};
BusSize::BusSize(const BusSize& size): r(size.r),c(size.c) {};
BusSize& BusSize::operator=(const BusSize &size) { r=size.r, c=size.c; return *this; }
bool BusSize::operator==(const BusSize &size) const { return ((r==size.r)&&(c==size.c))?true:false; }
bool BusSize::operator<(const BusSize &size) const {
    if (r>=size.r) return false;
    if (c>=size.c) return false;
    return true;
}


/*********************
Mux module.
**********************/
Mux::~Mux() { delete[] _next; }
u8 Mux::Get_State() const { return _state; }
BusSize Mux::Get_OutputBusSize() const { return _size; }
bool Mux::Initialize() { return true; }
void Mux::connect(const PMatModule m) { TraceLog(LOG_FATAL, "Simucpp: internal error: Mux."); }
void Mux::connect(const PUnitModule m, BusSize size) { _next[size.r*_size.c+size.c] = m; }
Mux::Mux(Simulator *sim, BusSize size, std::string name)
    :MatModule(sim, name), _size(size) {
    _state = BUS_INITIALIZED;
    if (_size<BusSize(1, 1)) TraceLog(LOG_FATAL, "Size of multiplex module \"%s\" is too small!", _name);
    _next = new PUnitModule[_size.c*_size.r];
    for (int i=_size.c*_size.r-1; i>=0; --i) _next[i] = nullptr;
    MATMODULE_INIT();
}
PUnitModule Mux::Get_OutputPort(BusSize size) const {
    if (_next==nullptr) TraceLog(LOG_FATAL, "Multiplex module \"%s\" doesn't have a child module!", _name);
    if (_size<size) return nullptr;
    return _next[size.r*_size.c+size.c];
}


/*********************
DeMux module.
**********************/
DeMux::~DeMux() {}
u8 DeMux::Get_State() const { return _state; }
BusSize DeMux::Get_OutputBusSize() const { return _size; }
void DeMux::connect(const PMatModule m) { _next = m; }
void DeMux::connect(const PUnitModule m, BusSize size) {
    _sim->connectU(m, _gains[size.r*_size.c+size.c]);
}
DeMux::DeMux(Simulator *sim, BusSize size, std::string name)
    : MatModule(sim, name), _size(size) {
    _state = BUS_SIZED;
    if (_size<BusSize(1, 1)) TraceLog(LOG_FATAL, "Size of multiplex module \"%s\" is too small!", _name);
    _gains = new PUGain[_size.c*_size.r];
    for (int i=_size.c*_size.r-1; i>=0; --i)
        _gains[i] = new UGain(sim, name+"connector"+std::to_string(i));
    MATMODULE_INIT();
}
PUnitModule DeMux::Get_OutputPort(BusSize size) const {
    if (_size<size) return nullptr;
    return _gains[size.r*_size.c+size.c];
}
bool DeMux::Initialize() {
    if (_state == BUS_INITIALIZED) return true;
    if (!_next->Get_State()) return false;
    bool full = true;
    PUnitModule m;
    for (int i=0; i<_size.r; ++i) {
        for (int j=0; j<_size.c; ++j) {
            if (_gains[i*_size.c+j] == nullptr) {
                full = false;
                continue;
            }
            _sim->connectU(_next->Get_OutputPort(BusSize(i, j)), _gains[i*_size.c+j]);
        }
    }
    if (!full) TraceLog(LOG_WARNING, "Demultiplex module \"%s\" was not fully connected.", _name);
    _state = BUS_INITIALIZED; return true;
}


/*********************
STATESPACE module.
**********************/
MStateSpace::~MStateSpace() {}
u8 MStateSpace::Get_State() const { return _state; }
BusSize MStateSpace::Get_OutputBusSize() const { return _size; }
void MStateSpace::connect(const PMatModule m) { _next=m; }
MStateSpace::MStateSpace(Simulator *sim, BusSize size, bool isc, std::string name)
    :MatModule(sim, name), _size(size), _isc(isc) {
    _state = BUS_SIZED;
    int cnt = size.r * size.c;
    MATMODULE_INIT();
    if (isc) {
        _intx = new PUIntegrator[cnt];
        for (int i=0; i<cnt; ++i)
            _intx[i] = new UIntegrator(sim, _name+"_int"+std::to_string(i));
    }
    else {
        _udx = new PUUnitDelay[cnt];
        for (int i=0; i<cnt; ++i)
            _udx[i] = new UUnitDelay(sim, _name+"_ud"+std::to_string(i));
    }
}
PUnitModule MStateSpace::Get_OutputPort(BusSize size) const {
    if (_size<size) return nullptr;
    if (_isc) return _intx[size.r*_size.c+size.c];
    else return _udx[size.r*_size.c+size.c];
}
bool MStateSpace::Initialize() {
    if (_state == BUS_INITIALIZED) return true;
    if (_next==nullptr) TraceLog(LOG_FATAL, "State module \"%s\" doesn't have a child module!", _name);
    if (!_next->Get_State()) return false;
    if (!(_next->Get_OutputBusSize()==_size)) TraceLog(LOG_FATAL, "Bus size between state module \"%s\" and its child module are mismatch!", _name);
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            if (_isc) _sim->connectU(_next->Get_OutputPort(BusSize(i, j)), _intx[i*_size.c+j]);
            else _sim->connectU(_next->Get_OutputPort(BusSize(i, j)), _udx[i*_size.c+j]);
        }
    }
    _state = BUS_INITIALIZED; return true;
}
void MStateSpace::Set_SampleTime(double time) {
    if (_isc) return;
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            _udx[i*_size.c+j]->Set_SampleTime(time);
        }
    }
}
void MStateSpace::Set_InitialValue(const zhnmat::Mat& value) {
    if ((value.row()!=_size.r) || (value.col()!=_size.c))
        TraceLog(LOG_FATAL, "Matrix module \"%s\" accepted mismatched initial values!", _name);
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            if (_isc) _intx[i*_size.c+j]->Set_InitialValue(value.at(i, j));
            else _udx[i*_size.c+j]->Set_InitialValue(value.at(i, j));
        }
    }
}
zhnmat::Mat MStateSpace::Get_OutValue() {
    zhnmat::Mat ans;
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            if (_isc) ans.set(i, j, _intx[i*_size.c+j]->Get_OutValue());
            else ans.set(i, j, _udx[i*_size.c+j]->Get_OutValue());
        }
    }
    return ans;
}


/*********************
matrix Gain module.
**********************/
MGain::~MGain() {}
BusSize MGain::Get_OutputBusSize() const { return _sizeout; }
u8 MGain::Get_State() const { return _state; }
void MGain::connect(const PMatModule m) { _next=m; }
MGain::MGain(Simulator *sim, const zhnmat::Mat& G, bool isleft, std::string name)
    :MatModule(sim, name), _G(G), _isleft(isleft) {
    _state = 0;
    MATMODULE_INIT();
}
PUnitModule MGain::Get_OutputPort(BusSize size) const {
    if (_sumy==nullptr) TraceLog(LOG_FATAL, "internal error: MGain.");
    if (_sizeout<size) return nullptr;
    return _sumy[size.r*_sizeout.c+size.c];
}
bool MGain::Initialize() {
    if (_state == BUS_INITIALIZED) return true;  // This matrix module has been initialized.
    if (_next==nullptr) TraceLog(LOG_FATAL, "State module \"%s\" doesn't have a child module!", _name);
    if (!_next->Get_State()) return false;
    _sizein = _next->Get_OutputBusSize();
    if ((!_isleft || (_sizein.r!=_G.col())) && (_isleft || (_sizein.c!=_G.row())))
        TraceLog(LOG_FATAL, "Bus size between state module \"%s\" and its child module are mismatch!", _name);
    _sizeout = _isleft ? BusSize(_G.row(), _sizein.c) : BusSize(_sizein.r, _G.col());
    _sumy = new USum*[_sizeout.r*_sizeout.c];
    for (uint i=0; i<_sizeout.r; ++i) {
        for (uint j=0; j<_sizeout.c; ++j) {
            _sumy[i*_sizeout.c+j] = new USum(_sim, _name+"_inu"+std::to_string(i)+"_"+std::to_string(j));
            if (_isleft) {
                for (uint k=0; k<_sizein.r; ++k) {
                    _sim->connectU(_next->Get_OutputPort(BusSize(k, j)), _sumy[i*_sizeout.c+j]);
                    _sumy[i*_sizeout.c+j]->Set_InputGain(_G.at(i, k));
                }
            }
            else {
                for (uint k=0; k<_sizein.c; ++k) {
                    _sim->connectU(_next->Get_OutputPort(BusSize(i, k)), _sumy[i*_sizeout.c+j]);
                    _sumy[i*_sizeout.c+j]->Set_InputGain(_G.at(k, j));
                }
            }
        }
    }
    _state = BUS_INITIALIZED; return true;
}


/*********************
matrix Sum module.
**********************/
MSum::~MSum() {}
BusSize MSum::Get_OutputBusSize() const { return _size; }
u8 MSum::Get_State() const { return _state; }
void MSum::connect(const PMatModule m) { _nexts.push_back(m); }
MSum::MSum(Simulator *sim, std::string name): MatModule(sim, name) {
    _state = 0;
    MATMODULE_INIT();
}
PUnitModule MSum::Get_OutputPort(BusSize size) const {
    if (_sumy==nullptr) TraceLog(LOG_FATAL, "internal error: MSum.");
    if (_size<size) return nullptr;
    return _sumy[size.r*_size.c+size.c];
}
bool MSum::Initialize() {
    if (_state == BUS_INITIALIZED) return true;
    if (_nexts.size()==0) TraceLog(LOG_FATAL, "Matrix module \"%s\" doesn't have a child module!", _name);
    BusSize childBusSize;
    PUnitModule childBusPort;
    for (int b=_nexts.size()-1; b>=0; --b) {
        if (!_nexts[b]->Get_State()) continue;  // Bus size of child module is not determined
        childBusSize = _nexts[b]->Get_OutputBusSize();
        if (_state & BUS_SIZED) {  // Bus size of this module is determined
            if (!(childBusSize==_size))
                TraceLog(LOG_FATAL, "Bus size mismatch between child modules of matrix module \"%s\"!", _name);
        }
        else {  // Bus size of this module is not determined
            _size = childBusSize; _state |= BUS_SIZED;
            int totalsum = _size.r*_size.c;
            _sumy = new PUSum[totalsum];
            for (int i=0; i<totalsum; ++i)
                _sumy[i] = new USum(_sim, _name+"_inu"+std::to_string(i));
        }
        for (uint i=0; i<_size.r; ++i) {
            for (uint j=0; j<_size.c; ++j) {
                childBusPort = _nexts[b]->Get_OutputPort(BusSize(i, j));
                _sim->connectU(childBusPort, _sumy[i*_size.c + j]);
            }
        }
    }
    if (!(_state & BUS_SIZED)) return false;
    _state = BUS_INITIALIZED; return true;
}


NAMESPACE_SIMUCPP_R
#endif
