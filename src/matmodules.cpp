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
void Mux::connect(const PMatModule m) { TRACELOG(LOG_FATAL, "Simucpp: internal error: Mux."); }
void Mux::connect(const PUnitModule m, BusSize size) { _next[size.r*_size.c+size.c] = m; }
Mux::Mux(Simulator *sim, BusSize size, std::string name)
    :MatModule(sim, name), _size(size) {
    _state = BUS_INITIALIZED;
    if (_size<BusSize(1, 1)) TRACELOG(LOG_FATAL, "Mux: Size of \"%s\" too small!", _name.c_str());
    _next = new PUnitModule[_size.c*_size.r];
    for (int i=_size.c*_size.r-1; i>=0; --i) _next[i] = nullptr;
    MATMODULE_INIT();
}
PUnitModule Mux::Get_OutputPort(BusSize size) const {
    if (_next==nullptr) TRACELOG(LOG_FATAL, "Mux: \"%s\" doesn't have a child module!", _name.c_str());
    if (!(size<_size)) return nullptr;
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
    if (_size<BusSize(1, 1)) TRACELOG(LOG_FATAL, "DeMux: Size of \"%s\" is too small!", _name.c_str());
    _gains = new PUGain[_size.c*_size.r];
    for (int i=_size.c*_size.r-1; i>=0; --i)
        _gains[i] = new UGain(sim, name+"_connector_"+std::to_string(i));
    MATMODULE_INIT();
}
PUnitModule DeMux::Get_OutputPort(BusSize size) const {
    if (!(size<_size)) return nullptr;
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
    if (!full) TRACELOG(LOG_WARNING, "DeMux: \"%s\" was not fully connected.", _name.c_str());
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
    MATMODULE_INIT();
    if (isc) {
        _intx = new PUIntegrator[_size.r*_size.c];
        for (uint i=0; i<_size.r; ++i)
            for (uint j=0; j<_size.c; ++j)
            _intx[i*_size.c+j] = new UIntegrator(sim, _name+"_intx_"+std::to_string(i)+"_"+std::to_string(j));
    }
    else {
        _udx = new PUUnitDelay[_size.r*_size.c];
        for (uint i=0; i<_size.r; ++i)
            for (uint j=0; j<_size.c; ++j)
            _udx[i*_size.c+j] = new UUnitDelay(sim, _name+"_udx_"+std::to_string(i)+"_"+std::to_string(j));
    }
    _state = BUS_GENERATED;
}
PUnitModule MStateSpace::Get_OutputPort(BusSize size) const {
    if (!(size<_size)) return nullptr;
    if (_isc) return _intx[size.r*_size.c+size.c];
    else return _udx[size.r*_size.c+size.c];
}
bool MStateSpace::Initialize() {
    if (_state == BUS_INITIALIZED) return true;
    if (_next==nullptr) TRACELOG(LOG_FATAL, "StateSpace: \"%s\" doesn't have a child module!", _name.c_str());
    if (!(_next->Get_State() & BUS_GENERATED)) return false;
    BusSize childSize = _next->Get_OutputBusSize();
    if (!(childSize==_size))
        TRACELOG(LOG_FATAL, "StateSpace: Bus size of \"%s\" and its child modules are mismatch!\n    "
        "child:%d,%d; this:%d,%d", _name.c_str(), childSize.r, childSize.c, _size.r, _size.c);
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
        TRACELOG(LOG_FATAL, "StateSpace: \"%s\" accepted mismatched initial values!", _name.c_str());
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            if (_isc) _intx[i*_size.c+j]->Set_InitialValue(value.at(i, j));
            else _udx[i*_size.c+j]->Set_InitialValue(value.at(i, j));
        }
    }
}
zhnmat::Mat MStateSpace::Get_OutValue() {
    zhnmat::Mat ans(_size.r, _size.c);
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            if (_isc) ans.set(i, j, _intx[i*_size.c+j]->Get_OutValue());
            else ans.set(i, j, _udx[i*_size.c+j]->Get_OutValue());
        }
    }
    return ans;
}


/*********************
matrix Constant module.
**********************/
MConstant::~MConstant() {}
BusSize MConstant::Get_OutputBusSize() const { return _size; }
u8 MConstant::Get_State() const { return _state; }
void MConstant::connect(const PMatModule m) {}
bool MConstant::Initialize() { return true; }
MConstant::MConstant(Simulator *sim, const zhnmat::Mat& A, std::string name)
    :MatModule(sim, name) {
    MATMODULE_INIT();
    _size = BusSize(A.row(), A.col());
    _ucst = new PUConstant[_size.r*_size.c];
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            _ucst[i*_size.c+j] = new UConstant(_sim, _name+"_ucst_"+std::to_string(i)+"_"+std::to_string(j));
            _ucst[i*_size.c+j]->Set_OutValue(A.at(i, j));
        }
    }
    _state = BUS_INITIALIZED;
}
PUnitModule MConstant::Get_OutputPort(BusSize size) const {
    if (!(size<_size)) return nullptr;
    return _ucst[size.r*_size.c+size.c];
}
void MConstant::Set_OutValue(zhnmat::Mat A) {
    for (uint i=0; i<_size.r; ++i)
        for (uint j=0; j<_size.c; ++j)
            _ucst[i*_size.c+j]->Set_OutValue(A.at(i, j));
}


/*********************
matrix FcnMISO module.
**********************/
MFcnMISO::~MFcnMISO() {}
BusSize MFcnMISO::Get_OutputBusSize() const { return _size; }
u8 MFcnMISO::Get_State() const { return _state; }
void MFcnMISO::connect(const PMatModule m) { _nexts.push_back(m); }
void MFcnMISO::Set_Function(std::function<zhnmat::Mat(zhnmat::Mat*)> function) { _f=function; }
MFcnMISO::MFcnMISO(Simulator *sim, BusSize size, std::string name)
    :MatModule(sim, name), _size(size) {
    MATMODULE_INIT();
    _misoy = new PUFcnMISO[_size.r*_size.c];
    for (uint i=0; i<_size.r; ++i)
        for (uint j=0; j<_size.c; ++j)
            _misoy[i*_size.c+j] = new UFcnMISO(_sim, _name+"_misoy_"+std::to_string(i)+"_"+std::to_string(j));
    _state = BUS_GENERATED;
}
PUnitModule MFcnMISO::Get_OutputPort(BusSize size) const {
    if (_misoy==nullptr) TRACELOG(LOG_FATAL, "MFcnMISO: internal error.");
    if (!(size<_size)) return nullptr;
    return _misoy[size.r*_size.c+size.c];
}
bool MFcnMISO::Initialize() {
    if (_state == BUS_INITIALIZED) return true;
    if (_nexts.size()==0) TRACELOG(LOG_FATAL, "MFcnMISO: \"%s\" doesn't have a child module!", _name.c_str());
    bool success = true;
    for (PMatModule m: _nexts) { if (!(m->Get_State() & BUS_GENERATED)) { success = false; break; } }
    if (!success) return false;
    PUnitModule childPort;
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            _misoy[i*_size.c+j]->Set_Function([=](double *u){
                uint cntmat = 0;
                zhnmat::Mat *mats = new zhnmat::Mat[_nexts.size()];
                for (uint n = 0; n < _nexts.size(); n++) {
                    BusSize size = _nexts[n]->Get_OutputBusSize();
                    mats[n] = zhnmat::Mat(size.r, size.c);
                    for (int i = 0; i < size.r; i++)
                        for (int j = 0; j < size.c; j++)
                            mats[n].set(i, j, u[cntmat+i*size.c+j]);
                    cntmat += size.r * size.c;
                }
                zhnmat::Mat ansmat = _f(mats);
                delete[] mats;
                return ansmat.at(i, j);
            });
            for (int k = 0; k < _nexts.size(); k++) {
                BusSize childSize = _nexts[k]->Get_OutputBusSize();
                for (int m = 0; m < childSize.r; m++) {
                    for (int n = 0; n < childSize.c; n++) {
                        childPort = _nexts[k]->Get_OutputPort(BusSize(m, n));
                        _sim->connectU(childPort, _misoy[i*_size.c+j]);
                    }
                }
            }
        }
    }
    _state = BUS_INITIALIZED; return true;
}
zhnmat::Mat MFcnMISO::Get_OutValue() {
    if (_state != BUS_INITIALIZED) return zhnmat::Mat();
    zhnmat::Mat ans(_size.r, _size.c);
    for (uint i=0; i<_size.r; ++i)
        for (uint j=0; j<_size.c; ++j)
            ans.set(i, j, _misoy[i*_size.c+j]->Get_OutValue());
    return ans;
}


/*********************
matrix Gain module.
**********************/
MGain::~MGain() {}
BusSize MGain::Get_OutputBusSize() const { return _size; }
u8 MGain::Get_State() const { return _state; }
void MGain::connect(const PMatModule m) { _next=m; }
MGain::MGain(Simulator *sim, const zhnmat::Mat& G, bool isleft, std::string name)
    :MatModule(sim, name), _G(G), _isleft(isleft) {
    _state = 0;
    MATMODULE_INIT();
}
PUnitModule MGain::Get_OutputPort(BusSize size) const {
    if (_sumy==nullptr) TRACELOG(LOG_FATAL, "internal error: MGain.");
    if (!(size<_size)) return nullptr;
    return _sumy[size.r*_size.c+size.c];
}
bool MGain::Initialize() {
    if (_state == BUS_INITIALIZED) return true;  // This matrix module has been initialized.
    if (_next==nullptr) TRACELOG(LOG_FATAL, "MGain: \"%s\" doesn't have a child module!", _name.c_str());
    if (!(_next->Get_State() & BUS_SIZED)) return false;
    BusSize childSize = _next->Get_OutputBusSize();
    if ((!_isleft || (childSize.r!=_G.col())) && (_isleft || (childSize.c!=_G.row())))
        TRACELOG(LOG_FATAL, "MGain: Bus size of \"%s\" and its child module is mismatch!\n    "
        "child:%d,%d; this:%d,%d", _name.c_str(), childSize.r, childSize.c, _size.r, _size.c);
    _size = _isleft ? BusSize(_G.row(), childSize.c) : BusSize(childSize.r, _G.col());
    _sumy = new PUSum[_size.r*_size.c];
    for (uint i=0; i<_size.r; ++i)
        for (uint j=0; j<_size.c; ++j)
            _sumy[i*_size.c+j] = new USum(_sim, _name+"_inu"+std::to_string(i)+"_"+std::to_string(j));
    _state = BUS_GENERATED;
    if (!(_next->Get_State() & BUS_GENERATED)) return false;
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            if (_isleft) {
                for (uint k=0; k<childSize.r; ++k) {
                    _sim->connectU(_next->Get_OutputPort(BusSize(k, j)), _sumy[i*_size.c+j]);
                    _sumy[i*_size.c+j]->Set_InputGain(_G.at(i, k));
                }
            } else {
                for (uint k=0; k<childSize.c; ++k) {
                    _sim->connectU(_next->Get_OutputPort(BusSize(i, k)), _sumy[i*_size.c+j]);
                    _sumy[i*_size.c+j]->Set_InputGain(_G.at(k, j));
                }
            }
        }
    }
    _state = BUS_INITIALIZED; return true;
}


/*********************
matrix Output module.
**********************/
MOutput::~MOutput() {}
BusSize MOutput::Get_OutputBusSize() const { return _size; }
u8 MOutput::Get_State() const { return _state; }
PUnitModule MOutput::Get_OutputPort(BusSize size) const { return nullptr; }
void MOutput::connect(const PMatModule m) { _next=m; }
MOutput::MOutput(Simulator *sim, std::string name): MatModule(sim, name) {
    _state = 0;
    MATMODULE_INIT();
}
bool MOutput::Initialize() {
    if (_state == BUS_INITIALIZED) return true;
    if (_next==nullptr) TRACELOG(LOG_FATAL, "MOutput: \"%s\" doesn't have a child module!", _name.c_str());
    if (!(_next->Get_State() & BUS_SIZED)) return false;
    _size = _next->Get_OutputBusSize();
    _out = new PUOutput[_size.r*_size.c];
    for (uint i=0; i<_size.r; ++i) {
        for (uint j=0; j<_size.c; ++j) {
            _out[i*_size.c+j] = new UOutput(_sim, _name+"_out_"+std::to_string(i)+"_"+std::to_string(j));
            _sim->connectU(_next->Get_OutputPort(BusSize(i, j)), _out[i*_size.c+j]);
        }
    }
    _state = BUS_INITIALIZED; return true;
}


/*********************
matrix Product module.
**********************/
MProduct::~MProduct() {}
BusSize MProduct::Get_OutputBusSize() const { return _size; }
u8 MProduct::Get_State() const { return _state; }
MProduct::MProduct(Simulator *sim, std::string name): MatModule(sim, name) {
    _state = 0;
    _portcnt = 0;
    MATMODULE_INIT();
}
PUnitModule MProduct::Get_OutputPort(BusSize size) const {
    if (_misoy==nullptr) TRACELOG(LOG_FATAL, "internal error: MProduct.");
    if (!(size<_size)) return nullptr;
    return _misoy[size.r*_size.c+size.c];
}
bool MProduct::Initialize() {
    if (_state == BUS_INITIALIZED) return true;
    if (_nextL==nullptr) TRACELOG(LOG_FATAL, "MProduct: \"%s\" doesn't have 2 child modules!", _name.c_str());
    if (!(_nextL->Get_State() & BUS_SIZED)) return false;
    if (!(_nextR->Get_State() & BUS_SIZED)) return false;
    _size = _nextL->Get_OutputBusSize();
    BusSize childSize = _nextR->Get_OutputBusSize();
    if (_size.c != childSize.r)
        TRACELOG(LOG_FATAL, "MProduct: Bus size mismatch between child modules of \"%s\"!\n    "
        "left:%d,%d; right:%d,%d", _name.c_str(), _size.r, _size.c, childSize.r, childSize.c);
    _size.c = childSize.c;
    _misoy = new PUFcnMISO[_size.r*_size.c];
    for (uint i = 0; i < _size.r; i++)
        for (uint j = 0; j < _size.c; j++)
            _misoy[i*_size.c+j] = new UFcnMISO(_sim, _name+"_misoy_"+std::to_string(i)+"_"+std::to_string(j));
    _state = BUS_GENERATED;
    if (!(_nextL->Get_State() & BUS_GENERATED)) return false;
    if (!(_nextR->Get_State() & BUS_GENERATED)) return false;
    for (uint i = 0; i < _size.r; i++) {
        for (uint j = 0; j < _size.c; j++) {
            _misoy[i*_size.c+j]->Set_Function([childSize](double *u){
                double ans = 0;
                for (int i = 0; i < childSize.r; i++)
                    ans += u[i+i]*u[i+i+1];
                return ans;
            });
            for (uint k=0; k<childSize.r; ++k) {
                _sim->connectU(_nextL->Get_OutputPort(BusSize(i, k)), _misoy[i*_size.c+j]);
                _sim->connectU(_nextR->Get_OutputPort(BusSize(k, j)), _misoy[i*_size.c+j]);
            }
        }
    }
    _state = BUS_INITIALIZED; return true;
}
void MProduct::connect(const PMatModule m) {
    if (_portcnt==0) _nextR = m;
    else if (_portcnt==1) _nextL = m;
    else TRACELOG(LOG_WARNING, "MProduct: \"%s\" is repeatedly connected.", _name.c_str());
    _portcnt++;
}


/*********************
matrix Sum module.
**********************/
MSum::~MSum() {}
BusSize MSum::Get_OutputBusSize() const { return _size; }
u8 MSum::Get_State() const { return _state; }
void MSum::connect(const PMatModule m) { _nexts.push_back(m); _ingain.push_back(1); }
MSum::MSum(Simulator *sim, std::string name): MatModule(sim, name) {
    _state = 0;
    MATMODULE_INIT();
}
PUnitModule MSum::Get_OutputPort(BusSize size) const {
    if (_sumy==nullptr) TRACELOG(LOG_FATAL, "internal error: MSum.");
    if (!(size<_size)) return nullptr;
    return _sumy[size.r*_size.c+size.c];
}
bool MSum::Initialize() {
    if (_state == BUS_INITIALIZED) return true;
    if (_nexts.size()==0) TRACELOG(LOG_FATAL, "MSum: \"%s\" doesn't have a child module!", _name.c_str());
    BusSize childSize;
    PUnitModule childPort;
    for (int b=_nexts.size()-1; b>=0; --b) {
        if (!(_nexts[b]->Get_State() & BUS_SIZED)) continue;  // Bus size of child module is not determined
        childSize = _nexts[b]->Get_OutputBusSize();
        if (_state & BUS_GENERATED) {  // Bus size of this module is determined
            if (!(childSize==_size))
                TRACELOG(LOG_FATAL, "MSum: Bus size mismatch between child modules of \"%s\"!\n    "
                "child:%d,%d; this:%d,%d", _name.c_str(), childSize.r, childSize.c, _size.r, _size.c);
        } else {  // Bus size of this module is not determined
            _size = childSize;
            int totalsum = _size.r*_size.c;
            _sumy = new PUSum[totalsum];
            for (int i=0; i<totalsum; ++i)
                _sumy[i] = new USum(_sim, _name+"_inu"+std::to_string(i));
            _state = BUS_GENERATED;
        }
        for (uint i=0; i<_size.r; ++i) {
            for (uint j=0; j<_size.c; ++j) {
                childPort = _nexts[b]->Get_OutputPort(BusSize(i, j));
                _sim->connectU(childPort, _sumy[i*_size.c + j]);
                _sumy[i*_size.c + j]->Set_InputGain(_ingain[b]);
            }
        }
    }
    if (!(_state & BUS_SIZED)) return false;
    _state = BUS_INITIALIZED; return true;
}
void MSum::Set_InputGain(double inputgain, int port) {
    if (port==-1) {
        if (_nexts.size()<=0)
            TRACELOG(LOG_WARNING, "MSum: \"%s\" doesn't have enough child module.", _name.c_str());
        _ingain[_nexts.size()-1] = inputgain;
    } else {
        if (port<0 || port>=(int)_nexts.size())
            TRACELOG(LOG_WARNING, "MSum: \"%s\" doesn't have enough child module.", _name.c_str());
        _ingain[port] = inputgain;
    }
}


/*********************
matrix MTranspose module.
**********************/
MTranspose::~MTranspose() {}
BusSize MTranspose::Get_OutputBusSize() const { return _size; }
u8 MTranspose::Get_State() const { return _state; }
void MTranspose::connect(const PMatModule m) { _next = m; }
MTranspose::MTranspose(Simulator *sim, std::string name): MatModule(sim, name) {
    _state = 0;
    MATMODULE_INIT();
}
PUnitModule MTranspose::Get_OutputPort(BusSize size) const {
    if (!(size<_size)) return nullptr;
    return _next->Get_OutputPort(BusSize(size.c, size.r));
}
bool MTranspose::Initialize() {
    if (_state == BUS_INITIALIZED) return true;
    if (_next==nullptr) TRACELOG(LOG_FATAL, "MTranspose: \"%s\" doesn't have a child module!", _name.c_str());
    if (!(_next->Get_State() & BUS_SIZED)) return false;
    _size = _next->Get_OutputBusSize();
    _size = BusSize(_size.c, _size.r);
    _state = BUS_SIZED;
    if (!(_next->Get_State() & BUS_INITIALIZED)) return false;
    _state = BUS_INITIALIZED; return true;
}


NAMESPACE_SIMUCPP_R
#endif  // USE_ZHNMAT
