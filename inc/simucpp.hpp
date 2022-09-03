#ifndef SIMUCPP_HEADER_H
#define SIMUCPP_HEADER_H
#include "packmodules.hpp"

#define SIMUCPP_CONTINUOUS                        true
#define SIMUCPP_DISCRETE                          false


#define SUConstant(x, sim)                        x=new UConstant(sim, #x)
#define SUFcn(x, sim)                             x=new UFcn(sim, #x)
#define SUFcnMISO(x, sim)                         x=new UFcnMISO(sim, #x)
#define SUGain(x, sim)                            x=new UGain(sim, #x)
#define SUInput(x, sim)                           x=new UInput(sim, #x)
#define SUIntegrator(x, sim)                      x=new UIntegrator(sim, #x)
#define SUNoise(x, sim)                           x=new UNoise(sim, #x)
#define SUOutput(x, sim)                          x=new UOutput(sim, #x)
#define SUProduct(x, sim)                         x=new UProduct(sim, #x)
#define SUSum(x, sim)                             x=new USum(sim, #x)
#define SUTransportDelay(x, sim)                  x=new UTransportDelay(sim, #x)
#define SUUnitDelay(x, sim)                       x=new UUnitDelay(sim, #x)
#define SUZOH(x, sim)                             x=new UZOH(sim, #x)

#define FUConstant(x, sim)        UConstant       *SUConstant(x, sim)
#define FUFcn(x, sim)             UFcn            *SUFcn(x, sim)
#define FUFcnMISO(x, sim)         UFcnMISO        *SUFcnMISO(x, sim)
#define FUGain(x, sim)            UGain           *SUGain(x, sim)
#define FUInput(x, sim)           UInput          *SUInput(x, sim)
#define FUIntegrator(x, sim)      UIntegrator     *SUIntegrator(x, sim)
#define FUNoise(x, sim)           UNoise          *SUNoise(x, sim)
#define FUOutput(x, sim)          UOutput         *SUOutput(x, sim)
#define FUProduct(x, sim)         UProduct        *SUProduct(x, sim)
#define FUSum(x, sim)             USum            *SUSum(x, sim)
#define FUTransportDelay(x, sim)  UTransportDelay *SUTransportDelay(x, sim)
#define FUUnitDelay(x, sim)       UUnitDelay      *SUUnitDelay(x, sim)
#define FUZOH(x, sim)             UZOH            *SUZOH(x, sim)

#define CLASS_USERFUNC_SISO(funcname, funccontent) \
class funcname:public UserFunc { \
public: \
    virtual double Function(double u)const{ funccontent; } \
};
#define CLASS_USERFUNC_MISO(funcname, funccontent) \
class funcname:public UserFunc { \
public: \
    virtual double Function(double *u)const{ funccontent; } \
};

#endif  // SIMUCPP_H
