#ifndef SIMUCPP_HEADER_H
#define SIMUCPP_HEADER_H
#include "packmodules.hpp"

#define SIMUCPP_CONTINUOUS                        true
#define SIMUCPP_DISCRETE                          false
#define PRINT_VALUE(x)                            std::cout << x << std::endl

#define SMConnector(x, sim)                       x=new MConnector(sim, #x)
#define SMConstant(x, sim)                        x=new MConstant(sim, #x)
#define SMFcn(x, sim)                             x=new MFcn(sim, #x)
#define SMFcnMISO(x, sim)                         x=new MFcnMISO(sim, #x)
#define SMGain(x, sim)                            x=new MGain(sim, #x)
#define SMInput(x, sim)                           x=new MInput(sim, #x)
#define SMIntegrator(x, sim)                      x=new MIntegrator(sim, #x)
#define SMNoise(x, sim)                           x=new MNoise(sim, #x)
#define SMOutput(x, sim)                          x=new MOutput(sim, #x)
#define SMProduct(x, sim)                         x=new MProduct(sim, #x)
#define SMSum(x, sim)                             x=new MSum(sim, #x)
#define SMTransportDelay(x, sim)                  x=new MTransportDelay(sim, #x)
#define SMUnitDelay(x, sim)                       x=new MUnitDelay(sim, #x)
#define SMZOH(x, sim)                             x=new MZOH(sim, #x)

#define FMConnector(x, sim)       MConnector      *SMConnector(x, sim)     
#define FMConstant(x, sim)        MConstant       *SMConstant(x, sim)     
#define FMFcn(x, sim)             MFcn            *SMFcn(x, sim)           
#define FMFcnMISO(x, sim)         MFcnMISO        *SMFcnMISO(x, sim)       
#define FMGain(x, sim)            MGain           *SMGain(x, sim)          
#define FMInput(x, sim)           MInput          *SMInput(x, sim)         
#define FMIntegrator(x, sim)      MIntegrator     *SMIntegrator(x, sim)    
#define FMNoise(x, sim)           MNoise          *SMNoise(x, sim)         
#define FMOutput(x, sim)          MOutput         *SMOutput(x, sim)        
#define FMProduct(x, sim)         MProduct        *SMProduct(x, sim)       
#define FMSum(x, sim)             MSum            *SMSum(x, sim)           
#define FMTransportDelay(x, sim)  MTransportDelay *SMTransportDelay(x, sim)
#define FMUnitDelay(x, sim)       MUnitDelay      *SMUnitDelay(x, sim)     
#define FMZOH(x, sim)             MZOH            *SMZOH(x, sim)           

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

#endif  // SIMUCPP_HEADER_H
