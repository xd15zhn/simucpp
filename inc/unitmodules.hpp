/**********************
FILE DESCRIPTIONS
This file contains the class definations of UNITMODULES which will be frequently used in a
 simulation module.
The first line in code annotations of each unitmodule gives its name and abbreviation. The
 name is used for detail descriptions, the abbreviation is suggested for variable naming.
**********************/
#ifndef BASEMODULES_H
#define BASEMODULES_H
#include <iostream>
#include <random>
#include <ctime>
#include "baseclass.hpp"
NAMESPACE_SIMUCPP_L
#define UNITMODULE_VIRTUAL(classname, abbrname) \
    friend class Simulator; \
    public: \
        classname(Simulator *sim=nullptr, std::string name=#abbrname); \
        virtual ~classname(); \
        virtual double Get_OutValue() const; \
    private: \
        virtual void Set_Enable(bool enable=false); \
        virtual int Self_Check() const; \
        virtual void Module_Update(double time); \
        virtual void Module_Reset(); \
        virtual int Get_childCnt() const; \
        virtual PUnitModule Get_child(unsigned int n=0) const; \
        virtual void connect(const PUnitModule m); \
        bool _enable;


/**********************
CONNECTOR module.(io)
This module is used in pack modules.
If there exists sub PackModules that have multiple input or output ports in a PackModules,
then you can't specify which UnitModule will be used to build a connection with other modules.
Now a CONNECTOR module is the UnitModule you will need.
**********************/
class MConnector: public UnitModule
{
    UNITMODULE_VIRTUAL(MConnector, io);
private:
    PUnitModule _next;
};


/**********************
CONSTANT module.(cnst)
User can use INPUT module instead, and this module is added for convenience.
It generate a constant value.
**********************/
class MConstant: public UnitModule
{
    UNITMODULE_VIRTUAL(MConstant, cnst);
public:
    void Set_OutValue(double v);
private:
    double _outvalue;
};


/**********************
FCN module.(fcn)
**********************/
class MFcn: public UnitModule
{
    UNITMODULE_VIRTUAL(MFcn, fcn);
public:
    // Set the function. param "u" is the input value and can be null.
    void Set_Function(double(*function)(double u));
    void Set_Function(UserFunc *function);
private:
    double _outvalue;
    double(*_f)(double u);
    UserFunc *_fu;
    PUnitModule _next;
};


/**********************
FCNMISO module.(miso)
This module is similar to FCN module but it has multiple input ports.
**********************/
class MFcnMISO: public UnitModule
{
    UNITMODULE_VIRTUAL(MFcnMISO, miso);
public:
    // Set the function. param "inparam" is an array, which represents all the input values.
    void Set_Function(double(*function)(double* inparam));
    void Set_Function(UserFunc *function);
private:
    void disconnect(int n=0);
    double _outvalue;
    double(*_f)(double* inparam);
    UserFunc *_fu;
    std::vector<PUnitModule> _next;
};


/**********************
GAIN module.(gain)
This module is not recommended to use because a redundant module may slow down the
 simulation speed. Many necessary modules have an "input gain" paramter to replace
 this module. This module is kept for special use, for example see demo test18.
**********************/
class MGain: public UnitModule
{
    UNITMODULE_VIRTUAL(MGain, gain);
public:
    // Set the gain value.
    void Set_Gain(double gain);
private:
    double _outvalue, _gain;
    PUnitModule _next;
};


/**********************
INPUT module.(in)
In continuous mode, this module generate outputs by given input function,
 and in discrete mode by input data. It's default continuous, and remember
 to set sample time if change it into discrete mode.
**********************/
class MInput: public UnitModule
{
    UNITMODULE_VIRTUAL(MInput, in);
public:
    // User should provide an input function when it's in continuous mode.
    void Set_InputFunction(double(*function)(double u));
    void Set_InputFunction(UserFunc *function);
    // User should provide sample dataswhen it's in discrete mode.
    void Set_InputData(const std::vector<double>& data);
    // Set if this module is in continuous mode.
    void Set_Continuous(bool isContinuous=true);
    // User should set sample time if this module is in discrete mode.
    // It's useless in continuous mode.
    void Set_SampleTime(double time=-1);
private:
    double _outvalue;
    int _cnt;  // samples count. Only used when in discrete mode
    double _T;  // Sample time. Only used when in discrete mode
    bool _isc;  // Be in continuous mode when it's true
    double(*_f)(double t);  // Input function
    UserFunc *_fu;
    std::vector<double> _data;  // Input data
};


/**********************
INTEGRATOR module.(int)
**********************/
class MIntegrator: public UnitModule
{
    UNITMODULE_VIRTUAL(MIntegrator, integ);
public:
    // Set the initial value.
    void Set_InitialValue(double value=0);
private:
    double _outvalue, _iv;
    PUnitModule _next;
};


/**********************
NOISE module.(nse)
User can use INPUT module instead, and this module is added for convenience.
It generates a gaussian white noise.
**********************/
class MNoise: public UnitModule
{
    UNITMODULE_VIRTUAL(MNoise, nse);
public:
    // Set the mean value of the noise.
    void Set_Mean(double mean=0);
    // Set the variance value of the noise.
    void Set_Variance(double var=1);
    // How much does it generate a value. Default -1 represents that it generates values
    //  in every sample points.
    void Set_SampleTime(double time=-1);
private:
    DISCRETE_VARIABLES();
    double _outvalue;
    double _mean, _var;
    std::default_random_engine gen;
    std::normal_distribution<double> NormDis;
};


/**********************
OUTPUT module.(out)
**********************/
class MOutput: public UnitModule
{
    UNITMODULE_VIRTUAL(MOutput, out);
public:
    // Return all the stored data.
    std::vector<double>& Get_StoredData();

    // How long should this module collect a sample data.
    // All samples will be collected by default.
    void Set_SampleTime(double time=-1);

    // Used to control whether simulation data is printed to file or stored to memory.
    // Simulation may speed up if they are disabled.
    void Set_EnablePrint(bool print=true);
    void Set_EnableStore(bool store=false);

    // all input data will be multiplied by "inputgain" before stored.
    void Set_InputGain(double inputgain=1);

    // Set the maximum amount of data it can hold.
    // -1 for don't set a limitation.
    // If too many data was stored, then earliest data will be removed.
    void Set_MaxDataStorage(int n=-1);

private:
    DISCRETE_VARIABLES();

    // Stored data.
    std::vector<double> _values;

    // See public member function "Set_EnablePrint" and "Set_EnableStore".
    // @_maxstorage: How many samples will it store.
    bool _print, _store;
    int _maxstorage;

    // @_outvalue: value of its latest sample point.
    // @_ingain: See public member function "Set_InputGain".
    double _outvalue, _ingain;

    PUnitModule _next;
};


/**********************
PRODUCT module.(pord)
See SUM module for details.
**********************/
class MProduct: public UnitModule
{
    UNITMODULE_VIRTUAL(MProduct, prod);
public:
    // Set the input gain of current connection, or the connection specified by param "port".
    void Set_InputGain(double inputgain, int port=-1);
private:
    double disconnect(int n=0);
    double _outvalue;
    std::vector<PUnitModule> _next;
    std::vector<double> _ingain;
};


/**********************
SUM module.(sum)
SUM and PRODUCT module is the only two basic modules which can have more than one input ports.
If you use Simulator::connect() method to add a child module to them(SUM and PRODUCT module),
 then Set_InputGain() method default to set the input gain of the latest added child module.
**********************/
class MSum: public UnitModule
{
    UNITMODULE_VIRTUAL(MSum, sum);
public:
    // Set the input gain of current connection, or the connection specified by param "port".
    void Set_InputGain(double inputgain, int port=-1);
private:
    double disconnect(int n=0);
    double _outvalue;
    std::vector<PUnitModule> _next;
    std::vector<double> _ingain;
};


/**********************
TRANSPORTDELAY module.(trd)
**********************/
class MTransportDelay: public UnitModule
{
    UNITMODULE_VIRTUAL(MTransportDelay, trd);
public:
    // Set the stored value. Default 0.
    void Set_InitialValue(double value=0);
    // Set the delay time. The default value is one sample time. It's strongly recommanded
    //  to call this function and set a delay time before using this module.
    // You must be aware that the value you set may not be used accurately because
    //  computer simulation is discrete.
    void Set_DelayTime(double time);
private:
    double _outvalue, _iv;
    // delay times and previous values
    std::vector<double> _lv;
    double _simstep, _nexttime;
    PUnitModule _next;
};


/**********************
UNITDELAY module.(ud)
**********************/
class MUnitDelay: public UnitModule
{
    UNITMODULE_VIRTUAL(MUnitDelay, ud);
public:
    // Set the stored value. Default 0.
    void Set_InitialValue(double value=0);
    // Set the sample time. Default 1.
    void Set_SampleTime(double time=1);
private:
    DISCRETE_VARIABLES();
    void Output_Update(double time);
    double _outvalue;
    // previous value, initial value.
    double _lv, _iv;
    PUnitModule _next;
};


/**********************
ZOH module.(zoh)
**********************/
class MZOH: public UnitModule
{
    UNITMODULE_VIRTUAL(MZOH, zoh);
public:
    // Set the sample time. Default 1.
    void Set_SampleTime(double time);
private:
    DISCRETE_VARIABLES();
    double _outvalue;
    PUnitModule _next;
};


/**********************
The following is the user's macro definition.
**********************/
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

NAMESPACE_SIMUCPP_R
#endif // UNITMODULES_H
