/**********************
FILE DESCRIPTIONS
This file contains the class definations of UNITMODULES which will be frequently used in a
 simulation module.
The first line in code annotations of each unitmodule gives its name and abbreviation. The
 name is used for detail descriptions, the abbreviation is suggested for variable naming.
**********************/
#ifndef BASEMODULES_H
#define BASEMODULES_H
#include <vector>
#include <functional>
#include "baseclass.hpp"
NAMESPACE_SIMUCPP_L
#define UNITMODULE_VIRTUAL(classname, abbrname) \
        friend class Simulator; \
    public: \
        classname(Simulator *sim=nullptr, std::string name=#abbrname); \
        virtual ~classname() override; \
        virtual double Get_OutValue() const override; \
    private: \
        virtual int Self_Check() const override; \
        virtual void Module_Update(double time) override; \
        virtual void Module_Reset() override; \
        virtual int Get_childCnt() const override; \
        virtual PUnitModule Get_child(uint n=0) const override; \
        virtual void connect(const PUnitModule m) override;
typedef std::vector<double>  vecdble;


/**********************
 * CONSTANT module.(cnst)  
 * User can use INPUT module instead, and this module is added for convenience.  
 * It outputs a constant value.
 *********************/
class UConstant: public UnitModule {
    UNITMODULE_VIRTUAL(UConstant, cnst);
public:
    void Set_OutValue(double v);
private:
    double _outvalue;
};


/**********************
 * FCN module.(fcn)
 *********************/
class UFcn: public UnitModule {
    UNITMODULE_VIRTUAL(UFcn, fcn);
public:
    void Set_Function(std::function<double(double)> function);
private:
    double _outvalue;
    std::function<double(double)> _f=nullptr;
    PUnitModule _next=nullptr;
};


/**********************
 * @brief module.(miso)  
 * This module is similar to FCN module but it has multiple input ports.
 *********************/
class UFcnMISO: public UnitModule {
    UNITMODULE_VIRTUAL(UFcnMISO, miso);
public:
    // Set the function. param "inparam" is an array, which represents all the input values.
    void Set_Function(std::function<double(double*)> function);
private:
    void connect2(const PUnitModule m, uint n=0);
    void disconnect(uint n=0);
    double _outvalue;
    std::function<double(double*)> _f=nullptr;
    std::vector<PUnitModule> _next;
};


/**********************
GAIN module.(gain)
**********************/
class UGain: public UnitModule {
    UNITMODULE_VIRTUAL(UGain, gain);
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
class UInput: public UnitModule {
    UNITMODULE_VIRTUAL(UInput, in);
public:
    // User should provide an input function when it's in continuous mode.
    void Set_Function(std::function<double(double)> function);
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
    std::function<double(double)> _f=nullptr;  // Input function
    std::vector<double> _data;  // Input data
};


/**********************
INTEGRATOR module.(int)
**********************/
class UIntegrator: public UnitModule {
    UNITMODULE_VIRTUAL(UIntegrator, integ);
public:
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
class UNoise: public UnitModule {
    UNITMODULE_VIRTUAL(UNoise, nse);
public:
    // Set the mean value of the noise.
    void Set_Mean(double mean=0);
    // Set the variance value of the noise.
    void Set_Variance(double var=1);
    // How much does it generate a value. Default -1 represents that it generates values
    //  in every sample points.
    void Set_SampleTime(double time=-1);
private:
    DISCRETE_VARIABLES;
    double _outvalue;
    double _mean, _var;
};


/**********************
OUTPUT module.(out)
**********************/
class UOutput: public UnitModule {
    UNITMODULE_VIRTUAL(UOutput, out);
public:
    // Return all the stored data.
    std::vector<double>& Get_StoredData();

    // How long should this module collect a sample data.
    // All samples will be collected by default.
    void Set_SampleTime(double time=-1);

    // Is simulation data stored to memory.
    void Set_EnableStore(bool store=true);

    // all input data will be multiplied by "inputgain" before stored.
    void Set_InputGain(double inputgain=1);

    // Set the maximum amount of data it can hold.
    // -1 for don't set a limitation.
    // If too many data was stored, then earliest data will be removed.
    void Set_MaxDataStorage(int n=-1);

private:
    DISCRETE_VARIABLES;

    // Stored data.
    std::vector<double> _values;

    // See public member function "Set_EnableStore".
    bool _store;
    // _maxstorage: How many samples will it store.
    int _maxstorage;

    // @_outvalue: value of its latest sample point.
    // @_ingain: See public member function "Set_InputGain".
    double _outvalue, _ingain;

    PUnitModule _next;
};


/**********************
PRODUCT module.(prod)
See SUM module for details.
**********************/
class UProduct: public UnitModule {
    UNITMODULE_VIRTUAL(UProduct, prod);
public:
    // Set the input gain of current connection, or the connection specified by param "port".
    void Set_InputGain(double inputgain, int port=-1);
private:
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
class USum: public UnitModule {
    UNITMODULE_VIRTUAL(USum, sum);
public:
    // Set the input gain of current connection, or the connection specified by param "port".
    void Set_InputGain(double inputgain, int port=-1);
    void Set_Redundant(bool rdnt=true);
private:
    double _outvalue;
    std::vector<PUnitModule> _next;
    std::vector<double> _ingain;
    bool _rdnt;
};


/**********************
TRANSPORTDELAY module.(trd)
**********************/
class UTransportDelay: public UnitModule {
    UNITMODULE_VIRTUAL(UTransportDelay, trd);
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
class UUnitDelay: public UnitModule {
    UNITMODULE_VIRTUAL(UUnitDelay, ud);
public:
    // Set the stored value. Default 0.
    void Set_InitialValue(double value=0);
    // Set the sample time. Default 1.
    void Set_SampleTime(double time=1);
private:
    DISCRETE_VARIABLES;
    void Output_Update(double time);
    double _outvalue;
    // previous value, initial value.
    double _lv, _iv;
    PUnitModule _next;
};


/**********************
ZOH module.(zoh)
**********************/
class UZOH: public UnitModule {
    UNITMODULE_VIRTUAL(UZOH, zoh);
public:
    // Set the sample time. Default 1.
    void Set_SampleTime(double time);
private:
    DISCRETE_VARIABLES;
    double _outvalue;
    PUnitModule _next;
};

typedef UConstant*           PUConstant;
typedef UFcn*                PUFcn;
typedef UFcnMISO*            PUFcnMISO;
typedef UGain*               PUGain;
typedef UInput*              PUInput;
// typedef UIntegrator*      PUIntegrator;
typedef UNoise*              PUNoise;
// typedef UOutput*          PUOutput;
typedef UProduct*            PUProduct;
typedef USum*                PUSum;
typedef UTransportDelay*     PUTransportDelay;
// typedef UUnitDelay*       PUUnitDelay;
typedef UZOH*                PUZOH;

NAMESPACE_SIMUCPP_R
#endif // UNITMODULES_H
