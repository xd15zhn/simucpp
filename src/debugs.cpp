#include "simulator.hpp"
#ifdef SUPPORT_DEBUG
    #include <iostream>
#else
    #include "definitions.hpp"
#endif  // SUPPORT_DEBUG
NAMESPACE_SIMUCPP_L

void Simulator::Print_Modules() {
#if defined(SUPPORT_DEBUG)
    using namespace std;
    int childcnt;
    PUnitModule bm;  // pointer to child module
    cout << "Model structure print start." << endl;
    for (PUnitModule m: _modules) {
        cout << "name:" << m->_name << "  id:" << m->_id << "  type:" << typeid(*m).name() << endl;
        for (int i=0; i<m->Get_childCnt(); ++i) {
            bm = m->Get_child(i);
            cout << "    name:" << bm->_name << "  id:" << bm->_id;
            m->Print_DebugInfo(i);
        }
    }
    cout << "Model structure print finished." << endl;
#else
    TRACELOG(LOG_WARNING, "Simulator debug: You didn't add debug functions.");
#endif  // SUPPORT_DEBUG
}

void Simulator::Print_Connection(std::vector<int> &ids) {
#if defined(SUPPORT_DEBUG)
    std::cout << "Sequence table: ";
    for (int i: ids) {
        std::cout << i << ", ";
    }
    std::cout << std::endl;
#endif  // SUPPORT_DEBUG
}

#if defined(SUPPORT_DEBUG)
void UConstant::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
void UFcn::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
void UFcnMISO::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
void UGain::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
void UInput::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
void UIntegrator::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
void UNoise::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
void UOutput::Print_DebugInfo(unsigned int n) { std::cout << "  gain:" << _ingain << std::endl; }
void UProduct::Print_DebugInfo(unsigned int n) { std::cout << "  gain:" << _ingain[n] << std::endl; }
void USum::Print_DebugInfo(unsigned int n) { std::cout << "  gain:" << _ingain[n] << std::endl; }
void UTransportDelay::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
void UUnitDelay::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
void UZOH::Print_DebugInfo(unsigned int n) {std::cout<<std::endl;}
#endif  // SUPPORT_DEBUG

NAMESPACE_SIMUCPP_R
