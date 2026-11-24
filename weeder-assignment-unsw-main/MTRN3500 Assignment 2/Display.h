#pragma once
//inherits form NetworkedModule

#include <NetworkedModule.h>

constexpr int DISPLAY_PORT = 28000;

ref class Display : public NetworkedModule
{
public:
    Display(SM_ThreadManagement^ sm_tm, SM_Laser^ sm_laser);

    error_state connect(String^ hostName, int portNumber) override;
    error_state communicate() override;
    error_state processSharedMemory() override;
    bool getShutdownFlag() override;
    void threadFunction() override;
    //helper
    void sendDisplayData(array<double>^ xData, array<double>^ yData);
private:
    SM_Laser^ SM_Laser_;
    array<double>^ x_;
    array<double>^ y_;
};