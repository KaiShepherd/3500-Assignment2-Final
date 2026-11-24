#include "Controller.h"

Controller::Controller(SM_ThreadManagement^ sm_tm, SM_VehicleControl^ sm_vehiclecontrol)
{
	SM_TM_ = sm_tm;
	SM_VehicleControl_ = sm_vehiclecontrol;
	controller_ = new ControllerInterface(1, INPUT_MODE);
}


Controller::~Controller()
{
	delete controller_;
}


error_state Controller::processSharedMemory()
{
    controllerState state = controller_->GetState();

    double speed = 0;
    double steering = 0;
    bool connected = controller_->IsConnected();

    if (connected)
    {
        speed = state.rightTrigger - state.leftTrigger;  // -1 to 1 m/s
        steering = state.rightThumbX * 40.0;  // -40 to 40 degrees
    }

    // Always write: zeros if disconnected, so the vehicle stops
    Monitor::Enter(SM_VehicleControl_->lockObject);
    SM_VehicleControl_->Speed = speed;
    SM_VehicleControl_->Steering = steering;
    Monitor::Exit(SM_VehicleControl_->lockObject);

    // Shutdown button
    if (connected && state.back)
    {
        Monitor::Enter(SM_TM_->lockObject);
        SM_TM_->shutdown = bit_ALL;
        Monitor::Exit(SM_TM_->lockObject);
    }

    return connected ? SUCCESS : ERR_CONNECTION;
}


bool Controller::getShutdownFlag()
{
	return (SM_TM_->shutdown & bit_CONTROLLER) != 0;
}


void Controller::threadFunction()
{
	Console::WriteLine("Controller thread is starting");

	SM_TM_->ThreadBarrier->SignalAndWait();

	while (!getShutdownFlag())
	{
		Monitor::Enter(SM_TM_->lockObject);
		SM_TM_->heartbeat |= bit_CONTROLLER;
		Monitor::Exit(SM_TM_->lockObject);

		error_state result = processSharedMemory();
		
		if (result != SUCCESS)
		{
			printError(result);
		}

		Thread::Sleep(20);
	}

    Monitor::Enter(SM_VehicleControl_->lockObject);
    SM_VehicleControl_->Speed = 0;
    SM_VehicleControl_->Steering = 0;
    Monitor::Exit(SM_VehicleControl_->lockObject);

	Console::WriteLine("Controller thread shutting down");
}