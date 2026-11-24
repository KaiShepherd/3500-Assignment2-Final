#include "VC.h"

VehicleControl::VehicleControl(SM_ThreadManagement^ sm_tm, SM_VehicleControl^ sm_vehiclecontrol)
{
	SM_TM_ = sm_tm;
	SM_VehicleControl_ = sm_vehiclecontrol;
    speed_ = 0;
    steering_ = 0;
    watchdog_ = false;
    inhibit_ = false;
}

error_state VehicleControl::connect(String^ hostName, int portNumber)
{
    try
    {
        if (Stream != nullptr) Stream->Close();
        if (Client != nullptr) Client->Close();
        // Open the TCP connection and configure it
        Client = gcnew TcpClient(hostName, portNumber);
        Client->NoDelay = true;
        Client->ReceiveTimeout = 500;
        Client->SendTimeout = 500;
        Client->ReceiveBufferSize = 4096;
        Client->SendBufferSize = 1024;

        Stream = Client->GetStream();
        ReadData = gcnew array<unsigned char>(16);

        // Authenticate
        array<unsigned char>^ authData = Encoding::ASCII->GetBytes("5476861\n");
        Stream->Write(authData, 0, authData->Length);

        // Read and check the server's reply
        int bytesRead = Stream->Read(ReadData, 0, ReadData->Length);
        String^ response = Encoding::ASCII->GetString(ReadData, 0, bytesRead);

        if (response->Trim() != "OK")
        {
            Console::WriteLine("Vehicle control authentication failed, server replied: " + response);
            return ERR_CONNECTION;
        }

        Console::WriteLine("Vehicle control connected and authenticated");
    }
    catch (SocketException^ e)
    {
        Console::WriteLine("Vehicle control connection failed: " + e->Message);
        return ERR_CONNECTION;
    }
    catch (System::IO::IOException^ e)
    {
        Console::WriteLine("Vehicle control authentication timed out: " + e->Message);
        return ERR_CONNECTION;
    }

    return SUCCESS;
}

error_state VehicleControl::communicate()
{
    if (Stream == nullptr)
    {
        return ERR_CONNECTION;
    }

    String^ command = String::Format("# {0:F2} {1:F2} {2} #", steering_, speed_, watchdog_ ? 1 : 0);

    try
    {
        array<unsigned char>^ sendData = Encoding::ASCII->GetBytes(command);
        Stream->Write(sendData, 0, sendData->Length);
    }
    catch (System::IO::IOException^)
    {
        return ERR_CONNECTION;      // connection dropped or send timed out
    }

    watchdog_ = !watchdog_;         // flip only after a successful send
    return SUCCESS;
}

error_state VehicleControl::processSharedMemory()
{
    Monitor::Enter(SM_VehicleControl_->lockObject);
    speed_ = SM_VehicleControl_->Speed;
    steering_ = SM_VehicleControl_->Steering;
    inhibit_ = SM_VehicleControl_->Inhibit;
    speedScale_ = SM_VehicleControl_->SpeedScale;
    Monitor::Exit(SM_VehicleControl_->lockObject);

    if (speed_ > 0)
    {
        speed_ *= speedScale_;   // forward only; reverse untouched
    }

	return SUCCESS; 
}

bool VehicleControl::getShutdownFlag()
{
	return (SM_TM_->shutdown & bit_VC) != 0;
}

void VehicleControl::threadFunction()
{
    Console::WriteLine("Vehicle Control thread is starting");

    bool connected = (connect(WEEDER_ADDRESS, VC_PORT) == SUCCESS);
    if (!connected)
    {
        Console::WriteLine("Vehicle Control: connection failed, will retry");
    }

    SM_TM_->ThreadBarrier->SignalAndWait();

    Stopwatch^ retryTimer = Stopwatch::StartNew();

    while (!getShutdownFlag())
    {
        Monitor::Enter(SM_TM_->lockObject);
        SM_TM_->heartbeat |= bit_VC;
        Monitor::Exit(SM_TM_->lockObject);

        if (!connected)
        {
            // Retry about once a second, keep heartbeating meanwhile
            if (retryTimer->ElapsedMilliseconds > 1000)
            {
                connected = (connect(WEEDER_ADDRESS, VC_PORT) == SUCCESS);
                retryTimer->Restart();
            }
        }
        else
        {
            error_state result = processSharedMemory();
            if (result == SUCCESS)
            {
                result = communicate();
            }
            if (result == ERR_CONNECTION)
            {
                connected = false;      // lost connection: go back to retrying
            }
            if (result != SUCCESS)
            {
                printError(result);
            }
        }

        Thread::Sleep(100);
    }

    // Stop the vehicle and close the connection
    if (connected)
    {
        speed_ = 0;
        steering_ = 0;
        communicate();
    }
    if (Stream != nullptr) Stream->Close();
    if (Client != nullptr) Client->Close();

    Console::WriteLine("Vehicle Control thread is shutting down");
}