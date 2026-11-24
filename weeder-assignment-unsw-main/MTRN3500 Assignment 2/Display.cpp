#include "Display.h"

Display::Display(SM_ThreadManagement^ sm_tm, SM_Laser^ sm_laser)
{
	SM_TM_ = sm_tm;
	SM_Laser_ = sm_laser;
	x_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	y_ = gcnew array<double>(STANDARD_LASER_LENGTH);
}

error_state Display::connect(String^ hostName, int portNumber)
{
	try
	{
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();

		// Open the TCP connection to the local MATLAB display engine
		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->SendTimeout = 500;
		Client->SendBufferSize = 8192;

		Stream = Client->GetStream();

		Console::WriteLine("Display connected");
	}
	catch (SocketException^ e)
	{
		Console::WriteLine("Display connection failed: " + e->Message);
		return ERR_CONNECTION;
	}

	return SUCCESS;
}

error_state Display::communicate()
{
	if (Stream == nullptr)
	{
		return ERR_CONNECTION;
	}

	try
	{
		sendDisplayData(x_, y_);
	}
	catch (System::IO::IOException^)
	{
		return ERR_CONNECTION;
	}

	return SUCCESS;
}

error_state Display::processSharedMemory()
{
	Monitor::Enter(SM_Laser_->lockObject);
	try
	{
		Array::Copy(SM_Laser_->x, x_, STANDARD_LASER_LENGTH); //copying can throw an error so put in try
		Array::Copy(SM_Laser_->y, y_, STANDARD_LASER_LENGTH);
	}
	finally
	{
		Monitor::Exit(SM_Laser_->lockObject);
	}

	// Display engine expects millimetres; shared memory holds metres
	for (int i = 0; i < STANDARD_LASER_LENGTH; i++)
	{
		double forward = x_[i];
		double left = y_[i];

		x_[i] = -left * 1000;
		y_[i] = forward * 1000;
	}

	return SUCCESS;
}

bool Display::getShutdownFlag()
{
	return (SM_TM_->shutdown & bit_DISPLAY) != 0;
}

void Display::threadFunction()
{
	Console::WriteLine("Display thread is starting");

	bool connected = (connect(DISPLAY_ADDRESS, DISPLAY_PORT) == SUCCESS);
	if (!connected)
	{
		Console::WriteLine("Display: connection failed, will retry");
	}

	SM_TM_->ThreadBarrier->SignalAndWait();

	Stopwatch^ retryTimer = Stopwatch::StartNew();

	while (!getShutdownFlag())
	{
		Monitor::Enter(SM_TM_->lockObject);
		SM_TM_->heartbeat |= bit_DISPLAY;
		Monitor::Exit(SM_TM_->lockObject);

		if (!connected)
		{
			if (retryTimer->ElapsedMilliseconds > 1000)
			{
				connected = (connect(DISPLAY_ADDRESS, DISPLAY_PORT) == SUCCESS);
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
				connected = false;
			}
			if (result != SUCCESS)
			{
				printError(result);
			}
		}

		Thread::Sleep(100);
	}

	if (Stream != nullptr) Stream->Close();
	if (Client != nullptr) Client->Close();

	Console::WriteLine("Display thread is shutting down");
}

void Display::sendDisplayData(array<double>^ xData, array<double>^ yData)
{
	// Serialise the data arrays to byte arrays (format required by the display engine)
	array<Byte>^ dataX = gcnew array<Byte>(xData->Length * sizeof(double));
	Buffer::BlockCopy(xData, 0, dataX, 0, dataX->Length);

	array<Byte>^ dataY = gcnew array<Byte>(yData->Length * sizeof(double));
	Buffer::BlockCopy(yData, 0, dataY, 0, dataY->Length);

	// Send x then y over the connection
	Stream->Write(dataX, 0, dataX->Length);
	Thread::Sleep(10);
	Stream->Write(dataY, 0, dataY->Length);
}