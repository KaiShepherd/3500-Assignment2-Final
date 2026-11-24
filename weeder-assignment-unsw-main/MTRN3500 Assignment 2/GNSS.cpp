#include "GNSS.h"
#include <cstring>

#define GNSS_PRINT 1

GNSS::GNSS(SM_ThreadManagement^ sm_tm, SM_GPS^ sm_gps)
{
	SM_TM_ = sm_tm;
	SM_GPS_ = sm_gps;
	Northing_ = 0;
	Easting_ = 0;
	Height_ = 0;
}

error_state GNSS::connect(String^ hostName, int portNumber)
{
	try
	{
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();

		Client = gcnew TcpClient(hostName, portNumber);
		Client->NoDelay = true;
		Client->ReceiveTimeout = 500;
		Client->SendTimeout = 500;
		Client->ReceiveBufferSize = 4096;
		Client->SendBufferSize = 1024;

		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(512);

		Console::WriteLine("GNSS connected");
	}
	catch (SocketException^ e)
	{
		Console::WriteLine("GNSS connection failed: " + e->Message);
		return ERR_CONNECTION;
	}

	return SUCCESS;
}

error_state GNSS::communicate()
{
	if (Stream == nullptr)
	{
		return ERR_CONNECTION;
	}

	const unsigned char SYNC[3] = { 0xAA, 0x44, 0x12 };
	int matched = 0;
	int searched = 0;

	try
	{
		// find the sync bytes AA 44 12
		while (matched < 3)
		{
			int b = Stream->ReadByte();
			if (b == -1)
			{
				return ERR_CONNECTION; // connection closed
			}

			if (++searched > MAX_SEARCH_BYTES)
			{
				return ERR_NO_DATA; // no header found in a reasonable amount of data
			}

			if (b == SYNC[matched])
			{
				matched++; // got the next expected byte
			}
			else if (b == SYNC[0])
			{
				matched = 1; // false start, but this AA could begin a real header
			}
			else
			{
				matched = 0; // start again
			}
		}

		array<unsigned char>^ buffer = gcnew array<unsigned char>(GNSS_RECORD_LENGTH);

		buffer[0] = 0xAA;
		buffer[1] = 0x44;
		buffer[2] = 0x12;

		int total = 3;
		while (total < 112)
		{
			int n = Stream->Read(buffer, total, GNSS_RECORD_LENGTH - total);
			if (n == 0)
			{
				return ERR_CONNECTION;
			}

			total += n;
		}

		if (buffer[3] != 28)
		{
			return ERR_INVALID_DATA;
		}

		pin_ptr<unsigned char> pinned = &buffer[0]; // pin the array; points at its first byte
		unsigned char* raw = pinned; // plain native pointer, safe while 'pinned' exists

		unsigned long calculatedCRC = CalculateBlockCRC32(GNSS_RECORD_LENGTH - 4, raw);   // bytes 0-107

		GNSSData data;
		memcpy(&data, raw, sizeof(GNSSData));

		if (data.Checksum != calculatedCRC)
		{
			Console::WriteLine("GNSS CRC mismatch: received {0:X8}, calculated {1:X8}",
				data.Checksum, calculatedCRC);
			return ERR_INVALID_DATA;
		}

		Northing_ = data.Northing;
		Easting_ = data.Easting;
		Height_ = data.Height;

#if GNSS_Print
		Console::WriteLine("GNSS  N: {0,12:F3} m  E: {1,11:F3} m  H: {2,8:F3} m  CRC rx: {3:X8}  calc: {4:X8}",
			data.Northing, data.Easting, data.Height, data.Checksum, calculatedCRC);
#endif
	}
	catch (System::IO::IOException^)
	{
		return ERR_NO_DATA; // timed out waiting for data
	}

	return SUCCESS;
}

error_state GNSS::processSharedMemory()
{
	Monitor::Enter(SM_GPS_->lockObject);
	SM_GPS_->Northing = Northing_;
	SM_GPS_->Easting = Easting_;
	SM_GPS_->Height = Height_;
	Monitor::Exit(SM_GPS_->lockObject);

	return SUCCESS;
}

bool GNSS::getShutdownFlag()
{
	return (SM_TM_->shutdown & bit_GPS) != 0;
}

void GNSS::threadFunction()
{
	Console::WriteLine("GNSS thread is starting");

	bool connected = (connect(WEEDER_ADDRESS, GNSS_PORT) == SUCCESS);
	if (!connected)
	{
		Console::WriteLine("GNSS: connection failed, will retry");
	}

	if (!skipBarrier)
	{
		SM_TM_->ThreadBarrier->SignalAndWait();
	}

	Stopwatch^ retryTimer = Stopwatch::StartNew();

	while (!getShutdownFlag())
	{
		Monitor::Enter(SM_TM_->lockObject);
		SM_TM_->heartbeat |= bit_GPS;
		Monitor::Exit(SM_TM_->lockObject);

		if (!connected)
		{
			if (retryTimer->ElapsedMilliseconds > 1000)
			{
				connected = (connect(WEEDER_ADDRESS, GNSS_PORT) == SUCCESS);
				retryTimer->Restart();
			}
		}
		else
		{
			error_state result = communicate();
			if (result == SUCCESS)
			{
				result = processSharedMemory();
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

		Thread::Sleep(20);
	}

	if (Stream != nullptr) Stream->Close();
	if (Client != nullptr) Client->Close();

	Console::WriteLine("GNSS thread shutting down");
}

unsigned long GNSS::CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	ulCRC = i;
	for (j = 8; j > 0; j--)
	{
		if (ulCRC & 1)
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	
	return ulCRC;
}

unsigned long GNSS::CalculateBlockCRC32(
	unsigned long ulCount, /* Number of bytes in the data block */
	unsigned char* ucBuffer) /* Data block */
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while (ulCount-- != 0)
	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int)ulCRC ^ * ucBuffer++) & 0xff);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return(ulCRC);
}