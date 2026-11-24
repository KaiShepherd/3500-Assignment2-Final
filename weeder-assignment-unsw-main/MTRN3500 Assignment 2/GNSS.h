#pragma once
//inherits from NetworkedModule

#include <NetworkedModule.h>

constexpr int GNSS_PORT = 24000;
constexpr int MAX_SEARCH_BYTES = 1000;

#define CRC32_POLYNOMIAL 0xEDB88320L

#pragma pack(push, 1)
struct GNSSData
{
    unsigned char Sync[3];          // 0-2:   0xAA 0x44 0x12
    unsigned char HeaderLength;     // 3:     should be 28
    unsigned char Discard1[40];     // 4-43:  rest of header + sol status, pos type, zone
    double Northing;                // 44-51
    double Easting;                 // 52-59
    double Height;                  // 60-67
    unsigned char Discard2[40];     // 68-107: undulation, datum, std devs, etc.
    unsigned int Checksum;          // 108-111: CRC32 of bytes 0-107
};
#pragma pack(pop)

static_assert(sizeof(GNSSData) == 112, "GNSSData must match the 112-byte BESTUTM record");
constexpr int GNSS_RECORD_LENGTH = sizeof(GNSSData);

ref class GNSS : public NetworkedModule
{
public:
    GNSS(SM_ThreadManagement^ sm_tm, SM_GPS^ sm_gps);

    error_state connect(String^ hostName, int portNumber) override;
    error_state communicate() override;
    error_state processSharedMemory() override;
    bool getShutdownFlag() override;
    void threadFunction() override;
    bool skipBarrier = false;

    //provided helpers
    unsigned long CRC32Value(int i);
    unsigned long CalculateBlockCRC32(unsigned long ulCount,
        unsigned char* ucBuffer);

private:
    SM_GPS^ SM_GPS_;
    double Northing_;
    double Easting_;
    double Height_;
};