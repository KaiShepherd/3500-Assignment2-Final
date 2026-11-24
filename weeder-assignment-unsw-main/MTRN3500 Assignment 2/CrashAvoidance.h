#pragma once
//inherits from UGVModule

#include <UGVModule.h>

// crash avoidance parameter
constexpr double STOP_DIST = 1.0;
constexpr double RELEASE_DIST = 1.2;
constexpr double SLOW_DIST = 3.0;

// vehicle geometry parameters
constexpr double VEHICLE_HALF_WIDTH = 0.3;
constexpr double SAFETY_MARGIN = 0.1;
constexpr double CORRIDOR_HALF_WIDTH = VEHICLE_HALF_WIDTH + SAFETY_MARGIN;
constexpr double WHEELBASE = 0.5;

constexpr double STRAIGHT_STEER_DEGREES = 1.0; // steering angles < |1| to be treated as straight
constexpr double MIN_VALID_RANGE = 0.05; // closer points to be considered invalid
constexpr int MIN_CLUSTER_POINTS = 3; // minimum number of consectuive points required to count as an obstacle
constexpr int STALE_TIMEOUT_MS = 200; // no new scan for this long = inhibit motion
constexpr double NO_OBSTACLE = 1.0e9; // d = infinity i.e clear path


ref class CrashAvoidance : public UGVModule
{
public:
	CrashAvoidance(SM_ThreadManagement^ sm_tm, SM_Laser^ sm_laser, SM_VehicleControl^ sm_vehiclecontrol);
	error_state processSharedMemory() override;
	bool getShutdownFlag() override;
	void threadFunction() override;

private:
	bool isStale();
	double nearestObstacle();
	bool inCorridor(double x, double y, double% pathDist);
	double computeScale(double d);

	SM_Laser^ SM_Laser_;
	SM_VehicleControl^ SM_VehicleControl_;

	array<double>^ x_;
	array<double>^ y_;
	double steering_;
	unsigned int scanCount_;

	unsigned int lastScanCount_;
	Stopwatch^ staleTimer_;
	bool inhibited_;
};