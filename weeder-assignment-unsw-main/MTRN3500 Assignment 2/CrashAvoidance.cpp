#include "CrashAvoidance.h"
#include <math.h>

CrashAvoidance::CrashAvoidance(SM_ThreadManagement^ sm_tm, SM_Laser^ sm_laser, SM_VehicleControl^ sm_vehiclecontrol)
{
	SM_TM_ = sm_tm;
	SM_Laser_ = sm_laser;
	SM_VehicleControl_ = sm_vehiclecontrol;
	x_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	y_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	steering_ = 0;
	scanCount_ = 0;
	lastScanCount_ = 0;
	staleTimer_ = Stopwatch::StartNew();
	inhibited_ = false;
}

error_state CrashAvoidance::processSharedMemory()
{
	Monitor::Enter(SM_Laser_->lockObject);
	try
	{
		Array::Copy(SM_Laser_->x, x_, STANDARD_LASER_LENGTH);
		Array::Copy(SM_Laser_->y, y_, STANDARD_LASER_LENGTH);
		scanCount_ = SM_Laser_->ScanCount;
	}
	finally
	{
		Monitor::Exit(SM_Laser_->lockObject);
	}
	
	Monitor::Enter(SM_VehicleControl_->lockObject);
	steering_ = SM_VehicleControl_->Steering;
	Monitor::Exit(SM_VehicleControl_->lockObject);

	double scale;
	bool inhibit;
	error_state result = SUCCESS;

	if (isStale())
	{
		scale = 0.0;
		inhibit = true;
		result = ERR_NO_DATA;
	}
	else
	{
		double d = nearestObstacle();
		scale = computeScale(d);
		inhibit = inhibited_;
	}

	Monitor::Enter(SM_VehicleControl_->lockObject);
	SM_VehicleControl_->SpeedScale = scale;
	SM_VehicleControl_->Inhibit = inhibit;
	Monitor::Exit(SM_VehicleControl_->lockObject);

	return result;
}

bool CrashAvoidance::getShutdownFlag()
{
	return (SM_TM_->shutdown & bit_CRASHAVOIDANCE) != 0;
}

void CrashAvoidance::threadFunction()
{
	Console::WriteLine("Crash Avoidance thread is starting");

	SM_TM_->ThreadBarrier->SignalAndWait();

	while (!getShutdownFlag())
	{
		Monitor::Enter(SM_TM_->lockObject);
		SM_TM_->heartbeat |= bit_CRASHAVOIDANCE;
		Monitor::Exit(SM_TM_->lockObject);

		error_state result = processSharedMemory();

		if (result != SUCCESS)
		{
			printError(result);
		}

		Thread::Sleep(20);
	}

	Console::WriteLine("Crash Avoidance thread is shutting down");
}

bool CrashAvoidance::isStale()
{
	if (scanCount_ != lastScanCount_)
	{
		lastScanCount_ = scanCount_;
		staleTimer_->Restart();
	}

	return staleTimer_->ElapsedMilliseconds > STALE_TIMEOUT_MS;
}

double CrashAvoidance::computeScale(double d)
{

	if (d <= STOP_DIST)
	{
		inhibited_ = true;
	}
	else if (inhibited_ && d > RELEASE_DIST)
	{
		inhibited_ = false;
	}

	if (inhibited_)
	{
		return 0.0;
	}

	double ratio = (d - STOP_DIST) / (SLOW_DIST - STOP_DIST);
	ratio = Math::Max(0.0, Math::Min(1.0, ratio)); // clamping between [0 1]
	return Math::Sqrt(ratio);
}

bool CrashAvoidance::inCorridor(double x, double y, double% pathDist)
{
	if (x <= 0)
	{
		return false; // points behind can't be in the path
	}

	// straight case
	if (Math::Abs(steering_) < STRAIGHT_STEER_DEGREES)
	{
		if (Math::Abs(y) <= CORRIDOR_HALF_WIDTH)
		{
			pathDist = x; // distance straight ahead
			return true;
		}
		return false;
	}

	// steering case
	// from geometry lecture
	double steerRad = steering_ * Math::PI / 180.0;
	double R = WHEELBASE / Math::Tan(Math::Abs(steerRad));

	// mirror a right turn into a left turn, so one set of maths covers both
	if (steering_ < 0)
	{
		y = -y;
	}

	// turn centre is at (0, R), directly to the left of the vehicle
	double dx = x;
	double dy = y - R;
	double rho = Math::Sqrt(dx * dx + dy * dy); // point's distance from the turn centre

	// inside the corridor if within half-width of the path's circle
	if (Math::Abs(rho - R) > CORRIDOR_HALF_WIDTH)
	{
		return false;
	}

	// angle swept around the centre, from the vehicle's position to the point.
	double swept = Math::Atan2(dy, dx) + Math::PI / 2.0;
	if (swept < 0)
	{
		swept += 2.0 * Math::PI;
	}

	// ignore points more than half a turn 
	if (swept > Math::PI)
	{
		return false;
	}

	pathDist = R * swept; // arc length along the corridor
	return true;
}

double CrashAvoidance::nearestObstacle()
{
	double nearest = NO_OBSTACLE;
	int runLength = 0;
	double runMin = NO_OBSTACLE;

	for (int i = 0; i < STANDARD_LASER_LENGTH; i++)
	{
		double range = Math::Sqrt(x_[i] * x_[i] + y_[i] * y_[i]);
		double pathDist = 0;
		bool inside = false;

		if (range >= MIN_VALID_RANGE)            // ignore invalid points
		{
			inside = inCorridor(x_[i], y_[i], pathDist);
		}

		if (inside)
		{
			runLength++;
			runMin = Math::Min(runMin, pathDist);
		}
		else
		{
			if (runLength >= MIN_CLUSTER_POINTS)  // big enough to be real
			{
				nearest = Math::Min(nearest, runMin);
			}
			runLength = 0;
			runMin = NO_OBSTACLE;
		}
	}

	// a run may still be open at the end of the scan
	if (runLength >= MIN_CLUSTER_POINTS)
	{
		nearest = Math::Min(nearest, runMin);
	}

	return nearest;
}