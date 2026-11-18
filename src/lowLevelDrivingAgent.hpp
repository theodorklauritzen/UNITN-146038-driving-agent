#pragma once
#include "interfaces_data_structs.h"
#include "vehicleTrajectory.hpp"
#include <fstream>

struct VehicleControlSignal {
    double requestedThottle;
    double requestedSteeringWheelAngle;
};

std::ostream& operator<<(std::ostream& os, const VehicleControlSignal& controlSignal);

enum class LowLevelDrivingAgentState {
    ConstantVelocity,
    FollowTrajectory,
    Stop,
};

class LowLevelDrivingAgent {

    LowLevelDrivingAgentState state = LowLevelDrivingAgentState::ConstantVelocity;

    // For the state ConstantVelocity
    double targetVelocity = 0;

    const double calculateConstantVelocityThrottle(double timeStep, const input_data_str& scenario_message);

    // For the state FollowTrajectory
    std::ofstream trajectoryFile{"trajectories.txt"};
    VehicleTrajectory targetTrajectory;
    double calculateFollowTrajectoryAcceleration(double timeStep, const input_data_str& scenario_message);

    // For the state Stop
    const double calculateStopThrottle(const input_data_str& scenario_message);

    // Acceleration control
    double accelIntegratedError = 0;
    const double calculateThrottle(double timeStep, double targetAcceleration, const input_data_str& scenario_message);

    public:

    void setTargetVelocity(double velocity);
    void followTrajectory(VehicleTrajectory trajectory);
    void stop();

    const VehicleControlSignal calulateVehicleControl(double timeStep, const input_data_str& scenario_message);
};
