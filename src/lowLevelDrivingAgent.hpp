#pragma once
#include "interfaces_data_structs.h"
#include "vehicleTrajectory.hpp"

struct VehicleControlSignal {
    double requestedThottle;
    double requestedSteeringWheelAngle;
};

std::ostream& operator<<(std::ostream& os, const VehicleControlSignal& controlSignal);

enum class LowLevelDrivingAgentState {
    ConstantVelocity,
    FollowTrajectory,
};

class LowLevelDrivingAgent {

    LowLevelDrivingAgentState state = LowLevelDrivingAgentState::ConstantVelocity;

    // For the state ConstantVelocity
    double targetVelocity = 0;
    constexpr static double velocityRegulatorP = 1;

    VehicleControlSignal calculateConstantVelocityAcceleration(double timeStep, const input_data_str& scenario_message);

    // For the satet FollowTrajectory
    VehicleTrajectory targetTrajectory;
    VehicleControlSignal calculateFollowTrajectoryAcceleration(double timeStep, double time, const input_data_str& scenario_message);

    // Acceleration control
    constexpr static double accelRegulatorI = 0.001;
    constexpr static double accelRegulatorP = 0.005;
    double accelIntegratedError = 0;
    const double calulateThrottle(double timeStep, double targetAcceleration, const input_data_str& scenario_message);

    public:

    void setTargetVelocity(double velocity);
    void followTrajectory(VehicleTrajectory trajectory);

    const VehicleControlSignal calulateVehicleControl(double time, double timeStep, const input_data_str& scenario_message);
};
