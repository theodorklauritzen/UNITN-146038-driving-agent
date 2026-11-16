#pragma once
#include "interfaces_data_structs.h"
#include "vehicleTrajectory.hpp"

struct VehicleControlSignal {
    double requestedAcceleration;
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
    double integratedError = 0;
    constexpr static double regulatorI = 0.01;
    constexpr static double regulatorP = 1;

    VehicleControlSignal calculateConstantVelocityAcceleration(double timeStep, const input_data_str& scenario_message);

    // For the satet FollowTrajectory
    VehicleTrajectory targetTrajectory;
    VehicleControlSignal calculateFollowTrajectoryAcceleration(double time, const input_data_str& scenario_message);

    public:

    void setTargetVelocity(double velocity);
    void followTrajectory(VehicleTrajectory trajectory);

    const VehicleControlSignal calulateVehicleControl(double time, double timeStep, const input_data_str& scenario_message);
};
