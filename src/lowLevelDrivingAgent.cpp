#include "lowLevelDrivingAgent.hpp"
#include "interfaces_data_structs.h"
#include <stdexcept>

std::ostream& operator<<(std::ostream& os, const VehicleControlSignal& controlSignal) {
    return os << "VehicleControlSignal(acceleration=" << controlSignal.requestedAcceleration << ")";
}

void LowLevelDrivingAgent::setTargetVelocity(double velocity) {
    state = LowLevelDrivingAgentState::ConstantVelocity;
    targetVelocity = velocity;
}

void LowLevelDrivingAgent::followTrajectory(VehicleTrajectory trajectory) {
    state = LowLevelDrivingAgentState::FollowTrajectory;
    targetTrajectory = trajectory;
}

VehicleControlSignal LowLevelDrivingAgent::calculateConstantVelocityAcceleration(double timeStep, const input_data_str& scenario_message) {
    double error = targetVelocity - scenario_message.VLgtFild;
    integratedError += error * timeStep;

    VehicleControlSignal controlSignal;

    controlSignal.requestedSteeringWheelAngle = 0;
    controlSignal.requestedAcceleration = error * regulatorP + integratedError * regulatorI;

    return controlSignal;
}

VehicleControlSignal LowLevelDrivingAgent::calculateFollowTrajectoryAcceleration(double time, const input_data_str& scenario_message) {
    VehicleControlSignal controlSignal;
    controlSignal.requestedSteeringWheelAngle = 0;

    controlSignal.requestedAcceleration = targetTrajectory.aEval(time);

    if (targetTrajectory.getEndTime() < time) {
        double targetVelocity = targetTrajectory.vEval(time);
        std::cout << "targetVelocity: " << targetVelocity << std::endl;
        setTargetVelocity(targetVelocity);
    }

    return controlSignal;
}

const VehicleControlSignal LowLevelDrivingAgent::calulateVehicleControl(double time, double timeStep, const input_data_str& scenario_message) {
    // TODO: Maybe do a LP filter on the result?
    switch (state) {
        case LowLevelDrivingAgentState::ConstantVelocity:
            return calculateConstantVelocityAcceleration(timeStep, scenario_message);
        case LowLevelDrivingAgentState::FollowTrajectory:
            return calculateFollowTrajectoryAcceleration(time, scenario_message);
    }

    throw std::runtime_error("Could not calulate vehicle control signal, not in a valid state.");
}
