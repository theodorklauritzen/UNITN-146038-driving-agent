#include "lowLevelDrivingAgent.hpp"
#include "interfaces_data_structs.h"
#include <cmath>
#include <stdexcept>

std::ostream& operator<<(std::ostream& os, const VehicleControlSignal& controlSignal) {
    return os << "VehicleControlSignal(throttle=" << controlSignal.requestedThottle << ")";
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
    double targetAcceleration = velocityRegulatorP * error;

    VehicleControlSignal controlSignal;

    controlSignal.requestedSteeringWheelAngle = 0;
    controlSignal.requestedThottle = calulateThrottle(timeStep, targetAcceleration, scenario_message);

    return controlSignal;
}

VehicleControlSignal LowLevelDrivingAgent::calculateFollowTrajectoryAcceleration(double timeStep, double time, const input_data_str& scenario_message) {
    VehicleControlSignal controlSignal;
    controlSignal.requestedSteeringWheelAngle = 0;

    double targetAcceleration = targetTrajectory.aEval(time);

    controlSignal.requestedThottle = calulateThrottle(timeStep, targetAcceleration, scenario_message);

    if (targetTrajectory.getEndTime() < time) {
        double targetVelocity = targetTrajectory.vEval(time);
        std::cout << "targetVelocity: " << targetVelocity << std::endl;
        setTargetVelocity(targetVelocity);
    }

    return controlSignal;
}

const VehicleControlSignal LowLevelDrivingAgent::calulateVehicleControl(double time, double timeStep, const input_data_str& scenario_message) {
    // TODO: Maybe do a LP filter on the result?
    // switch (state) {
    //     case LowLevelDrivingAgentState::ConstantVelocity:
    //         return calculateConstantVelocityAcceleration(timeStep, scenario_message);
    //     case LowLevelDrivingAgentState::FollowTrajectory:
    //         return calculateFollowTrajectoryAcceleration(timeStep, time, scenario_message);
    // }
    //
    // throw std::runtime_error("Could not calulate vehicle control signal, not in a valid state.");

    double targetAcceleration = (time < 7) ? 1 : 0;
    VehicleControlSignal controlSignal;
    controlSignal.requestedSteeringWheelAngle = 0;
    controlSignal.requestedThottle = calulateThrottle(timeStep, targetAcceleration, scenario_message);
    std::cout << scenario_message.VLgtFild << "," << scenario_message.ALgtFild << std::endl;
    return controlSignal;

}

const double LowLevelDrivingAgent::calulateThrottle(double timeStep, double targetAcceleration, const input_data_str& scenario_message) {
    double error = targetAcceleration - scenario_message.ALgtFild;
    accelIntegratedError += error * timeStep;

    double throttle;
    // Regulator throttle
    throttle += accelRegulatorP * error + accelRegulatorI * accelIntegratedError;

    // Feedforward part, from the model a = G*u - k*v^2, where G = 7 and k = 0.004279
    constexpr double k = 0.004279;
    constexpr double gain = 7;
    double velocity = scenario_message.VLgtFild;
    throttle += (targetAcceleration + k * std::pow(velocity, 2)) / gain;

    return throttle;
}
