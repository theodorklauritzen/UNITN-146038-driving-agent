#include "lowLevelDrivingAgent.hpp"
#include "interfaces_data_structs.h"
#include "odometer.hpp"
#include <cmath>
#include <cstdlib>
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

void LowLevelDrivingAgent::stop() {
    state = LowLevelDrivingAgentState::Stop;
    targetVelocity = 0;
}

const double LowLevelDrivingAgent::calculateConstantVelocityAcceleration(double timeStep, const input_data_str& scenario_message) {
    double error = targetVelocity - scenario_message.VLgtFild;

    constexpr double Kp = 1;
    double targetAcceleration = Kp * error;

    return targetAcceleration;
}

double LowLevelDrivingAgent::calculateFollowTrajectoryAcceleration(double timeStep, const input_data_str& scenario_message) {
    double time = scenario_message.ECUupTime;
    if (targetTrajectory.getEndTime() < time) {
        double targetVelocity = targetTrajectory.vEval(targetTrajectory.getEndTime());
        std::cout << "Trajectory completed -> ";
        if (std::abs(targetVelocity) < 0.1) {
            stop();
            std::cout << "stopping";
        } else {
            setTargetVelocity(targetVelocity);
            std::cout << "new target velocity: " << targetVelocity;
        }
        std::cout << std::endl;
        time = targetTrajectory.getEndTime();
    }

    double aRef = targetTrajectory.aEval(time);
    double vRef = targetTrajectory.vEval(time);
    double sRef = targetTrajectory.sEval(time);

    double v = scenario_message.VLgtFild;
    double s = getDistanceTraveled(scenario_message);

    constexpr double Kp = 0.1;
    constexpr double Kv = 1;

    double aCmd = aRef + Kv * (vRef - v) + Kp * (sRef - s);
    return aCmd;
}

const double LowLevelDrivingAgent::calculateStopAcceleration(const input_data_str& scenario_message) {
    std::cout << "Distance left: " << scenario_message.TrfLightDist << std::endl;
    return -1; // This will brake, it seems like the vehicle cannot go in reverse.
}

const VehicleControlSignal LowLevelDrivingAgent::calulateVehicleControl(double timeStep, const input_data_str& scenario_message) {
    double targetAcceleration = NAN;
    switch (state) {
        case LowLevelDrivingAgentState::ConstantVelocity:
            targetAcceleration = calculateConstantVelocityAcceleration(timeStep, scenario_message);
            break;
        case LowLevelDrivingAgentState::FollowTrajectory:
            targetAcceleration = calculateFollowTrajectoryAcceleration(timeStep, scenario_message);
            break;
        case LowLevelDrivingAgentState::Stop:
            targetAcceleration = calculateStopAcceleration(scenario_message);
            break;
    }

    if (std::isnan(targetAcceleration)) {
        throw std::runtime_error("The target acceleration is not calculated.");
    }

    VehicleControlSignal controlSignal;
    controlSignal.requestedSteeringWheelAngle = 0;
    controlSignal.requestedThottle = calulateThrottle(timeStep, targetAcceleration, scenario_message);

    return controlSignal;
}

const double LowLevelDrivingAgent::calulateThrottle(double timeStep, double targetAcceleration, const input_data_str& scenario_message) {
    double throttle = 0;

    // Feedforward part, from the model a = G*u - k*v^2
    constexpr double k = 0.004279;
    constexpr double gasGain = 7;
    constexpr double brakeGain = 20;

    double velocity = scenario_message.VLgtFild;
    double aeroAdjusted = targetAcceleration + k * std::pow(velocity, 2);

    if (aeroAdjusted > 0) { // We need more gas
        throttle += aeroAdjusted / gasGain;
    } else { // We need to brake
        throttle += aeroAdjusted / brakeGain;
    }

    // Regulator throttle
    double error = targetAcceleration - scenario_message.ALgtFild;
    accelIntegratedError += error * timeStep;

    constexpr double Ki = 0.01;
    constexpr double Kp = 0.05;

    throttle += Kp * error + Ki * accelIntegratedError;

    return throttle;
}
