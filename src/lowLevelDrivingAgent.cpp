#include "lowLevelDrivingAgent.hpp"
#include "interfaces_data_structs.h"
#include "odometer.hpp"
#include <cmath>
#include <cstdlib>

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
    double targetAcceleration = velocityRegulatorP * error;

    return targetAcceleration;
}

double LowLevelDrivingAgent::calculateFollowTrajectoryAcceleration(double timeStep, const input_data_str& scenario_message) {
    double time = scenario_message.ECUupTime;
    if (targetTrajectory.getEndTime() < time) {
        double targetVelocity = targetTrajectory.vEval(targetTrajectory.getEndTime());
        std::cout << "Trajecgory completed -> ";
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

    std::cout << "s = " << s << ", trf dist = " << scenario_message.TrfLightDist << std::endl;

    constexpr double wn = 1.5, zeta = 1.0;
    constexpr double Kp = wn * wn;
    constexpr double Kv = 2.0 * zeta * wn;

    double aCmd = aRef + Kv * (vRef - v) + Kp * (sRef - s);
    return aCmd;
}

const double LowLevelDrivingAgent::calculateStopAcceleration(const input_data_str& scenario_message) {
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

    VehicleControlSignal controlSignal;
    controlSignal.requestedSteeringWheelAngle = 0;
    controlSignal.requestedThottle = calulateThrottle(timeStep, targetAcceleration, scenario_message);

    return controlSignal;
}

const double LowLevelDrivingAgent::calulateThrottle(double timeStep, double targetAcceleration, const input_data_str& scenario_message) {
    double error = targetAcceleration - scenario_message.ALgtFild;
    accelIntegratedError += error * timeStep;

    double throttle;
    // Regulator throttle
    throttle += accelRegulatorP * error + accelRegulatorI * accelIntegratedError;

    // Feedforward part, from the model a = G*u - k*v^2
    constexpr double k = 0.004279;
    constexpr double gain = 7;
    double velocity = scenario_message.VLgtFild;
    throttle += (targetAcceleration + k * std::pow(velocity, 2)) / gain;

    return throttle;
}
