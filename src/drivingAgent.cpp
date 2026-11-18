#include "drivingAgent.hpp"
#include "interfaces_data_structs.h"
#include "odometer.hpp"
#include "primitive.h"
#include <algorithm>
#include <iostream>
#include "additional_logging.h"

DrivingAgent::DrivingAgent() {
    lowLevelControl.setTargetVelocity(20);
}

const double DrivingAgent::lookaheadDistance(const input_data_str& scenario_message) {
    return std::max(50.0, 5 * scenario_message.VLgtFild);
}

void DrivingAgent::changeSpeed(double speed, double distance, const input_data_str& scenario_message) {
    double s0 = getDistanceTraveled(scenario_message);
    double t0 = scenario_message.ECUupTime;

    double v0 = scenario_message.VLgtFild;
    double a0 = scenario_message.ALgtFild;
    double sf = distance;
    double vf = speed;

    double tf = finalOptTime(v0, a0, sf, vf);

    double results[5];
    evalPrimitiveCoeffs(v0, a0, sf, vf, 0, tf, results);
    VehicleTrajectory trajectory(t0, tf, s0, results);

    std::cout << "state: Speed: " << speed << " Cruise -> " << trajectory << std::endl;

    lowLevelControl.followTrajectory(trajectory);
}

void DrivingAgent::enterStateCruise(const input_data_str& scenario_message) {
    state = DrivingAgentState::Cruise;
    changeSpeed(cruisingSpeed, 50, scenario_message);
}

void DrivingAgent::loopStateCruise(const input_data_str& scenario_message) {
    if (scenario_message.TrfLightDist < lookaheadDistance(scenario_message)) {
        enterStateTrafficLight(scenario_message);
        return;
    }

    // This function assumes that the speed limits are in order
    if (nextSpeedLimitHandle >= scenario_message.AdasisSpeedLimitNr) {
        return; // All speed limits are handeled
    }

    double nextSpeedLimitDist = scenario_message.AdasisSpeedLimitDist[nextSpeedLimitHandle];
    if (nextSpeedLimitDist > lookaheadDistance(scenario_message)) {
        return; // The next speed limits is not seen yet
    }

    double nextSpeedLimit = scenario_message.AdasisSpeedLimitValues[nextSpeedLimitHandle] / 3.6; // Convert to m/s, assumes that these values are in km/t
    if (nextSpeedLimit <= cruisingSpeed) {
        // A lower in the speed limit
        cruisingSpeed = nextSpeedLimit;
        nextSpeedLimitHandle++;
        changeSpeed(nextSpeedLimit, nextSpeedLimitDist - safetyDistance, scenario_message);
    }

    if (nextSpeedLimit > cruisingSpeed && nextSpeedLimitDist < 0) {
        cruisingSpeed = nextSpeedLimit;
        nextSpeedLimitHandle++;
        changeSpeed(nextSpeedLimit, 50, scenario_message);
    }
}

void DrivingAgent::enterStateTrafficLight(const input_data_str& scenario_message) {
    state = DrivingAgentState::TrafficLight;

    double s0 = getDistanceTraveled(scenario_message);
    double t0 = scenario_message.ECUupTime;

    double v0 = scenario_message.VLgtFild;
    double a0 = scenario_message.ALgtFild;
    double sf = scenario_message.TrfLightDist - safetyDistance;

    double results[5];
    double sf_max;
    double tf;
    stop_primitive(v0, a0, sf, results, &sf_max, &tf);
    VehicleTrajectory trajectory(t0, tf, s0, results);

    std::cout << "state: TrafficLight -> " << trajectory << std::endl;

    lowLevelControl.followTrajectory(trajectory);
}

void DrivingAgent::planManoeuvre(const input_data_str& scenario_message) {
    switch (state) {
        case DrivingAgentState::Initial:
            std::cout << scenario_message << std::endl;
            enterStateCruise(scenario_message);
            return;
        case DrivingAgentState::TrafficLight:
            return;
        case DrivingAgentState::Cruise:
            loopStateCruise(scenario_message);
            return;
    }
}

output_data_str DrivingAgent::computeManoeuvre(const input_data_str& scenario_message) {
    output_data_str manoeuvre;
    manoeuvre.CycleNumber = scenario_message.CycleNumber;
    manoeuvre.Status = scenario_message.Status;

    planManoeuvre(scenario_message);

    double timeStep = scenario_message.ECUupTime - lastMessageTime;

    VehicleControlSignal controlSignal = lowLevelControl.calulateVehicleControl(timeStep, scenario_message);
    manoeuvre.RequestedAcc = controlSignal.requestedThottle;
    manoeuvre.RequestedSteerWhlAg = controlSignal.requestedSteeringWheelAngle;

    lastMessageTime = scenario_message.ECUupTime;
    return manoeuvre;
}

