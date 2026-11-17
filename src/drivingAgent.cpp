#include "drivingAgent.hpp"
#include "interfaces_data_structs.h"
#include "odometer.hpp"
#include "primitive.h"
#include <algorithm>
#include <iostream>

DrivingAgent::DrivingAgent() {
    lowLevelControl.setTargetVelocity(20);
}

const double DrivingAgent::lookaheadDistance(const input_data_str& scenario_message) {
    return std::max(50.0, 5 * scenario_message.VLgtFild);
}

void DrivingAgent::planManoeuvre(const input_data_str& scenario_message) {
    switch (state) {
        case DrivingAgentState::Initial:

            state = DrivingAgentState::Cruise;
            return;
        case DrivingAgentState::TrafficLight:
            return;
        case DrivingAgentState::Cruise:
            if (scenario_message.TrfLightDist > lookaheadDistance(scenario_message)) {
                return;
            }
            state = DrivingAgentState::TrafficLight;
            break;
    }

    double s0 = getDistanceTraveled(scenario_message);
    double t0 = scenario_message.ECUupTime;

    double v0 = scenario_message.VLgtFild;
    double a0 = scenario_message.ALgtFild;
    double sf = scenario_message.TrfLightDist;
    double vf = 5;

    double results[5];
    double sf_max;
    double tf;
    stop_primitive(v0, a0, sf, results, &sf_max, &tf);
    VehicleTrajectory trajectory(t0, tf, s0, results);

    std::cout << trajectory << std::endl;

    lowLevelControl.followTrajectory(trajectory);
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

