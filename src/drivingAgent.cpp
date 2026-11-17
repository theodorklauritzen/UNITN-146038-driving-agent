#include "drivingAgent.hpp"
#include "interfaces_data_structs.h"
#include "primitive.h"
#include <algorithm>
#include <iostream>

DrivingAgent::DrivingAgent() {
    lowLevelControl.setTargetVelocity(20);
}

const double DrivingAgent::lookaheadDistance(const input_data_str& scenario_message) {
    return std::max(50.0, 5 * scenario_message.VLgtFild);
}

void DrivingAgent::planManoeuvre(const double time, const input_data_str& scenario_message) {
    if (state != DrivingAgentState::Initial) {
        return;
    }
    state = DrivingAgentState::Cruise;


    double v0 = scenario_message.VLgtFild;
    double a0 = scenario_message.ALgtFild;
    double sf = 10;
    double vf = 5;

    double tf = finalOptTime(v0, a0, sf, vf);

    double results[5];
    evalPrimitiveCoeffs(v0, a0, sf, vf, 0, tf, results);
    VehicleTrajectory trajectory(time, tf, results);

    // std::cout << trajectory << std::endl;

    lowLevelControl.followTrajectory(trajectory);
}

output_data_str DrivingAgent::computeManoeuvre(const double time, const input_data_str& scenario_message) {
    output_data_str manoeuvre;
    manoeuvre.CycleNumber = scenario_message.CycleNumber;
    manoeuvre.Status = scenario_message.Status;

    planManoeuvre(time, scenario_message);

    double timeStep = time - lastMessageTime;

    VehicleControlSignal controlSignal = lowLevelControl.calulateVehicleControl(time, timeStep, scenario_message);
    manoeuvre.RequestedAcc = controlSignal.requestedThottle;
    manoeuvre.RequestedSteerWhlAg = controlSignal.requestedSteeringWheelAngle;

    lastMessageTime = time;
    return manoeuvre;
}

