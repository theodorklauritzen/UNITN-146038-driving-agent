#include "drivingAgent.hpp"
#include "interfaces_data_structs.h"

DrivingAgent::DrivingAgent() {
    lowLevelControl.setTargetVelocity(20);
}

output_data_str DrivingAgent::computeManoeuvre(const double time, const input_data_str& scenario_message) {
    output_data_str manoeuvre;
    manoeuvre.CycleNumber = scenario_message.CycleNumber;
    manoeuvre.Status = scenario_message.Status;

    double timeStep = time - lastMessageTime;

    VehicleControlSignal controlSignal = lowLevelControl.calulateVehicleControl(timeStep, scenario_message);
    manoeuvre.RequestedAcc = controlSignal.requestedAcceleration;
    manoeuvre.RequestedSteerWhlAg = controlSignal.requestedSteeringWheelAngle;

    lastMessageTime = time;
    return manoeuvre;
}

