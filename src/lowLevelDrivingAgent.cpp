#include "lowLevelDrivingAgent.hpp"

const VehicleControlSignal LowLevelDrivingAgent::calulateVehicleControl(double timeStep, const input_data_str& scenario_message) {
    VehicleControlSignal controlSignal;

    controlSignal.requestedSteeringWheelAngle = 0;

    double error = targetVelocity - scenario_message.VLgtFild;
    integratedError += error * timeStep;

    controlSignal.requestedAcceleration = error * regulatorP + integratedError * regulatorI;

    return controlSignal;
}
