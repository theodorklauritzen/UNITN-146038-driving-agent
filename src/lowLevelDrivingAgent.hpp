#pragma once
#include "interfaces_data_structs.h"

struct VehicleControlSignal {
    double requestedAcceleration;
    double requestedSteeringWheelAngle;
};

class LowLevelDrivingAgent {
    double targetVelocity;

    constexpr static double regulatorI = 0.01;
    constexpr static double regulatorP = 1;

    double integratedError;

    public:

    void setTargetVelocity(double velocity) { targetVelocity = velocity; }

    const VehicleControlSignal calulateVehicleControl(double timeStep, const input_data_str& scenario_message);
};
