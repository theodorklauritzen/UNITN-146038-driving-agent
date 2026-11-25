#include "drivingAgent.hpp"
#include "interfaces_data_structs.h"
#include "odometer.hpp"
#include "primitive.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "additional_logging.h"
#include "scenario_constants.h"
#include "vehicleTrajectory.hpp"

double removeNear0Value(double value) {
    return (std::abs(value) < 1e-3) ? std::copysign(1e-3, value) : value;
}

DrivingAgent::DrivingAgent() {
    lowLevelControl.setTargetVelocity(20);
}

const double DrivingAgent::lookaheadDistance(const input_data_str& scenario_message) {
    return std::max(50.0, 5 * scenario_message.VLgtFild);
}

void DrivingAgent::changeSpeed(double speed, double distance, const input_data_str& scenario_message) {
    cruisingSpeed = speed;
    double s0 = getDistanceTraveled(scenario_message);
    double t0 = scenario_message.ECUupTime;

    double v0 = scenario_message.VLgtFild;
    double a0 = removeNear0Value(scenario_message.ALgtFild);
    double sf = distance;
    double vf = speed;

    double tf = finalOptTime(v0, a0, sf, vf);

    double results[5] = {0};
    evalPrimitiveCoeffs(v0, a0, sf, vf, 0, tf, results);
    VehicleTrajectory trajectory(t0, tf, s0, results);

    std::cout << "Speed change from " << v0 << " to " << speed << " " << trajectory << std::endl;

    lowLevelControl.followTrajectory(trajectory);
}

void DrivingAgent::enterStateCruise(const input_data_str& scenario_message) {
    state = DrivingAgentState::Cruise;
    RecommendedSpeedReturn speedReturn = getRecommendedUpperSpeed(scenario_message);
    changeSpeed(speedReturn.speed, std::max(speedReturn.distance, 50.0), scenario_message);
}

void DrivingAgent::loopStateCruise(const input_data_str& scenario_message) {
    if (scenario_message.TrfLightDist < lookaheadDistance(scenario_message) && scenario_message.TrfLightDist > 0) {
        enterStateTrafficLight(scenario_message);
        return;
    }

    RecommendedSpeedReturn speedReturn = getRecommendedUpperSpeed(scenario_message);

    if (speedReturn.speed != cruisingSpeed) {
        changeSpeed(speedReturn.speed, std::max(speedReturn.distance, 50.0), scenario_message);
    }
}

void DrivingAgent::enterStateTrafficLight(const input_data_str& scenario_message) {
    state = DrivingAgentState::TrafficLight;

    double s0 = getDistanceTraveled(scenario_message);
    double t0 = scenario_message.ECUupTime;

    double v0 = scenario_message.VLgtFild;
    double a0 = removeNear0Value(scenario_message.ALgtFild);
    double sf = scenario_message.TrfLightDist;

    nextTrafficLightPosition = s0 + sf;

    double T_green;
    double T_red;

    double v_min = 2;
    double v_max = getRecommendedUpperSpeed(scenario_message).speed;

    TrafficLightState currentState = static_cast<TrafficLightState>(scenario_message.TrfLightCurrState);

    double firstTimeToChange = scenario_message.TrfLightFirstTimeToChange;
    double secondTimeToChange = scenario_message.TrfLightSecondTimeToChange;
    double thirdTimeToChange = scenario_message.TrfLightThirdTimeToChange;

    switch(currentState) {
        case TrafficLightState::GREEN:
            T_green = 0;
            T_red = firstTimeToChange - (safetyDistance + lengthOfIntersection) / v_min;
            break;
        case TrafficLightState::YELLOW: // next is red, then green
            T_green = secondTimeToChange + safetyDistance / v_max;
            T_red = thirdTimeToChange - (safetyDistance + lengthOfIntersection) / v_min;
            break;
        case TrafficLightState::RED: // Next is green
            T_green = firstTimeToChange + safetyDistance / v_max;
            T_red = secondTimeToChange - (safetyDistance + lengthOfIntersection) / v_min;
            break;
        default:
            throw std::runtime_error("The traffic light is in an unimplemented state.");
    }

    // TODO: Handle if the state is green, and we are under safetydistance to the trafficLight.

    double stopCoeffs[5];
    double sf_max;
    double tf;
    stop_primitive(v0, a0, sf - safetyDistance, stopCoeffs, &sf_max, &tf);

    double passCoeffs1[5];
    double v1;
    double T1;
    double passCoeffs2[5];
    double v2;
    double T2;
    pass_primitive(a0, v0, sf, v_min, v_max, T_green, T_red, passCoeffs1, &v1, &T1, passCoeffs2, &v2, &T2);

    bool passPrimiteIsValid = !std::isnan(T1);

    if (!passPrimiteIsValid) { // We need to stop
        VehicleTrajectory trajectory(t0, tf, s0, stopCoeffs);
        trafficLightIntendToStop = true;
        std::cout << "state: TrafficLight stopping -> " << trajectory << std::endl;
        lowLevelControl.followTrajectory(trajectory);
        return;
    }

    trafficLightIntendToStop = false;

    if (passCoeffs1[2] * passCoeffs2[2] < 0) { // Different signs, should do a j(0) = 0 instead
        double passJ0Coeffs[5];
        double tf_passj0;
        pass_primitive_j0(a0, v0, sf, v_min, v_max, passJ0Coeffs, &tf_passj0);

        VehicleTrajectory trajectory(t0, tf_passj0, s0, passJ0Coeffs);
        std::cout << "state: TrafficLight passing j0 -> " << trajectory << std::endl;
        lowLevelControl.followTrajectory(trajectory);
        return;
    }

    if (std::abs(passCoeffs1[2]) < std::abs(passCoeffs2[2])) {
        VehicleTrajectory trajectory(t0, T1, s0, passCoeffs1);
        std::cout << "state: TrafficLight passing 1 -> " << trajectory << std::endl;
        lowLevelControl.followTrajectory(trajectory);
        return;
    }

    VehicleTrajectory trajectory(t0, T1, s0, passCoeffs1);
    std::cout << "state: TrafficLight passing 2 -> " << trajectory << std::endl;
    lowLevelControl.followTrajectory(trajectory);
}

void DrivingAgent::loopStateTrafficLight(const input_data_str &scenario_message) {
    if (getDistanceTraveled(scenario_message) >= nextTrafficLightPosition + lengthOfIntersection) {
        enterStateCruise(scenario_message);
        return;
    }

    TrafficLightState currentState = static_cast<TrafficLightState>(scenario_message.TrfLightCurrState);
    if (
        trafficLightIntendToStop &&
        scenario_message.TrfLightDist < safetyDistance * 2 &&
        currentState == TrafficLightState::GREEN &&
        scenario_message.TrfLightFirstTimeToChange > 5 // The light is green for 8 seconds
    ) {
        trafficLightIntendToStop = false;
        RecommendedSpeedReturn recommendedSpeed = getRecommendedUpperSpeed(scenario_message);
        changeSpeed(recommendedSpeed.speed, std::max(recommendedSpeed.distance, 50.0), scenario_message);
    }
}

void DrivingAgent::planManoeuvre(const input_data_str& scenario_message) {
    switch (state) {
        case DrivingAgentState::Initial:
            std::cout << scenario_message << std::endl;
            enterStateCruise(scenario_message);
            return;
        case DrivingAgentState::TrafficLight:
            loopStateTrafficLight(scenario_message);
            return;
        case DrivingAgentState::Cruise:
            loopStateCruise(scenario_message);
            return;
    }
}

void DrivingAgent::perceptSpeedLimit(const input_data_str &scenario_message) {
    // This function assumes that the speed limits are in order
    if (nextSpeedLimitHandle < scenario_message.AdasisSpeedLimitNr) {
        double nextSpeedLimitDist = scenario_message.AdasisSpeedLimitDist[nextSpeedLimitHandle];
        if (nextSpeedLimitDist < lookaheadDistance(scenario_message)) {
            nextSpeedLimitHandle++;
        }
    }

    if (nextPassingSpeedLimit < nextSpeedLimitHandle) {
        double distToNextpassingSpeedLimit = scenario_message.AdasisSpeedLimitDist[nextPassingSpeedLimit];
        if (distToNextpassingSpeedLimit < 0) {
            nextPassingSpeedLimit++;
        }
    }
}

RecommendedSpeedReturn DrivingAgent::getRecommendedUpperSpeed(const input_data_str& scenario_message) {
    double distanceToLowestSpeed = lookaheadDistance(scenario_message);
    if (nextSpeedLimitHandle == 0) { // We have not passed or seen any signs yet
        return RecommendedSpeedReturn{90 / 3.6, distanceToLowestSpeed};
    }

    double lowestSpeed = 1000;
    for (int i = std::max(nextPassingSpeedLimit - 1, 0); i < nextSpeedLimitHandle; i++) {
        double speedLimitValue = scenario_message.AdasisSpeedLimitValues[i] / 3.6;
        if (speedLimitValue < lowestSpeed) {
            lowestSpeed = speedLimitValue;
            distanceToLowestSpeed = scenario_message.AdasisSpeedLimitDist[i];
        }
    }

    return RecommendedSpeedReturn{lowestSpeed, std::max(distanceToLowestSpeed, 0.0)};
}

output_data_str DrivingAgent::computeManoeuvre(const input_data_str& scenario_message) {
    output_data_str manoeuvre;
    manoeuvre.CycleNumber = scenario_message.CycleNumber;
    manoeuvre.Status = scenario_message.Status;

    perceptSpeedLimit(scenario_message);

    planManoeuvre(scenario_message);

    double timeStep = scenario_message.ECUupTime - lastMessageTime;

    VehicleControlSignal controlSignal = lowLevelControl.calulateVehicleControl(timeStep, scenario_message);
    manoeuvre.RequestedAcc = controlSignal.requestedThottle;
    manoeuvre.RequestedSteerWhlAg = controlSignal.requestedSteeringWheelAngle;

    lastMessageTime = scenario_message.ECUupTime;
    return manoeuvre;
}

