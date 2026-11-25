#include "odometer.hpp"
#include "interfaces_data_structs.h"
#include <cmath>
#include <stdexcept>

double getTravelMeasurement(const input_data_str& scenario_message) {
    if (scenario_message.AdasisSpeedLimitNr > 0) {
        return scenario_message.AdasisSpeedLimitDist[0];
    }

    throw std::runtime_error("Could not find a reference point to measure the distance traveled.");
}

double getDistanceTraveled(const input_data_str& scenario_message) {
    static double referencePoint = NAN;
    if (std::isnan(referencePoint)) {
        referencePoint = getTravelMeasurement(scenario_message);
    }

    return referencePoint - getTravelMeasurement(scenario_message);
}
