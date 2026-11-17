#include "odometer.hpp"
#include <cmath>

double getDistanceTraveled(const input_data_str& scenario_message) {
    static double referencePoint = NAN;
    if (std::isnan(referencePoint)) {
        referencePoint = scenario_message.TrfLightDist;
    }

    return referencePoint - scenario_message.TrfLightDist;
}
