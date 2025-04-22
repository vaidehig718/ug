#ifndef EBUS_VNS_CONSTANTS_H
#define EBUS_VNS_CONSTANTS_H

#include "logger.h"
#include <array>
#include <string>

constexpr double VEHICLE_COST = 381500.0; // Acquisition cost of a bus
constexpr double CHARGE_LOC_COST = 218000.0;  // Cost of opening one charging location
constexpr double COST_PER_KM = 2100.0;  // Per km travel cost

constexpr double MAX_CHARGE_LEVEL = 300.0;  // Maximum allowed charge level in kWh
constexpr double MIN_CHARGE_LEVEL = 45.0;  // Minimum charge level to be maintained in kWh
constexpr double CHARGE_RATE = 1.67;  // Rate at which charging happens km/min
constexpr double MAX_ENERGY_PER_MIN = 2.505;  // Maximum energy IN kWh that can be charged in a minute
constexpr double ENERGY_PER_KM = 1.5;  // Energy consumed in kWh/km

constexpr int NUM_PRICE_INTERVALS = 6;  // Number of energy price points
constexpr std::array<int, NUM_PRICE_INTERVALS+1> ENERGY_LEFT_INTERVAL = {0, 540, 840, 960, 1260, 1440, 1980};
constexpr std::array<double, NUM_PRICE_INTERVALS> ENERGY_PRICE = {555.0, 444.0, 555.0, 1355.0, 555.0, 555.0};
//TODO: Do we need a last interval since the prices are the same?

constexpr double POWER_CAPACITY_PRICE = 654;  // in Cost/kW

constexpr double INF = 1e12; // Large number to represent infinity
constexpr double EPSILON = 1e-12; // Small number to compare doubles
constexpr double SMALL_EPSILON = 1e-15; // Small number to compare doubles

constexpr double IDLE_TIME_THRESHOLD = 0.0;  // Threshold for idle time in minutes used in opening and closing stations
constexpr int SHIFT_MULTIPLE_TRIPS_THRESHOLD = 7; // Threshold for number of trips in a rotation to perform shift multiple trips (includes depots)

constexpr bool SOLVE_EVSP_CSP = true;  // Flag to solve the CSP jointly or separately in the exchanges and shifts
constexpr bool SOLVE_CLP_CSP = true; // Flag to solve CSP with CLP

constexpr int NUM_SHORTLISTED_SOLUTIONS = 500; // Set this preferably to be a multiple of the number of cores used
constexpr bool USE_HYBRID_OPERATORS = false;  // Flag to use hybrid operators

#endif //EBUS_VNS_CONSTANTS_H
