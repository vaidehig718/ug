#ifndef EBUS_VNS_OPERATORS_H
#define EBUS_VNS_OPERATORS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include "csp.h"
#include <omp.h>
#include <random>
#include <sstream>
#include <climits>
#include <utility>
#include <map>

int extern NUM_THREADS;  // Number of threads to use for parallelization

class Exchange {
public:
    int first_vehicle_index;
    int first_trip_index;
    int second_vehicle_index;
    int second_trip_index;
    double savings;

    // Create a constructor that initializes these variables with maximum integer values
    Exchange()
            :first_vehicle_index(INT_MAX),
             first_trip_index(INT_MAX),
             second_vehicle_index(INT_MAX),
             second_trip_index(INT_MAX) { }
};

class Shift {
public:
    int source_vehicle_index;
    int source_trip_index;
    int dest_vehicle_index;
    int dest_trip_index; // The new trip is inserted after this index
    double savings;

    // Create a constructor that initializes these variables with maximum integer values
    Shift()
            :source_vehicle_index(INT_MAX),
             source_trip_index(INT_MAX),
             dest_vehicle_index(INT_MAX),
             dest_trip_index(INT_MAX) { }
};

class ThreeExchange {
public:
    int first_vehicle_index;
    int first_trip_index;
    int second_vehicle_index;
    int second_trip_index;
    int third_vehicle_index;
    int third_trip_index;
};

namespace scheduling {
double exchange_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&, Exchange&);
double exchange_depots(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&, Exchange&);
double shift_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&, Shift&);

double exchange_trips_hybrid(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&,
        Exchange&);
double exchange_depots_hybrid(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&,
        Exchange&);
double shift_trips_hybrid(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&, Shift&);

void perform_exchange(std::vector<Vehicle>&, Exchange&);
void perform_shift(std::vector<Vehicle>&, Shift&);

void apply_best_improvement(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);
void optimize_rotations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);
}

namespace locations {
void split_trips(std::vector<Vehicle>&, std::vector<int>&, int, int);
bool are_rotations_charge_feasible(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&,
        std::vector<int>&);
void open_charging_station(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&, int);
void open_charging_stations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&,
        std::vector<int>&);
void close_charging_station(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&, int);

double swap_charging_station(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&, int,
        int);
void perform_station_swap(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);

void optimize_stations_using_utilization(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&,
        ProcessedData&);
void optimize_stations_using_energy(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);
void optimize_stations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);
}

namespace diversification {
double exchange_three_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ThreeExchange&);
void perform_three_exchange(std::vector<Vehicle>&, ThreeExchange&);
void apply_three_exchanges(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);
void optimize_three_exchanges(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);

double shift_multiple_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal, ProcessedData& processed_data, int source_vehicle_index);
void apply_multiple_shifts(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal, ProcessedData& processed_data);
void optimize_multiple_shifts(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal, ProcessedData& processed_data);
}

namespace evaluation {
double calculate_objective(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);
void calculate_utilization(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);

bool is_two_exchange_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
bool is_three_exchange_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int, int, int);
bool is_shift_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);

bool is_charge_adequate_next_trip(std::vector<Trip>&, int, int, bool, bool, int, double&);
bool are_rotations_charge_feasible(std::vector<Trip>&, std::vector<Terminal>&, std::vector<std::vector<int>>);

double calculate_trip_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_end_depot_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_start_depot_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_addition_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_removal_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int);

bool is_savings_maximum(double, double, int, int, int, int, Exchange&);
bool is_savings_maximum(double, double, int, int, int, int, Shift&);
}

#endif //EBUS_VNS_OPERATORS_H
