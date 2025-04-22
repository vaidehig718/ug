#include "csp.h"
#include "operators.h"
#include "logger.h"
#include "vehicle.h"
#include "helpers.h"
#include <string>
#include <vector>
#include <chrono>
#include <omp.h>

#define _CRT_SECURE_NO_WARNINGS
#include <setjmp.h>  

#ifdef _MSC_VER
#define longjmp _longjmp  
#endif

#include <ilconcert/ilosys.h>
#include <ilcplex/ilocplex.h>

#ifdef _MSC_VER
#undef longjmp  // 🔧 Restore longjmp after CPLEX includes
#endif

/* TODOs:
 * Check logging outputs for different levels*/

// Global variables
Logger logger(true);
int NUM_THREADS = omp_get_num_procs();  // Set this to appropriate number of threads

int main(int argc, char* argv[])
{
    // Read the instance as command line argument. If not provided, use the default instance
    ProcessedData processed_data; // Vector of parameters
    processed_data.instance = (argc > 1) ? argv[1] : "Ann_Arbor";

    // Delete any old log files if present and create a new one. Set logging level.
    std::remove(("../output/" + processed_data.instance + "/log.txt").c_str());
    logger.set_file_path("../output/" + processed_data.instance + "/log.txt");
    logger.set_log_level_threshold(LogLevel::Info);

    logger.log(LogLevel::Info, "Starting local search for instance " + processed_data.instance + "...");
    processed_data.start_time_stamp = std::chrono::steady_clock::now();
    postprocessing::write_summary(processed_data.instance, std::time(nullptr));
    postprocessing::create_output_directory(processed_data.instance);

    // Initialize variables
    std::vector<Trip> trip;  // Vector of trips
    std::vector<Terminal> terminal;  // Vector of terminals
    std::vector<Vehicle> vehicle;  // Vector of vehicles

    // Read input processed_data on trips and stops and initialize bus rotations
    preprocessing::initialize_inputs(vehicle, trip, terminal, processed_data);

    // Diversify the solution by optimizing rotations. No changes to charging locations are made here.
    diversification::optimize_multiple_shifts(vehicle, trip, terminal, processed_data);

    // Local search for charging locations which also includes scheduling operators
    locations::optimize_stations(vehicle, trip, terminal, processed_data);

    // Solve the charge scheduling problem
    processed_data.log_csp_solution = true;
    double csp_cost = csp::select_optimization_model(vehicle, trip, terminal, processed_data, "csp-split");

    // Find runtime
    logger.log(LogLevel::Info, "Finishing local search...");
    processed_data.end_time_stamp = std::chrono::steady_clock::now();
    processed_data.runtime = double(std::chrono::duration_cast<std::chrono::milliseconds>(
        processed_data.end_time_stamp - processed_data.start_time_stamp).count()) / 1000.0; // in seconds

    // Log final vehicle rotations
    evaluation::calculate_utilization(vehicle, trip, terminal, processed_data);
    for (auto& curr_vehicle : vehicle)
        curr_vehicle.log_member_data();

    // Postprocessing
    postprocessing::check_solution(vehicle, trip, terminal, processed_data);
    postprocessing::write_summary(vehicle, trip, terminal, processed_data, csp_cost);
    postprocessing::write_vehicle_results(vehicle, processed_data);
    postprocessing::write_terminal_results(terminal, processed_data);
    postprocessing::write_iteration_stats(processed_data);
    logger.log(LogLevel::Info, "Local search completed in " + std::to_string(processed_data.runtime) + " seconds.");
}
