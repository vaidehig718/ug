// CPLEX code for charge scheduling problem
#include "csp.h"

/* INITIALIZATION
 * The following functions are used to update the CSP members of the vehicle class.
 * They also create subsets of vehicles and terminals that are relevant for the CSP model.*/

// This function updates the CSP members of the vehicle class for all vehicles
void initialization::update_vehicles(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, const std::string& type)
{
    //logger.log(LogLevel::Debug, "Updating CSP variables for all vehicles");

    // Clear old CSP variables (if any) and update them
    for (auto& curr_vehicle : vehicle) {
        curr_vehicle.clear_csp_parameters();
        curr_vehicle.populate_csp_parameters(trip, terminal);
    }

    if (type=="csp-uniform") {
        for (auto& curr_vehicle : vehicle) {
            curr_vehicle.clear_price_interval_parameters();
            curr_vehicle.populate_price_interval_parameters();
        }
    }

    // Log the results from the initialization step
    /*logger.log(LogLevel::Info, "CSP variables updated for all vehicles");
    logger.log(LogLevel::Info, "Number of vehicles: "+std::to_string(vehicle.size()));
    logger.log(LogLevel::Info,
            "Number of vehicles requiring charging: "+std::to_string(std::count_if(vehicle.begin(), vehicle.end(),
                    [](Vehicle& bus) { return bus.is_charging_required; })));
    for (const auto& curr_vehicle : vehicle) {
        logger.log(LogLevel::Info, "Vehicle ID: "+std::to_string(curr_vehicle.id));
        logger.log(LogLevel::Info, "Is charging required: "+std::to_string(curr_vehicle.is_charging_required));
        logger.log(LogLevel::Info, "Charge opportunity terminals: "+vector_to_string(curr_vehicle.charge_terminal));
        logger.log(LogLevel::Info,
                "Charge opportunity start times: "+vector_to_string(curr_vehicle.start_charge_time));
        logger.log(LogLevel::Info, "Charge opportunity end times: "+vector_to_string(curr_vehicle.end_charge_time));
        logger.log(LogLevel::Info,
                "Vehicle energy till charge terminals: "+vector_to_string(curr_vehicle.energy_till_charge_terminal));
    }*/
}

// Function to update the CSP members of the vehicle class for a subset of vehicles
void initialization::update_vehicles(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, std::vector<int>& update_vehicle_indices, const std::string& type)
{
    //logger.log(LogLevel::Info, "Updating CSP variables for all vehicles");

    // Clear old CSP variables (if any) and update them
    size_t num_vehicles_updated = update_vehicle_indices.size();
    for (int index = 0; index<num_vehicles_updated; ++index) {
        vehicle[update_vehicle_indices[index]].clear_csp_parameters();
        vehicle[update_vehicle_indices[index]].populate_csp_parameters(trip, terminal);
    }

    // Clear and populate CSP variables associated with price intervals
    if (type=="csp-uniform") {
        for (int index = 0; index<num_vehicles_updated; ++index) {
            vehicle[update_vehicle_indices[index]].clear_price_interval_parameters();
            vehicle[update_vehicle_indices[index]].populate_price_interval_parameters();
        }
    }

    // Log the results from the initialization step
    /*logger.log(LogLevel::Info, "CSP variables updated for all vehicles");
    logger.log(LogLevel::Info, "Number of vehicles: "+std::to_string(vehicle.size()));
    logger.log(LogLevel::Info,
            "Number of vehicles requiring charging: "+std::to_string(std::count_if(vehicle.begin(), vehicle.end(),
                    [](Vehicle& bus) { return bus.is_charging_required; })));
    for (const auto& curr_vehicle : vehicle) {
        logger.log(LogLevel::Info, "Vehicle ID: "+std::to_string(curr_vehicle.id));
        logger.log(LogLevel::Info, "Is charging required: "+std::to_string(curr_vehicle.is_charging_required));
        logger.log(LogLevel::Info, "Charge opportunity terminals: "+vector_to_string(curr_vehicle.charge_terminal));
        logger.log(LogLevel::Info,
                "Charge opportunity start times: "+vector_to_string(curr_vehicle.start_charge_time));
        logger.log(LogLevel::Info, "Charge opportunity end times: "+vector_to_string(curr_vehicle.end_charge_time));
        logger.log(LogLevel::Info,
                "Vehicle energy till charge terminals: "+vector_to_string(curr_vehicle.energy_till_charge_terminal));
    }*/
}

// This function creates subsets of vehicles and terminals for the CSP model and a terminal to index map
void initialization::create_subsets(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal,
        std::vector<int>& vehicles_requiring_charging, std::vector<int>& terminals_with_charging_station,
        std::vector<int>& terminal_to_index)
{
    // logger.log(LogLevel::Debug, "Preprocessing the data for the CSP problem");
    // Determine the indices of vehicles which require charging
    size_t num_vehicles = vehicle.size();
    for (int v = 0; v<num_vehicles; ++v)
        if (vehicle[v].is_charging_required)
            vehicles_requiring_charging.emplace_back(v);

    // Determine the terminal indices with charging stations
    // Initialize a terminal to index vector with -1s and length equal to the number of terminals
    terminal_to_index.resize(terminal.size(), -1);
    int index = 0;
    for (const auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station) {
            terminals_with_charging_station.emplace_back(curr_terminal.id-1);
            terminal_to_index[curr_terminal.id-1] = index;
            ++index;
        }
    }

    // Log the indices of the vehicles that require charging and the terminals that have charging stations
    /*logger.log(LogLevel::Debug, "Vehicles requiring charging: "+vector_to_string(vehicles_requiring_charging));
    logger.log(LogLevel::Debug, "Terminals with charging: "+vector_to_string(terminals_with_charging_station)));*/
}

/* UNIFORM MODEL
 * The uniform CSP problem assumes that the rate of energy transfer in each charging opportunity is constant.
 * Electricity prices can vary over time.
 * Price intervals where the prices are constant are created sub-intervals from the charging opportunities.
 * Bron-Kerbosch algorithm is used to find the time instances where more than one bus is at a charging station */

// This function is used to create three types of variables -- charge levels after different opportunities, energy
// input during different opportunities, and power capacity at charging stations.
void uniform::set_variables(IloEnv& env, const std::vector<Vehicle>& vehicle,
        UniformModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    // Charging level of vehicle with index v and after charging at opportunity k
    // CPLEX index is b since some of the vehicles do not require charging
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];  // Vehicle index
        variable.charge_level[b].resize(vehicle[v].num_charge_opportunities);  // No. of charge opportunities
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {  // Iterate over charging opportunities
            variable.charge_level[b][k] = IloNumVar(env, MIN_CHARGE_LEVEL, MAX_CHARGE_LEVEL, IloNumVar::Float);
            //std::string var_name = "l("+std::to_string(v)+","+std::to_string(k)+")";  // Names are wrt original indices
            //variable.charge_level[b][k].setName(var_name.c_str());
        }
    }

    // Energy input variables for vehicle v during opportunity k
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        variable.energy_input[b].resize(vehicle[v].charge_terminal.size());
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            double charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            variable.energy_input[b][k] = IloNumVar(env, 0.00, MAX_ENERGY_PER_MIN*charge_time_window, IloNumVar::Float);
            //std::string var_name = "w("+std::to_string(v)+","+std::to_string(k)+")";  // Names are wrt original indices
            //variable.energy_input[b][k].setName(var_name.c_str());
        }
    }

    // Power input variables for each charging terminal. Names are wrt original indices
    for (int s = 0; s<num_subset_terminals; ++s) {
        variable.terminal_charge_capacity[s] = IloNumVar(env, 0.00, IloInfinity, IloNumVar::Float);
        //std::string var_name = "z("+std::to_string(terminals_with_charging_station[s])+")";
        //variable.terminal_charge_capacity[s].setName(var_name.c_str());
    }
}

void uniform::set_constraints(IloEnv& env, IloModel& model, const std::vector<Vehicle>& vehicle,
        const std::vector<Terminal>& terminal, UniformModelVariable& variable,
        const std::vector<int>& vehicles_requiring_charging, const std::vector<int>& terminals_with_charging_station,
        const std::vector<int>& terminal_to_index)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    // Constraint 1: Energy balance constraint
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];

        // First opportunity
        //std::string constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(0)+")";
        //model.add(variable.charge_level[b][0]-variable.energy_input[b][0]==
        //        MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]).setName(constraint_name.c_str());

        model.add(variable.charge_level[b][0]-variable.energy_input[b][0]
                ==MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]);

        // Not the first opportunity
        for (int k = 0; k<vehicle[v].num_charge_opportunities-1; ++k) {
            //std::string constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(k+1)+")";
            //model.add(variable.charge_level[b][k+1]-variable.charge_level[b][k]-variable.energy_input[b][k+1]==
            //vehicle[v].energy_till_charge_terminal[k]-vehicle[v].energy_till_charge_terminal[k+1]).setName(
            //        constraint_name.c_str());
            model.add(variable.charge_level[b][k+1]-variable.charge_level[b][k]-variable.energy_input[b][k+1]==
                    vehicle[v].energy_till_charge_terminal[k]-vehicle[v].energy_till_charge_terminal[k+1]);
        }
    }

    // Constraint 2: Minimum charge level constraint
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) { // Includes trip to the depot, hence +1
            //std::string constraint_name = "min_charge_level("+std::to_string(v)+","+std::to_string(k)+")";
            //model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
            //        -vehicle[v].energy_till_charge_terminal[k]).setName(constraint_name.c_str());
            model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
                    -vehicle[v].energy_till_charge_terminal[k]);
        }
    }

    // Constraint 3: Determine the max power required at each charging location
    // Save information on the rotation opportunity pairs for different terminals that have charging stations
    std::vector<std::vector<std::pair<int, int>>> terminal_rotation_opportunity(num_subset_terminals);

    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            int s = terminal_to_index[vehicle[v].charge_terminal[k]-1];  // Terminal index
            terminal_rotation_opportunity[s].emplace_back(b, k);
        }
    }

    // Loop through all terminals with charging stations
    for (int s = 0; s<num_subset_terminals; ++s) {
        // Create a graph where nodes are rotation-opportunity pairs and edges connect them if they overlap
        size_t num_pairs = terminal_rotation_opportunity[s].size();
        Graph g(num_pairs);

        // Loop through all trip rotations and charge opportunities for two nested loops and check if they overlap
        for (int i = 0; i<num_pairs; ++i) {
            int u = vehicles_requiring_charging[terminal_rotation_opportunity[s][i].first];  // This is original vehicle index
            int k = terminal_rotation_opportunity[s][i].second;
            boost::add_edge(i, i, g);
            for (int j = i+1; j<num_pairs; ++j) {
                int v = vehicles_requiring_charging[terminal_rotation_opportunity[s][j].first];
                int l = terminal_rotation_opportunity[s][j].second;
                if (u==v) // Check if v is different from u. If yes, check if the charge opportunities overlap
                    continue;

                // If they overlap add an edge between the (rotation, charge opportunity) pair in graph g
                if (vehicle[u].start_charge_time[k]<vehicle[v].end_charge_time[l]
                        && vehicle[v].start_charge_time[l]<vehicle[u].end_charge_time[k])
                    boost::add_edge(i, j, g);
            }
        }

        std::vector<Clique> cliques;
        struct {
          Cliques& target;
          void clique(std::deque<V> clique, Graph const&) const { target.push_back(std::move(clique)); }
        } collect{cliques};
        bron_kerbosch_all_cliques(g, collect, 1);

        // Iterate across the cliques and populate max power constraints
        size_t num_cliques = cliques.size();
        for (int c = 0; c<num_cliques; ++c) {
            IloExpr max_power(env);
            // Select the rotation and charge opportunity pair from the clique elements
            for (auto pair_index : cliques[c]) {
                int b = terminal_rotation_opportunity[s][pair_index].first;
                int k = terminal_rotation_opportunity[s][pair_index].second;
                int v = vehicles_requiring_charging[b];

                // Populate the max power constraint. Check if denominator is zero, if so throw an error and exit
                double denominator = double(vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k])/60.0;
                if (denominator<0.0) {
                    std::cerr << "Error: Check charge time windows of vehicle index " << v << std::endl;
                    exit(1);
                }
                max_power += variable.energy_input[b][k]/denominator;
            }

            // Add the max power constraint to the model
            //std::string constraint_name = "max_power("+std::to_string(terminals_with_charging_station[s])+","+std::to_string(c)+")";
            //model.add(max_power<=variable.terminal_charge_capacity[s]).setName(constraint_name.c_str());
            model.add(max_power<=variable.terminal_charge_capacity[s]);
            max_power.end();
        }
    }
}

// This function creates the objective function for the CSP model
void uniform::set_objective(IloExpr& objective, const std::vector<Vehicle>& vehicle,
        UniformModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];

        // Add objective expressions for dynamic energy prices
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            // Charging opportunity k lies in a single price period
            if (vehicle[v].price_intervals[k].period_index.size()==1)
                objective += variable.energy_input[b][k]*ENERGY_PRICE[vehicle[v].price_intervals[k].period_index[0]];
            else {  // Charging opportunity k lies in multiple price periods
                // Find the fraction of time the vehicle is charging in each price period
                double denominator_inverse =
                        1.0/double(vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k]);
                for (int p = 0; p<vehicle[v].price_intervals[k].period_index.size(); ++p) {
                    double time_fraction =
                            double(vehicle[v].price_intervals[k].within_period_duration[p])*denominator_inverse;
                    objective += variable.energy_input[b][k]*(time_fraction)
                            *(ENERGY_PRICE[vehicle[v].price_intervals[k].period_index[p]]);
                }
            }
        }
    }

    // Add objective expressions for static capacity costs
    for (int s = 0; s<num_subset_terminals; ++s)
        objective += variable.terminal_charge_capacity[s]*POWER_CAPACITY_PRICE;
}

void uniform::log_solution(IloCplex& cplex, const std::vector<Vehicle>& vehicle,
        const UniformModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    // Log the charging level solution
    logger.log(LogLevel::Debug, "Charging level solution");
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            logger.log(LogLevel::Debug, "Charge level for vehicle "+std::to_string(v+1)+" at opportunity "
                    +std::to_string(k)+" = "+std::to_string(cplex.getValue(variable.charge_level[b][k])));
        }
    }

    // Log the energy input solution
    logger.log(LogLevel::Debug, "Energy level solution");
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            logger.log(LogLevel::Debug, "Energy input for vehicle "+std::to_string(v+1)+" at opportunity "
                    +std::to_string(k)+" = "+std::to_string(cplex.getValue(variable.energy_input[b][k])));
        }
    }

    // Log the power limit solutions
    logger.log(LogLevel::Debug, "Power limit solution");
    for (int s = 0; s<num_subset_terminals; ++s) {
        logger.log(LogLevel::Debug, "Power limit for terminal "+std::to_string(terminals_with_charging_station[s])
                +" = "+std::to_string(cplex.getValue(variable.terminal_charge_capacity[s])));
    }
}

double uniform::solve_lp(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal, ProcessedData& processed_data)
{
    logger.log(LogLevel::Debug, "Solving the uniform charging CSP problem");
    IloEnv env; // Create the CPLEX environment
    double optimal_objective;

    try {
        // Create a CPLEX model
        IloModel model(env);
        IloCplex cplex(model);

        // Turn off CPLEX output
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());

        std::vector<int> vehicles_requiring_charging;
        std::vector<int> terminals_with_charging_station;
        std::vector<int> terminal_to_index;
        initialization::create_subsets(vehicle, terminal, vehicles_requiring_charging, terminals_with_charging_station,
                terminal_to_index);

        // Create decision variables
        logger.log(LogLevel::Debug, "Creating decision variables");
        UniformModelVariable variable(vehicles_requiring_charging.size(), terminals_with_charging_station.size());
        uniform::set_variables(env, vehicle, variable, vehicles_requiring_charging,
                terminals_with_charging_station);

        // Add constraints
        logger.log(LogLevel::Debug, "Adding constraints");
        uniform::set_constraints(env, model, vehicle, terminal, variable, vehicles_requiring_charging,
                terminals_with_charging_station, terminal_to_index);

        // Add objective
        logger.log(LogLevel::Debug, "Adding objective function");
        IloExpr objective(env);
        uniform::set_objective(objective, vehicle, variable, vehicles_requiring_charging,
                terminals_with_charging_station);

        // Solve the linear program
        IloObjective obj = IloMinimize(env, objective);
        model.add(obj);
        objective.end();

        logger.log(LogLevel::Debug, "Solving the LP");
        if (!cplex.solve()) {
            csp::log_model_rotations_terminals(cplex, vehicle, terminal, processed_data);
            exit(1); // Terminate with error
        }

        if (processed_data.log_csp_solution) {  // Write the LP to a .lp file and save the results in a .sol file
            cplex.exportModel(("../output/"+processed_data.instance+"/csp_uniform.lp").c_str());
            cplex.writeSolution(("../output/"+processed_data.instance+"/csp_uniform.sol").c_str());
        }

        //Display results and log the solution
        optimal_objective = cplex.getObjValue();
        //logger.log(LogLevel::Debug, "Solution status = "+std::to_string(cplex.getStatus()));
        //logger.log(LogLevel::Debug, "Solution value = "+std::to_string(cplex.getObjValue()));
        //uniform::log_solution(cplex, vehicle, variable, vehicles_requiring_charging, terminals_with_charging_station);
    }
        // Catch exceptions thrown by CPLEX
    catch (IloException& exception) {
        std::cerr << "Error: " << exception << std::endl;
    }
    env.end();  // Clean up
    return optimal_objective;
}

/* SPLIT MODEL
 * In this model, charging decisions can be made at one-minute intervals.
 * The charging levels are still tracked at the end of each charging opportunity.
 * Terminal charge capacity decisions are evaluated at each time step. */

// Function that creates the variables required for the split model. They include charge levels at the end of each
// opportunity, energy input at each time step, and terminal charge capacity
void split::set_variables(IloEnv& env, const std::vector<Vehicle>& vehicle, SplitModelVariable& variable,
        const std::vector<int>& vehicles_requiring_charging, const std::vector<int>& terminals_with_charging_station)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    // Charging level of vehicle with index v and after charging at opportunity k
    // CPLEX index is b since some of the vehicles do not require charging
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];  // Vehicle index
        variable.charge_level[b].resize(vehicle[v].num_charge_opportunities);  // No. of charge opportunities
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {  // Iterate over charging opportunities
            variable.charge_level[b][k] = IloNumVar(env, MIN_CHARGE_LEVEL, MAX_CHARGE_LEVEL, IloNumVar::Float);
            //std::string var_name = "l("+std::to_string(v)+","+std::to_string(k)+")";  // Names are wrt original indices
            //variable.charge_level[b][k].setName(var_name.c_str());
        }
    }

    // Energy input variables for vehicle v during opportunity k
    int charge_time_window;
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        variable.energy_input[b].resize(vehicle[v].num_charge_opportunities);  // No. of charge opportunities
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {  // Iterate over charging opportunities
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            variable.energy_input[b][k].resize(charge_time_window);
            for (int t = 0; t<charge_time_window; ++t) {
                variable.energy_input[b][k][t] = IloNumVar(env, 0.00, MAX_ENERGY_PER_MIN, IloNumVar::Float);
                //std::string var_name = "w("+std::to_string(v)+","+std::to_string(vehicle[v].charge_terminal[k]-1)+","
                //        +std::to_string(vehicle[v].start_charge_time[k]+t)+")";
                //variable.energy_input[b][k][t].setName(var_name.c_str());
            }
        }
    }

    // Power input variables for each charging terminal. Names are wrt original indices
    for (int s = 0; s<num_subset_terminals; ++s) {
        variable.terminal_charge_capacity[s] = IloNumVar(env, 0.00, IloInfinity, IloNumVar::Float);
        //std::string var_name = "z("+std::to_string(terminals_with_charging_station[s])+")";
        //variable.terminal_charge_capacity[s].setName(var_name.c_str());
    }
}

// Function that creates the constraints required for the split model. They include energy balance, minimum charge level,
// and max power constraints
void split::set_constraints(IloEnv& env, IloModel& model, const std::vector<Vehicle>& vehicle,
        const ProcessedData& processed_data, SplitModelVariable& variable,
        const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station, const std::vector<int>& terminal_to_index)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    // Constraint 1: Energy balance constraint
    int charge_time_window;
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];

        // First opportunity -- run a loop from start charge time to end charge time at this opportunity
        IloExpr lhs(env);
        lhs += variable.charge_level[b][0];
        charge_time_window = vehicle[v].end_charge_time[0]-vehicle[v].start_charge_time[0];
        for (int t = 0; t<charge_time_window; ++t)
            lhs -= variable.energy_input[b][0][t];

        //constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(0)+")";
        //model.add(lhs==MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]).setName(
        //        constraint_name.c_str());
        model.add(lhs==MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]);
        lhs.end();

        // Not the first opportunity -- run a loop from start charge time to end charge time at this opportunity
        for (int k = 0; k<vehicle[v].num_charge_opportunities-1; ++k) {
            int s = terminal_to_index[vehicle[v].charge_terminal[k]-1];  // Terminal index in the subset of indices
            IloExpr lhs(env);
            lhs += variable.charge_level[b][k+1];
            lhs -= variable.charge_level[b][k];
            charge_time_window = vehicle[v].end_charge_time[k+1]-vehicle[v].start_charge_time[k+1];
            for (int t = 0; t<charge_time_window; ++t)
                lhs -= variable.energy_input[b][k+1][t];

            //constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(k+1)+")";
            //model.add(lhs==vehicle[v].energy_till_charge_terminal[k]-
            //        vehicle[v].energy_till_charge_terminal[k+1]).setName(constraint_name.c_str());
            model.add(lhs==vehicle[v].energy_till_charge_terminal[k]-vehicle[v].energy_till_charge_terminal[k+1]);
            lhs.end();
        }
    }

    // Constraint 2: Minimum charge level constraint
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) { // Includes trip to the depot, hence +1
            //constraint_name = "min_charge_level("+std::to_string(v)+","+std::to_string(k)+")";
            //model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
            //        -vehicle[v].energy_till_charge_terminal[k]).setName(constraint_name.c_str());
            model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
                    -vehicle[v].energy_till_charge_terminal[k]);
        }
    }

    // Constraint 3: Determine the max power required at each charging location
    IloArray<IloExprArray> total_energy_input(env, terminals_with_charging_station.size());
    for (int s = 0; s<num_subset_terminals; ++s) {
        total_energy_input[s] = IloExprArray(env, processed_data.time_steps_length);
        for (int t = 0; t<processed_data.time_steps_length; ++t)
            total_energy_input[s][t] = IloExpr(env);
    }

    std::vector<std::vector<bool>> include_lhs(vehicles_requiring_charging.size(),
            std::vector<bool>(processed_data.time_steps_length, false));

    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            int s = terminal_to_index[vehicle[v].charge_terminal[k]-1];  // Terminal index in the subset of indices
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            for (int t = 0; t<charge_time_window; ++t) {
                total_energy_input[s][vehicle[v].start_charge_time[k]+t
                        -processed_data.first_trip_start_time] += variable.energy_input[b][k][t];
                include_lhs[s][vehicle[v].start_charge_time[k]+t-processed_data.first_trip_start_time] = true;
            }
        }
    }

    for (int s = 0; s<num_subset_terminals; ++s) {
        for (int t = 0; t<processed_data.time_steps_length; ++t) {
            if (include_lhs[s][t]) {
                //constraint_name =
                //        "max_power("+std::to_string(terminals_with_charging_station[s])+","
                //                +std::to_string(t+processed_data.first_trip_start_time)+")";
                //model.add(total_energy_input[s][t]<=variable.terminal_charge_capacity[s]/60.0).setName(
                //        constraint_name.c_str());
                model.add(total_energy_input[s][t]<=variable.terminal_charge_capacity[s]/60.0);
            }
        }
    }

    // Clear the expression variables
    for (int s = 0; s<num_subset_terminals; ++s) {
        total_energy_input[s].endElements();  // End individual expressions in each row
        total_energy_input[s].end();  // End the row
    }
}

// Function that creates the objective function for the split model. It includes energy input for each rotation and
// fixed costs due to terminal charge capacity
void split::create_objective(IloExpr& objective, const std::vector<Vehicle>& vehicle, ProcessedData& processed_data,
        SplitModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            int charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            for (int t = 0; t<charge_time_window; ++t)
                objective +=
                        variable.energy_input[b][k][t]
                                *processed_data.energy_price_per_min[vehicle[v].start_charge_time[k]+t];
        }
    }

    // Add objective expressions for static capacity costs
    for (int s = 0; s<num_subset_terminals; ++s)
        objective += variable.terminal_charge_capacity[s]*POWER_CAPACITY_PRICE;
}

// Function that logs the solution of the split model for debugging
void split::log_solution(IloCplex& cplex, const std::vector<Vehicle>& vehicle,
        SplitModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    // Log the charging level solution
    logger.log(LogLevel::Debug, "Charging level solution");
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            logger.log(LogLevel::Debug, "Charge level for vehicle index "+std::to_string(v)+" at opportunity "
                    +std::to_string(k)+" = "+std::to_string(cplex.getValue(variable.charge_level[b][k])));
        }
    }

    // Log the energy input solution
    int charge_time_window;
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {  // Iterate over charging opportunities
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            for (int t = 0; t<charge_time_window; ++t) {
                logger.log(LogLevel::Debug, "Energy input for vehicle index "+std::to_string(v)+" at terminal index "
                        +std::to_string(vehicle[v].charge_terminal[k]-1)+" at time "
                        +std::to_string(vehicle[v].start_charge_time[k]+t)+" = "
                        +std::to_string(cplex.getValue(variable.energy_input[b][k][t])));
            }
        }
    }

    // Log the power limit solutions
    logger.log(LogLevel::Debug, "Power limit solution");
    for (int s = 0; s<num_subset_terminals; ++s) {
        logger.log(LogLevel::Debug, "Power limit for terminal index"+std::to_string(terminals_with_charging_station[s])
                +" = "+std::to_string(cplex.getValue(variable.terminal_charge_capacity[s])));
    }
}

// Function that solves the split model
double split::solve_lp(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal, ProcessedData& processed_data)
{
    logger.log(LogLevel::Debug, "Solving the split charging CSP problem");
    IloEnv env; // Create the CPLEX environment
    double optimal_objective;

    try {
        // Create a CPLEX model
        IloModel model(env);
        IloCplex cplex(model);

        // Turn off CPLEX output
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());

        std::vector<int> vehicles_requiring_charging;
        std::vector<int> terminals_with_charging_station;
        std::vector<int> terminal_to_index;
        initialization::create_subsets(vehicle, terminal, vehicles_requiring_charging, terminals_with_charging_station,
                terminal_to_index);

        // Create decision variables
        logger.log(LogLevel::Debug, "Creating decision variables");
        SplitModelVariable variable(vehicles_requiring_charging.size(), terminals_with_charging_station.size());
        split::set_variables(env, vehicle, variable, vehicles_requiring_charging, terminals_with_charging_station);

        // Add constraints
        logger.log(LogLevel::Debug, "Adding constraints");
        split::set_constraints(env, model, vehicle, processed_data, variable, vehicles_requiring_charging,
                terminals_with_charging_station, terminal_to_index);

        // Add objective
        logger.log(LogLevel::Debug, "Adding objective function");
        IloExpr objective(env);
        split::create_objective(objective, vehicle, processed_data, variable, vehicles_requiring_charging,
                terminals_with_charging_station);

        // Solve the linear program
        IloObjective obj = IloMinimize(env, objective);
        model.add(obj);
        objective.end();

        logger.log(LogLevel::Debug, "Solving the LP");
        if (!cplex.solve()) {
            csp::log_model_rotations_terminals(cplex, vehicle, terminal, processed_data);
            exit(1); // Terminate with error
        }

        if (processed_data.log_csp_solution) {  // Write the LP to a .lp file and save the results in a .sol file
            cplex.exportModel(("../output/"+processed_data.instance+"/csp_split.lp").c_str());
            cplex.writeSolution(("../output/"+processed_data.instance+"/csp_split.sol").c_str());
        }

        // Query and print the values of the variables
        for (int s = 0; s<terminals_with_charging_station.size(); ++s) {
            terminal[terminals_with_charging_station[s]].charge_capacity = cplex.getValue(
                    variable.terminal_charge_capacity[s]);
            logger.log(LogLevel::Debug,
                    "Terminal index "+std::to_string(terminals_with_charging_station[s])+" charge capacity = "
                            +std::to_string(terminal[terminals_with_charging_station[s]].charge_capacity));
        }

        //Display results and log the solution
        optimal_objective = cplex.getObjValue();
        /*logger.log(LogLevel::Debug, "Solution status = "+std::to_string(cplex.getStatus()));
        logger.log(LogLevel::Debug, "Solution value = "+std::to_string(cplex.getObjValue()));*/
        //split::log_solution(cplex, vehicle, variable, vehicles_requiring_charging,
        //        terminals_with_charging_station);
    }
        // Catch exceptions thrown by CPLEX
    catch (IloException& exception) {
        std::cerr << "Error: " << exception << std::endl;
    }
    env.end();  // Clean up
    return optimal_objective;
}

/* SPLIT MODEL WITH INTEGRATED LOCATION DECISIONS
 * This model is same as the split model for the CSP part.
 * It however contains additional charging location decisions for all the charging stations that are open.
 * Extra constraints relate charge capacities with 0/1 decisions on opening/closing candidate charging stations. */

// Function that creates the variables required for the split model. They include charge levels at the end of each
// opportunity, energy input at each time step, terminal charge capacity, and charge station activation
void integrated::set_variables(IloEnv& env, const std::vector<Vehicle>& vehicle, IntegratedModelVariable& variable,
        const std::vector<int>& vehicles_requiring_charging, const std::vector<int>& terminals_with_charging_station)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    // Charging level of vehicle with index v and after charging at opportunity k
    // CPLEX index is b since some of the vehicles do not require charging
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];  // Vehicle index
        variable.charge_level[b].resize(vehicle[v].num_charge_opportunities);  // No. of charge opportunities
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {  // Iterate over charging opportunities
            variable.charge_level[b][k] = IloNumVar(env, MIN_CHARGE_LEVEL, MAX_CHARGE_LEVEL, IloNumVar::Float);
            //std::string var_name = "l("+std::to_string(v)+","+std::to_string(k)+")";  // Names are wrt original indices
            //variable.charge_level[b][k].setName(var_name.c_str());
        }
    }

    // Energy input variables for vehicle v during opportunity k
    int charge_time_window;
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        variable.energy_input[b].resize(vehicle[v].num_charge_opportunities);  // No. of charge opportunities
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {  // Iterate over charging opportunities
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            variable.energy_input[b][k].resize(charge_time_window);
            for (int t = 0; t<charge_time_window; ++t) {
                variable.energy_input[b][k][t] = IloNumVar(env, 0.00, MAX_ENERGY_PER_MIN, IloNumVar::Float);
                //std::string var_name = "w("+std::to_string(v)+","+std::to_string(vehicle[v].charge_terminal[k]-1)+","
                //        +std::to_string(vehicle[v].start_charge_time[k]+t)+")";
                //variable.energy_input[b][k][t].setName(var_name.c_str());
            }
        }
    }

    // Power input variables for each charging terminal. Names are wrt original indices
    for (int s = 0; s<num_subset_terminals; ++s) {
        variable.terminal_charge_capacity[s] = IloNumVar(env, 0.00, IloInfinity, IloNumVar::Float);
        //std::string var_name = "z("+std::to_string(terminals_with_charging_station[s])+")";
        //variable.terminal_charge_capacity[s].setName(var_name.c_str());
    }

    // Binary variables for opening/closing charging stations
    for (int s = 0; s<num_subset_terminals; ++s) {
        variable.activate_charge_station[s] = IloNumVar(env, 0, 1, IloNumVar::Int);
        //std::string var_name = "y("+std::to_string(terminals_with_charging_station[s])+")";
        //variable.activate_charge_station[s].setName(var_name.c_str());
    }
}

// Function that creates the constraints required for the split model. They include energy balance, minimum charge level,
// max power constraints, and constraints for activating charging stations
void integrated::set_constraints(IloEnv& env, IloModel& model, const std::vector<Vehicle>& vehicle,
        const ProcessedData& processed_data, IntegratedModelVariable& variable,
        const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station, const std::vector<int>& terminal_to_index)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    // Constraint 1: Energy balance constraint
    int charge_time_window;
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];

        // First opportunity -- run a loop from start charge time to end charge time at this opportunity
        IloExpr lhs(env);
        lhs += variable.charge_level[b][0];
        charge_time_window = vehicle[v].end_charge_time[0]-vehicle[v].start_charge_time[0];
        for (int t = 0; t<charge_time_window; ++t)
            lhs -= variable.energy_input[b][0][t];

        //constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(0)+")";
        //model.add(lhs==MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]).setName(
        //        constraint_name.c_str());
        model.add(lhs==MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]);
        lhs.end();

        // Not the first opportunity -- run a loop from start charge time to end charge time at this opportunity
        for (int k = 0; k<vehicle[v].num_charge_opportunities-1; ++k) {
            int s = terminal_to_index[vehicle[v].charge_terminal[k]-1];  // Terminal index in the subset of indices
            IloExpr lhs(env);
            lhs += variable.charge_level[b][k+1];
            lhs -= variable.charge_level[b][k];
            charge_time_window = vehicle[v].end_charge_time[k+1]-vehicle[v].start_charge_time[k+1];
            for (int t = 0; t<charge_time_window; ++t)
                lhs -= variable.energy_input[b][k+1][t];

            //constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(k+1)+")";
            //model.add(lhs==vehicle[v].energy_till_charge_terminal[k]-
            //        vehicle[v].energy_till_charge_terminal[k+1]).setName(constraint_name.c_str());
            model.add(lhs==vehicle[v].energy_till_charge_terminal[k]-vehicle[v].energy_till_charge_terminal[k+1]);
            lhs.end();
        }
    }

    // Constraint 2: Minimum charge level constraint
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) { // Includes trip to the depot, hence +1
            //constraint_name = "min_charge_level("+std::to_string(v)+","+std::to_string(k)+")";
            //model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
            //        -vehicle[v].energy_till_charge_terminal[k]).setName(constraint_name.c_str());
            model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
                    -vehicle[v].energy_till_charge_terminal[k]);
        }
    }

    // Constraint 3: Determine the max power required at each charging location
    IloArray<IloExprArray> total_energy_input(env, terminals_with_charging_station.size());
    for (int s = 0; s<num_subset_terminals; ++s) {
        total_energy_input[s] = IloExprArray(env, processed_data.time_steps_length);
        for (int t = 0; t<processed_data.time_steps_length; ++t)
            total_energy_input[s][t] = IloExpr(env);
    }

    std::vector<std::vector<bool>> include_lhs(vehicles_requiring_charging.size(),
            std::vector<bool>(processed_data.time_steps_length, false));

    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            int s = terminal_to_index[vehicle[v].charge_terminal[k]-1];  // Terminal index in the subset of indices
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            for (int t = 0; t<charge_time_window; ++t) {
                total_energy_input[s][vehicle[v].start_charge_time[k]+t
                        -processed_data.first_trip_start_time] += variable.energy_input[b][k][t];
                include_lhs[s][vehicle[v].start_charge_time[k]+t-processed_data.first_trip_start_time] = true;
            }
        }
    }

    for (int s = 0; s<num_subset_terminals; ++s) {
        for (int t = 0; t<processed_data.time_steps_length; ++t) {
            if (include_lhs[s][t]) {
                //constraint_name =
                //        "max_power("+std::to_string(terminals_with_charging_station[s])+","
                //                +std::to_string(t+processed_data.first_trip_start_time)+")";
                //model.add(total_energy_input[s][t]<=variable.terminal_charge_capacity[s]/60.0).setName(
                //        constraint_name.c_str());
                model.add(total_energy_input[s][t]<=variable.terminal_charge_capacity[s]/60.0);
            }
        }
    }

    // Clear the expression variables
    for (int s = 0; s<num_subset_terminals; ++s) {
        total_energy_input[s].endElements();  // End individual expressions in each row
        total_energy_input[s].end();  // End the row
    }

    // Constraint 4: Set the charge capacity of terminals that are not open to 0
    for (int s = 0; s<num_subset_terminals; ++s) {
        IloExpr lhs(env);
        lhs += variable.terminal_charge_capacity[s];
        lhs -= INF*variable.activate_charge_station[s];  // TODO: Upper bound can be tightened
        //constraint_name = "activate_station("+std::to_string(terminals_with_charging_station[s])+")";
        //model.add(lhs==0).setName(constraint_name.c_str());
        model.add(lhs<=0);
        lhs.end();
    }
}

// Function that creates the objective function for the split model. It includes energy input for each rotation,
// fixed costs due to terminal charge capacity, and fixed costs due to activating charging stations
void integrated::create_objective(IloExpr& objective, const std::vector<Vehicle>& vehicle,
        ProcessedData& processed_data, IntegratedModelVariable& variable,
        const std::vector<int>& vehicles_requiring_charging, const std::vector<int>& terminals_with_charging_station)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            int charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            for (int t = 0; t<charge_time_window; ++t)
                objective +=
                        variable.energy_input[b][k][t]
                                *processed_data.energy_price_per_min[vehicle[v].start_charge_time[k]+t];
        }
    }

    // Add objective expressions for static capacity costs
    for (int s = 0; s<num_subset_terminals; ++s)
        objective += variable.terminal_charge_capacity[s]*POWER_CAPACITY_PRICE;

    // Add objective expressions for activating charging stations
    for (int s = 0; s<num_subset_terminals; ++s)
        objective += variable.activate_charge_station[s]*CHARGE_LOC_COST;
}

// Function that logs the solution of the split model for debugging
void integrated::log_solution(IloCplex& cplex, const std::vector<Vehicle>& vehicle,
        IntegratedModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    size_t num_subset_vehicles = vehicles_requiring_charging.size();
    size_t num_subset_terminals = terminals_with_charging_station.size();

    // Log the charging level solution
    logger.log(LogLevel::Debug, "Charging level solution");
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {
            logger.log(LogLevel::Debug, "Charge level for vehicle index "+std::to_string(v)+" at opportunity "
                    +std::to_string(k)+" = "+std::to_string(cplex.getValue(variable.charge_level[b][k])));
        }
    }

    // Log the energy input solution
    int charge_time_window;
    for (int b = 0; b<num_subset_vehicles; ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].num_charge_opportunities; ++k) {  // Iterate over charging opportunities
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            for (int t = 0; t<charge_time_window; ++t) {
                logger.log(LogLevel::Debug, "Energy input for vehicle index "+std::to_string(v)+" at terminal index "
                        +std::to_string(vehicle[v].charge_terminal[k]-1)+" at time "
                        +std::to_string(vehicle[v].start_charge_time[k]+t)+" = "
                        +std::to_string(cplex.getValue(variable.energy_input[b][k][t])));
            }
        }
    }

    // Log the power limit solutions
    logger.log(LogLevel::Debug, "Power limit solution");
    for (int s = 0; s<num_subset_terminals; ++s) {
        logger.log(LogLevel::Debug, "Power limit for terminal index"+std::to_string(terminals_with_charging_station[s])
                +" = "+std::to_string(cplex.getValue(variable.terminal_charge_capacity[s])));
    }

    // Log the charge station activation solutions
    logger.log(LogLevel::Debug, "Charge station activation solution");
    for (int s = 0; s<num_subset_terminals; ++s) {
        logger.log(LogLevel::Debug, "Charge station activation for terminal index"
                +std::to_string(terminals_with_charging_station[s])+" = "
                +std::to_string(cplex.getValue(variable.activate_charge_station[s])));
    }
}

// Function that solves the split model
double integrated::solve_mip(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal,
        ProcessedData& processed_data)
{
    logger.log(LogLevel::Debug, "Solving the split charging CSP problem");
    IloEnv env; // Create the CPLEX environment
    double optimal_objective;

    try {
        // Create a CPLEX model
        IloModel model(env);
        IloCplex cplex(model);

        // Turn off CPLEX output
        // cplex.setOut(env.getNullStream());
        // cplex.setWarning(env.getNullStream());

        std::vector<int> vehicles_requiring_charging;
        std::vector<int> terminals_with_charging_station;
        std::vector<int> terminal_to_index;
        initialization::create_subsets(vehicle, terminal, vehicles_requiring_charging, terminals_with_charging_station,
                terminal_to_index);

        // Create decision variables
        logger.log(LogLevel::Debug, "Creating decision variables");
        IntegratedModelVariable variable(vehicles_requiring_charging.size(), terminals_with_charging_station.size());
        integrated::set_variables(env, vehicle, variable, vehicles_requiring_charging, terminals_with_charging_station);

        // Add constraints
        logger.log(LogLevel::Debug, "Adding constraints");
        integrated::set_constraints(env, model, vehicle, processed_data, variable, vehicles_requiring_charging,
                terminals_with_charging_station, terminal_to_index);

        // Add objective
        logger.log(LogLevel::Debug, "Adding objective function");
        IloExpr objective(env);
        integrated::create_objective(objective, vehicle, processed_data, variable, vehicles_requiring_charging,
                terminals_with_charging_station);

        // Solve the linear program
        IloObjective obj = IloMinimize(env, objective);
        model.add(obj);
        objective.end();

        logger.log(LogLevel::Debug, "Solving the MIP");
        if (!cplex.solve()) {
            csp::log_model_rotations_terminals(cplex, vehicle, terminal, processed_data);
            exit(1); // Terminate with error
        }

        if (processed_data.log_csp_solution) {  // Write the LP to a .lp file and save the results in a .sol file
            cplex.exportModel(("../output/"+processed_data.instance+"/clp_csp_mip.lp").c_str());
            cplex.writeSolution(("../output/"+processed_data.instance+"/clp_csp_mip.sol").c_str());
        }

        // Query and print the values of the variables
        for (int s = 0; s<terminals_with_charging_station.size(); ++s) {
            terminal[terminals_with_charging_station[s]].charge_capacity = cplex.getValue(
                    variable.terminal_charge_capacity[s]);
            logger.log(LogLevel::Info,
                    "Terminal ID "+std::to_string(terminals_with_charging_station[s]+1)+" charge capacity = "
                            +std::to_string(terminal[terminals_with_charging_station[s]].charge_capacity));
        }

        // Using the charge station activation variables, update the is charge station variables of terminals
        logger.log(LogLevel::Info, "Closing charge stations that are not in use...");
        int num_charge_stations_closed = 0;
        for (int s = 0; s<terminals_with_charging_station.size(); ++s) {
            terminal[terminals_with_charging_station[s]].is_charge_station = static_cast<bool>(std::round(
                    cplex.getValue(variable.activate_charge_station[s])));
            if (!terminal[terminals_with_charging_station[s]].is_charge_station)
                ++num_charge_stations_closed;
        }
        logger.log(LogLevel::Info, "Number of charge stations closed = "+std::to_string(num_charge_stations_closed));


        //Display results and log the solution
        optimal_objective = cplex.getObjValue();
        /*logger.log(LogLevel::Debug, "Solution status = "+std::to_string(cplex.getStatus()));
        logger.log(LogLevel::Debug, "Solution value = "+std::to_string(cplex.getObjValue()));*/
        //split::log_solution(cplex, vehicle, variable, vehicles_requiring_charging,
        //        terminals_with_charging_station);
    }
        // Catch exceptions thrown by CPLEX
    catch (IloException& exception) {
        std::cerr << "Error: " << exception << std::endl;
    }
    env.end();  // Clean up
    return optimal_objective;
}

/* OPTIMIZATION SELECTOR FUNCTIONS
 * These functions select the CSP model based on user parameters.
 * Split model takes more time since it has more variables and constraints.
 * Hence, it is better used sparingly or for smaller instances */

// Function that updates all vehicle rotations with CSP processed_data and selects the CSP model based on user parameters
double csp::select_optimization_model(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, ProcessedData& processed_data, const std::string& type)
{
    initialization::update_vehicles(vehicle, trip, terminal, type);

    if (type=="csp-uniform")
        return uniform::solve_lp(vehicle, terminal, processed_data);
    else if (type=="csp-split")
        return split::solve_lp(vehicle, terminal, processed_data);
    else if (type=="clp-csp-split")
        return integrated::solve_mip(vehicle, terminal, processed_data);
    else {
        std::cerr << "Error: Invalid charging model" << std::endl;
        exit(1);
    }
}

// Function that updates a subset of vehicle rotations with CSP processed_data and selects the CSP model based on user parameters
double csp::select_optimization_model(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, ProcessedData& processed_data, std::vector<int>& update_vehicle_indices,
        const std::string& type)
{
    initialization::update_vehicles(vehicle, trip, terminal, type);

    if (type=="csp-uniform")
        return uniform::solve_lp(vehicle, terminal, processed_data);
    else if (type=="csp-split")
        return split::solve_lp(vehicle, terminal, processed_data);
    else if (type=="clp-csp-split")
        return integrated::solve_mip(vehicle, terminal, processed_data);
    else {
        std::cerr << "Error: Invalid charging model" << std::endl;
        exit(1);
    }
}

// Logs the solution in case CPLEX does not find an optimal solution
void csp::log_model_rotations_terminals(IloCplex& cplex, std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal,
        ProcessedData& processed_data)
{
    cplex.exportModel(("../output/"+processed_data.instance+"/csp_error.lp").c_str());  // Write the LP to a .lp file
    logger.log(LogLevel::Error, "Failed to optimize LP");

    // Log terminal data
    logger.log(LogLevel::Info, "Terminal data and utilization");
    for (const auto& curr_terminal : terminal)
        curr_terminal.log_member_data();

    // Log the vehicle rotations
    logger.log(LogLevel::Info, "Vehicle rotations");
    for (const auto& curr_vehicle : vehicle)
        curr_vehicle.log_member_data();
    logger.log(LogLevel::Info, "Vehicle CSP member data");
    for (const auto& curr_vehicle : vehicle)
        curr_vehicle.log_csp_member_data();
}

