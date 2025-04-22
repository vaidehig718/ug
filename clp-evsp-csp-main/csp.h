#ifndef EBUS_VNS_CSP_H
#define EBUS_VNS_CSP_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include "helpers.h"
#include <sstream>
#include <ilcplex/ilocplex.h>
#include <vector>
#include <map>
#include <set>
#include <deque>
#include <utility>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/bron_kerbosch_all_cliques.hpp>

class IntegratedModelVariable {
public:
    std::vector<std::vector<IloNumVar>> charge_level;
    std::vector<std::vector<std::vector<IloNumVar>>> energy_input;
    std::vector<IloNumVar> terminal_charge_capacity;
    std::vector<IloNumVar> activate_charge_station;

    IntegratedModelVariable(int num_vehicles, int num_terminals)
            :charge_level(num_vehicles),
             energy_input(num_vehicles),
             terminal_charge_capacity(num_terminals),
             activate_charge_station(num_terminals)
    {

    }
};

class SplitModelVariable {
public:
    std::vector<std::vector<IloNumVar>> charge_level;
    std::vector<std::vector<std::vector<IloNumVar>>> energy_input;
    std::vector<IloNumVar> terminal_charge_capacity;

    SplitModelVariable(int num_vehicles, int num_terminals)
            :charge_level(num_vehicles),
             energy_input(num_vehicles),
             terminal_charge_capacity(num_terminals)
    {

    }
};

class UniformModelVariable {
public:
    std::vector<std::vector<IloNumVar>> charge_level;
    std::vector<std::vector<IloNumVar>> energy_input;
    std::vector<IloNumVar> terminal_charge_capacity;

    UniformModelVariable(int num_vehicles, int num_terminals)
            :charge_level(num_vehicles),
             energy_input(num_vehicles),
             terminal_charge_capacity(num_terminals)
    {

    }
};

using Graph = boost::adjacency_matrix<boost::undirectedS>;
using V = Graph::vertex_descriptor;
using Clique = std::deque<V>;
using Cliques = std::vector<Clique>;

namespace initialization {
void update_vehicles(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&,
        const std::string&);
void update_vehicles(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&,
        std::vector<int>&, const std::string&);
void create_subsets(std::vector<Vehicle>&, std::vector<Terminal>&, std::vector<int>&, std::vector<int>&,
        std::vector<int>&);
void create_subsets(std::vector<Vehicle>&, std::vector<Terminal>&, std::vector<int>&, std::vector<int>&);
}

namespace uniform {
void set_variables(IloEnv&, const std::vector<Vehicle>&, UniformModelVariable&, const std::vector<int>&,
        const std::vector<int>&);
void set_constraints(IloEnv&, IloModel&, const std::vector<Vehicle>&, const std::vector<Terminal>&,
        UniformModelVariable&, const std::vector<int>&, const std::vector<int>&, const std::vector<int>&);
void set_objective(IloExpr&, const std::vector<Vehicle>&, UniformModelVariable&, const std::vector<int>&,
        const std::vector<int>&);
void log_solution(IloCplex&, const std::vector<Vehicle>&, const UniformModelVariable&, const std::vector<int>&,
        const std::vector<int>&);
double solve_lp(std::vector<Vehicle>&, std::vector<Terminal>&, ProcessedData&);
}

namespace split {
void set_variables(IloEnv&, const std::vector<Vehicle>&, SplitModelVariable&, const std::vector<int>&,
        const std::vector<int>&);
void set_constraints(IloEnv&, IloModel&, const std::vector<Vehicle>&, const ProcessedData&, SplitModelVariable&,
        const std::vector<int>&, const std::vector<int>&, const std::vector<int>&);
void create_objective(IloExpr&, const std::vector<Vehicle>&, ProcessedData&, SplitModelVariable&,
        const std::vector<int>&, const std::vector<int>&);
void log_solution(IloCplex&, const std::vector<Vehicle>&, SplitModelVariable&, const std::vector<int>&,
        const std::vector<int>&);
double solve_lp(std::vector<Vehicle>&, std::vector<Terminal>&, ProcessedData&);
}

namespace integrated {
void set_variables(IloEnv&, const std::vector<Vehicle>&, IntegratedModelVariable&, const std::vector<int>&,
        const std::vector<int>&);
void set_constraints(IloEnv&, IloModel&, const std::vector<Vehicle>&, const ProcessedData&, IntegratedModelVariable&,
        const std::vector<int>&, const std::vector<int>&, const std::vector<int>&);
void create_objective(IloExpr&, const std::vector<Vehicle>&, ProcessedData&, IntegratedModelVariable&,
        const std::vector<int>&, const std::vector<int>&);
void log_solution(IloCplex&, const std::vector<Vehicle>&, IntegratedModelVariable&, const std::vector<int>&,
        const std::vector<int>&);
double solve_mip(std::vector<Vehicle>&, std::vector<Terminal>&, ProcessedData&);
}

namespace csp {
double select_optimization_model(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&,
        const std::string&);
double select_optimization_model(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&,
        std::vector<int>&, const std::string&);
void log_model_rotations_terminals(IloCplex&, std::vector<Vehicle>&, std::vector<Terminal>&,
        ProcessedData& processed_data);
}

#endif  //EBUS_VNS_CSP_H
