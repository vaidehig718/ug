cmake_minimum_required(VERSION 3.10)
project(EbusVNS VERSION 1.0)

# ==== Boost ====
set(BOOST_ROOT "C:/local/boost_1_84_0/boost_1_88_0")
include_directories("${BOOST_ROOT}")
link_directories("${BOOST_ROOT}/stage/lib")
find_package(Boost REQUIRED COMPONENTS graph)

# ==== CPLEX ====
set(CPLEX_ROOT "C:/Program Files/IBM/ILOG/CPLEX_Studio2212")
include_directories("${CPLEX_ROOT}/cplex/include")
include_directories("${CPLEX_ROOT}/concert/include")
link_directories("${CPLEX_ROOT}/cplex/lib/x64_windows_vs2017/stat_mda")
link_directories("${CPLEX_ROOT}/concert/lib/x64_windows_vs2017/stat_mda")

# ==== Executable ====
add_executable(ebus_vns main.cpp)

# ✅ Define IL_STD directly for this target
target_compile_definitions(ebus_vns PRIVATE IL_STD)

# ==== Linking ====
target_link_libraries(ebus_vns PRIVATE Boost::graph ilocplex cplex concert)

# ==== C++ Version ====
set_target_properties(ebus_vns PROPERTIES CXX_STANDARD 17)
