# Optimization of Electric Bus Fleets

## Description

This repository contains code for co-optimizing locations, vehicle schedules, and charging schedules for electric bus fleets. It aims to integrate optimization processes for determining optimal charging station locations alongside vehicle scheduling, taking into account the constraints of range and the necessity for opportunity charging.

## Features

- Integrated optimization of charging station locations and vehicle schedules, considering range limitations and opportunity charging.
- Provides multiple charging scheduling formulations to optimize charging costs with varying levels of granularity.

## Installation

- **CPLEX**: Used for solving linear and mixed-integer linear programs for the scheduling algorithms. Please ensure CPLEX is installed on your system. You may need to adjust the Makefiles and CMakeLists depending on your CPLEX version.
- **Boost Libraries**: Required for clique enumeration. Ensure the Boost libraries are properly installed on your system. Visit [Boost Getting Started](https://www.boost.org/doc/libs/release/more/getting_started/index.html) for installation instructions.
- Makefiles and CMakeLists are provided. Modifications may be necessary to accommodate your specific CPLEX version and system configuration.

## Contact

- Email: tarunrambha@iisc.ac.in
