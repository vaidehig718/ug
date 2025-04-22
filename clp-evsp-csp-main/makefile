# Compiler settings
CXX = g++
CXXFLAGS = -O3 -std=c++2a
# CXXFLAGS = -O3 -std=c++17 # Use this if there are compatibility issues with Boost

# Directories for CPLEX and Concert
CPLEXDIR = /opt/ibm/ILOG/CPLEX_Studio2211/cplex
CONCERTDIR = /opt/ibm/ILOG/CPLEX_Studio2211/concert

# Library directories
CPLEXLIBDIR = $(CPLEXDIR)/lib/x86-64_linux/static_pic
CONCERTLIBDIR = $(CONCERTDIR)/lib/x86-64_linux/static_pic

# Link options and libraries
LDFLAGS = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR) -lilocplex -lcplex -lconcert -lm -pthread -ldl

# OpenMP flag
OPENMPFLAG = -fopenmp

# Include directories
INCLUDES = -I$(CPLEXDIR)/include -I$(CONCERTDIR)/include -I./include

# Source files
SOURCES = main.cpp vehicle.cpp helpers.cpp operators.cpp csp.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Executable name
EXECUTABLE = cmake-build-debug/ebus_vns

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OPENMPFLAG) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(OPENMPFLAG) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(EXECUTABLE) $(OBJECTS)

.PHONY: all clean
