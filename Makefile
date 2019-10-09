CCLNFLAGS = -lconcert -lilocplex -lcplex -lm -lpthread

CCLNDIRS = -L/opt/ibm/ILOG/CPLEX_Studio1263/concert/lib/x86-64_linux/static_pic -L/opt/ibm/ILOG/CPLEX_Studio1263/cplex/lib/x86-64_linux/static_pic -L./qpOASES-3.2.1/bin

CCOPT = -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/include/eigen3 -I/opt/ibm/ILOG/CPLEX_Studio1263/cplex/include/ -I/opt/ibm/ILOG/CPLEX_Studio1263/concert/include/ -I./qpOASES-3.2.1/include/

CCC = g++ -O0 -std=c++11

CC = gcc

libCPLEXsolver_cwrapper.so: CPLEXsolver_cwrapper.cpp
	$(CCC) -c $(CCFLAGS) CPLEXsolver_cwrapper.cpp -o CPLEXsolver_cwrapper.o
	$(CCC) -c $(CCFLAGS) CPLEXsolver.cpp -o CPLEXsolver.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -shared -Wl,-soname,libCPLEXsolver_cwrapper.so -o ./lib/libCPLEXsolver_cwrapper.so CPLEXsolver.o CPLEXsolver_cwrapper.o $(CCLNFLAGS)

libMPCsolver.so: CPLEXsolver.cpp qpOASESsolver.cpp
	$(CCC) -c $(CCFLAGS) CPLEXsolver.cpp -o CPLEXsolver.o
	$(CCC) -c $(CCFLAGS) qpOASESsolver.cpp -o qpOASESsolver.o
	$(CCC) -shared -Wl,-soname,libMPCsolver.so -o ./lib/libMPCsolver.so CPLEXsolver.o qpOASESsolver.o

all: libMPCsolver.so libMPCsolver_cwrapper.so

clean:
	@rm -rf *.o ./lib/*.so
