CCLNFLAGS = -lconcert -lilocplex -lcplex -lm -lpthread -lCPLEXsolver_cwrapper

CCLNDIRS = -L/opt/ibm/ILOG/CPLEX_Studio1263/concert/lib/x86-64_linux/static_pic -L/opt/ibm/ILOG/CPLEX_Studio1263/cplex/lib/x86-64_linux/static_pic -L./qpOASES-3.2.1/bin -L../../../lib

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/include/eigen3 -I/opt/ibm/ILOG/CPLEX_Studio1263/cplex/include/ -I/opt/ibm/ILOG/CPLEX_Studio1263/concert/include/ -I./qpOASES-3.2.1/include/ -I../../../

CCC = g++ -O0 -std=c++11

CC = gcc

test_1: test_1.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_1 test_1.o $(CCLNFLAGS)
test_1.o: test_1.cpp
	$(CC) -c $(CCFLAGS) test_1.cpp -o test_1.o

all: test_1

clean:
	@rm -rf *.o ./lib/*.so
	@rm -rf test_1
