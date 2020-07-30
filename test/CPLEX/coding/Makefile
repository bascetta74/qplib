CCLNFLAGS = -lMPCsolver -lconcert -lilocplex -lcplex -lm -lpthread -ldl -lstdc++

CCLNDIRS = -L/opt/ibm/ILOG/CPLEX_Studio1210/concert/lib/x86-64_linux/static_pic -L/opt/ibm/ILOG/CPLEX_Studio1210/cplex/lib/x86-64_linux/static_pic

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/include/eigen3 -I/opt/ibm/ILOG/CPLEX_Studio1210/cplex/include/ -I/opt/ibm/ILOG/CPLEX_Studio1210/concert/include/

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
