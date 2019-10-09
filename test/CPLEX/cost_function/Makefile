CCLNFLAGS = -lconcert -lilocplex -lcplex -lm -lpthread

CCLNDIRS = -L/opt/ibm/ILOG/CPLEX_Studio1263/concert/lib/x86-64_linux/static_pic -L/opt/ibm/ILOG/CPLEX_Studio1263/cplex/lib/x86-64_linux/static_pic -L/opt/qpOASES-3.2.0/bin

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/include/eigen3 -I/opt/ibm/ILOG/CPLEX_Studio1263/cplex/include/ -I/opt/ibm/ILOG/CPLEX_Studio1263/concert/include/ -I/opt/qpOASES-3.2.1/include/ -I../../../ -I../../../util/

CCC = g++ -O0 -std=c++11

test_1: test_1.o CPLEXsolver.o writeMatlabScript.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_1 test_1.o CPLEXsolver.o writeMatlabScript.o $(CCLNFLAGS)
test_1.o: test_1.cpp
	$(CCC) -c $(CCFLAGS) test_1.cpp -o test_1.o

test_2: test_2.o CPLEXsolver.o writeMatlabScript.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_2 test_2.o CPLEXsolver.o writeMatlabScript.o $(CCLNFLAGS)
test_2.o: test_2.cpp
	$(CCC) -c $(CCFLAGS) test_2.cpp -o test_2.o

test_3: test_3.o CPLEXsolver.o writeMatlabScript.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_3 test_3.o CPLEXsolver.o writeMatlabScript.o $(CCLNFLAGS)
test_3.o: test_3.cpp
	$(CCC) -c $(CCFLAGS) test_3.cpp -o test_3.o

CPLEXsolver.o: ../../../CPLEXsolver.cpp
	$(CCC) -c $(CCFLAGS) ../../../CPLEXsolver.cpp -o CPLEXsolver.o
writeMatlabScript.o: ../../../util/writeMatlabScript.cpp
	$(CCC) -c $(CCFLAGS) ../../../util/writeMatlabScript.cpp -o writeMatlabScript.o

all: test_1 test_2 test_3

clean:
	@rm -rf *.o *.lp
	@rm -rf test_1 test_2 test_3

