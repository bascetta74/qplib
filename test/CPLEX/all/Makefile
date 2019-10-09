CCLNFLAGS = -lconcert -lilocplex -lcplex -lm -lpthread

CCLNDIRS = -L/opt/ibm/ILOG/CPLEX_Studio1263/concert/lib/x86-64_linux/static_pic -L/opt/ibm/ILOG/CPLEX_Studio1263/cplex/lib/x86-64_linux/static_pic -L/opt/qpOASES-3.2.0/bin

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/include/eigen3 -I/opt/ibm/ILOG/CPLEX_Studio1263/cplex/include/ -I/opt/ibm/ILOG/CPLEX_Studio1263/concert/include/ -I/opt/qpOASES-3.2.0/include/ -I../../../ -I../../../util/

CCC = g++ -O0 -std=c++11

test_all_qp: test_all_qp.o CPLEXsolver.o writeMatlabScript.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_all_qp test_all_qp.o CPLEXsolver.o writeMatlabScript.o $(CCLNFLAGS)
test_all_qp.o: test_all_qp.cpp
	$(CCC) -c $(CCFLAGS) test_all_qp.cpp -o test_all_qp.o

test_all_qcp: test_all_qcp.o CPLEXsolver.o writeMatlabScript.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_all_qcp test_all_qcp.o CPLEXsolver.o writeMatlabScript.o $(CCLNFLAGS)
test_all_qcp.o: test_all_qcp.cpp
	$(CCC) -c $(CCFLAGS) test_all_qcp.cpp -o test_all_qcp.o

CPLEXsolver.o: ../../../CPLEXsolver.cpp
	$(CCC) -c $(CCFLAGS) ../../../CPLEXsolver.cpp -o CPLEXsolver.o
writeMatlabScript.o: ../../../util/writeMatlabScript.cpp
	$(CCC) -c $(CCFLAGS) ../../../util/writeMatlabScript.cpp -o writeMatlabScript.o

all: test_all_qp test_all_qcp

clean:
	@rm -rf *.o *.lp
	@rm -rf test_all_qp test_all_qcp
