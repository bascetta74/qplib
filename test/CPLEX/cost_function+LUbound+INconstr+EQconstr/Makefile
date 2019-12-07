CCLNFLAGS = -lconcert -lilocplex -lcplex -lm -lpthread -ldl

CCLNDIRS = -L/opt/ibm/ILOG/CPLEX_Studio129/concert/lib/x86-64_linux/static_pic -L/opt/ibm/ILOG/CPLEX_Studio129/cplex/lib/x86-64_linux/static_pic -L/opt/qpOASES-3.2.1/bin

CCOPT = -m64 -O0 -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD -std=c++11

CCFLAGS = $(CCOPT) -I/usr/include/eigen3 -I/opt/ibm/ILOG/CPLEX_Studio129/cplex/include/ -I/opt/ibm/ILOG/CPLEX_Studio129/concert/include/ -I/opt/qpOASES-3.2.1/include/ -I../../../ -I../../../util/

CCC = g++

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

test_4: test_4.o CPLEXsolver.o writeMatlabScript.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_4 test_4.o CPLEXsolver.o writeMatlabScript.o $(CCLNFLAGS)
test_4.o: test_4.cpp
	$(CCC) -c $(CCFLAGS) test_4.cpp -o test_4.o

test_5: test_5.o CPLEXsolver.o writeMatlabScript.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_5 test_5.o CPLEXsolver.o writeMatlabScript.o $(CCLNFLAGS)
test_5.o: test_5.cpp
	$(CCC) -c $(CCFLAGS) test_5.cpp -o test_5.o

test_6: test_6.o CPLEXsolver.o writeMatlabScript.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_6 test_6.o CPLEXsolver.o writeMatlabScript.o $(CCLNFLAGS)
test_6.o: test_6.cpp
	$(CCC) -c $(CCFLAGS) test_6.cpp -o test_6.o

test_7: test_7.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_7 test_7.o $(CCLNFLAGS)
test_7.o: test_7.cpp
	$(CCC) -c $(CCFLAGS) test_7.cpp -o test_7.o

test_rand: test_rand.o CPLEXsolver.o writeMatlabScript.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o test_rand test_rand.o CPLEXsolver.o writeMatlabScript.o $(CCLNFLAGS)
test_rand.o: test_rand.cpp
	$(CCC) -c $(CCFLAGS) test_rand.cpp -o test_rand.o

CPLEXsolver.o: ../../../CPLEXsolver.cpp
	$(CCC) -c $(CCFLAGS) ../../../CPLEXsolver.cpp -o CPLEXsolver.o
writeMatlabScript.o: ../../../util/writeMatlabScript.cpp
	$(CCC) -c $(CCFLAGS) ../../../util/writeMatlabScript.cpp -o writeMatlabScript.o

all: test_1 test_2 test_3 test_4 test_5 test_6 test_7 test_rand

clean:
	@rm -rf *.o *.lp test_*.m *.mat
	@rm -rf test_1 test_2 test_3 test_4 test_5 test_6 test_7 test_rand
	@rm -rf ./script/* ./output/*

