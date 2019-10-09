#include "MPCsolver.h"
#include "CPLEXsolver.h"
#include "writeMatlabScript.h"

using namespace std;

MPCsolver* solver = NULL;


int main(int argc, char **argv)
{
    const int numVar          = 5;
    const int numConstr       = 5;
    const int numQConstr      = 0;
    const int numEqConstraint = 0;

    std::vector<double> lB(numVar); lB.at(0) = -1.0; lB.at(1) = -1.0; lB.at(2) = -1.0; lB.at(3) = -1.0; lB.at(4) = -1.0;
    std::vector<double> uB(numVar); uB.at(0) = 5.0; uB.at(1) = 5.0; uB.at(2) = 5.0; uB.at(3) = 5.0; uB.at(4) = 5.0;

    MatrixXd H(numVar,numVar);      H << 4.0, 0.0,  1.0,  0.0, -1.0,
                                         0.0, 5.0,  1.0,  0.0,  0.0,
                                         1.0, 1.0,  3.0, -1.0,  1.0,
                                         0.0, 0.0, -1.0,  2.0, -1.0,
                                        -1.0, 0.0,  1.0, -1.0,  5.0;

    VectorXd f(numVar);             f << 1.0, 0.0, 1.0 ,2.0 ,-1.0;

    MatrixXd Ain(numConstr,numVar); Ain << 1.0, -1.0,  1.0,  0.0, -1.0,
                                           2.0, -2.0,  1.0,  0.0,  0.0,
                                          -1.0,  0.0,  1.0,  0.0,  1.0,
                                           0.0,  1.0, -1.0,  2.0,  0.0,
                                           1.0,  0.0,  1.0, -1.0, -2.0;
    VectorXd Bin(numConstr);        Bin << 1.0, 0.0 ,-1.0, 0.0, -2.0;


    /** CPLEX solver example */
    solver = new CPLEXsolver(numVar, numConstr, numEqConstraint, numQConstr, CPLEXsolver::AUTO);
    cout << "CPLEX solver created" << endl;

    if (solver->initProblem())
        cout << "CPLEX solver initialized" << endl;
    else
        cout << "Cannot initialize CPLEX solver" << endl;
    solver->set_printLevel(MPCsolver::NONE);

    if (solver->setProblem(lB, uB,H, f, Ain, Bin))
        cout << "CPLEX solver problem setted" << endl;
    else
        cout << "Cannot set CPLEX problem" << endl;

    VectorXd result_CPLEX(numVar);
    int optimizerStatus = -1;
    if (solver->solveProblem(result_CPLEX, optimizerStatus))
    {
        cout << "CPLEX solver problem solved" << endl;
        cout << "Solution: [ " << result_CPLEX.transpose() << " ]" << endl;
        cout << "Solver status: " << optimizerStatus << endl;

    }
    else
        cout << "Cannot solve CPLEX problem" << endl;

    solver->saveProblem("problem");

    if (solver)
    {
        delete solver;
        solver = NULL;

        cout << "CPLEX solver deleted" << endl;
    }

    /** Generate Matlab script */
    QP_writeMatlabScript("test_4_script.m", true, lB, uB, H, f, Ain, Bin, result_CPLEX, optimizerStatus);

    cout << "Matlab file generated" << endl << endl;

    return 0;
}
