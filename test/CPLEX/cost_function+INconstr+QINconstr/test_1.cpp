#include "MPCsolver.h"
#include "CPLEXsolver.h"

#include "writeMatlabScript.h"

using namespace std;

MPCsolver* solver = NULL;


int main(int argc, char **argv)
{
    const int numVar          = 2;
    const int numConstr       = 2;
    const int numQConstr      = 2;
    const int numEqConstraint = 0;

    MatrixXd H(numVar,numVar);      H << 2.0, 0.0,
                                         0.0, 5.0;
    VectorXd f(numVar);             f << -1.0, 3.0;

    MatrixXd Ain(numConstr,numVar); Ain << 1.0, -1.0,
                                           2.0, -2.0;
    VectorXd Bin(numConstr);        Bin << 1.0, 0.0;

    VectorXd l1(numVar);            l1 << 2.0, 1.0;
    MatrixXd q1(numVar,numVar);     q1 << 2.0, 0.5,
                                          0.5, 1.0;
    double r1 = 4.0;
    
    VectorXd l2(numVar);            l2 << 1.0, -1.5;
    MatrixXd q2(numVar,numVar);     q2 << 1.0, 0.1,
                                          0.1, 4.0;
    double r2 = 2.0;

    std::vector<VectorXd> l = { l1, l2 };
    std::vector<MatrixXd> Q = { q1, q2 };
    std::vector<double>   r = { r1, r2 };

    /** CPLEX solver example */
    solver = new CPLEXsolver(numVar, numConstr, numEqConstraint, numQConstr, CPLEXsolver::AUTO);
    cout << "CPLEX solver created" << endl;

    if (solver->initProblem())
        cout << "CPLEX solver initialized" << endl;
    else
        cout << "Cannot initialize CPLEX solver" << endl;
    solver->set_printLevel(MPCsolver::NONE);

    if (solver->setProblem(H, f, Ain, Bin, l, Q, r))
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
    QCP_writeMatlabScript("test_1_script.m", true, H, f, Ain, Bin, l, Q, r, result_CPLEX, optimizerStatus);

    cout << "Matlab file generated" << endl << endl;

    return 0;
}
