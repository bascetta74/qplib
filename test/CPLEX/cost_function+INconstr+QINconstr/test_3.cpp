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

    MatrixXd H(numVar,numVar);      H <<  2.0, 0.25,
                                         0.25, 5.0;
    VectorXd f(numVar);             f << 0.5, -1.5;

    MatrixXd Ain(numConstr,numVar); Ain << 1.7, 2.3,
                                           -1.5, -1.5;
    VectorXd Bin(numConstr);        Bin << 0.4, 1.2;

    vector<VectorXd> l;
    vector<MatrixXd> Q;
    vector<double> r;

    VectorXd l1(numVar);            l1 << -1.0, 1.0;         l.push_back(l1);
    VectorXd l2(numVar);            l2 << 0.5, -0.5;         l.push_back(l2);
    MatrixXd Q1(numVar,numVar);     Q1 << 2.0, 1.0,
                                          1.0, 1.0;          Q.push_back(Q1);
    MatrixXd Q2(numVar,numVar);     Q2 << 1.0, 0.5,
                                          0.5, 2.0;          Q.push_back(Q2);
    double                          r1 = 1.0;                r.push_back(r1);
    double                          r2 = 5.0;                r.push_back(r2);

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
    QCP_writeMatlabScript("test_3_script.m", true, H, f, Ain, Bin, l, Q, r, result_CPLEX, optimizerStatus);

    cout << "Matlab file generated" << endl << endl;

    return 0;
}
