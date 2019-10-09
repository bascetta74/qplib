#include "MPCsolver.h"
#include "CPLEXsolver.h"
#include "writeMatlabScript.h"

using namespace std;

MPCsolver* solver = NULL;


int main(int argc, char **argv)
{
    const int numVar          = 5;
    const int numConstr       = 2;
    const int numQConstr      = 1;
    const int numEqConstraint = 2;

    std::vector<double> lB(numVar); lB.at(0) = -1.0; lB.at(1) = -1.0; lB.at(2) = -1.0; lB.at(3) = -1.0; lB.at(4) = -1.0;
    std::vector<double> uB(numVar); uB.at(0) = 5.0; uB.at(1) = 5.0; uB.at(2) = 5.0; uB.at(3) = 5.0; uB.at(4) = 5.0;

    MatrixXd H(numVar,numVar);      H << 4.0, 0.0,  1.0,  0.0, -1.0,
                                         0.0, 5.0,  1.0,  0.0,  0.0,
                                         1.0, 1.0,  3.0, -1.0,  1.0,
                                         0.0, 0.0, -1.0,  2.0, -1.0,
                                        -1.0, 0.0,  1.0, -1.0,  5.0;
    VectorXd f(numVar);             f << 1.0, 0.0, 1.0 ,2.0 ,-1.0;

    MatrixXd Ain(numConstr,numVar); Ain << 1.0, -1.0,  1.0,  0.0, -1.0,
                                           2.0, -2.0,  1.0,  0.0,  0.0;
    VectorXd Bin(numConstr);        Bin << 10.0, 20.0;

    MatrixXd Aeq(numEqConstraint,numVar); Aeq << 1.0,  2.0, 1.0, 0.5, 1.0,
                                                 0.0, -2.0, 0.0, 0.0, 0.0;
    VectorXd Beq(numEqConstraint);        Beq << 1.0, 1.0;

    vector<VectorXd> l;
    vector<MatrixXd> Q;
    vector<double> r;

    VectorXd l1(numVar);            l1 << 1.0, 5.0, 2.0, 0.5, -0.5;        l.push_back(l1);
    MatrixXd Q1(numVar,numVar);     Q1 << 2.1, 1.0, 0.5,  5.5,  1.5,
                                          1.0, 1.0, 0.1,  1.0,  0.4,
                                          0.5, 0.1, 5.0,  0.2,  2.5,
                                          5.5, 1.0, 0.2, 25.5,  2.1,
                                          1.5, 0.4, 2.5,  2.1, 12.3;         Q.push_back(Q1);
    double                          r1 = 10.0;                             r.push_back(r1);

    /** CPLEX solver example */
    solver = new CPLEXsolver(numVar, numConstr, numEqConstraint, numQConstr, CPLEXsolver::AUTO);
    cout << "CPLEX solver created" << endl;

    if (solver->initProblem())
        cout << "CPLEX solver initialized" << endl;
    else
        cout << "Cannot initialize CPLEX solver" << endl;
    solver->set_printLevel(MPCsolver::NONE);

    if (solver->setProblem(lB, uB, H, f, Ain, Bin, Aeq, Beq, l, Q, r))
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
    QCP_writeMatlabScript("test_5_script.m", true, lB, uB, H, f, Ain, Bin, Aeq, Beq, l, Q, r, result_CPLEX, optimizerStatus);

    cout << "Matlab file generated" << endl << endl;

    return 0;
}
