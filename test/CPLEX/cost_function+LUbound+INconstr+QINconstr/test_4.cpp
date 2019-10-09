#include "MPCsolver.h"
#include "CPLEXsolver.h"
#include "writeMatlabScript.h"

using namespace std;

MPCsolver* solver = NULL;


int main(int argc, char **argv)
{
    const int numVar          = 5;
    const int numConstr       = 5;
    const int numQConstr      = 2;
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

    vector<VectorXd> l;
    vector<MatrixXd> Q;
    vector<double> r;

    VectorXd l1(numVar);            l1 << 0.5, -0.5, 1.5, -1.0, 2.0;    l.push_back(l1);
    MatrixXd Q1(numVar,numVar);     Q1 << 5.1, 1.0,  0.5, 1.5,  3.2,
                                          1.0, 2.1,  1.5, 0.4,  1.7,
                                          0.5, 1.5, 10.1, 0.4,  2.8, 
                                          1.5, 0.4,  0.4, 1.7,  4.3,
                                          3.2, 1.7,  2.8, 4.3, 15.0;     Q.push_back(Q1);
    double                          r1 = 25.0;                          r.push_back(r1);

    VectorXd l2(numVar);            l2 << -0.5, 0.5, -1.5, 1.0, -2.0;   l.push_back(l2);
    MatrixXd Q2(numVar,numVar);     Q2 << 4.1, 1.0, 0.5, 1.5,  1.2,
                                          1.0, 7.1, 1.5, 5.4,  1.7,
                                          0.5, 1.5, 5.1, 0.4,  2.8, 
                                          1.5, 0.4, 5.4, 3.7,  4.3,
                                          1.2, 1.7, 2.8, 4.3, 25.5;      Q.push_back(Q2);
    double                          r2 = 55.0;                          r.push_back(r2);

    /** CPLEX solver example */
    solver = new CPLEXsolver(numVar, numConstr, numEqConstraint, numQConstr, CPLEXsolver::AUTO);
    cout << "CPLEX solver created" << endl;

    if (solver->initProblem())
        cout << "CPLEX solver initialized" << endl;
    else
        cout << "Cannot initialize CPLEX solver" << endl;
    solver->set_printLevel(MPCsolver::NONE);

    if (solver->setProblem(lB, uB,H, f, Ain, Bin, l, Q, r))
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
    QCP_writeMatlabScript("test_4_script.m", true, lB, uB, H, f, Ain, Bin, l, Q, r, result_CPLEX, optimizerStatus);

    cout << "Matlab file generated" << endl << endl;

    return 0;
}
