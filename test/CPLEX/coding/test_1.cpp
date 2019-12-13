#include "MPCsolver.h"
#include "CPLEXsolver.h"

using namespace std;

MPCsolver* solver = NULL;

#define MAX_DECIMAL 5


int main(int argc, char **argv)
{
    const int numVar          = 2;
    const int numConstr       = 2;
    const int numQConstr      = 1;
    const int numEqConstraint = 2;

    const double convergence_tolerance_QP = 1.0e-6;
    const double convergence_tolerance_QCP = 1.0e-6;
    const double optimality_tolerance = 1.0e-6;
    const double feasibility_tolerance = 1.0e-6;

    std::vector<double> lB(numVar); lB.at(0) = -1.0; lB.at(1) = -2.0;
    std::vector<double> uB(numVar); uB.at(0) =  5.0; uB.at(1) =  5.0;

    MatrixXd H(numVar,numVar);      H << 2.0, 0.0,
                                         0.0, 5.0;
    VectorXd f(numVar);             f << -2.0, 3.0, 1.0;

    MatrixXd Ain(numConstr,numVar); Ain << 1.0, -1.0,
                                           2.0, -2.0;
    VectorXd Bin(numConstr);        Bin << 1.0, 0.0;

    MatrixXd Aeq(numEqConstraint,numVar); Aeq <<  1.0, 0.0,
                                                 -1.0, 0.0;
    VectorXd Beq(numEqConstraint);        Beq << 1.0, -1.0;

    vector<VectorXd> l;
    vector<MatrixXd> Q;
    vector<double> r;

    VectorXd l1(numVar);            l1 << 0.5, -0.5;     l.push_back(l1);
    MatrixXd Q1(numVar,numVar);     Q1 << 5.1, 1.0,
                                          1.0, 5.1;      Q.push_back(Q1);
    double                          r1 = 100.0;          r.push_back(r1);

    MPCsolver::solverType solver_algo = MPCsolver::BARRIER;

    /** CPLEX solver example */
    solver = new CPLEXsolver(numVar, numConstr, numEqConstraint, numQConstr, CPLEXsolver::AUTO);
    cout << "CPLEX solver created" << endl;

    solver->set_solverMethod(solver_algo);
    switch (solver_algo)
    {
        case MPCsolver::AUTO:
        std::cout << "CPLEX solver method: AUTO" << std::endl;
        break;

        case MPCsolver::PRIMAL:
        std::cout << "CPLEX solver method: PRIMAL" << std::endl;
        break;

        case MPCsolver::DUAL:
        std::cout << "CPLEX solver method: DUAL" << std::endl;
        break;

        case MPCsolver::NETWORK:
        std::cout << "CPLEX solver method: NETWORK" << std::endl;
        break;

        case MPCsolver::BARRIER:
        std::cout << "CPLEX solver method: BARRIER" << std::endl;
        break;
        
        case MPCsolver::SIFTING:
        std::cout << "CPLEX solver method: SIFTING" << std::endl;
        break;

        case MPCsolver::CONCURRENT:
        std::cout << "CPLEX solver method: CONCURRENT" << std::endl;
        break;
    }

    solver->set_solverParams(convergence_tolerance_QP, convergence_tolerance_QCP, optimality_tolerance, feasibility_tolerance);
    std::cout << "CPLEX optimality/feasibility thresholds set" << std::endl;

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

    if (solver)
    {
        delete solver;
        solver = NULL;

        cout << "CPLEX solver deleted" << endl;
    }

    return 0;
}