#include "MPCsolver.h"
#include "CPLEXsolver.h"

#include "writeMatlabScript.h"

using namespace std;

MPCsolver* solver = NULL;

#define MAX_DECIMAL 5


int main(int argc, char **argv)
{
    const int numVar          = 2;
    const int numConstr       = 2;
    const int numQConstr      = 1;
    const int numEqConstraint = 0;

    const double convergence_tolerance_QP = 1.0e-6;
    const double convergence_tolerance_QCP = 1.0e-6;
    const double optimality_tolerance = 1.0e-6;
    const double feasibility_tolerance = 1.0e-6;

    MatrixXd H(numVar,numVar);      H <<  2.0, 0.25,
                                         0.25, 5.0;
    VectorXd f(numVar);             f << 0.5, -1.5;

    MatrixXd Ain(numConstr,numVar); Ain << 0.7, -0.3,
                                           1.5, 2.25;
    VectorXd Bin(numConstr);        Bin << 1.4, -1.2;

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

    if (solver->setProblem(H, f, Ain, Bin))
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

    solver->saveProblem(argv[0]);

    if (solver)
    {
        delete solver;
        solver = NULL;

        cout << "CPLEX solver deleted" << endl;
    }

    /** Generate Matlab script */
    optim_algo_t algo_type;
    switch (solver_algo)
    {
        case MPCsolver::AUTO:
        algo_type = AUTO;
        break;

        case MPCsolver::PRIMAL:
        algo_type = PRIMAL;
        break;

        case MPCsolver::DUAL:
        algo_type = DUAL;
        break;

        case MPCsolver::NETWORK:
        algo_type = NETWORK;
        break;

        case MPCsolver::BARRIER:
        algo_type = BARRIER;
        break;

        case MPCsolver::SIFTING:
        algo_type = SIFTING;
        break;

        case MPCsolver::CONCURRENT:
        algo_type = CONCURRENT;
        break;
    }

    optim_algo_tol_t algo_tol;
    algo_tol.optimality_tolerance = optimality_tolerance;
    algo_tol.feasibility_tolerance = feasibility_tolerance;
    algo_tol.QP_convergence_tolerance = convergence_tolerance_QP;
    algo_tol.QCP_convergence_tolerance = convergence_tolerance_QCP;

    QP_writeMatlabScript(argv[0], true, algo_type, algo_tol, H, f, Ain, Bin, result_CPLEX, optimizerStatus, MAX_DECIMAL);

    cout << "Matlab file generated" << endl << endl;

    return 0;
}
