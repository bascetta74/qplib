#include "MPCsolver.h"
#include "CPLEXsolver.h"
#include "writeMatlabScript.h"


MPCsolver* solver = NULL;

#define MAX_DECIMAL 5


int main(int argc, char **argv)
{
    const int numVar          = 7;
    const int numConstr       = 1;
    const int numQConstr      = 0;
    const int numEqConstraint = 5;

    const double convergence_tolerance_QP = 1.0e-6;
    const double convergence_tolerance_QCP = 1.0e-6;
    const double optimality_tolerance = 1.0e-6;
    const double feasibility_tolerance = 1.0e-6;

    std::vector<double> lB{-6.9507, -7.700, -16.834, -3.9797, -4.2436, -11.317, -5.7158};
    std::vector<double> uB{ 5.8284, 11.719,  2.0219, 18.2750,  9.4704,  11.646, 14.2270};

    MatrixXd H(numVar,numVar);      H << 7.277, 0.1235, 0.2224, 0.2175, 0.1771, 0.1806, 0.0358,
                                         0.1235, 7.1116, 0.1272, 0.1043, 0.0883, 0.0983, 0.0291,
                                         0.2224, 0.1272, 7.2202, 0.1699, 0.1099, 0.1807, 0.0399,
                                         0.2175, 0.1043, 0.1699, 7.2518, 0.2086, 0.1215, 0.0497,
                                         0.1771, 0.0883, 0.1099, 0.2086, 7.2085, 0.0706, 0.032,
                                         0.1806, 0.0983, 0.1807, 0.1215, 0.0706, 7.1549, 0.0312,
                                         0.0358, 0.0291, 0.0399, 0.0497, 0.032, 0.0312, 7.0186;
    VectorXd f(numVar);             f << -0.0047, 0.004, -0.0191, -0.0113, 0.0094, -0.0041, -0.0185;

    MatrixXd Ain(numConstr,numVar); Ain << -0.449, 0.1602, 0.5401, -0.2458, 0.1598, -0.5479, 0.3081;
    VectorXd Bin(numConstr);        Bin << -0.2284;

    MatrixXd Aeq(numEqConstraint,numVar); Aeq << -0.0985, -0.5071, 0.5169, 0.4079, 0.4007, 0.1645, -0.3347,
                                                  0.3681, 0.2274, -0.1843, -0.5899, 0.0428, -0.3576, -0.5488,
                                                  0.4227, -0.2469, -0.5271, -0.1867, 0.2167, 0.4735, 0.4201,
                                                  0.4821, -0.2989, 0.2266, 0.4402, 0.4241, 0.1384, -0.4838,
                                                  0.4096, -0.1629, 0.2833, 0.4346, -0.3171, 0.4887, -0.444;
    VectorXd Beq(numEqConstraint);        Beq << 0.4041, -0.8668, 0.472, -0.936, -0.4808;

    MPCsolver::solverType solver_algo = MPCsolver::AUTO;

    /** CPLEX solver example */
    solver = new CPLEXsolver(numVar, numConstr, numEqConstraint, numQConstr, CPLEXsolver::AUTO);
    std::cout << "CPLEX solver created" << std::endl;

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
        std::cout << "CPLEX solver initialized" << std::endl;
    else
        std::cout << "Cannot initialize CPLEX solver" << std::endl;
    solver->set_printLevel(MPCsolver::HIGH);

    if (solver->setProblem(lB, uB, H, f, Ain, Bin, Aeq, Beq))
        std::cout << "CPLEX solver problem setted" << std::endl;
    else
        std::cout << "Cannot set CPLEX problem" << std::endl;

    VectorXd result_CPLEX(numVar);
    int optimizerStatus = -1;
    if (solver->solveProblem(result_CPLEX, optimizerStatus))
    {
        std::cout << "CPLEX solver problem solved" << std::endl;
        std::cout << "Solution: [ " << result_CPLEX.transpose() << " ]" << std::endl;
        std::cout << "Cost function value: " << 0.5*result_CPLEX.transpose()*H*result_CPLEX+f.transpose()*result_CPLEX << std::endl;
        std::cout << "Equality constraint satisfaction: " << (Aeq*result_CPLEX-Beq).norm() << std::endl;
        std::cout << "Solver status: " << optimizerStatus << std::endl;
    }
    else
        std::cout << "Cannot solve CPLEX problem" << std::endl;

    solver->saveProblem(argv[0]);

    if (solver)
    {
        delete solver;
        solver = NULL;

        std::cout << "CPLEX solver deleted" << std::endl;
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

    QP_writeMatlabScript(argv[0], true, algo_type, algo_tol, lB, uB, H, f, Ain, Bin, Aeq, Beq, result_CPLEX,
                         optimizerStatus, MAX_DECIMAL);

    std::cout << "Matlab file generated" << std::endl << std::endl;

    return 0;
}
