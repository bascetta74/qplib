#include "MPCsolver.h"
#include "CPLEXsolver.h"

#include "writeMatlabScript.h"

using namespace std;

MPCsolver* solver = NULL;

#define MAX_DECIMAL 5


int main(int argc, char **argv)
{
    const int numVar          = 7;
    const int numConstr       = 5;
    const int numQConstr      = 1;
    const int numEqConstraint = 0;

    const double convergence_tolerance_QP = 1.0e-6;
    const double convergence_tolerance_QCP = 1.0e-6;
    const double optimality_tolerance = 1.0e-6;
    const double feasibility_tolerance = 1.0e-6;

    MatrixXd H(numVar,numVar);      H << 7.13099, 0.06350, 0.12037, 0.10880, 0.14199, 0.14245, 0.10259,
                                         0.06350, 7.05669, 0.05295, 0.04431, 0.09066, 0.07930, 0.08221,
                                         0.12037, 0.05295, 7.22595, 0.13791, 0.16536, 0.17132, 0.15146,
                                         0.10880, 0.04431, 0.13791, 7.14869, 0.15104, 0.17486, 0.12852,
                                         0.14199, 0.09066, 0.16536, 0.15104, 7.22290, 0.19553, 0.19382,
                                         0.14245, 0.07930, 0.17132, 0.17486, 0.19553, 7.23506, 0.17586,
                                         0.10259, 0.08221, 0.15146, 0.12852, 0.19382, 0.17586, 7.19031;
    VectorXd f(numVar);             f << 0.00299, -0.01145, 0.01835, 0.00871, -0.0113, 0.01615, -0.0046;

    MatrixXd Ain(numConstr,numVar); Ain <<  0.24742, -0.49315,  0.49529,  0.20114,  0.42162,  0.03185,  0.48066,
                                            0.26593,  0.01619, -0.54224,  0.65967, -0.34331, -0.28516, -0.02553,
                                           -0.46914, -0.45507,  0.41639,  0.42633, -0.17573,  0.10147,  0.42012,
                                           -0.16710, -0.47569,  0.46871, -0.06755, -0.19251,  0.31112, -0.62264,
                                           -0.46256, -0.28306,  0.20027,  0.49984,  0.41854, -0.28243,  0.40128;
    VectorXd Bin(numConstr);        Bin << 0.99337, -0.93614, -0.26695, -0.89053, -0.66429;

    VectorXd l1(numVar);            l1 << 0.01581, 0.01119, 0.00649, 0.01868, -0.00237, -0.01296, 0.00098;
    MatrixXd q1(numVar,numVar);     q1 << 7.11460, 0.11302, 0.10870, 0.11998,  0.15320,  0.12116, 0.06823,
                                          0.11302, 7.21912, 0.17320, 0.14256,  0.17787,  0.17976, 0.11378,
                                          0.10870, 0.17320, 7.14727, 0.13170,  0.16831,  0.15145, 0.09971,
                                          0.11998, 0.14256, 0.13170, 7.14410,  0.17366,  0.14855, 0.09237,
                                          0.15320, 0.17787, 0.16831, 0.17366,  7.22583,  0.17859, 0.10951,
                                          0.12116, 0.17976, 0.15145, 0.14855,  0.17859,  7.18761, 0.09384,
                                          0.06823, 0.11378, 0.09971, 0.09237,  0.10951,  0.09384, 7.09037;
    double r1 = 143.254;

    std::vector<VectorXd> l = { l1 };
    std::vector<MatrixXd> Q = { q1 };
    std::vector<double>   r = { r1 };

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

    QCP_writeMatlabScript(argv[0], true, algo_type, algo_tol, H, f, Ain, Bin, l, Q, r, result_CPLEX, optimizerStatus, MAX_DECIMAL);

    cout << "Matlab file generated" << endl << endl;

    return 0;
}
