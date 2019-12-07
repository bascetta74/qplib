#include <sstream>
#include <time.h>
#include <tgmath.h>

#include "MPCsolver.h"
#include "CPLEXsolver.h"
#include "writeMatlabScript.h"

#define NUM_TEST            1000
#define MAX_NUM_VAR         100
#define MAX_NUM_CONSTRAINT  50
#define MAX_DECIMAL         5

MPCsolver* solver = NULL;


int main(int argc, char **argv)
{
    int num_problem_solved         = 0;
    int num_problem_notinitialized = 0;
    int num_problem_notsetted      = 0;
    int num_problem_unfeasible     = 0;

    // Initialize random seed
    srand(time(NULL));

    // Test loop
    for (int k=0; k<NUM_TEST; k++)
    {
        /** Generate problem data */
        int numVar          = rand() % MAX_NUM_VAR + 1;
        int numConstraint   = rand() % MAX_NUM_CONSTRAINT + 1;
        int numQConstraint  = 0;
        int numEqConstraint = rand() % MAX_NUM_CONSTRAINT + 1;

        double convergence_tolerance_QP = 1.0e-6;
        double convergence_tolerance_QCP = 1.0e-6;
        double optimality_tolerance = 1.0e-6;
        double feasibility_tolerance = 1.0e-6;

        MPCsolver::solverType solver_algo = MPCsolver::DUAL;

        MatrixXd T = MatrixXd::Random(numVar,numVar) + MatrixXd::Constant(numVar,numVar,1.0);
        VectorXd eigen = VectorXd::Random(numVar) + VectorXd::Constant(numVar,1.0);
        MatrixXd H(numVar,numVar);
        H = T*eigen.asDiagonal()*T.transpose();

        VectorXd f = VectorXd::Random(numVar);

        double Hnorm = H.norm();
        for (int i=0; i<H.rows(); i++)
        {
            for (int j=0; j<H.cols(); j++)
            {
                H(i,j) = round(H(i,j)/Hnorm*pow(10.0,MAX_DECIMAL))/pow(10.0,MAX_DECIMAL);
            }
            
            f(i) = round(f(i)/Hnorm*pow(10.0,MAX_DECIMAL))/pow(10.0,MAX_DECIMAL);
        }
        H = 0.5*(H+H.transpose());
        H = H+numVar*MatrixXd::Identity(numVar,numVar);

        VectorXd lBvect = VectorXd::Random(numVar)*10.0 - VectorXd::Constant(numVar,10.0);
        VectorXd uBvect = VectorXd::Random(numVar)*10.0 + VectorXd::Constant(numVar,10.0);
        for (int i=0; i<lBvect.size(); i++)
        {
            lBvect(i) = round(lBvect(i)*pow(10.0,MAX_DECIMAL))/pow(10.0,MAX_DECIMAL);
            uBvect(i) = round(uBvect(i)*pow(10.0,MAX_DECIMAL))/pow(10.0,MAX_DECIMAL);
        }
        std::vector<double> lB(lBvect.data(), lBvect.data() + lBvect.rows() * lBvect.cols());
        std::vector<double> uB(uBvect.data(), uBvect.data() + uBvect.rows() * uBvect.cols());

        MatrixXd Ain = MatrixXd::Random(numConstraint,numVar);
        VectorXd Bin = VectorXd::Random(numConstraint);
        
        for (int i=0; i<Ain.rows(); i++)
        {
            Ain.row(i).normalize();
            Bin(i) = Bin(i)/Ain.row(i).norm();

            for (int j=0; j<Ain.cols(); j++)
            {
                Ain(i,j) = round(Ain(i,j)*pow(10.0,MAX_DECIMAL))/pow(10.0,MAX_DECIMAL);
            }
            Bin(i) = round(Bin(i)*pow(10.0,MAX_DECIMAL))/pow(10.0,MAX_DECIMAL);
        }

        MatrixXd Aeq = MatrixXd::Random(numEqConstraint,numVar);
        VectorXd Beq = VectorXd::Random(numEqConstraint);

        for (int i=0; i<Aeq.rows(); i++)
        {
            Aeq.row(i).normalize();
            Beq(i) = Beq(i)/Aeq.row(i).norm();

            for (int j=0; j<Aeq.cols(); j++)
            {
                Aeq(i,j) = round(Aeq(i,j)*pow(10.0,MAX_DECIMAL))/pow(10.0,MAX_DECIMAL);
            }
            Beq(i) = round(Beq(i)*pow(10.0,MAX_DECIMAL))/pow(10.0,MAX_DECIMAL);
        }

        /** CPLEX solve problem */
        solver = new CPLEXsolver(numVar, numConstraint, numEqConstraint, numQConstraint, CPLEXsolver::AUTO);

        solver->set_solverMethod(solver_algo);
        solver->set_solverParams(convergence_tolerance_QP, convergence_tolerance_QCP, optimality_tolerance, feasibility_tolerance);

        if (!solver->initProblem())
        {
            num_problem_notinitialized++;

            delete solver;
            solver = NULL;

            continue;
        }
        solver->set_printLevel(MPCsolver::NONE);

        if (!solver->setProblem(lB, uB, H, f, Ain, Bin, Aeq, Beq))
        {
            num_problem_notsetted++;

            delete solver;
            solver = NULL;

            continue;
        }

        VectorXd result_CPLEX(numVar);
        int optimizerStatus = -1;
        if (solver->solveProblem(result_CPLEX, optimizerStatus))
            num_problem_solved++;
        else
            num_problem_unfeasible++;

        if (solver)
        {
            delete solver;
            solver = NULL;
        }

        /** Generate Matlab script */
        optim_algo_t algo_type;
        switch (solver_algo) {
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

        char fileName[255];
        sprintf(fileName, "%s%d", "./script/test_", k+1);
        QP_writeMatlabScript(fileName, false, algo_type, algo_tol, lB, uB, H, f, Ain, Bin, Aeq, Beq, result_CPLEX, optimizerStatus,
                             MAX_DECIMAL);

        std::cout << "Problem " << k+1 << "/" << NUM_TEST << " completed" << std::endl << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Num. problems:\t\t\t\t\t" << NUM_TEST << std::endl;
    std::cout << "Num. problems solved:\t\t\t\t" << num_problem_solved << std::endl;
    std::cout << "Num. problems with initialization errors:\t" << num_problem_notinitialized << std::endl;
    std::cout << "Num. problems with setting up errors:\t\t" << num_problem_notsetted << std::endl;
    std::cout << "Num. problems unfeasible:\t\t\t" << num_problem_unfeasible << std::endl << std::endl;

    return 0;
}
