#include <sstream>
#include <time.h>

#include "MPCsolver.h"
#include "CPLEXsolver.h"
#include "writeMatlabScript.h"

#define NUM_TEST            1000
#define MAX_NUM_VAR         100
#define MAX_NUM_CONSTRAINT  50

using namespace std;

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
        int numEqConstraint = rand() % std::max(numVar / 2, 1) + 1;

        VectorXd lBvect = VectorXd::Random(numVar)*10.0 - VectorXd::Constant(numVar,10.0);
        VectorXd uBvect = VectorXd::Random(numVar)*10.0 + VectorXd::Constant(numVar,10.0);
        std::vector<double> lB(lBvect.data(), lBvect.data() + lBvect.rows() * lBvect.cols());
        std::vector<double> uB(uBvect.data(), uBvect.data() + uBvect.rows() * uBvect.cols());

        MatrixXd Q = MatrixXd::Random(numVar,numVar) + MatrixXd::Constant(numVar,numVar,1.0);
        VectorXd eigen = VectorXd::Random(numVar) + VectorXd::Constant(numVar,1.0);
        MatrixXd H(numVar,numVar);
        H = Q*eigen.asDiagonal()*Q.transpose();

        VectorXd f = VectorXd::Random(numVar);

        double Hnorm = H.norm();
        for (int i=0; i<H.rows(); i++)
        {
            for (int j=0; j<H.cols(); j++)
            {
                H(i,j) = H(i,j)/Hnorm;
            }
            
            f(i) = f(i)/Hnorm;
        }

        MatrixXd Ain = MatrixXd::Random(numConstraint,numVar);
        VectorXd Bin = VectorXd::Random(numConstraint);
        
        for (int i=0; i<Ain.rows(); i++)
        {
            Ain.row(i).normalize();
            Bin(i) = Bin(i)/Ain.row(i).norm();
        }

        MatrixXd Aeq = MatrixXd::Random(numEqConstraint,numVar);
        VectorXd Beq = VectorXd::Random(numEqConstraint);

        for (int i=0; i<Aeq.rows(); i++)
        {
            Aeq.row(i).normalize();
            Beq(i) = Beq(i)/Aeq.row(i).norm();
        }
        
        /** CPLEX solve problem */
        solver = new CPLEXsolver(numVar, numConstraint, numEqConstraint, CPLEXsolver::AUTO);

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
        char fileName[255];
        sprintf(fileName, "%s%d%s", "./script/test_", k+1, "_script.m");
        QP_writeMatlabScript(fileName, false, lB, uB, H, f, Ain, Bin, Aeq, Beq, result_CPLEX, optimizerStatus);

        cout << "Problem " << k+1 << "/" << NUM_TEST << " completed" << endl;
    }

    cout << endl;
    cout << "Num. problems:\t\t\t\t\t" << NUM_TEST << endl;
    cout << "Num. problems solved:\t\t\t\t" << num_problem_solved << endl;
    cout << "Num. problems with initialization errors:\t" << num_problem_notinitialized << endl;
    cout << "Num. problems with setting up errors:\t\t" << num_problem_notsetted << endl;
    cout << "Num. problems unfeasible:\t\t\t" << num_problem_unfeasible << endl << endl;

    return 0;
}
