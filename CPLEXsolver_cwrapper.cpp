#include "CPLEXsolver_cwrapper.h"
#include "MPCsolver.h"
#include "CPLEXsolver.h"

#include <cstddef>
#include <iostream>
#include <Eigen/Dense>

//#define DEBUG

extern "C" {
    /* Internal function prototypes */
    void doublePtr_eigenMatrix(double* mat, int row, int col, Ref<MatrixXd> eigen_mat);
    void doublePtr_eigenVector(double* vect, int length, Ref<VectorXd> eigen_vect);
    void doublePtr_stdVector(double* vect, int length, std::vector<double> std_vect);
    void eigenVector_doublePtr(const Ref<const VectorXd> eigen_vect, double* vect, int length);
    void print_eigenMatrix(const Ref<const MatrixXd> eigen_mat, char matrix_name[]);
    void print_eigenVector(const Ref<const VectorXd> eigen_vect, char vector_name[]);
    void print_stdVector(std::vector<double> std_vect, char vector_name[]);

    /* Wrapper global variables */
    typedef void* CCPLEXsolver;
    CCPLEXsolver solver = NULL;

    int numOptimVar = 0;
    int numConstEq = 0;
    int numConstIneq = 0;

    /* Wrapper initialization functions */
    int init_CPLEXsolver_cwrapper(const int numVariable, const int numIneqConstraint, const int numEqConstraint, const int numQIneqConstraint)
    {
        if (solver) {
            return -1;
        } else {
            solver = reinterpret_cast<void*>(new CPLEXsolver(numVariable, numIneqConstraint, numEqConstraint, numQIneqConstraint));
            numOptimVar = numVariable;
            numConstEq = numEqConstraint;
            numConstIneq = numIneqConstraint;

#ifdef DEBUG
            printf("\n --> CPLEXsolver_cwrapper: initializing wrapper\n");
            printf("numVariable: %d - numIneqConstraint: %d - numEqConstraint: %d\n", numVariable, numIneqConstraint, numEqConstraint, numQIneqConstraint);
#endif
        }

        return 1;
    }

    int deinit_CPLEXsolver_cwrapper()
    {
        if (!solver) {
            return -1;
        } else {
            delete reinterpret_cast<CPLEXsolver*>(solver);

            solver = NULL;
            numOptimVar = 0;

#ifdef DEBUG
            printf("\n --> CPLEXsolver_cwrapper: deinitializing wrapper\n");
#endif
        }

        return 1;
    }

    /* Library functions */
    int initProblem()
    {
        if (!solver) {
            return -1;
        }

#ifdef DEBUG       
        printf("\n --> CPLEXsolver_cwrapper: initializing QP problem\n");
#endif

        return reinterpret_cast<CPLEXsolver*>(solver)->initProblem();
    }

    int setProblem(double* hessian_mat, double* gradient_vect)
    {
        /* Convert matrix/vector to eigen */
        Eigen::MatrixXd hessian(numOptimVar, numOptimVar);
        doublePtr_eigenMatrix(hessian_mat, numOptimVar, numOptimVar, hessian);

        Eigen::VectorXd gradient(numOptimVar);
        doublePtr_eigenVector(gradient_vect, numOptimVar, gradient);

#ifdef DEBUG
        printf("\n --> CPLEXsolver_cwrapper: setting QP problem (hessian, gradient)\n");

        char H_name[] = "H";
        char g_name[] = "f";
        print_eigenMatrix(hessian, H_name);
        print_eigenVector(gradient, g_name);
#endif

        if (!solver) {
            return -1;
        }
        
        return reinterpret_cast<CPLEXsolver*>(solver)->setProblem(hessian, gradient);
    }

    int setProblem_ineqConst(double* hessian_mat, double* gradient_vect, double* A_mat, double* B_vect)
    {
        /* Convert matrix/vector to eigen */
        Eigen::MatrixXd hessian(numOptimVar, numOptimVar);
        doublePtr_eigenMatrix(hessian_mat, numOptimVar, numOptimVar, hessian);

        Eigen::VectorXd gradient(numOptimVar);
        doublePtr_eigenVector(gradient_vect, numOptimVar, gradient);

        Eigen::MatrixXd A(numConstIneq, numOptimVar);
        doublePtr_eigenMatrix(A_mat, numConstIneq, numOptimVar, A);

        Eigen::VectorXd B(numConstIneq);
        doublePtr_eigenVector(B_vect, numConstIneq, B);

#ifdef DEBUG
        printf("\n --> CPLEXsolver_cwrapper: setting QP problem (hessian, gradient, A, b)\n");

        char H_name[] = "H";
        char g_name[] = "f";
        print_eigenMatrix(hessian, H_name);
        print_eigenVector(gradient, g_name);

        char A_name[] = "A";
        char b_name[] = "b";
        print_eigenMatrix(A, A_name);
        print_eigenVector(B, b_name);
#endif

        if (!solver) {
            return -1;
        }
        
        return reinterpret_cast<CPLEXsolver*>(solver)->setProblem(hessian, gradient, A, B);
    }

    int setProblem_bound(double* hessian_mat, double* gradient_vect, double* lowerbound_vect, double* upperbound_vect)
    {
        /* Convert matrix/vector to eigen */
        Eigen::MatrixXd hessian(numOptimVar, numOptimVar);
        doublePtr_eigenMatrix(hessian_mat, numOptimVar, numOptimVar, hessian);

        Eigen::VectorXd gradient(numOptimVar);
        doublePtr_eigenVector(gradient_vect, numOptimVar, gradient);

        std::vector<double> lowerBound;
        doublePtr_stdVector(lowerbound_vect, numOptimVar, lowerBound);

        std::vector<double> upperBound;
        doublePtr_stdVector(upperbound_vect, numOptimVar, upperBound);

#ifdef DEBUG
        printf("\n --> CPLEXsolver_cwrapper: setting QP problem (hessian, gradient, lower bound, upper bound)\n");

        char H_name[] = "H";
        char g_name[] = "f";
        print_eigenMatrix(hessian, H_name);
        print_eigenVector(gradient, g_name);

        char ub_name[] = "lower bound";
        char lb_name[] = "upper bound";
        print_stdVector(lowerBound, ub_name);
        print_stdVector(upperBound, lb_name);
#endif

        if (!solver) {
            return -1;
        }
        
        return reinterpret_cast<CPLEXsolver*>(solver)->setProblem(lowerBound, upperBound, hessian, gradient);
    }

    int setProblem_ineqConst_bound(double* hessian_mat, double* gradient_vect, double* A_mat, double* B_vect, double* lowerbound_vect, double* upperbound_vect)
    {
        /* Convert matrix/vector to eigen */
        Eigen::MatrixXd hessian(numOptimVar, numOptimVar);
        doublePtr_eigenMatrix(hessian_mat, numOptimVar, numOptimVar, hessian);

        Eigen::VectorXd gradient(numOptimVar);
        doublePtr_eigenVector(gradient_vect, numOptimVar, gradient);

        Eigen::MatrixXd A(numConstIneq, numOptimVar);
        doublePtr_eigenMatrix(A_mat, numConstIneq, numOptimVar, A);

        Eigen::VectorXd B(numConstIneq);
        doublePtr_eigenVector(B_vect, numConstIneq, B);

        std::vector<double> lowerBound;
        doublePtr_stdVector(lowerbound_vect, numOptimVar, lowerBound);

        std::vector<double> upperBound;
        doublePtr_stdVector(upperbound_vect, numOptimVar, upperBound);

#ifdef DEBUG
        printf("\n --> CPLEXsolver_cwrapper: setting QP problem (hessian, gradient, lower bound, upper bound, A, b)\n");

        char H_name[] = "H";
        char g_name[] = "f";
        print_eigenMatrix(hessian, H_name);
        print_eigenVector(gradient, g_name);

        char ub_name[] = "lower bound";
        char lb_name[] = "upper bound";
        print_stdVector(lowerBound, ub_name);
        print_stdVector(upperBound, lb_name);

        char A_name[] = "A";
        char b_name[] = "b";
        print_eigenMatrix(A, A_name);
        print_eigenVector(B, b_name);        
#endif

        if (!solver) {
            return -1;
        }
        
        return reinterpret_cast<CPLEXsolver*>(solver)->setProblem(lowerBound, upperBound, hessian, gradient, A, B);
    }

    int setProblem_ineqConst_eqConst_bound(double* hessian_mat, double* gradient_vect, double* A_mat, double* B_vect, double* Aeq_mat, double* Beq_vect,
        double* lowerbound_vect, double* upperbound_vect)
    {
        /* Convert matrix/vector to eigen */
        Eigen::MatrixXd hessian(numOptimVar, numOptimVar);
        doublePtr_eigenMatrix(hessian_mat, numOptimVar, numOptimVar, hessian);

        Eigen::VectorXd gradient(numOptimVar);
        doublePtr_eigenVector(gradient_vect, numOptimVar, gradient);

        Eigen::MatrixXd A(numConstIneq, numOptimVar);
        doublePtr_eigenMatrix(A_mat, numConstIneq, numOptimVar, A);

        Eigen::VectorXd B(numConstIneq);
        doublePtr_eigenVector(B_vect, numConstIneq, B);

        Eigen::MatrixXd Aeq(numConstEq, numOptimVar);
        doublePtr_eigenMatrix(Aeq_mat, numConstEq, numOptimVar, Aeq);

        Eigen::VectorXd Beq(numConstEq);
        doublePtr_eigenVector(Beq_vect, numConstEq, Beq);

        std::vector<double> lowerBound;
        doublePtr_stdVector(lowerbound_vect, numOptimVar, lowerBound);

        std::vector<double> upperBound;
        doublePtr_stdVector(upperbound_vect, numOptimVar, upperBound);

#ifdef DEBUG
        printf("\n --> CPLEXsolver_cwrapper: setting QP problem (hessian, gradient, lower bound, upper bound, A, b)\n");

        char H_name[] = "H";
        char g_name[] = "f";
        print_eigenMatrix(hessian, H_name);
        print_eigenVector(gradient, g_name);

        char ub_name[] = "lower bound";
        char lb_name[] = "upper bound";
        print_stdVector(lowerBound, ub_name);
        print_stdVector(upperBound, lb_name);

        char A_name[] = "A";
        char b_name[] = "b";
        print_eigenMatrix(A, A_name);
        print_eigenVector(B, b_name);        

        char Aeq_name[] = "Aeq";
        char beq_name[] = "beq";
        print_eigenMatrix(Aeq, Aeq_name);
        print_eigenVector(Beq, beq_name);        
#endif

        if (!solver) {
            return -1;
        }
        
        return reinterpret_cast<CPLEXsolver*>(solver)->setProblem(lowerBound, upperBound, hessian, gradient, A, B, Aeq, Beq);
    }

    int solveProblem(double* result_vect)
    {
        if (!solver) {
            return -1;
        }

        Eigen::VectorXd result(numOptimVar);
        int optimizerStatus;
        
        int res = reinterpret_cast<CPLEXsolver*>(solver)->solveProblem(result, optimizerStatus);
        if (res)
        {
            eigenVector_doublePtr(result, result_vect, numOptimVar);

#ifdef DEBUG
            printf("CPLEXsolver_cwrapper: solving QP problem\n");

            char sol_name[] = "solution";
            print_eigenVector(result, sol_name);
#endif

            return optimizerStatus;
        }

        return res;
    }

    /* Internal functions */
    void doublePtr_eigenMatrix(double* mat, int row, int col, Ref<MatrixXd> eigen_mat)
    {
        for(int j=0; j<col; j++)
        {
            for(int i=0; i<row; i++)
            {
                eigen_mat(i,j) = mat[j*row+i];
            }
        }
    }

    void doublePtr_eigenVector(double* vect, int length, Ref<VectorXd> eigen_vect)
    {
        for(int j=0; j<length; j++)
        {
            eigen_vect(j) = vect[j];
        }
    }

    void doublePtr_stdVector(double* vect, int length, std::vector<double> std_vect)
    {
        std_vect.clear();

        for(int j=0; j<length; j++)
        {
            std_vect.push_back(vect[j]);
        }
    }

    void eigenVector_doublePtr(const Ref<const VectorXd> eigen_vect, double* vect, int length)
    {
        for(int j=0; j<length; j++)
        {
            vect[j] = eigen_vect(j);
        }
    }

    void print_eigenMatrix(const Ref<const MatrixXd> eigen_mat, char matrix_name[])
    {
        printf("%s = [", matrix_name);
        for(int i=0; i<eigen_mat.rows(); i++)
        {
            for(int j=0; j<eigen_mat.cols(); j++)
            {
                printf("%.5f", eigen_mat(i,j));

                if ((j == eigen_mat.cols()-1) && (i != eigen_mat.rows()-1))
                    printf("; ");
                else if ((j != eigen_mat.cols()-1) || (i != eigen_mat.rows()-1))
                    printf(", ");
            }
        }
        printf("]\n");
    }

    void print_eigenVector(const Ref<const VectorXd> eigen_vect, char vector_name[])
    {
        printf("%s = [", vector_name);
        for(int j=0; j<eigen_vect.size(); j++)
        {
            printf("%.5f", eigen_vect(j));

            if (j != eigen_vect.size()-1)
                printf(", ");
        }
        printf("]\n");
    }

    void print_stdVector(std::vector<double> std_vect, char vector_name[])
    {
        printf("%s = [", vector_name);
        for(int j=0; j<std_vect.size(); j++)
        {
            printf("%.5f", std_vect.at(j));

            if (j != std_vect.size()-1)
                printf(", ");
        }
        printf("]\n");
    }
}