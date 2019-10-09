#ifndef CPLEXSOLVER_CWRAPPER_H_
#define CPLEXSOLVER_CWRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

int init_CPLEXsolver_cwrapper(const int numVariable, const int numIneqConstraint, const int numEqConstraint, const int numQIneqConstraint);
int deinit_CPLEXsolver_cwrapper();

int initProblem();

int setProblem(double* hessian_mat, double* gradient_vect);
int setProblem_ineqConst(double* hessian_mat, double* gradient_vect, double* A_mat, double* B_vect);
int setProblem_bound(double* hessian_mat, double* gradient_vect, double* lowerbound_vect, double* upperbound_vect);
int setProblem_ineqConst_bound(double* hessian_mat, double* gradient_vect, double* A_mat, double* B_vect, double* lowerbound_vect, double* upperbound_vect);
int setProblem_ineqConst_eqConst_bound(double* hessian_mat, double* gradient_vect, double* A_mat, double* B_vect, double* Aeq_mat, double* Beq_vect,
        double* lowerbound_vect, double* upperbound_vect);

int solveProblem(double* result_vect);

#ifdef __cplusplus
}
#endif

#endif /* CPLEXSOLVER_CWRAPPER_H_ */