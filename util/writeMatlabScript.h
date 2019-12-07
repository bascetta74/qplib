#ifndef WRITEMATLABSCRIPT_H_
#define WRITEMATLABSCRIPT_H_

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>

typedef enum { AUTO=0, PRIMAL=1, DUAL=2, NETWORK=3, BARRIER=4, SIFTING=5, CONCURRENT=6 } optim_algo_t;
typedef struct {
    double optimality_tolerance;
    double feasibility_tolerance;
    double QP_convergence_tolerance;
    double QCP_convergence_tolerance;
} optim_algo_tol_t;

void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol,
                           const Eigen::Ref<const Eigen::MatrixXd> H, const Eigen::Ref<const Eigen::MatrixXd> f, 
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal);
void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol,
                           const Eigen::Ref<const Eigen::MatrixXd> H, const Eigen::Ref<const Eigen::VectorXd> f,
                           const Eigen::Ref<const Eigen::MatrixXd> Ain, const Eigen::Ref<const Eigen::VectorXd> Bin, 
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal);
void QCP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol,
                           const Eigen::Ref<const Eigen::MatrixXd> H, const Eigen::Ref<const Eigen::VectorXd> f, 
                           const Eigen::Ref<const Eigen::MatrixXd> Ain, const Eigen::Ref<const Eigen::VectorXd> Bin, 
                           const std::vector<Eigen::VectorXd>& l, const std::vector<Eigen::MatrixXd> Q, const std::vector<double>& r, 
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal);
void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol,
                           const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                           const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag,
                           int nDecimal);
void QCP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol,
                           const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                           const Eigen::Ref<const Eigen::VectorXd> f, const std::vector<Eigen::VectorXd>& l, 
                           const std::vector<Eigen::MatrixXd> Q, const std::vector<double>& r, 
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal);
void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol,
                           const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                           const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::MatrixXd> Ain, 
                           const Eigen::Ref<const Eigen::VectorXd> Bin, const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag,
                           int nDecimal);
void QCP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol,
                           const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                           const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::MatrixXd> Ain, 
                           const Eigen::Ref<const Eigen::VectorXd> Bin, const std::vector<Eigen::VectorXd>& l, 
                           const std::vector<Eigen::MatrixXd> Q, const std::vector<double>& r,
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal);
void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol,
                           const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                           const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::MatrixXd> Ain, 
                           const Eigen::Ref<const Eigen::VectorXd> Bin, const Eigen::Ref<const Eigen::MatrixXd> Aeq, 
                           const Eigen::Ref<const Eigen::VectorXd> Beq, const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag,
                           int nDecimal);
void QCP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol, 
                           const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                           const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::MatrixXd> Ain, 
                           const Eigen::Ref<const Eigen::VectorXd> Bin, const Eigen::Ref<const Eigen::MatrixXd> Aeq, 
                           const Eigen::Ref<const Eigen::VectorXd> Beq, const std::vector<Eigen::VectorXd>& l, 
                           const std::vector<Eigen::MatrixXd> Q, const std::vector<double>& r, 
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal);

void writeVector(const Eigen::Ref<const Eigen::VectorXd> vect, std::string name, int nDecimal, std::ofstream& stream);
void writeVector(const std::vector<double>& vect, std::string name, int nDecimal, std::ofstream& stream);
void writeMatrix(const Eigen::Ref<const Eigen::MatrixXd> mat, std::string name, int nDecimal, std::ofstream& stream);

#endif /* WRITEMATLABSCRIPT_H_ */
