#ifndef WRITEMATLABSCRIPT_H_
#define WRITEMATLABSCRIPT_H_

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>

using namespace Eigen;


void QP_writeMatlabScript(std::string name, bool clearStatements, const Ref<const MatrixXd> H, const Ref<const MatrixXd> f, const Ref<const VectorXd> solution, int exitFlag);
void QP_writeMatlabScript(std::string name, bool clearStatements, const Ref<const MatrixXd> H, const Ref<const VectorXd> f, const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin,
                           const Ref<const VectorXd> solution, int exitFlag);
void QCP_writeMatlabScript(std::string name, bool clearStatements, const Ref<const MatrixXd> H, const Ref<const VectorXd> f, const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin,
                           const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r, const Ref<const VectorXd> solution, int exitFlag);
void QP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                           const Ref<const VectorXd> solution, int exitFlag);
void QCP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                           const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r, const Ref<const VectorXd> solution, int exitFlag);
void QP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                           const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin, const Ref<const VectorXd> solution, int exitFlag);
void QCP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                           const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin, const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r,
                           const Ref<const VectorXd> solution, int exitFlag);
void QP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                           const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin, const Ref<const MatrixXd> Aeq, const Ref<const VectorXd> Beq,
                           const Ref<const VectorXd> solution, int exitFlag);
void QCP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                           const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin, const Ref<const MatrixXd> Aeq, const Ref<const VectorXd> Beq, 
                           const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r, const Ref<const VectorXd> solution, int exitFlag);

void writeVector(const Ref<const VectorXd> vect, std::string name, std::ofstream& stream);
void writeVector(const std::vector<double>& vect, std::string name, std::ofstream& stream);
void writeMatrix(const Ref<const MatrixXd> mat, std::string name, std::ofstream& stream);

#endif /* WRITEMATLABSCRIPT_H_ */
