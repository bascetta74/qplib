#ifndef MPCSOLVER_H_
#define MPCSOLVER_H_

#include <string>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;


class MPCsolver
{
public:
    enum printLevelType {NONE, LOW, MEDIUM, HIGH};
    enum solverType {AUTO, PRIMAL, DUAL, NETWORK, BARRIER, SIFTING, CONCURRENT};

    virtual bool initProblem() =0;
    virtual bool setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient) =0;
    virtual bool setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B) =0;
    virtual bool setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B,
                            const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r) =0;
    virtual bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                            const Ref<const VectorXd> gradient) =0;
    virtual bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                            const Ref<const VectorXd> gradient, const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r) =0;
    virtual bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                            const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B) =0;
    virtual bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                    const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B,
                    const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r) =0;
    virtual bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                            const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B,
                            const Ref<const MatrixXd> Aeq, const Ref<const VectorXd> Beq) =0;
    virtual bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                            const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B, const Ref<const MatrixXd> Aeq,
                            const Ref<const VectorXd> Beq, const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r) =0;

    virtual bool solveProblem(Ref<VectorXd> result, int& optimizerStatus) =0;
    virtual void saveProblem(std::string filename) =0;

    int get_numVariable() { return _numVariable; }
    int get_numEqConstraint() { return _numEqConstraint; }
    int get_numIneqConstraint() { return _numIneqConstraint; }

    virtual solverType get_solverMethod() =0;

    virtual void set_solverMethod(const solverType solver) =0;
    virtual void set_solverParams(const double convergence_tolerance_QP, const double convergence_tolerance_QCP,
                                  const double optimality_tolerance, const double feasibility_tolerance) =0;
    virtual void set_printLevel(const printLevelType printLevel) =0;

protected:
    int _numVariable;
    int _numEqConstraint;
    int _numIneqConstraint;
    int _numQIneqConstraint;

    double _convergenceTolQP, _convergenceTolQCP;
    double _optimalityTol;
    double _feasibilityTol;
};

#endif /* MPCSOLVER_H_ */
