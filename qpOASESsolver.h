#ifndef QPOASESSOLVER_H_
#define QPOASESSOLVER_H_

#include "MPCsolver.h"

#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES;


class qpOASESsolver : public MPCsolver
{
public:
    qpOASESsolver(int numVariable, int numConstraint, int numWorkingsetComputation = 100);
    qpOASESsolver();
    ~qpOASESsolver();

    bool initProblem();
    bool setProblem(const Ref<const VectorXd> lowerBound, const Ref<const VectorXd> upperBound, const Ref<const MatrixXd> hessian,
                    const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B);
    bool solveProblem(Ref<VectorXd> result, int& optimizerStatus);

    void set_printLevel(printLevelType printLevel);
private:
    bool              _qpOASESInitialized;

    SQProblem _problem;                 /** QPOASES problem */
    int_t _numWorkingsetComputation;    /** Maximum number of working set computations */
};


#endif /* QPOASESSOLVER_H_ */
