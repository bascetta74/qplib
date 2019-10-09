#ifndef CPLEXSOLVER_H_
#define CPLEXSOLVER_H_

#include "MPCsolver.h"

#include <string>
#include <ilcplex/ilocplex.h>


class CPLEXsolver : public MPCsolver
{
public:
    enum solverType {AUTO, PRIMAL, DUAL, NETWORK, BARRIER, SIFTING, CONCURRENT};

    CPLEXsolver(const int numVariable, const int numIneqConstraint, const int numEqConstraint, const int numQIneqConstraint, const solverType solverMethod);
    CPLEXsolver(const int numVariable, const int numIneqConstraint, const int numEqConstraint, const int numQIneqConstraint);
    CPLEXsolver();
    ~CPLEXsolver();

    bool initProblem();

    bool setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient);                                                                         // H, f
    bool setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B);               // H, f, Ain, Bin
    bool setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B,                // H, f, Ain, Bin, l, Q, r
                    const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r);
    bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,                                // H, f, lB. uB
                    const Ref<const VectorXd> gradient);
    bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,                                // H, f, lB, uB, l, Q, r
                    const Ref<const VectorXd> gradient, const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r);
    bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,                                // H, f, lB. uB, Ain, Bin
                    const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B);
    bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,                                // H, f, lB. uB, Ain, Bin, l, Q, r
                    const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B,
                    const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r);
    bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,                                // H, f, lB. uB, Ain, Bin, 
                    const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B, const Ref<const MatrixXd> Aeq,                    // Aeq, Beq
                    const Ref<const VectorXd> Beq);
    bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,                                // H, f, lB. uB, Ain, Bin,
                    const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B, const Ref<const MatrixXd> Aeq,                    // Aeq, Beq, l, Q, r
                    const Ref<const VectorXd> Beq, const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r);

    bool solveProblem(Ref<VectorXd> result, int& optimizerStatus);

    void saveProblem(const std::string filename);

    void set_printLevel(const printLevelType printLevel);

    void set_solverMethod(const solverType solverMethod) { _solverMethod = solverMethod; }
    int get_solverMethod() { return _solverMethod; }

private:
    solverType        _solverMethod;
    bool              _IloInitialized;

    IloEnv            _IloEnv;          /** Ilo environment */
    IloModel          _IloModel;        /** Ilo model */
    IloNumVarArray    _IloVar;          /** Ilo unknowns */
    IloRangeArray     _IloConstrEq;     /** Ilo inequality constraints */
    IloRangeArray     _IloConstrIneq;   /** Ilo inequality constraints */
    IloObjective      _IloObj;		    /** Ilo objective function */
    IloCplex          _IloCplex;	    /** Cplex environment */
};

#endif /* CPLEXSOLVER_H_ */
