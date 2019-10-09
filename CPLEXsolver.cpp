#include <iostream>

#include "CPLEXsolver.h"


CPLEXsolver::~CPLEXsolver()
{
    /** Destroy CPLEX environment */
    _IloEnv.end();
    _IloInitialized = false;
}

CPLEXsolver::CPLEXsolver(const int numVariable, const int numIneqConstraint, const int numEqConstraint, const int numQIneqConstraint, const solverType solverMethod)
{
    /** Initialize class variables */
    _numVariable        = numVariable;
    _numEqConstraint    = numEqConstraint;
    _numIneqConstraint  = numIneqConstraint;
    _numQIneqConstraint = numQIneqConstraint;
    _solverMethod       = solverMethod;
    _IloInitialized     = false;
}

CPLEXsolver::CPLEXsolver(const int numVariable, const int numIneqConstraint, const int numEqConstraint, const int numQIneqConstraint)
{
    /** Initialize class variables */
    _numVariable        = numVariable;
    _numEqConstraint    = numEqConstraint;
    _numIneqConstraint  = numIneqConstraint;
    _numQIneqConstraint = numQIneqConstraint;
    _solverMethod       = AUTO;
    _IloInitialized     = false;
}

CPLEXsolver::CPLEXsolver()
{
    /** Initialize class variables */
    _numVariable        = -1;
    _numEqConstraint    = -1;
    _numIneqConstraint  = -1;
    _numQIneqConstraint = -1;
    _solverMethod       = AUTO;
    _IloInitialized     = false;
}

bool CPLEXsolver::initProblem()
{
    /** Check the problem has been initialized */
    if ((_numVariable<0) || (_numEqConstraint<0) || (_numIneqConstraint<0) || (_numQIneqConstraint<0))
    {
        std::cout << "[CPLEXsolver] Number of variables/constraints not initialized" << std::endl;
        return false;
    }

    /** Create and initialize a CPLEX environment */
    try
    {
    	// Create environment variables
        _IloModel      = IloModel(_IloEnv);
        _IloVar        = IloNumVarArray(_IloEnv);
        _IloConstrIneq = IloRangeArray(_IloEnv);
        _IloConstrEq   = IloRangeArray(_IloEnv);

        // Create unknowns without lower/upper bounds
        for (int i=0; i<_numVariable; i++)
        	_IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        // Create a dummy objective function
        IloExpr objExpr(_IloEnv, 0.0);
        _IloObj = IloObjective(_IloEnv, objExpr, IloObjective::Minimize);
        _IloModel.add(_IloObj);
        objExpr.end();

        // Create a dummy set of equality constraints
        for (int i=0; i<_numEqConstraint; i++)
            _IloConstrEq.add(-1.0 <= _IloVar[0] <= 1.0);
        if (_numEqConstraint>0)
            _IloModel.add(_IloConstrEq);

        // Create a dummy problem
        _IloCplex = IloCplex(_IloModel);
        _IloInitialized = true;

        // Set Cplex solver
        switch(_solverMethod)
        {
        case AUTO:
        	_IloCplex.setParam(IloCplex::RootAlg, IloCplex::Auto);
        	_IloCplex.setParam(IloCplex::NodeAlg, IloCplex::Auto);
        	break;
        case PRIMAL:
        	_IloCplex.setParam(IloCplex::RootAlg, IloCplex::Primal);
        	_IloCplex.setParam(IloCplex::NodeAlg, IloCplex::Primal);
        	break;
        case DUAL:
        	_IloCplex.setParam(IloCplex::RootAlg, IloCplex::Dual);
        	_IloCplex.setParam(IloCplex::NodeAlg, IloCplex::Dual);
        	break;
        case NETWORK:
        	_IloCplex.setParam(IloCplex::RootAlg, IloCplex::Network);
        	_IloCplex.setParam(IloCplex::NodeAlg, IloCplex::Network);
        	break;
        case BARRIER:
        	_IloCplex.setParam(IloCplex::RootAlg, IloCplex::Barrier);
        	_IloCplex.setParam(IloCplex::NodeAlg, IloCplex::Barrier);
        	break;
        case SIFTING:
        	_IloCplex.setParam(IloCplex::RootAlg, IloCplex::Sifting);
        	_IloCplex.setParam(IloCplex::NodeAlg, IloCplex::Sifting);
        	break;
        case CONCURRENT:
        	_IloCplex.setParam(IloCplex::RootAlg, IloCplex::Concurrent);
        	_IloCplex.setParam(IloCplex::NodeAlg, IloCplex::Concurrent);
        	break;
        default:
        	_IloCplex.setParam(IloCplex::RootAlg, IloCplex::Auto);
        	_IloCplex.setParam(IloCplex::NodeAlg, IloCplex::Auto);
        }
    }
    catch (IloException& e)
    {
        std::cout << "[CPLEXsolver] Error initializing Ilog CPLEX:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
    }

    return true;
}


bool CPLEXsolver::setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient)
// Set an optimization problem characterized by cost function only
// Cost function        min 0.5*xT*H*x+fT*x
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before setProblem" << std::endl;
        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()<1) || (gradient.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty gradient" << std::endl;
        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a non square hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()!=hessian.rows()) || (gradient.cols()>1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a gradient having wrong dimensions" << std::endl;
        return false;
    }

    /** Set the QP problem */
    try
    {
        // Check and update the number of unknowns
        if (_numVariable>hessian.cols())
            _IloVar.remove(_IloVar.getSize()-(_numVariable-hessian.cols()), _numVariable-hessian.cols());
        else if (_numVariable<hessian.cols())
            for (int i=0; i<hessian.cols()-_numVariable; i++)
                _IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        _numVariable = _IloVar.getSize();

    	// Remove unknowns lower/upper bounds
    	for (int i=0; i<_numVariable; i++)
    		_IloVar[i].setBounds(-IloInfinity, +IloInfinity);

        // Check and remove inequality constraints
        if (_numIneqConstraint>0)
        {
            _IloModel.remove(_IloConstrIneq);
            _IloConstrIneq.endElements();

            _numIneqConstraint = 0;
        }

        // Check and remove equality constraints
        if (_numEqConstraint>0)
        {
            _IloModel.remove(_IloConstrEq);
            _IloConstrEq.endElements();

            _numEqConstraint = 0;
        }

    	// Modify the objective function
        IloExpr objExpr(_IloEnv, 0.0);

        for (int i=0; i<_numVariable; i++)			// Quadratic part of the objective function
        	for (int j=i; j<_numVariable; j++)
        	{
        		if (i==j)
        			objExpr += 0.5*hessian(i,j)*_IloVar[i]*_IloVar[j];
        		else
        			objExpr += hessian(i,j)*_IloVar[i]*_IloVar[j];
        	}

        for (int i=0; i<_numVariable; i++)			// Linear part of the objective function
        	objExpr += gradient(i)*_IloVar[i];

        _IloObj.setExpr(objExpr);

        objExpr.end();
    }
    catch (IloException& e)
	{
        std::cout << "[CPLEXsolver] CPlex failed setting up the QP problem:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
	}

	return true;
}


bool CPLEXsolver::setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B)
// Set an optimization problem characterized by cost function and linear inequality constraints
// Cost function           min 0.5*xT*H*x+fT*x
// Inequality constraints       Ax <= B
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before setProblem" << std::endl;
        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()<1) || (gradient.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty gradient" << std::endl;
        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a non square hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()!=hessian.rows()) || (gradient.cols()>1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a gradient having wrong dimensions" << std::endl;
        return false;
    }
    if ((A.rows()<1) || (A.cols()<1) || (B.rows()<1) || (B.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of inequality constraints" << std::endl;
        return false;
    }
    if ((A.cols()!=hessian.rows()) || (A.rows()!=B.rows()) || (B.cols()!=1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a set of inequality constraints having wrong dimensions" << std::endl;
        return false;
    }

    /** Set the QP problem */
    try
    {
        // Check and update the number of optimization variables
        if (_numVariable>hessian.cols())
            _IloVar.remove(_IloVar.getSize()-(_numVariable-hessian.cols()), _numVariable-hessian.cols());
        else if (_numVariable<hessian.cols())
            for (int i=0; i<hessian.cols()-_numVariable; i++)
                _IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        _numVariable = _IloVar.getSize();

    	// Remove unknowns lower/upper bounds
    	for (int i=0; i<_numVariable; i++)
    		_IloVar[i].setBounds(-IloInfinity, +IloInfinity);

        // Check and remove equality constraints
        if (_numEqConstraint>0)
        {
            _IloModel.remove(_IloConstrEq);
            _IloConstrEq.endElements();

            _numEqConstraint = 0;
        }

        // Modify the objective function
        IloExpr objExpr(_IloEnv, 0.0);

        for (int i=0; i<_numVariable; i++)			// Quadratic part of the objective function
        	for (int j=i; j<_numVariable; j++)
        	{
        		if (i==j)
        			objExpr += 0.5*hessian(i,j)*_IloVar[i]*_IloVar[j];
        		else
        			objExpr += hessian(i,j)*_IloVar[i]*_IloVar[j];
        	}

        for (int i=0; i<_numVariable; i++)			// Linear part of the objective function
        	objExpr += gradient(i)*_IloVar[i];

        _IloObj.setExpr(objExpr);

        objExpr.end();

    	// Add the inequality constraints
    	if (_numIneqConstraint!=A.rows())
    	{
            // Remove previous constraints
            _IloModel.remove(_IloConstrIneq);
            _IloConstrIneq.endElements();

            _numIneqConstraint = A.rows();
    	}

        for (int i=0; i<_numIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

      		for (int j=0; j<_numVariable; j++)
      			conExpr += A(i,j)*_IloVar[j];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, B(i)));

        	conExpr.end();
        }

       _IloModel.add(_IloConstrIneq);
    }
    catch (IloException& e)
	{
        std::cout << "[CPLEXsolver] CPlex failed setting up the QP problem:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
	}

	return true;
}


bool CPLEXsolver::setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B,
                    const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r)
// Set an optimization problem characterized by cost function, linear inequality constraints and quadratic inequality constraints
// Cost function                     min 0.5*xT*H*x+fT*x
// Linear inequality constraints         Ax <= B
// Quadratic inequality constraints      aTi*x+xT*Qi*x <= ri   i=1,...,q
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before setProblem" << std::endl;
        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()<1) || (gradient.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty gradient" << std::endl;
        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a non square hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()!=hessian.rows()) || (gradient.cols()>1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a gradient having wrong dimensions" << std::endl;
        return false;
    }
    if ((A.rows()<1) || (A.cols()<1) || (B.rows()<1) || (B.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of linear inequality constraints" << std::endl;
        return false;
    }
    if ((A.cols()!=hessian.rows()) || (A.rows()!=B.rows()) || (B.cols()!=1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a set of linear inequality constraints having wrong dimensions" << std::endl;
        return false;
    }
    if ((l.size()!=_numQIneqConstraint) || (Q.size()!=_numQIneqConstraint) || (r.size()!=_numQIneqConstraint))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong number of quadratic inequality constraints" << std::endl;
        return false;
    }
    for (int k=0; k<l.size(); k++)
    {
        if ((l.at(k).rows()<1) || (l.at(k).cols()<1) || (Q.at(k).rows()<1) || (Q.at(k).cols()<1))
        {
            std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of quadratic inequality constraints" << std::endl;
            return false;
        }
        if ((l.at(k).rows()!=hessian.rows()) || (l.at(k).cols()!=1) || (Q.at(k).rows()!=hessian.rows()) || (Q.at(k).cols()!=hessian.rows()))
        {
            std::cout << "[CPLEXsolver] Cannot setProblem with a set of quadratic inequality constraints having wrong dimensions" << std::endl;
            return false;
        }
    }

    /** Set the QP problem */
    try
    {
        // Check and update the number of optimization variables
        if (_numVariable>hessian.cols())
            _IloVar.remove(_IloVar.getSize()-(_numVariable-hessian.cols()), _numVariable-hessian.cols());
        else if (_numVariable<hessian.cols())
            for (int i=0; i<hessian.cols()-_numVariable; i++)
                _IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        _numVariable = _IloVar.getSize();

    	// Remove unknowns lower/upper bounds
    	for (int i=0; i<_numVariable; i++)
    		_IloVar[i].setBounds(-IloInfinity, +IloInfinity);

        // Check and remove equality constraints
        if (_numEqConstraint>0)
        {
            _IloModel.remove(_IloConstrEq);
            _IloConstrEq.endElements();

            _numEqConstraint = 0;
        }

        // Modify the objective function
        IloExpr objExpr(_IloEnv, 0.0);

        for (int i=0; i<_numVariable; i++)			// Quadratic part of the objective function
        	for (int j=i; j<_numVariable; j++)
        	{
        		if (i==j)
        			objExpr += 0.5*hessian(i,j)*_IloVar[i]*_IloVar[j];
        		else
        			objExpr += hessian(i,j)*_IloVar[i]*_IloVar[j];
        	}

        for (int i=0; i<_numVariable; i++)			// Linear part of the objective function
        	objExpr += gradient(i)*_IloVar[i];

        _IloObj.setExpr(objExpr);

        objExpr.end();

    	// Add the inequality constraints
    	if ((_numIneqConstraint+_numQIneqConstraint)!=(A.rows()+r.size()))
    	{
            // Remove previous constraints
            _IloModel.remove(_IloConstrIneq);
            _IloConstrIneq.endElements();

            _numIneqConstraint  = A.rows();
            _numQIneqConstraint = r.size();
    	}

        for (int i=0; i<_numIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

      		for (int j=0; j<_numVariable; j++)
      			conExpr += A(i,j)*_IloVar[j];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, B(i)));

        	conExpr.end();
        }

        for (int i=0; i<_numQIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

            // Linear part of the constraint aiT*x
      		for (int j=0; j<_numVariable; j++)
      			conExpr += l.at(i)(j)*_IloVar[j];

            // Quadratic part of the constraint xT*Q*x
            for (int j=0; j<_numVariable; j++)
        	    for (int k=0; k<_numVariable; k++)
                    conExpr += Q.at(i)(j,k)*_IloVar[j]*_IloVar[k];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, r.at(i)));

        	conExpr.end();
        }

        _IloModel.add(_IloConstrIneq);
    }
    catch (IloException& e)
	{
        std::cout << "[CPLEXsolver] CPlex failed setting up the QP problem:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
	}

	return true;
}


bool CPLEXsolver::setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                             const Ref<const VectorXd> gradient)
// Set an optimization problem characterized by cost function  and lower/upper bound
// Cost function        min 0.5*xT*H*x+fT*x
// Variable constraints     lb <= x <= ub
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before setProblem" << std::endl;
        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()<1) || (gradient.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty gradient" << std::endl;
        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a non square hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()!=hessian.rows()) || (gradient.cols()>1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a gradient having wrong dimensions" << std::endl;
        return false;
    }
    if ((lowerBound.size()!=hessian.rows()) || (upperBound.size()!=hessian.rows()))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong dimension on lower/upper bounds" << std::endl;
        return false;
    }

    /** Set the QP problem */
    try
    {
        // Check and update the number of optimization variables
        if (_numVariable>hessian.cols())
            _IloVar.remove(_IloVar.getSize()-(_numVariable-hessian.cols()), _numVariable-hessian.cols());
        else if (_numVariable<hessian.cols())
            for (int i=0; i<hessian.cols()-_numVariable; i++)
                _IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        _numVariable = _IloVar.getSize();

    	// Modify unknowns lower/upper bounds
    	for (int i=0; i<_numVariable; i++)
    		_IloVar[i].setBounds(lowerBound.at(i), upperBound.at(i));

        // Check and remove inequality constraints
        if (_numIneqConstraint>0)
        {
            _IloModel.remove(_IloConstrIneq);
            _IloConstrIneq.endElements();

            _numIneqConstraint = 0;
        }

        // Check and remove equality constraints
        if (_numEqConstraint>0)
        {
            _IloModel.remove(_IloConstrEq);
            _IloConstrEq.endElements();

            _numEqConstraint = 0;
        }

    	// Modify the objective function
        IloExpr objExpr(_IloEnv, 0.0);

        for (int i=0; i<_numVariable; i++)			// Quadratic part of the objective function
        	for (int j=i; j<_numVariable; j++)
        	{
        		if (i==j)
        			objExpr += 0.5*hessian(i,j)*_IloVar[i]*_IloVar[j];
        		else
        			objExpr += hessian(i,j)*_IloVar[i]*_IloVar[j];
        	}

        for (int i=0; i<_numVariable; i++)			// Linear part of the objective function
        	objExpr += gradient(i)*_IloVar[i];

        _IloObj.setExpr(objExpr);

        objExpr.end();
    }
    catch (IloException& e)
	{
        std::cout << "[CPLEXsolver] CPlex failed setting up the QP problem:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
	}

	return true;
}

bool CPLEXsolver::setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                const Ref<const VectorXd> gradient, const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r)
// Set an optimization problem characterized by cost function, lower/upper bound and quadratic constraints
// Cost function                              min 0.5*xT*H*x+fT*x
// Variable constraints                          lb <= x <= ub
// Quadratic inequality constraints      aTi*x+xT*Qi*x <= ri   i=1,...,q
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before setProblem" << std::endl;
        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()<1) || (gradient.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty gradient" << std::endl;
        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a non square hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()!=hessian.rows()) || (gradient.cols()>1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a gradient having wrong dimensions" << std::endl;
        return false;
    }
    if ((lowerBound.size()!=hessian.rows()) || (upperBound.size()!=hessian.rows()))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong dimension on lower/upper bounds" << std::endl;
        return false;
    }
    if ((l.size()!=_numQIneqConstraint) || (Q.size()!=_numQIneqConstraint) || (r.size()!=_numQIneqConstraint))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong number of quadratic inequality constraints" << std::endl;
        return false;
    }
    for (int k=0; k<l.size(); k++)
    {
        if ((l.at(k).rows()<1) || (l.at(k).cols()<1) || (Q.at(k).rows()<1) || (Q.at(k).cols()<1))
        {
            std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of quadratic inequality constraints" << std::endl;
            return false;
        }
        if ((l.at(k).rows()!=hessian.rows()) || (l.at(k).cols()!=1) || (Q.at(k).rows()!=hessian.rows()) || (Q.at(k).cols()!=hessian.rows()))
        {
            std::cout << "[CPLEXsolver] Cannot setProblem with a set of quadratic inequality constraints having wrong dimensions" << std::endl;
            return false;
        }
    }

    /** Set the QP problem */
    try
    {
        // Check and update the number of optimization variables
        if (_numVariable>hessian.cols())
            _IloVar.remove(_IloVar.getSize()-(_numVariable-hessian.cols()), _numVariable-hessian.cols());
        else if (_numVariable<hessian.cols())
            for (int i=0; i<hessian.cols()-_numVariable; i++)
                _IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        _numVariable = _IloVar.getSize();

    	// Modify unknowns lower/upper bounds
    	for (int i=0; i<_numVariable; i++)
    		_IloVar[i].setBounds(lowerBound.at(i), upperBound.at(i));

        // Check and remove equality constraints
        if (_numEqConstraint>0)
        {
            _IloModel.remove(_IloConstrEq);
            _IloConstrEq.endElements();

            _numEqConstraint = 0;
        }

    	// Modify the objective function
        IloExpr objExpr(_IloEnv, 0.0);

        for (int i=0; i<_numVariable; i++)			// Quadratic part of the objective function
        	for (int j=i; j<_numVariable; j++)
        	{
        		if (i==j)
        			objExpr += 0.5*hessian(i,j)*_IloVar[i]*_IloVar[j];
        		else
        			objExpr += hessian(i,j)*_IloVar[i]*_IloVar[j];
        	}

        for (int i=0; i<_numVariable; i++)			// Linear part of the objective function
        	objExpr += gradient(i)*_IloVar[i];

        _IloObj.setExpr(objExpr);

        objExpr.end();

    	// Update and modify the inequality constraints
    	if (_numQIneqConstraint!=r.size())
    	{
            // Remove previous constraints
            _IloModel.remove(_IloConstrIneq);
            _IloConstrIneq.endElements();

            _numQIneqConstraint = r.size();
    	}

        for (int i=0; i<_numQIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

            // Linear part of the constraint aiT*x
      		for (int j=0; j<_numVariable; j++)
      			conExpr += l.at(i)(j)*_IloVar[j];

            // Quadratic part of the constraint xT*Q*x
            for (int j=0; j<_numVariable; j++)
        	    for (int k=0; k<_numVariable; k++)
                    conExpr += Q.at(i)(j,k)*_IloVar[j]*_IloVar[k];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, r.at(i)));

        	conExpr.end();
        }

        _IloModel.add(_IloConstrIneq);
    }
    catch (IloException& e)
	{
        std::cout << "[CPLEXsolver] CPlex failed setting up the QP problem:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
	}

	return true;
}

bool CPLEXsolver::setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                             const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B)
// Set an optimization problem characterized by cost function and inequality constraints
// Cost function            min 0.5*xT*H*x+fT*x
// Variable constraints         lb <= x <= ub
// Inequality constraints       Ax <= B
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before setProblem" << std::endl;
        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()<1) || (gradient.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty gradient" << std::endl;
        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a non square hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()!=hessian.rows()) || (gradient.cols()>1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a gradient having wrong dimensions" << std::endl;
        return false;
    }
    if ((lowerBound.size()!=hessian.rows()) || (upperBound.size()!=hessian.rows()))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong dimension on lower/upper bounds" << std::endl;
        return false;
    }
    if ((A.rows()<1) || (A.cols()<1) || (B.rows()<1) || (B.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of inequality constraints" << std::endl;
        return false;
    }
    if ((A.cols()!=hessian.rows()) || (A.rows()!=B.rows()) || (B.cols()!=1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a set of inequality constraints having wrong dimensions" << std::endl;
        return false;
    }

    /** Set the QP problem */
    try
    {
        // Check and update the number of optimization variables
        if (_numVariable>hessian.cols())
            _IloVar.remove(_IloVar.getSize()-(_numVariable-hessian.cols()), _numVariable-hessian.cols());
        else if (_numVariable<hessian.cols())
            for (int i=0; i<hessian.cols()-_numVariable; i++)
                _IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        _numVariable = _IloVar.getSize();

    	// Modify unknowns lower/upper bounds
    	for (int i=0; i<_numVariable; i++)
    		_IloVar[i].setBounds(lowerBound.at(i), upperBound.at(i));

        // Check and remove equality constraints
        if (_numEqConstraint>0)
        {
            _IloModel.remove(_IloConstrEq);
            _IloConstrEq.endElements();

            _numEqConstraint = 0;
        }

    	// Modify the objective function
        IloExpr objExpr(_IloEnv, 0.0);

        for (int i=0; i<_numVariable; i++)			// Quadratic part of the objective function
        	for (int j=i; j<_numVariable; j++)
        	{
        		if (i==j)
        			objExpr += 0.5*hessian(i,j)*_IloVar[i]*_IloVar[j];
        		else
        			objExpr += hessian(i,j)*_IloVar[i]*_IloVar[j];
        	}

        for (int i=0; i<_numVariable; i++)			// Linear part of the objective function
        	objExpr += gradient(i)*_IloVar[i];

        _IloObj.setExpr(objExpr);

        objExpr.end();

    	// Add the inequality constraints
    	if (_numIneqConstraint!=A.rows())
    	{
            // Remove previous constraints
            _IloModel.remove(_IloConstrIneq);
            _IloConstrIneq.endElements();

            _numIneqConstraint = A.rows();
    	}

        for (int i=0; i<_numIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

      		for (int j=0; j<_numVariable; j++)
      			conExpr += A(i,j)*_IloVar[j];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, B(i)));

        	conExpr.end();
        }

        _IloModel.add(_IloConstrIneq);
    }
    catch (IloException& e)
	{
        std::cout << "[CPLEXsolver] CPlex failed setting up the QP problem:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
	}

	return true;
}

bool CPLEXsolver::setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                             const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B,
                             const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r)
// Set an optimization problem characterized by cost function and inequality constraints
// Cost function                           min 0.5*xT*H*x+fT*x
// Variable constraints                       lb <= x <= ub
// Linear inequality constraints              Ax <= B
// Quadratic inequality constraints    aTi*x+xT*Qi*x <= ri   i=1,...,q
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before setProblem" << std::endl;
        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()<1) || (gradient.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty gradient" << std::endl;
        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a non square hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()!=hessian.rows()) || (gradient.cols()>1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a gradient having wrong dimensions" << std::endl;
        return false;
    }
    if ((lowerBound.size()!=hessian.rows()) || (upperBound.size()!=hessian.rows()))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong dimension on lower/upper bounds" << std::endl;
        return false;
    }
    if ((A.rows()<1) || (A.cols()<1) || (B.rows()<1) || (B.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of inequality constraints" << std::endl;
        return false;
    }
    if ((A.cols()!=hessian.rows()) || (A.rows()!=B.rows()) || (B.cols()!=1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a set of inequality constraints having wrong dimensions" << std::endl;
        return false;
    }
    if ((l.size()!=_numQIneqConstraint) || (Q.size()!=_numQIneqConstraint) || (r.size()!=_numQIneqConstraint))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong number of quadratic inequality constraints" << std::endl;
        return false;
    }
    for (int k=0; k<l.size(); k++)
    {
        if ((l.at(k).rows()<1) || (l.at(k).cols()<1) || (Q.at(k).rows()<1) || (Q.at(k).cols()<1))
        {
            std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of quadratic inequality constraints" << std::endl;
            return false;
        }
        if ((l.at(k).rows()!=hessian.rows()) || (l.at(k).cols()!=1) || (Q.at(k).rows()!=hessian.rows()) || (Q.at(k).cols()!=hessian.rows()))
        {
            std::cout << "[CPLEXsolver] Cannot setProblem with a set of quadratic inequality constraints having wrong dimensions" << std::endl;
            return false;
        }
    }

    /** Set the QP problem */
    try
    {
        // Check and update the number of optimization variables
        if (_numVariable>hessian.cols())
            _IloVar.remove(_IloVar.getSize()-(_numVariable-hessian.cols()), _numVariable-hessian.cols());
        else if (_numVariable<hessian.cols())
            for (int i=0; i<hessian.cols()-_numVariable; i++)
                _IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        _numVariable = _IloVar.getSize();

    	// Modify unknowns lower/upper bounds
    	for (int i=0; i<_numVariable; i++)
    		_IloVar[i].setBounds(lowerBound.at(i), upperBound.at(i));

        // Check and remove equality constraints
        if (_numEqConstraint>0)
        {
            _IloModel.remove(_IloConstrEq);
            _IloConstrEq.endElements();

            _numEqConstraint = 0;
        }

    	// Modify the objective function
        IloExpr objExpr(_IloEnv, 0.0);

        for (int i=0; i<_numVariable; i++)			// Quadratic part of the objective function
        	for (int j=i; j<_numVariable; j++)
        	{
        		if (i==j)
        			objExpr += 0.5*hessian(i,j)*_IloVar[i]*_IloVar[j];
        		else
        			objExpr += hessian(i,j)*_IloVar[i]*_IloVar[j];
        	}

        for (int i=0; i<_numVariable; i++)			// Linear part of the objective function
        	objExpr += gradient(i)*_IloVar[i];

        _IloObj.setExpr(objExpr);

        objExpr.end();

    	// Add the inequality constraints
    	if ((_numIneqConstraint+_numQIneqConstraint)!=(A.rows()+r.size()))
    	{
            // Remove previous constraints
            _IloModel.remove(_IloConstrIneq);
            _IloConstrIneq.endElements();

            _numIneqConstraint  = A.rows();
            _numQIneqConstraint = r.size();
    	}

        for (int i=0; i<_numIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

      		for (int j=0; j<_numVariable; j++)
      			conExpr += A(i,j)*_IloVar[j];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, B(i)));

        	conExpr.end();
        }

        for (int i=0; i<_numQIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

            // Linear part of the constraint aiT*x
      		for (int j=0; j<_numVariable; j++)
      			conExpr += l.at(i)(j)*_IloVar[j];

            // Quadratic part of the constraint xT*Q*x
            for (int j=0; j<_numVariable; j++)
        	    for (int k=0; k<_numVariable; k++)
                    conExpr += Q.at(i)(j,k)*_IloVar[j]*_IloVar[k];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, r.at(i)));

        	conExpr.end();
        }

        _IloModel.add(_IloConstrIneq);
    }
    catch (IloException& e)
	{
        std::cout << "[CPLEXsolver] CPlex failed setting up the QP problem:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
	}

	return true;
}

bool CPLEXsolver::setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                             const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B,
                             const Ref<const MatrixXd> Aeq, const Ref<const VectorXd> Beq)
// Set an optimization problem characterized by cost function and inequality constraints
// Cost function            min 0.5*xT*H*x+fT*x
// Variable constraints         lb <= x <= ub
// Inequality constraints       Ax <= B
// Equality constraints         Aeqx = Beq
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before setProblem" << std::endl;
        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()<1) || (gradient.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty gradient" << std::endl;
        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a non square hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()!=hessian.rows()) || (gradient.cols()>1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a gradient having wrong dimensions" << std::endl;
        return false;
    }
    if ((lowerBound.size()!=hessian.rows()) || (upperBound.size()!=hessian.rows()))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong dimension on lower/upper bounds" << std::endl;
        return false;
    }
    if ((A.rows()<1) || (A.cols()<1) || (B.rows()<1) || (B.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of inequality constraints" << std::endl;
        return false;
    }
    if ((A.cols()!=hessian.rows()) || (A.rows()!=B.rows()) || (B.cols()!=1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a set of inequality constraints having wrong dimensions" << std::endl;
        return false;
    }
    if ((Aeq.rows()<1) || (Aeq.cols()<1) || (Beq.rows()<1) || (Beq.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of equality constraints" << std::endl;
        return false;
    }
    if ((Aeq.cols()!=hessian.rows()) || (Aeq.rows()!=Beq.rows()) || (Beq.cols()!=1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a set of equality constraints having wrong dimensions" << std::endl;
        return false;
    }

    /** Set the QP problem */
    try
    {
        // Check and update the number of optimization variables
        if (_numVariable>hessian.cols())
            _IloVar.remove(_IloVar.getSize()-(_numVariable-hessian.cols()), _numVariable-hessian.cols());
        else if (_numVariable<hessian.cols())
            for (int i=0; i<hessian.cols()-_numVariable; i++)
                _IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        _numVariable = _IloVar.getSize();

    	// Modify unknowns lower/upper bounds
    	for (int i=0; i<_numVariable; i++)
    		_IloVar[i].setBounds(lowerBound.at(i), upperBound.at(i));

    	// Modify the objective function
        IloExpr objExpr(_IloEnv, 0.0);

        for (int i=0; i<_numVariable; i++)			// Quadratic part of the objective function
        	for (int j=i; j<_numVariable; j++)
        	{
        		if (i==j)
        			objExpr += 0.5*hessian(i,j)*_IloVar[i]*_IloVar[j];
        		else
        			objExpr += hessian(i,j)*_IloVar[i]*_IloVar[j];
        	}

        for (int i=0; i<_numVariable; i++)			// Linear part of the objective function
        	objExpr += gradient(i)*_IloVar[i];

        _IloObj.setExpr(objExpr);

        objExpr.end();

    	// Add the inequality constraints
    	if (_numIneqConstraint!=A.rows())
    	{
            // Remove previous constraints
            _IloModel.remove(_IloConstrIneq);
            _IloConstrIneq.endElements();

            _numIneqConstraint = A.rows();
    	}

        for (int i=0; i<_numIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

      		for (int j=0; j<_numVariable; j++)
      			conExpr += A(i,j)*_IloVar[j];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, B(i)));

        	conExpr.end();
        }

        _IloModel.add(_IloConstrIneq);

    	// Add the equality constraints
    	if (_numEqConstraint!=Aeq.rows())
    	{
            // Remove previous constraints
            _IloModel.remove(_IloConstrEq);
            _IloConstrEq.endElements();

            _numEqConstraint = Aeq.rows();
    	}

        for (int i=0; i<_numEqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

      		for (int j=0; j<_numVariable; j++)
      			conExpr += Aeq(i,j)*_IloVar[j];

            _IloConstrEq.add(IloRange(_IloEnv, Beq(i), conExpr, Beq(i)));

        	conExpr.end();
        }

        _IloModel.add(_IloConstrEq);
    }
    catch (IloException& e)
	{
        std::cout << "[CPLEXsolver] CPlex failed setting up the QP problem:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
	}

	return true;
}

bool CPLEXsolver::setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                             const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B, const Ref<const MatrixXd> Aeq,
                             const Ref<const VectorXd> Beq, const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r)
// Set an optimization problem characterized by cost function and inequality constraints
// Cost function                    min 0.5*xT*H*x+fT*x
// Variable constraints                lb <= x <= ub
// Linear inequality constraints       Ax <= B
// Equality constraints                Aeqx = Beq
// Quadratic inequality constraints    aTi*x+xT*Qi*x <= ri   i=1,...,q
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before setProblem" << std::endl;
        return false;
    }
    if ((hessian.rows()<1) || (hessian.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()<1) || (gradient.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty gradient" << std::endl;
        return false;
    }
    if (hessian.rows()!=hessian.cols())
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a non square hessian" << std::endl;
        return false;
    }
    if ((gradient.rows()!=hessian.rows()) || (gradient.cols()>1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a gradient having wrong dimensions" << std::endl;
        return false;
    }
    if ((lowerBound.size()!=hessian.rows()) || (upperBound.size()!=hessian.rows()))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong dimension on lower/upper bounds" << std::endl;
        return false;
    }
    if ((A.rows()<1) || (A.cols()<1) || (B.rows()<1) || (B.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of inequality constraints" << std::endl;
        return false;
    }
    if ((A.cols()!=hessian.rows()) || (A.rows()!=B.rows()) || (B.cols()!=1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a set of inequality constraints having wrong dimensions" << std::endl;
        return false;
    }
    if ((Aeq.rows()<1) || (Aeq.cols()<1) || (Beq.rows()<1) || (Beq.cols()<1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of equality constraints" << std::endl;
        return false;
    }
    if ((Aeq.cols()!=hessian.rows()) || (Aeq.rows()!=Beq.rows()) || (Beq.cols()!=1))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a set of equality constraints having wrong dimensions" << std::endl;
        return false;
    }
    if ((l.size()!=_numQIneqConstraint) || (Q.size()!=_numQIneqConstraint) || (r.size()!=_numQIneqConstraint))
    {
        std::cout << "[CPLEXsolver] Cannot setProblem with a wrong number of quadratic inequality constraints" << std::endl;
        return false;
    }
    for (int k=0; k<l.size(); k++)
    {
        if ((l.at(k).rows()<1) || (l.at(k).cols()<1) || (Q.at(k).rows()<1) || (Q.at(k).cols()<1))
        {
            std::cout << "[CPLEXsolver] Cannot setProblem with an empty set of quadratic inequality constraints" << std::endl;
            return false;
        }
        if ((l.at(k).rows()!=hessian.rows()) || (l.at(k).cols()!=1) || (Q.at(k).rows()!=hessian.rows()) || (Q.at(k).cols()!=hessian.rows()))
        {
            std::cout << "[CPLEXsolver] Cannot setProblem with a set of quadratic inequality constraints having wrong dimensions" << std::endl;
            return false;
        }
    }

    /** Set the QP problem */
    try
    {
        // Check and update the number of optimization variables
        if (_numVariable>hessian.cols())
            _IloVar.remove(_IloVar.getSize()-(_numVariable-hessian.cols()), _numVariable-hessian.cols());
        else if (_numVariable<hessian.cols())
            for (int i=0; i<hessian.cols()-_numVariable; i++)
                _IloVar.add(IloNumVar(_IloEnv, -IloInfinity, +IloInfinity));

        _numVariable = _IloVar.getSize();

    	// Modify unknowns lower/upper bounds
    	for (int i=0; i<_numVariable; i++)
    		_IloVar[i].setBounds(lowerBound.at(i), upperBound.at(i));

    	// Modify the objective function
        IloExpr objExpr(_IloEnv, 0.0);

        for (int i=0; i<_numVariable; i++)			// Quadratic part of the objective function
        	for (int j=i; j<_numVariable; j++)
        	{
        		if (i==j)
        			objExpr += 0.5*hessian(i,j)*_IloVar[i]*_IloVar[j];
        		else
        			objExpr += hessian(i,j)*_IloVar[i]*_IloVar[j];
        	}

        for (int i=0; i<_numVariable; i++)			// Linear part of the objective function
        	objExpr += gradient(i)*_IloVar[i];

        _IloObj.setExpr(objExpr);

        objExpr.end();

    	// Add the inequality constraints
    	if ((_numIneqConstraint+_numQIneqConstraint)!=(A.rows()+r.size()))
    	{
            // Remove previous constraints
            _IloModel.remove(_IloConstrIneq);
            _IloConstrIneq.endElements();

            _numIneqConstraint  = A.rows();
            _numQIneqConstraint = r.size();
    	}

        for (int i=0; i<_numIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

      		for (int j=0; j<_numVariable; j++)
      			conExpr += A(i,j)*_IloVar[j];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, B(i)));

        	conExpr.end();
        }

        for (int i=0; i<_numQIneqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

            // Linear part of the constraint aiT*x
      		for (int j=0; j<_numVariable; j++)
      			conExpr += l.at(i)(j)*_IloVar[j];

            // Quadratic part of the constraint xT*Q*x
            for (int j=0; j<_numVariable; j++)
        	    for (int k=0; k<_numVariable; k++)
                    conExpr += Q.at(i)(j,k)*_IloVar[j]*_IloVar[k];

            _IloConstrIneq.add(IloRange(_IloEnv, conExpr, r.at(i)));

        	conExpr.end();
        }

        _IloModel.add(_IloConstrIneq);

    	// Add the equality constraints
    	if (_numEqConstraint!=Aeq.rows())
    	{
            // Remove previous constraints
            _IloModel.remove(_IloConstrEq);
            _IloConstrEq.endElements();

            _numEqConstraint = Aeq.rows();
    	}

        for (int i=0; i<_numEqConstraint; i++)
        {
        	IloExpr conExpr(_IloEnv, 0.0);

      		for (int j=0; j<_numVariable; j++)
      			conExpr += Aeq(i,j)*_IloVar[j];

            _IloConstrEq.add(IloRange(_IloEnv, Beq(i), conExpr, Beq(i)));

        	conExpr.end();
        }

        _IloModel.add(_IloConstrEq);
    }
    catch (IloException& e)
	{
        std::cout << "[CPLEXsolver] CPlex failed setting up the QP problem:" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;
        return false;
	}

	return true;
}

bool CPLEXsolver::solveProblem(Ref<VectorXd> result, int& optimizerStatus)
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before solveProblem" << std::endl;
        return false;
    }

    /** Solve the QP problem */
    try
    {
    	if (_IloCplex.solve())
    	{
    		// Get the solution
    		for (int i=0; i<_numVariable; i++)
    			result(i) = _IloCplex.getValue(_IloVar[i]);

            // Get the optimizer status
            optimizerStatus = _IloCplex.getCplexStatus();
    	}
    	else
    	{
    		// Set the solution to zero
    		for (int i=0; i<_numVariable; i++)
    			result(i) = 0.0;

            // Get the optimizer status
            optimizerStatus = _IloCplex.getCplexStatus();

    		std::cout << "[CPLEXsolver] CPlex failed solving QP problem" << std::endl;
            return false;
    	}
    }
    catch (IloException& e)
    {
    	std::cout << "[CPLEXsolver] CPlex failed solving the QP problem" << std::endl;
        std::cout << "\t [" << e << "]" << std::endl;

    	optimizerStatus = -1;

    	return false;
    }

    return true;
}

void CPLEXsolver::saveProblem(const std::string filename)
{
    std::string fileName = filename+".lp";
    _IloCplex.exportModel(fileName.c_str());
}

void CPLEXsolver::set_printLevel(const printLevelType printLevel)
{
    /** Check that the problem has been initialized */
    if (!_IloInitialized)
    {
        std::cout << "[CPLEXsolver] Call initProblem before set_printLevel" << std::endl;
        return;
    }

    switch (printLevel)
    {
    case NONE:
        switch(_solverMethod)
        {
        case AUTO:
            _IloCplex.setParam(IloCplex::SimDisplay, 0);
            _IloCplex.setParam(IloCplex::NetDisplay, 0);
            _IloCplex.setParam(IloCplex::BarDisplay, 0);
            _IloCplex.setParam(IloCplex::SiftDisplay, 0);
          	break;
        case PRIMAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 0);
        	break;
        case DUAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 0);
        	break;
        case NETWORK:
            _IloCplex.setParam(IloCplex::NetDisplay, 0);
        	break;
        case BARRIER:
            _IloCplex.setParam(IloCplex::BarDisplay, 0);
        	break;
        case SIFTING:
            _IloCplex.setParam(IloCplex::SiftDisplay, 0);
        	break;
        case CONCURRENT:
            _IloCplex.setParam(IloCplex::SimDisplay, 0);
            _IloCplex.setParam(IloCplex::NetDisplay, 0);
            _IloCplex.setParam(IloCplex::BarDisplay, 0);
            _IloCplex.setParam(IloCplex::SiftDisplay, 0);
        	break;
        }
        break;

    case LOW:
        switch(_solverMethod)
        {
        case AUTO:
            _IloCplex.setParam(IloCplex::SimDisplay, 1);
            _IloCplex.setParam(IloCplex::NetDisplay, 1);
            _IloCplex.setParam(IloCplex::BarDisplay, 1);
            _IloCplex.setParam(IloCplex::SiftDisplay, 1);
          	break;
        case PRIMAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 1);
        	break;
        case DUAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 1);
        	break;
        case NETWORK:
            _IloCplex.setParam(IloCplex::NetDisplay, 1);
        	break;
        case BARRIER:
            _IloCplex.setParam(IloCplex::BarDisplay, 1);
        	break;
        case SIFTING:
            _IloCplex.setParam(IloCplex::SiftDisplay, 1);
        	break;
        case CONCURRENT:
            _IloCplex.setParam(IloCplex::SimDisplay, 1);
            _IloCplex.setParam(IloCplex::NetDisplay, 1);
            _IloCplex.setParam(IloCplex::BarDisplay, 1);
            _IloCplex.setParam(IloCplex::SiftDisplay, 1);
        	break;
        }
        break;

    case MEDIUM:
        switch(_solverMethod)
        {
        case AUTO:
            _IloCplex.setParam(IloCplex::SimDisplay, 2);
            _IloCplex.setParam(IloCplex::NetDisplay, 2);
            _IloCplex.setParam(IloCplex::BarDisplay, 2);
            _IloCplex.setParam(IloCplex::SiftDisplay, 2);
          	break;
        case PRIMAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 2);
        	break;
        case DUAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 2);
        	break;
        case NETWORK:
            _IloCplex.setParam(IloCplex::NetDisplay, 2);
        	break;
        case BARRIER:
            _IloCplex.setParam(IloCplex::BarDisplay, 2);
        	break;
        case SIFTING:
            _IloCplex.setParam(IloCplex::SiftDisplay, 2);
        	break;
        case CONCURRENT:
            _IloCplex.setParam(IloCplex::SimDisplay, 2);
            _IloCplex.setParam(IloCplex::NetDisplay, 2);
            _IloCplex.setParam(IloCplex::BarDisplay, 2);
            _IloCplex.setParam(IloCplex::SiftDisplay, 2);
        	break;
        }
        break;

    case HIGH:
        switch(_solverMethod)
        {
        case AUTO:
            _IloCplex.setParam(IloCplex::SimDisplay, 2);
            _IloCplex.setParam(IloCplex::NetDisplay, 2);
            _IloCplex.setParam(IloCplex::BarDisplay, 2);
            _IloCplex.setParam(IloCplex::SiftDisplay, 2);
          	break;
        case PRIMAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 2);
        	break;
        case DUAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 2);
        	break;
        case NETWORK:
            _IloCplex.setParam(IloCplex::NetDisplay, 2);
        	break;
        case BARRIER:
            _IloCplex.setParam(IloCplex::BarDisplay, 2);
        	break;
        case SIFTING:
            _IloCplex.setParam(IloCplex::SiftDisplay, 2);
        	break;
        case CONCURRENT:
            _IloCplex.setParam(IloCplex::SimDisplay, 2);
            _IloCplex.setParam(IloCplex::NetDisplay, 2);
            _IloCplex.setParam(IloCplex::BarDisplay, 2);
            _IloCplex.setParam(IloCplex::SiftDisplay, 2);
        	break;
        }
        break;

    default:
        switch(_solverMethod)
        {
        case AUTO:
            _IloCplex.setParam(IloCplex::SimDisplay, 0);
            _IloCplex.setParam(IloCplex::NetDisplay, 0);
            _IloCplex.setParam(IloCplex::BarDisplay, 0);
            _IloCplex.setParam(IloCplex::SiftDisplay, 0);
          	break;
        case PRIMAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 0);
        	break;
        case DUAL:
            _IloCplex.setParam(IloCplex::SimDisplay, 0);
        	break;
        case NETWORK:
            _IloCplex.setParam(IloCplex::NetDisplay, 0);
        	break;
        case BARRIER:
            _IloCplex.setParam(IloCplex::BarDisplay, 0);
        	break;
        case SIFTING:
            _IloCplex.setParam(IloCplex::SiftDisplay, 0);
        	break;
        case CONCURRENT:
            _IloCplex.setParam(IloCplex::SimDisplay, 0);
            _IloCplex.setParam(IloCplex::NetDisplay, 0);
            _IloCplex.setParam(IloCplex::BarDisplay, 0);
            _IloCplex.setParam(IloCplex::SiftDisplay, 0);
        	break;
        }
    }
}
