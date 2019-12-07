#include "writeMatlabScript.h"


void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol, 
                          const Eigen::Ref<const Eigen::MatrixXd> H, const Eigen::Ref<const Eigen::MatrixXd> f, 
                          const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal)
{
    std::string filename;
    filename = name+".m";

    std::ofstream matlab_script;
    matlab_script.open(filename.c_str());

    // File header
    matlab_script << "% Automatically generated matlab script" << std::endl;
    if (clearStatements)
    {
        matlab_script << "clear all;" << std::endl;
        matlab_script << "close all;" << std::endl;
    }
    matlab_script << "clc;" << std::endl << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", nDecimal, matlab_script);
    writeVector(f, "f", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Solver options
    matlab_script << "% Solver options" << std::endl;
    matlab_script << "Options = cplexoptimset('cplex');" << std::endl;
    matlab_script << "Options.qpmethod = " << optim_algorithm << ";" << std::endl;

    switch (optim_algorithm)
    {
        case AUTO:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case PRIMAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case DUAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case NETWORK:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;
        
        case BARRIER:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
        break;

        case SIFTING:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;

        case CONCURRENT:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;
    }
    matlab_script << "Options.display = 'on';" << std::endl;
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,[],[],[],[],[],[],[],Options);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", nDecimal, matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    matlab_script << "disp(newline);" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Percentage solution error: ', mat2str(norm(x-x_cpp)/norm(x)*100)]);" << std::endl;
    else
        matlab_script << "disp(['Percentage solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol,
                          const Eigen::Ref<const Eigen::MatrixXd> H, const Eigen::Ref<const Eigen::VectorXd> f, 
                          const Eigen::Ref<const Eigen::MatrixXd> Ain, const Eigen::Ref<const Eigen::VectorXd> Bin, 
                          const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal)
{
    std::string filename;
    filename = name+".m";

    std::ofstream matlab_script;
    matlab_script.open(filename.c_str());

    // File header
    matlab_script << "% Automatically generated matlab script" << std::endl;
    if (clearStatements)
    {
        matlab_script << "clear all;" << std::endl;
        matlab_script << "close all;" << std::endl;
    }
    matlab_script << "clc;" << std::endl << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", nDecimal, matlab_script);
    writeVector(f, "f", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", nDecimal, matlab_script);
    writeVector(Bin, "Bin", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Solver options
    matlab_script << "% Solver options" << std::endl;
    matlab_script << "Options = cplexoptimset('cplex');" << std::endl;
    matlab_script << "Options.qpmethod = " << optim_algorithm << ";" << std::endl;

    switch (optim_algorithm)
    {
        case AUTO:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case PRIMAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case DUAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case NETWORK:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;
        
        case BARRIER:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
        break;

        case SIFTING:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;

        case CONCURRENT:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;
    }
    matlab_script << "Options.display = 'on';" << std::endl;
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,Ain,Bin,[],[],[],[],[],Options);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", nDecimal, matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    matlab_script << "disp(newline);" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Percentage solution error: ', mat2str(norm(x-x_cpp)/norm(x)*100)]);" << std::endl;
    else
        matlab_script << "disp(['Percentage solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QCP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol, 
                           const Eigen::Ref<const Eigen::MatrixXd> H, const Eigen::Ref<const Eigen::VectorXd> f, 
                           const Eigen::Ref<const Eigen::MatrixXd> Ain, const Eigen::Ref<const Eigen::VectorXd> Bin, 
                           const std::vector<Eigen::VectorXd>& l, const std::vector<Eigen::MatrixXd> Q, const std::vector<double>& r, 
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal)
{
    std::string filename;
    filename = name+".m";

    std::ofstream matlab_script;
    matlab_script.open(filename.c_str());

    // File header
    matlab_script << "% Automatically generated matlab script" << std::endl;
    if (clearStatements)
    {
        matlab_script << "clear all;" << std::endl;
        matlab_script << "close all;" << std::endl;
    }
    matlab_script << "clc;" << std::endl << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", nDecimal, matlab_script);
    writeVector(f, "f", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Linear inequality constraints
    matlab_script << "% Linear inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", nDecimal, matlab_script);
    writeVector(Bin, "Bin", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Quadratic inequality constraints
    matlab_script << "% Quadratic inequality constraints" << std::endl;
    for (int i=0; i<r.size(); i++)
    {
        std::ostringstream l_name; l_name << "l" << i+1;
        writeVector(l.at(i), l_name.str(), nDecimal, matlab_script);

        std::ostringstream Q_name; Q_name << "Q" << i+1;
        writeMatrix(Q.at(i), Q_name.str(), nDecimal, matlab_script);

        matlab_script << "r" << i+1 << "=" << r.at(i) << ";" << std::endl;
    }

    if (r.size()>1)
    {
        matlab_script << "l=[";
        for (int i=0; i<r.size(); i++)
            matlab_script << "l" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "];" << std::endl;
    }
    else
        matlab_script << "l=l1;" << std::endl;

    if (r.size()>1)
    {
        matlab_script << "Q={";
        for (int i=0; i<r.size(); i++)
            matlab_script << "Q" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "};" << std::endl;
    }
    else
        matlab_script << "Q=Q1;" << std::endl;

    if (r.size()>1)
    {
        matlab_script << "r=[";
        for (int i=0; i<r.size(); i++)
            matlab_script << "r" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "];" << std::endl;
    }
    else
        matlab_script << "r=r1;" << std::endl;

    matlab_script << std::endl;

    // Solver options
    matlab_script << "% Solver options" << std::endl;
    matlab_script << "Options = cplexoptimset('cplex');" << std::endl;
    matlab_script << "Options.qpmethod = " << optim_algorithm << ";" << std::endl;

    switch (optim_algorithm)
    {
        case AUTO:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case PRIMAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case DUAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case NETWORK:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;
        
        case BARRIER:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
        break;

        case SIFTING:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;

        case CONCURRENT:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;
    }
    matlab_script << "Options.display = 'on';" << std::endl;
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqcp(H,f,Ain,Bin,[],[],l,Q,r,[],[],[],Options);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", nDecimal, matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    matlab_script << "disp(newline);" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Percentage solution error: ', mat2str(norm(x-x_cpp)/norm(x)*100)]);" << std::endl;
    else
        matlab_script << "disp(['Percentage solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol, 
                          const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                          const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag,
                          int nDecimal)
{
    std::string filename;
    filename = name+".m";

    std::ofstream matlab_script;
    matlab_script.open(filename.c_str());

    // File header
    matlab_script << "% Automatically generated matlab script" << std::endl;
    if (clearStatements)
    {
        matlab_script << "clear all;" << std::endl;
        matlab_script << "close all;" << std::endl;
    }
    matlab_script << "clc;" << std::endl << std::endl;

    // Lower/Upper bounds
    matlab_script << "% Lower/upper bounds" << std::endl;
    writeVector(lB, "lB", nDecimal, matlab_script);
    writeVector(uB, "uB", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", nDecimal, matlab_script);
    writeVector(f, "f", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Solver options
    matlab_script << "% Solver options" << std::endl;
    matlab_script << "Options = cplexoptimset('cplex');" << std::endl;
    matlab_script << "Options.qpmethod = " << optim_algorithm << ";" << std::endl;

    switch (optim_algorithm)
    {
        case AUTO:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case PRIMAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case DUAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case NETWORK:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;
        
        case BARRIER:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
        break;

        case SIFTING:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;

        case CONCURRENT:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;
    }
    matlab_script << "Options.display = 'on';" << std::endl;
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,[],[],[],[],lB,uB,[],Options);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", nDecimal, matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    matlab_script << "disp(newline);" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Percentage solution error: ', mat2str(norm(x-x_cpp)/norm(x)*100)]);" << std::endl;
    else
        matlab_script << "disp(['Percentage solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QCP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol, 
                           const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                           const Eigen::Ref<const Eigen::VectorXd> f, const std::vector<Eigen::VectorXd>& l, 
                           const std::vector<Eigen::MatrixXd> Q, const std::vector<double>& r, 
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal)
{
    std::string filename;
    filename = name+".m";

    std::ofstream matlab_script;
    matlab_script.open(filename.c_str());

    // File header
    matlab_script << "% Automatically generated matlab script" << std::endl;
    if (clearStatements)
    {
        matlab_script << "clear all;" << std::endl;
        matlab_script << "close all;" << std::endl;
    }
    matlab_script << "clc;" << std::endl << std::endl;

    // Lower/Upper bounds
    matlab_script << "% Lower/upper bounds" << std::endl;
    writeVector(lB, "lB", nDecimal, matlab_script);
    writeVector(uB, "uB", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", nDecimal, matlab_script);
    writeVector(f, "f", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Quadratic inequality constraints
    matlab_script << "% Quadratic inequality constraints" << std::endl;
    for (int i=0; i<r.size(); i++)
    {
        std::ostringstream l_name; l_name << "l" << i+1;
        writeVector(l.at(i), l_name.str(), nDecimal, matlab_script);

        std::ostringstream Q_name; Q_name << "Q" << i+1;
        writeMatrix(Q.at(i), Q_name.str(), nDecimal, matlab_script);

        matlab_script << "r" << i+1 << "=" << r.at(i) << ";" << std::endl;
    }

    if (r.size()>1)
    {
        matlab_script << "l=[";
        for (int i=0; i<r.size(); i++)
            matlab_script << "l" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "];" << std::endl;
    }
    else
        matlab_script << "l=l1;" << std::endl;

    if (r.size()>1)
    {
        matlab_script << "Q={";
        for (int i=0; i<r.size(); i++)
            matlab_script << "Q" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "};" << std::endl;
    }
    else
        matlab_script << "Q=Q1;" << std::endl;

    if (r.size()>1)
    {
        matlab_script << "r=[";
        for (int i=0; i<r.size(); i++)
            matlab_script << "r" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "];" << std::endl;
    }
    else
        matlab_script << "r=r1;" << std::endl;

    matlab_script << std::endl;

    // Solver options
    matlab_script << "% Solver options" << std::endl;
    matlab_script << "Options = cplexoptimset('cplex');" << std::endl;
    matlab_script << "Options.qpmethod = " << optim_algorithm << ";" << std::endl;

    switch (optim_algorithm)
    {
        case AUTO:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case PRIMAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case DUAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case NETWORK:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;
        
        case BARRIER:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
        break;

        case SIFTING:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;

        case CONCURRENT:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;
    }
    matlab_script << "Options.display = 'on';" << std::endl;
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqcp(H,f,[],[],[],[],l,Q,r,lB,uB,[],Options);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", nDecimal, matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    matlab_script << "disp(newline);" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Percentage solution error: ', mat2str(norm(x-x_cpp)/norm(x)*100)]);" << std::endl;
    else
        matlab_script << "disp(['Percentage solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol, 
                          const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                          const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::MatrixXd> Ain, 
                          const Eigen::Ref<const Eigen::VectorXd> Bin, const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag,
                          int nDecimal)
{
    std::string filename;
    filename = name+".m";

    std::ofstream matlab_script;
    matlab_script.open(filename.c_str());

    // File header
    matlab_script << "% Automatically generated matlab script" << std::endl;
    if (clearStatements)
    {
        matlab_script << "clear all;" << std::endl;
        matlab_script << "close all;" << std::endl;
    }
    matlab_script << "clc;" << std::endl << std::endl;

    // Lower/Upper bounds
    matlab_script << "% Lower/upper bounds" << std::endl;
    writeVector(lB, "lB", nDecimal, matlab_script);
    writeVector(uB, "uB", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", nDecimal, matlab_script);
    writeVector(f, "f", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", nDecimal, matlab_script);
    writeVector(Bin, "Bin", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Solver options
    matlab_script << "% Solver options" << std::endl;
    matlab_script << "Options = cplexoptimset('cplex');" << std::endl;
    matlab_script << "Options.qpmethod = " << optim_algorithm << ";" << std::endl;

    switch (optim_algorithm)
    {
        case AUTO:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case PRIMAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case DUAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case NETWORK:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;
        
        case BARRIER:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
        break;

        case SIFTING:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;

        case CONCURRENT:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;
    }
    matlab_script << "Options.display = 'on';" << std::endl;
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,Ain,Bin,[],[],lB,uB,[],Options);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", nDecimal, matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    matlab_script << "disp(newline);" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Percentage solution error: ', mat2str(norm(x-x_cpp)/norm(x)*100)]);" << std::endl;
    else
        matlab_script << "disp(['Percentage solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QCP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol, 
                           const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                           const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::MatrixXd> Ain, 
                           const Eigen::Ref<const Eigen::VectorXd> Bin, const std::vector<Eigen::VectorXd>& l, 
                           const std::vector<Eigen::MatrixXd> Q, const std::vector<double>& r,
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal)
{
    std::string filename;
    filename = name+".m";

    std::ofstream matlab_script;
    matlab_script.open(filename.c_str());

    // File header
    matlab_script << "% Automatically generated matlab script" << std::endl;
    if (clearStatements)
    {
        matlab_script << "clear all;" << std::endl;
        matlab_script << "close all;" << std::endl;
    }
    matlab_script << "clc;" << std::endl << std::endl;

    // Lower/Upper bounds
    matlab_script << "% Lower/upper bounds" << std::endl;
    writeVector(lB, "lB", nDecimal, matlab_script);
    writeVector(uB, "uB", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", nDecimal, matlab_script);
    writeVector(f, "f", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Linear inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", nDecimal, matlab_script);
    writeVector(Bin, "Bin", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Quadratic inequality constraints
    matlab_script << "% Quadratic inequality constraints" << std::endl;
    for (int i=0; i<r.size(); i++)
    {
        std::ostringstream l_name; l_name << "l" << i+1;
        writeVector(l.at(i), l_name.str(), nDecimal, matlab_script);

        std::ostringstream Q_name; Q_name << "Q" << i+1;
        writeMatrix(Q.at(i), Q_name.str(), nDecimal, matlab_script);

        matlab_script << "r" << i+1 << "=" << r.at(i) << ";" << std::endl;
    }

    if (r.size()>1)
    {
        matlab_script << "l=[";
        for (int i=0; i<r.size(); i++)
            matlab_script << "l" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "];" << std::endl;
    }
    else
        matlab_script << "l=l1;" << std::endl;

    if (r.size()>1)
    {
        matlab_script << "Q={";
        for (int i=0; i<r.size(); i++)
            matlab_script << "Q" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "};" << std::endl;
    }
    else
        matlab_script << "Q=Q1;" << std::endl;

    if (r.size()>1)
    {
        matlab_script << "r=[";
        for (int i=0; i<r.size(); i++)
            matlab_script << "r" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "];" << std::endl;
    }
    else
        matlab_script << "r=r1;" << std::endl;

    matlab_script << std::endl;

    // Solver options
    matlab_script << "% Solver options" << std::endl;
    matlab_script << "Options = cplexoptimset('cplex');" << std::endl;
    matlab_script << "Options.qpmethod = " << optim_algorithm << ";" << std::endl;

    switch (optim_algorithm)
    {
        case AUTO:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case PRIMAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case DUAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case NETWORK:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;
        
        case BARRIER:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
        break;

        case SIFTING:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;

        case CONCURRENT:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;
    }
    matlab_script << "Options.display = 'on';" << std::endl;
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqcp(H,f,Ain,Bin,[],[],l,Q,r,lB,uB,[],Options);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", nDecimal, matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    matlab_script << "disp(newline);" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Percentage solution error: ', mat2str(norm(x-x_cpp)/norm(x)*100)]);" << std::endl;
    else
        matlab_script << "disp(['Percentage solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol, 
                          const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                          const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::MatrixXd> Ain, 
                          const Eigen::Ref<const Eigen::VectorXd> Bin, const Eigen::Ref<const Eigen::MatrixXd> Aeq, 
                          const Eigen::Ref<const Eigen::VectorXd> Beq, const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag,
                          int nDecimal)
{
    std::string filename;
    filename = name+".m";

    std::ofstream matlab_script;
    matlab_script.open(filename.c_str());

    // File header
    matlab_script << "% Automatically generated matlab script" << std::endl;
    if (clearStatements)
    {
        matlab_script << "clear all;" << std::endl;
        matlab_script << "close all;" << std::endl;
    }
    matlab_script << "clc;" << std::endl << std::endl;

    // Lower/Upper bounds
    matlab_script << "% Lower/upper bounds" << std::endl;
    writeVector(lB, "lB", nDecimal, matlab_script);
    writeVector(uB, "uB", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", nDecimal, matlab_script);
    writeVector(f, "f", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", nDecimal, matlab_script);
    writeVector(Bin, "Bin", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Equality constraints
    matlab_script << "% Equality constraints" << std::endl;
    writeMatrix(Aeq, "Aeq", nDecimal, matlab_script);
    writeVector(Beq, "Beq", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Solver options
    matlab_script << "% Solver options" << std::endl;
    matlab_script << "Options = cplexoptimset('cplex');" << std::endl;
    matlab_script << "Options.qpmethod = " << optim_algorithm << ";" << std::endl;

    switch (optim_algorithm)
    {
        case AUTO:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case PRIMAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case DUAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case NETWORK:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;
        
        case BARRIER:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
        break;

        case SIFTING:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;

        case CONCURRENT:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;
    }
    matlab_script << "Options.display = 'on';" << std::endl;
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,Ain,Bin,Aeq,Beq,lB,uB,[],Options);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", nDecimal, matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    matlab_script << "disp(newline);" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Percentage solution error: ', mat2str(norm(x-x_cpp)/norm(x)*100)]);" << std::endl;
    else
        matlab_script << "disp(['Percentage solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QCP_writeMatlabScript(std::string name, bool clearStatements, optim_algo_t optim_algorithm, optim_algo_tol_t algorithm_tol, 
                           const std::vector<double>& lB, const std::vector<double>& uB, const Eigen::Ref<const Eigen::MatrixXd> H, 
                           const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::MatrixXd> Ain, 
                           const Eigen::Ref<const Eigen::VectorXd> Bin, const Eigen::Ref<const Eigen::MatrixXd> Aeq, 
                           const Eigen::Ref<const Eigen::VectorXd> Beq, const std::vector<Eigen::VectorXd>& l, 
                           const std::vector<Eigen::MatrixXd> Q, const std::vector<double>& r, 
                           const Eigen::Ref<const Eigen::VectorXd> solution, int exitFlag, int nDecimal)
{
    std::string filename;
    filename = name+".m";

    std::ofstream matlab_script;
    matlab_script.open(filename.c_str());

    // File header
    matlab_script << "% Automatically generated matlab script" << std::endl;
    if (clearStatements)
    {
        matlab_script << "clear all;" << std::endl;
        matlab_script << "close all;" << std::endl;
    }
    matlab_script << "clc;" << std::endl << std::endl;

    // Lower/Upper bounds
    matlab_script << "% Lower/upper bounds" << std::endl;
    writeVector(lB, "lB", nDecimal, matlab_script);
    writeVector(uB, "uB", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", nDecimal, matlab_script);
    writeVector(f, "f", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", nDecimal, matlab_script);
    writeVector(Bin, "Bin", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Equality constraints
    matlab_script << "% Equality constraints" << std::endl;
    writeMatrix(Aeq, "Aeq", nDecimal, matlab_script);
    writeVector(Beq, "Beq", nDecimal, matlab_script);
    matlab_script << std::endl;

    // Quadratic inequality constraints
    matlab_script << "% Quadratic inequality constraints" << std::endl;
    for (int i=0; i<r.size(); i++)
    {
        std::ostringstream l_name; l_name << "l" << i+1;
        writeVector(l.at(i), l_name.str(), nDecimal, matlab_script);

        std::ostringstream Q_name; Q_name << "Q" << i+1;
        writeMatrix(Q.at(i), Q_name.str(), nDecimal, matlab_script);

        matlab_script << "r" << i+1 << "=" << r.at(i) << ";" << std::endl;
    }

    if (r.size()>1)
    {
        matlab_script << "l=[";
        for (int i=0; i<r.size(); i++)
            matlab_script << "l" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "];" << std::endl;
    }
    else
        matlab_script << "l=l1;" << std::endl;

    if (r.size()>1)
    {
        matlab_script << "Q={";
        for (int i=0; i<r.size(); i++)
            matlab_script << "Q" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "};" << std::endl;
    }
    else
        matlab_script << "Q=Q1;" << std::endl;

    if (r.size()>1)
    {
        matlab_script << "r=[";
        for (int i=0; i<r.size(); i++)
            matlab_script << "r" << i+1 << ",";
        matlab_script.seekp(-1,matlab_script.cur);
        matlab_script << "];" << std::endl;
    }
    else
        matlab_script << "r=r1;" << std::endl;

    matlab_script << std::endl;

    // Solver options
    matlab_script << "% Solver options" << std::endl;
    matlab_script << "Options = cplexoptimset('cplex');" << std::endl;
    matlab_script << "Options.qpmethod = " << optim_algorithm << ";" << std::endl;

    switch (optim_algorithm)
    {
        case AUTO:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case PRIMAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case DUAL:
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;

        case NETWORK:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;
        
        case BARRIER:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
        break;

        case SIFTING:
            std::cout << "Warning: optimization/feasibility tolerances for network algorithm cannot be set in Matlab" << std::endl;
        break;

        case CONCURRENT:
            matlab_script << "Options.barrier.convergetol = " << algorithm_tol.QP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.barrier.qcpconvergetol = " << algorithm_tol.QCP_convergence_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.optimality = " << algorithm_tol.optimality_tolerance << ";" << std::endl;
            matlab_script << "Options.simplex.tolerances.feasibility = " << algorithm_tol.feasibility_tolerance << ";" << std::endl;
        break;
    }
    matlab_script << "Options.display = 'on';" << std::endl;
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqcp(H,f,Ain,Bin,Aeq,Beq,l,Q,r,lB,uB,[],Options);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", nDecimal, matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    matlab_script << "disp(newline);" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Percentage solution error: ', mat2str(norm(x-x_cpp)/norm(x)*100)]);" << std::endl;
    else
        matlab_script << "disp(['Percentage solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}


void writeVector(const Eigen::Ref<const Eigen::VectorXd> vect, std::string name, int nDecimal, std::ofstream& stream)
{
    stream << name << "=[";
    for (int k=0; k<vect.size(); k++)
    {
        if (k<vect.size()-1)
            stream << std::setprecision(nDecimal + 1) << vect(k) << ", ";
        else
            stream << std::setprecision(nDecimal + 1) << vect(k);
    }
    stream << "]';" << std::endl;
}

void writeVector(const std::vector<double>& vect, std::string name, int nDecimal, std::ofstream& stream)
{
    stream << name << "=[";
    for (int k=0; k<vect.size(); k++)
    {
        if (k<vect.size()-1)
            stream << std::setprecision(nDecimal + 1) << vect.at(k) << ", ";
        else
            stream << std::setprecision(nDecimal + 1) << vect.at(k);
    }
    stream << "]';" << std::endl;
}

void writeMatrix(const Eigen::Ref<const Eigen::MatrixXd> mat, std::string name, int nDecimal, std::ofstream& stream)
{
    stream << name << "=[";
    for (int j=0; j<mat.rows(); j++)
    {
        for (int k=0; k<mat.cols(); k++)
        {
            if (k<mat.cols()-1)
                stream << std::setprecision(nDecimal + 1) << mat(j,k) << ", ";
            else
                stream << std::setprecision(nDecimal + 1) << mat(j,k);
        }

        if (j<mat.rows()-1)
            stream << "; ";
        else
            stream << "];" << std::endl;
    }
}
