#include "writeMatlabScript.h"


void QP_writeMatlabScript(std::string name, bool clearStatements, const Ref<const MatrixXd> H, const Ref<const MatrixXd> f, const Ref<const VectorXd> solution, int exitFlag)
{
    std::ofstream matlab_script;
    matlab_script.open(name.c_str());

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
    writeMatrix(H, "H", matlab_script);
    writeVector(f, "f", matlab_script);
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "H=(H+H')/2;" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,[],[],[],[],[],[]);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Solution error: ', mat2str(norm(x-x_cpp))]);" << std::endl;
    else
        matlab_script << "disp(['Solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QP_writeMatlabScript(std::string name, bool clearStatements, const Ref<const MatrixXd> H, const Ref<const VectorXd> f, const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin,
                          const Ref<const VectorXd> solution, int exitFlag)
{
    std::ofstream matlab_script;
    matlab_script.open(name.c_str());

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
    writeMatrix(H, "H", matlab_script);
    writeVector(f, "f", matlab_script);
    matlab_script << std::endl;

    // Inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", matlab_script);
    writeVector(Bin, "Bin", matlab_script);
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "H=(H+H')/2;" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,Ain,Bin,[],[],[],[]);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Solution error: ', mat2str(norm(x-x_cpp))]);" << std::endl;
    else
        matlab_script << "disp(['Solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QCP_writeMatlabScript(std::string name, bool clearStatements, const Ref<const MatrixXd> H, const Ref<const VectorXd> f, const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin,
                           const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r, const Ref<const VectorXd> solution, int exitFlag)
{
    std::ofstream matlab_script;
    matlab_script.open(name.c_str());

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
    writeMatrix(H, "H", matlab_script);
    writeVector(f, "f", matlab_script);
    matlab_script << std::endl;

    // Linear inequality constraints
    matlab_script << "% Linear inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", matlab_script);
    writeVector(Bin, "Bin", matlab_script);
    matlab_script << std::endl;

    // Quadratic inequality constraints
    matlab_script << "% Quadratic inequality constraints" << std::endl;
    for (int i=0; i<r.size(); i++)
    {
        std::ostringstream l_name; l_name << "l" << i+1;
        writeVector(l.at(i), l_name.str(), matlab_script);

        std::ostringstream Q_name; Q_name << "Q" << i+1;
        writeMatrix(Q.at(i), Q_name.str(), matlab_script);

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

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "H=(H+H')/2;" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqcp(H,f,Ain,Bin,[],[],l,Q,r);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Solution error: ', mat2str(norm(x-x_cpp))]);" << std::endl;
    else
        matlab_script << "disp(['Solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                          const Ref<const VectorXd> solution, int exitFlag)
{
    std::ofstream matlab_script;
    matlab_script.open(name.c_str());

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
    writeVector(lB, "lB", matlab_script);
    writeVector(uB, "uB", matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", matlab_script);
    writeVector(f, "f", matlab_script);
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "H=(H+H')/2;" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,[],[],[],[],lB,uB);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Solution error: ', mat2str(norm(x-x_cpp))]);" << std::endl;
    else
        matlab_script << "disp(['Solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QCP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                           const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r, const Ref<const VectorXd> solution, int exitFlag)
{
    std::ofstream matlab_script;
    matlab_script.open(name.c_str());

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
    writeVector(lB, "lB", matlab_script);
    writeVector(uB, "uB", matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", matlab_script);
    writeVector(f, "f", matlab_script);
    matlab_script << std::endl;

    // Quadratic inequality constraints
    matlab_script << "% Quadratic inequality constraints" << std::endl;
    for (int i=0; i<r.size(); i++)
    {
        std::ostringstream l_name; l_name << "l" << i+1;
        writeVector(l.at(i), l_name.str(), matlab_script);

        std::ostringstream Q_name; Q_name << "Q" << i+1;
        writeMatrix(Q.at(i), Q_name.str(), matlab_script);

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

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "H=(H+H')/2;" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqcp(H,f,[],[],[],[],l,Q,r,lB,uB);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Solution error: ', mat2str(norm(x-x_cpp))]);" << std::endl;
    else
        matlab_script << "disp(['Solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                          const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin, const Ref<const VectorXd> solution, int exitFlag)
{
    std::ofstream matlab_script;
    matlab_script.open(name.c_str());

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
    writeVector(lB, "lB", matlab_script);
    writeVector(uB, "uB", matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", matlab_script);
    writeVector(f, "f", matlab_script);
    matlab_script << std::endl;

    // Inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", matlab_script);
    writeVector(Bin, "Bin", matlab_script);
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "H=(H+H')/2;" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,Ain,Bin,[],[],lB,uB);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Solution error: ', mat2str(norm(x-x_cpp))]);" << std::endl;
    else
        matlab_script << "disp(['Solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QCP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                           const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin, const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r,
                           const Ref<const VectorXd> solution, int exitFlag)
{
    std::ofstream matlab_script;
    matlab_script.open(name.c_str());

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
    writeVector(lB, "lB", matlab_script);
    writeVector(uB, "uB", matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", matlab_script);
    writeVector(f, "f", matlab_script);
    matlab_script << std::endl;

    // Linear inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", matlab_script);
    writeVector(Bin, "Bin", matlab_script);
    matlab_script << std::endl;

    // Quadratic inequality constraints
    matlab_script << "% Quadratic inequality constraints" << std::endl;
    for (int i=0; i<r.size(); i++)
    {
        std::ostringstream l_name; l_name << "l" << i+1;
        writeVector(l.at(i), l_name.str(), matlab_script);

        std::ostringstream Q_name; Q_name << "Q" << i+1;
        writeMatrix(Q.at(i), Q_name.str(), matlab_script);

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

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "H=(H+H')/2;" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqcp(H,f,Ain,Bin,[],[],l,Q,r,lB,uB);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Solution error: ', mat2str(norm(x-x_cpp))]);" << std::endl;
    else
        matlab_script << "disp(['Solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                          const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin, const Ref<const MatrixXd> Aeq, const Ref<const VectorXd> Beq, const Ref<const VectorXd> solution, int exitFlag)
{
    std::ofstream matlab_script;
    matlab_script.open(name.c_str());

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
    writeVector(lB, "lB", matlab_script);
    writeVector(uB, "uB", matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", matlab_script);
    writeVector(f, "f", matlab_script);
    matlab_script << std::endl;

    // Inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", matlab_script);
    writeVector(Bin, "Bin", matlab_script);
    matlab_script << std::endl;

    // Equality constraints
    matlab_script << "% Equality constraints" << std::endl;
    writeMatrix(Aeq, "Aeq", matlab_script);
    writeVector(Beq, "Beq", matlab_script);
    matlab_script << std::endl;

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "H=(H+H')/2;" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqp(H,f,Ain,Bin,Aeq,Beq,lB,uB);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Solution error: ', mat2str(norm(x-x_cpp))]);" << std::endl;
    else
        matlab_script << "disp(['Solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}

void QCP_writeMatlabScript(std::string name, bool clearStatements, const std::vector<double>& lB, const std::vector<double>& uB, const Ref<const MatrixXd> H, const Ref<const VectorXd> f,
                           const Ref<const MatrixXd> Ain, const Ref<const VectorXd> Bin, const Ref<const MatrixXd> Aeq, const Ref<const VectorXd> Beq, 
                           const std::vector<VectorXd>& l, const std::vector<MatrixXd> Q, const std::vector<double>& r, const Ref<const VectorXd> solution, int exitFlag)
{
    std::ofstream matlab_script;
    matlab_script.open(name.c_str());

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
    writeVector(lB, "lB", matlab_script);
    writeVector(uB, "uB", matlab_script);
    matlab_script << std::endl;

    // Cost function
    matlab_script << "% Cost function" << std::endl;
    writeMatrix(H, "H", matlab_script);
    writeVector(f, "f", matlab_script);
    matlab_script << std::endl;

    // Inequality constraints
    matlab_script << "% Inequality constraints" << std::endl;
    writeMatrix(Ain, "Ain", matlab_script);
    writeVector(Bin, "Bin", matlab_script);
    matlab_script << std::endl;

    // Equality constraints
    matlab_script << "% Equality constraints" << std::endl;
    writeMatrix(Aeq, "Aeq", matlab_script);
    writeVector(Beq, "Beq", matlab_script);
    matlab_script << std::endl;

    // Quadratic inequality constraints
    matlab_script << "% Quadratic inequality constraints" << std::endl;
    for (int i=0; i<r.size(); i++)
    {
        std::ostringstream l_name; l_name << "l" << i+1;
        writeVector(l.at(i), l_name.str(), matlab_script);

        std::ostringstream Q_name; Q_name << "Q" << i+1;
        writeMatrix(Q.at(i), Q_name.str(), matlab_script);

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

    // Solver call
    matlab_script << "% Solver call" << std::endl;
    matlab_script << "H=(H+H')/2;" << std::endl;
    matlab_script << "[x,fval,exitflag]=cplexqcp(H,f,Ain,Bin,Aeq,Beq,l,Q,r,lB,uB);" << std::endl << std::endl;

    // C++ solution
    matlab_script << "% C++ solution" << std::endl;
    writeVector(solution, "x_cpp", matlab_script);
    matlab_script << "exitflag_cpp=" << exitFlag << ";" << std::endl;
    matlab_script << std::endl;

    // Checking result
    matlab_script << "% Checking result" << std::endl;
    if (exitFlag==1)
        matlab_script << "disp(['Solution error: ', mat2str(norm(x-x_cpp))]);" << std::endl;
    else
        matlab_script << "disp(['Solution error: unfeasible problem']);" << std::endl;
    matlab_script << "disp(['Matlab exitflag: ', mat2str(exitflag)]);" << std::endl;
    matlab_script << "disp(['C++ exitflag: ', mat2str(exitflag_cpp)]);" << std::endl;

    matlab_script.close();
}


void writeVector(const Ref<const VectorXd> vect, std::string name, std::ofstream& stream)
{
    stream << name << "=[";
    for (int k=0; k<vect.size(); k++)
    {
        if (k<vect.size()-1)
            stream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << vect(k) << ", ";
        else
            stream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << vect(k);
    }
    stream << "]';" << std::endl;
}

void writeVector(const std::vector<double>& vect, std::string name, std::ofstream& stream)
{
    stream << name << "=[";
    for (int k=0; k<vect.size(); k++)
    {
        if (k<vect.size()-1)
            stream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << vect.at(k) << ", ";
        else
            stream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << vect.at(k);
    }
    stream << "]';" << std::endl;
}

void writeMatrix(const Ref<const MatrixXd> mat, std::string name, std::ofstream& stream)
{
    stream << name << "=[";
    for (int j=0; j<mat.rows(); j++)
    {
        for (int k=0; k<mat.cols(); k++)
        {
            if (k<mat.cols()-1)
                stream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << mat(j,k) << ", ";
            else
                stream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << mat(j,k);
        }

        if (j<mat.rows()-1)
            stream << "; ";
        else
            stream << "];" << std::endl;
    }
}
