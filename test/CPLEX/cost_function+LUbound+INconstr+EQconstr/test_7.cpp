#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

static void populatebyrow     (IloModel model, IloNumVarArray var, IloRangeArray con);

int main (int argc, char **argv)
{
   IloEnv   env;
   try {
      IloModel model(env);
      IloNumVarArray var(env);
      IloRangeArray con(env);

      populatebyrow (model, var, con);

      IloCplex cplex(model);

      // Optimize the problem and obtain solution.
      if ( !cplex.solve() ) {
         env.error() << "Failed to optimize LP" << endl;
         throw(-1);
      }

      IloNumArray vals(env);
      env.out() << "Solution status = " << cplex.getStatus() << endl;
      env.out() << "Solution value  = " << cplex.getObjValue() << endl;
      cplex.getValues(vals, var);
      env.out() << "Values        = " << vals << endl;
      cplex.getSlacks(vals, con);
      env.out() << "Slacks        = " << vals << endl;
      cplex.getDuals(vals, con);
      env.out() << "Duals         = " << vals << endl;
      cplex.getReducedCosts(vals, var);
      env.out() << "Reduced Costs = " << vals << endl;

      cplex.exportModel("test_7.lp");
   }
   catch (IloException& e) {
      cerr << "Concert exception caught: " << e << endl;
   }
   catch (...) {
      cerr << "Unknown exception caught" << endl;
   }

   env.end();

   return 0;
}


static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c)
{
   IloEnv env = model.getEnv();

   x.add(IloNumVar(env,  -6.9507,  5.8284));
   x.add(IloNumVar(env,  -7.7000, 11.7190));
   x.add(IloNumVar(env, -16.8340,  2.0219));
   x.add(IloNumVar(env,  -3.9797, 18.2750));
   x.add(IloNumVar(env,  -4.2436,  9.4704));
   x.add(IloNumVar(env, -11.3170, 11.6460));
   x.add(IloNumVar(env,  -5.7158, 14.2270));
   
   model.add(IloMaximize(env, -0.0047*x[0] + 0.004*x[1] - 0.0191*x[2] - 0.0113*x[3] + 0.0094*x[4] - 0.0041*x[5] - 0.0185*x[6]
                              - 0.5 * (  7.277*x[0]*x[0] + 7.1116*x[1]*x[1] + 7.2202*x[2]*x[2] + 7.2518*x[3]*x[3]
                                       + 7.2085*x[4]*x[4] + 7.1549*x[5]*x[5] + 7.0186*x[6]*x[6] )
                              - 1.0 * (  0.1235*x[0]*x[1] + 0.2224*x[0]*x[2] + 0.2175*x[0]*x[3] + 0.1771*x[0]*x[4] + 0.1806*x[0]*x[5]
                                       + 0.0358*x[0]*x[6] + 0.1272*x[1]*x[2] + 0.1043*x[1]*x[3] + 0.0883*x[1]*x[4] + 0.0983*x[1]*x[5]
                                       + 0.0291*x[1]*x[6] + 0.1699*x[2]*x[3] + 0.1099*x[2]*x[4] + 0.1807*x[2]*x[5] + 0.0399*x[2]*x[6]
                                       + 0.2086*x[3]*x[4] + 0.1215*x[3]*x[5] + 0.0497*x[3]*x[6] + 0.0706*x[4]*x[5] + 0.032*x[4]*x[6]
                                       + 0.0312*x[5]*x[6] ) ));

   c.add( - 0.449*x[0] + 0.1602*x[1] + 0.5401*x[2] - 0.2458*x[3] + 0.1598*x[4] - 0.5479*x[5] + 0.3081*x[6] <= -0.2284);

   c.add(   0.4041 <= - 0.0985*x[0] - 0.5071* x[1] + 0.5169*x[2] + 0.4079*x[3] + 0.4007*x[4] + 0.1645*x[5] - 0.3347*x[6] <=  0.4041);
   c.add(  -0.8668 <=   0.3681*x[0] + 0.2274* x[1] - 0.1843*x[2] - 0.5899*x[3] + 0.0428*x[4] - 0.3576*x[5] - 0.5488*x[6] <= -0.8668);
   c.add(   0.4720 <=   0.4227*x[0] - 0.2469* x[1] - 0.5271*x[2] - 0.1867*x[3] + 0.2167*x[4] + 0.4735*x[5] + 0.4201*x[6] <=  0.4720);
   c.add(  -0.9360 <=   0.4821*x[0] - 0.2989* x[1] + 0.2266*x[2] + 0.4402*x[3] + 0.4241*x[4] + 0.1384*x[5] - 0.4838*x[6] <= -0.9360);
   c.add(  -0.4808 <=   0.4096*x[0] - 0.1629* x[1] + 0.2833*x[2] + 0.4346*x[3] - 0.3171*x[4] + 0.4887*x[5] - 0.4440*x[6] <= -0.4808);

   model.add(c);
}
