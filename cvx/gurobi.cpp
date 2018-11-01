/* Copyright 2018, Gurobi Optimization, LLC */

/* Facility location: a company currently ships its product from 5 plants
   to 4 warehouses. It is considering closing some plants to reduce
   costs. What plant(s) should the company close, in order to minimize
   transportation and fixed costs?

   Based on an example from Frontline Systems:
   http://www.solver.com/disfacility.htm
   Used with permission.
 */

#include "gurobi_c++.h"
#include <sstream>
using namespace std;

int main(int argc, char* argv[])
{
  GRBEnv* env = 0;
  GRBVar* open = 0;
  GRBVar** transport = 0;
  int transportCt = 0;
  try
  {
    // Number of plants and warehouses
    const int nPlants = 5;
    const int nWarehouses = 4;

    // Warehouse demand in thousands of units
    double Demand[] = { 15, 18, 14, 20 };

    // Plant capacity in thousands of units
    double Capacity[] = { 20, 22, 17, 19, 18 };

    // Fixed costs for each plant
    double FixedCosts[] = { 12000, 15000, 17000, 13000, 16000 };

    // Transportation costs per thousand units
    double TransCosts[][nPlants] = { { 4000, 2000, 3000, 2500, 4500 },
                                     { 2500, 2600, 3400, 3000, 4000 },
                                     { 1200, 1800, 2600, 4100, 3000 },
                                     { 2200, 2600, 3100, 3700, 3200 } };

    // Model
    env = new GRBEnv();
    GRBModel model = GRBModel(*env);
    model.set(GRB_StringAttr_ModelName, "facility");

    // Plant open decision variables: open[p] == 1 if plant p is open.
    open = model.addVars(nPlants, GRB_BINARY);

    int p;
    for (p = 0; p < nPlants; ++p)
    {
      ostringstream vname;
      vname << "Open" << p;
      open[p].set(GRB_DoubleAttr_Obj, FixedCosts[p]);
      open[p].set(GRB_StringAttr_VarName, vname.str());
    }

    // Transportation decision variables: how much to transport from
    // a plant p to a warehouse w
    transport = new GRBVar*[nWarehouses];
    int w;
    for (w = 0; w < nWarehouses; ++w)
    {
      transport[w] = model.addVars(nPlants);
      transportCt++;

      for (p = 0; p < nPlants; ++p)
      {
        ostringstream vname;
        vname << "Trans" << p << "." << w;
        transport[w][p].set(GRB_DoubleAttr_Obj, TransCosts[w][p]);
        transport[w][p].set(GRB_StringAttr_VarName, vname.str());
      }
    }

    // The objective is to minimize the total fixed and variable costs
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Production constraints
    // Note that the right-hand limit sets the production to zero if
    // the plant is closed
    for (p = 0; p < nPlants; ++p)
    {
      GRBLinExpr ptot = 0;
      for (w = 0; w < nWarehouses; ++w)
      {
        ptot += transport[w][p];
      }
      ostringstream cname;
      cname << "Capacity" << p;
      model.addConstr(ptot <= Capacity[p] * open[p], cname.str());
    }

    // Demand constraints
    for (w = 0; w < nWarehouses; ++w)
    {
      GRBLinExpr dtot = 0;
      for (p = 0; p < nPlants; ++p)
      {
        dtot += transport[w][p];
      }
      ostringstream cname;
      cname << "Demand" << w;
      model.addConstr(dtot == Demand[w], cname.str());
    }

    // Guess at the starting point: close the plant with the highest
    // fixed costs; open all others

    // First, open all plants
    for (p = 0; p < nPlants; ++p)
    {
      open[p].set(GRB_DoubleAttr_Start, 1.0);
    }

    // Now close the plant with the highest fixed cost
    cout << "Initial guess:" << endl;
    double maxFixed = -GRB_INFINITY;
    for (p = 0; p < nPlants; ++p)
    {
      if (FixedCosts[p] > maxFixed)
      {
        maxFixed = FixedCosts[p];
      }
    }
    for (p = 0; p < nPlants; ++p)
    {
      if (FixedCosts[p] == maxFixed)
      {
        open[p].set(GRB_DoubleAttr_Start, 0.0);
        cout << "Closing plant " << p << endl << endl;
        break;
      }
    }

    // Use barrier to solve root relaxation
    model.set(GRB_IntParam_Method, GRB_METHOD_BARRIER);

    // Solve
    model.optimize();

    // Print solution
    cout << "\nTOTAL COSTS: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
    cout << "SOLUTION:" << endl;
    for (p = 0; p < nPlants; ++p)
    {
      if (open[p].get(GRB_DoubleAttr_X) > 0.99)
      {
        cout << "Plant " << p << " open:" << endl;
        for (w = 0; w < nWarehouses; ++w)
        {
          if (transport[w][p].get(GRB_DoubleAttr_X) > 0.0001)
          {
            cout << "  Transport " << transport[w][p].get(GRB_DoubleAttr_X) << " units to warehouse " << w << endl;
          }
        }
      }
      else
      {
        cout << "Plant " << p << " closed!" << endl;
      }
    }
  }
  catch (GRBException e)
  {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  }
  catch (...)
  {
    cout << "Exception during optimization" << endl;
  }

  delete[] open;
  for (int i = 0; i < transportCt; ++i)
  {
    delete[] transport[i];
  }
  delete[] transport;
  delete env;
  return 0;
}