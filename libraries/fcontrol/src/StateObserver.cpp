#include "StateObserver.h"

StateObserver::StateObserver(std::vector<vector<double>> G,std::vector<vector<double>> H,std::vector<vector<double>>C,std::vector<vector<double>>D,double dts)
{
    StateSpace(G,H,C,D,dts);
}

double StateObserver::output(double new_input, double _y)
{
  y=C*x+D*new_input;
  error=_y-y(0);
  x=G*x +H*u+K*error;
  return 2;
}
