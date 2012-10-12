/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <boost/format.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include "dynamic-graph/tutorial/table-cart.hh"
#include "command-increment.hh"
#include "constant.hh"

using namespace dynamicgraph;
using namespace dynamicgraph::tutorial;

const double Constant::gravity = 9.81;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TableCart, "TableCart");

TableCart::TableCart(const std::string& inName) :
  Entity(inName),
  forceSIN_(NULL, "TableCart("+inName+")::input(float)::force"),
  controlSIN_(NULL, "TableCart("+inName+")::input(float)::control"),
  stateSOUT_("TableCart("+inName+")::output(vector)::state"),
  zmpSOUT_ ("TableCart("+inName+")::output(double)::zmp"),
  cartMass_(58.0), stiffness_ (100.), viscosity_(0.1)
{
  // Register signals into the entity.
  signalRegistration (forceSIN_);
  signalRegistration (controlSIN_);
  signalRegistration (stateSOUT_);
  signalRegistration (zmpSOUT_);

  // Set signals as constant to size them
  Vector state (2); state.fill (0.);
  stateSOUT_.setConstant(state);
  double force = 0;
  forceSIN_.setConstant(force);
  double control =0;
  controlSIN_.setConstant (control);
  double zmp = 0;
  zmpSOUT_.setConstant (zmp);

  // Commands
  std::string docstring;

  // Incr
  docstring =
    "\n"
    "    Integrate dynamics for time step provided as input\n"
    "\n"
    "      take one floating point number as input\n"
    "\n";
  addCommand(std::string("incr"),
	     new command::Increment(*this, docstring));

  // setCartMass
  docstring =
    "\n"
    "    Set cart mass\n"
    "\n";
  addCommand(std::string("setCartMass"),
	     new ::dynamicgraph::command::Setter<TableCart, double>
	     (*this, &TableCart::setCartMass, docstring));

  // getCartMass
  docstring =
    "\n"
    "    Get cart mass\n"
    "\n";
  addCommand(std::string("getCartMass"),
	     new ::dynamicgraph::command::Getter<TableCart, double>
	     (*this, &TableCart::getCartMass, docstring));
  // setCartHeight
  docstring =
    "\n"
    "    Set cart height\n"
    "\n";
  addCommand(std::string("setCartHeight"),
	     new ::dynamicgraph::command::Setter<TableCart, double>
	     (*this, &TableCart::setCartHeight, docstring));

  // getCartHeight
  docstring =
    "\n"
    "    Get cart height\n"
    "\n";
  addCommand(std::string("getCartHeight"),
	     new ::dynamicgraph::command::Getter<TableCart, double>
	     (*this, &TableCart::getCartHeight, docstring));
  // setStiffness
  docstring =
    "\n"
    "    Set stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("setStiffness"),
	     new ::dynamicgraph::command::Setter<TableCart, double>
	     (*this, &TableCart::setStiffness, docstring));

  // getStiffness
  docstring =
    "\n"
    "    Get cart stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("getStiffness"),
	     new ::dynamicgraph::command::Getter<TableCart, double>
	     (*this, &TableCart::getStiffness, docstring));
  // setViscosity
  docstring =
    "\n"
    "    Set viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("setViscosity"),
	     new ::dynamicgraph::command::Setter<TableCart, double>
	     (*this, &TableCart::setViscosity, docstring));

  // getViscosity
  docstring =
    "\n"
    "    Get cart viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("getViscosity"),
	     new ::dynamicgraph::command::Getter<TableCart, double>
	     (*this, &TableCart::getViscosity, docstring));

}

TableCart::~TableCart()
{
}

Vector TableCart::computeDynamics(const Vector& inState,
				  const double& inControl,
				  const double&,
				  double inTimeStep,
				  double& outZmp)
{
  if (inState.size() != 4)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 4.",
					inState.size());

  double dt = inTimeStep;
  double g = Constant::gravity;
  double m = cartMass_;
  double kth = stiffness_;
  double kdth = viscosity_;
  double xi = inState (0);
  double th = inState (1);
  double dxi = inState (2);
  double dth = inState (3);
  double zeta = cartHeight_;
  double ddxi = inControl;
  double ddth = (-kth*th -kdth*dth - m*(cos(th)*xi - sin(th)*zeta)*g + m*(zeta*ddxi - 2*dth*xi*dxi))/(m*(xi*xi+zeta*zeta));
  Vector nextState (4);
  nextState (0) = inState (0) + dt*dxi;
  nextState (1) = inState (1) + dt*dth;
  nextState (2) = inState (2) + dt*ddxi;
  nextState (3) = inState (3) + dt*(ddth);

  double x = cos (th)*xi - sin (th)*zeta;
  double z = sin (th)*xi + cos (th)*zeta;
  double dx = cos (th)*dxi + dth*(-sin(th)*xi - cos (th)*zeta);

  double ddz = sin(th)*ddxi + 2*dth*dx + dth*dth*z + ddth*x;
  double fz = m * (ddz + g);
  double My = kth*inState (1) + kdth*inState (3);
  outZmp = -My/fz;
  return nextState;
}

void TableCart::incr(double inTimeStep)
{
  int t = stateSOUT_.getTime();
  double zmp;
  Vector nextState = computeDynamics (stateSOUT_ (t), controlSIN_ (t),
				      forceSIN_ (t), inTimeStep,
				      zmp);
  stateSOUT_.setConstant (nextState);
  stateSOUT_.setTime (t+1);
  zmpSOUT_.setConstant (zmp);
  zmpSOUT_.setTime (t+1);
  forceSIN_.setTime (t+1);
}
