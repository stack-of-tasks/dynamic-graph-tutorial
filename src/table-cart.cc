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
  cartMass_(58.0), viscosity_(0.1)
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

}

TableCart::~TableCart()
{
}

Vector TableCart::computeDynamics(const Vector& inState,
				  const double& inControl,
				  const double& inForce,
				  double inTimeStep,
				  double& outZmp)
{
  if (inState.size() != 2)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 2.",
					inState.size());

  double dt = inTimeStep;
  double g = Constant::gravity;
  double z = cartHeight_;
  const double& f = inForce;
  double ddx = inControl;
  double m = cartMass_;

  Vector nextState (2);
  nextState (0) = inState (0) + dt*inState (1);
  nextState (1) = inState (1) + dt*inControl;
  outZmp = inState (0) - (z/g)*(ddx - (1/m)*f);
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
