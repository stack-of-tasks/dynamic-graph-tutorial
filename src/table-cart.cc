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
  forceSIN_(NULL, "TableCart("+inName+")::input(double)::force"),
  controlSIN_(NULL, "TableCart("+inName+")::input(Vector)::control"),
  stateSOUT_("TableCart("+inName+")::output(vector)::state"),
  zmpSOUT_ ("TableCart("+inName+")::output(vector)::zmp"),
  cartMass_(58.0), viscosity_(0.1)
{
  // Register signals into the entity.
  signalRegistration (forceSIN_);
  signalRegistration (controlSIN_);
  signalRegistration (stateSOUT_);
  signalRegistration (zmpSOUT_);

  // Set signals as constant to size them
  Vector state (1); state.fill (0.);
  stateSOUT_.setConstant(state);
  Vector force (1); force.fill (0.);
  forceSIN_.setConstant(force);
  Vector control (1); control.fill (0.);
  controlSIN_.setConstant (control);
  Vector zmp (1); zmp.fill (0.);
  zmpSOUT_.setConstant (zmp);

  prevControl_ = control;

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
				  const Vector& inControl,
				  const Vector& inForce,
				  const Vector& inPrevControl,
				  double inTimeStep,
				  Vector& outZmp)
{
  if (inState.size() != 1)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 1.",
					inState.size());
  if (inControl.size() != 1)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"control signal size is ",
					"%d, should be 1.",
					inControl.size());
  if (inForce.size() != 1)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"force signal size is ",
					"%d, should be 1.",
					inForce.size());

  if (inForce.size() != 1)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"force signal size is ",
					"%d, should be 1.",
					inForce.size());

  if (outZmp.size() != 1)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"force signal size is ",
					"%d, should be 1.",
					outZmp.size());

  double dt = inTimeStep;
  double g = Constant::gravity;
  double z = cartHeight_;
  const Vector& f = inForce;
  Vector ddx = (1/dt)*(inControl - inPrevControl);
  double m = cartMass_;

  Vector nextState = inState + dt*inControl;
  outZmp = nextState - (z/g)*(ddx - (1/m)*f);
  return nextState;
}

void TableCart::incr(double inTimeStep)
{
  int t = stateSOUT_.getTime();
  Vector zmp (1);
  Vector nextState = computeDynamics (stateSOUT_ (t), controlSIN_ (t),
				      forceSIN_ (t), prevControl_, inTimeStep,
				      zmp);
  stateSOUT_.setConstant (nextState);
  stateSOUT_.setTime (t+1);
  zmpSOUT_.setConstant (zmp);
  zmpSOUT_.setTime (t+1);
  forceSIN_.setTime (t+1);
  prevControl_ = controlSIN_ (t);
}
