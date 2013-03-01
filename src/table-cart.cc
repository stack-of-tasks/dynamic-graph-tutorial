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
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include "dynamic-graph/tutorial/table-cart.hh"
#include "command-increment.hh"
#include "constant.hh"

using namespace dynamicgraph;
using namespace dynamicgraph::tutorial;
using dynamicgraph::command::makeDirectSetter;
using dynamicgraph::command::makeDirectGetter;

const double Constant::gravity = 9.81;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TableCart, "TableCart");

TableCart::TableCart(const std::string& inName) :
  Entity(inName),
  forceSIN_(NULL, "TableCart("+inName+")::input(double)::force"),
  controlSIN_(NULL, "TableCart("+inName+")::input(vector)::control"),
  stateSOUT_("TableCart("+inName+")::output(vector)::state"),
  outputSOUT_ ("TableCart("+inName+")::output(vector)::output"),
  cartMass_(58.0), stiffness_ (100.), viscosity_(0.1), Iyy_ (0.),
  A_ (4, 4), B_ (4, 1)
{
  A_.setZero (); B_.setZero ();
  // Register signals into the entity.
  signalRegistration (forceSIN_);
  signalRegistration (controlSIN_);
  signalRegistration (stateSOUT_);
  signalRegistration (outputSOUT_);

  // Set signals as constant to size them
  Vector state (2); state.fill (0.);
  stateSOUT_.setConstant(state);
  double force = 0;
  forceSIN_.setConstant(force);
  Vector control (1); control.setZero ();
  controlSIN_.setConstant (control);
  Vector output (2); output.setZero ();
  outputSOUT_.setConstant (output);

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

  // setMomentOfInertia
  docstring =
    "\n"
    "    Set moment of inertia around y axis\n"
    "\n";
  addCommand ("setMomentOfInertia",
	      makeDirectSetter (*this, &Iyy_, docstring));

  // setMomentOfInertia
  docstring =
    "\n"
    "    Get moment of inertia around y axis\n"
    "\n";
  addCommand ("getMomentOfInertia",
	      makeDirectGetter (*this, &Iyy_, docstring));
}

TableCart::~TableCart()
{
}

Vector TableCart::computeDynamics(const Vector& inState,
				  const Vector& inControl,
				  const double&,
				  double inTimeStep,
				  Vector& output)
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
  double Iyy = Iyy_;
  double zeta = cartHeight_;

  A_ (0, 2) = 1.; A_ (1, 3) = 1.;
  A_ (3, 0) = - g/(zeta*zeta + m*Iyy);
  A_ (3, 1) = (m*g*zeta - kth)/(zeta*zeta + m*Iyy);
  A_ (3, 3) = -kdth/(zeta*zeta + m*Iyy);
  B_ (2, 0) = 1.; B_ (3, 0) = 1/zeta;

  double xi = inState (0);
  double th = inState (1);
  double dth = inState (3);

  double My = kth*th + kdth*dth;

  const Vector& xn = inState;
  const Vector& k1 = A_*xn + B_*inControl;
  const Vector& k2 = A_*(xn + k1*(dt/2)) + B_*inControl;
  const Vector& k3 = A_*(xn + k2*(dt/2)) + B_*inControl;
  const Vector& k4 = A_*(xn + k3*dt) + B_*inControl;

  Vector nextState = xn + (k1 + k2*2 + k3*2 + k4)*(dt/6.);

  output.resize (2);
  output (0) = xi;
  output (1) = My;
  return nextState;
}

void TableCart::incr(double inTimeStep)
{
  int t = stateSOUT_.getTime();
  Vector output;
  Vector nextState = computeDynamics (stateSOUT_ (t), controlSIN_ (t),
				      forceSIN_ (t), inTimeStep,
				      output);
  stateSOUT_.setConstant (nextState);
  stateSOUT_.setTime (t+1);
  outputSOUT_.setConstant (output);
  outputSOUT_.setTime (t+1);
  forceSIN_.setTime (t+1);
}
