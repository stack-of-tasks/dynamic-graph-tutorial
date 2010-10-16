/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <boost/numeric/ublas/io.hpp>
#include "dynamic-graph/tutorial/inverted-pendulum.hh"
#include "dynamic-graph/factory.h"

using namespace dynamicgraph;
using namespace dynamicgraph::tutorial;

const double InvertedPendulum::gravity = 9.81;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(InvertedPendulum, "InvertedPendulum");

InvertedPendulum::InvertedPendulum(const std::string& inName) :
  Entity(inName),
  forceSIN(NULL, "InvertedPendulum("+inName+")::input(vector)::forcein"),
  stateSOUT("InvertedPendulum("+name+")::output(vector)::state"),
  cartMass_(1.0), pendulumMass_(1.0), pendulumLength_(1.0), viscosity_(0.1)
{
  // Register signals into the entity.
  signalRegistration (forceSIN);
  signalRegistration (stateSOUT);

  // Set signals as constant to size them
  Vector state = ZeroVector(4);
  Vector input = ZeroVector(1);
  stateSOUT.setConstant(state);
  forceSIN.setConstant(input);
}

InvertedPendulum::~InvertedPendulum()
{
}

const InvertedPendulum::Vector& InvertedPendulum::state()
{
  return stateSOUT.accessCopy();
}

InvertedPendulum::Vector
InvertedPendulum::computeDynamics(const Vector& inState,
				  const Vector& inControl,
				  double inTimeStep)
{
  if (inState.size() != 4)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 4.",
					inState.size());

  if (inControl.size() != 1)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"force signal size is ",
					"%d, should be 1.",
					inControl.size());
  double dt = inTimeStep;
  double dt2 = dt*dt;
  double g = gravity;
  double x = inState[0];
  double th = inState[1];
  double dx = inState[2];
  double dth = inState[3];
  double F = inControl[0];
  double m = pendulumMass_;
  double M = cartMass_;
  double l = pendulumLength_;
  double lambda = viscosity_;
  double l2 = l*l;
  double dth2 = dth*dth;
  double sth = sin(th);
  double cth = cos(th);
  double sth2 = sth*sth;

  double b1 = F - m*l*dth2*sth - lambda*dx;
  double b2 = m*l*g*sth - lambda*dth;

  double det = m*l2*(M + m*sth2);

  double ddx = (b1*m*l2 + b2*m*l*cth)/det;
  double ddth = ((M+m)*b2 + m*l*cth*b1)/det;

  Vector nextState(4);
  nextState[0] = x + dx*dt + .5*ddx*dt2;
  nextState[1] = th + dth*dt + .5*ddth*dt2;
  nextState[2] = dx + dt*ddx;
  nextState[3] = dth + dt*ddth;

  return nextState;
}

void InvertedPendulum::incr(double inTimeStep)
{
  int t = stateSOUT.getTime();
  Vector nextState = computeDynamics(stateSOUT(t), forceSIN(t), inTimeStep);
  stateSOUT.setConstant(nextState);
  stateSOUT.setTime(t+1);
}

dynamicgraph::DefaultCastRegisterer<InvertedPendulum::Vector> IPVectorCast;
