/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include "dynamic-graph/tutorial/inverted-pendulum.hh"

using namespace dynamicgraph::tutorial;

const std::string InvertedPendulum::CLASS_NAME = "InvertedPendulum";

InvertedPendulum::InvertedPendulum(const std::string& inName) :
  Entity(inName),
  forceSIN(NULL, "InvertedPendulum("+inName+")::input(vector)::forcein"),
  stateSOUT("InvertedPendulum("+name+")::output(vector)::state"),
  cartMass_(1.0), pendulumMass_(1.0)
{
  // Register signals into the entity.
  signalRegistration (forceSIN);
  signalRegistration (stateSOUT);
}

InvertedPendulum::~InvertedPendulum()
{
}

InvertedPendulum::Vector
InvertedPendulum::computeDynamics(const Vector& inState,
				  const Vector& inControl)
{
  return inState;
}

void InvertedPendulum::incr()
{
  int t = stateSOUT.getTime();
  Vector nextState = computeDynamics(stateSOUT, forceSIN(t));
  stateSOUT.setConstant(nextState);
  stateSOUT.setTime(t+1);
}
