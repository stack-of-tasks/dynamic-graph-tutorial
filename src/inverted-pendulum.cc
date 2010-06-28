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

  // Set signals as constant to size them
  Vector state(4);
  Vector input(1);
  stateSOUT.setConstant(state);
  forceSIN.setConstant(input);
}

InvertedPendulum::~InvertedPendulum()
{
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
  double x = inState[0];
  double theta = inState[1];
  double dx = inState[2];
  double dtheta = inState[3];
  double F = inControl[0];

  return inState;
}

void InvertedPendulum::incr(double inTimeStep)
{
  int t = stateSOUT.getTime();
  Vector nextState = computeDynamics(stateSOUT, forceSIN(t), inTimeStep);
  stateSOUT.setConstant(nextState);
  stateSOUT.setTime(t+1);
}
