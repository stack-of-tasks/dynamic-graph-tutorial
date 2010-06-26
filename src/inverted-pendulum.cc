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
  stateSOUT(boost::bind(&InvertedPendulum::computeDynamics, this, _1, _2),
	    sotNOSIGNAL, "InvertedPendulum("+name+")::output(vector)::state"),
  cartMass_(1.0), pendulumMass_(1.0)
{
  std::cout << "InvertedPendulum constructor" << std::endl;
}

InvertedPendulum::~InvertedPendulum()
{
  std::cout << "InvertedPendulum destructor" << std::endl;
}

InvertedPendulum::Vector& InvertedPendulum::
computeDynamics(Vector& outState, int time)
{
  return outState;
}

