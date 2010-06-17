/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include "sot/tutorial/inverted-pendulum.hh"

using sot::tutorial;

InvertedPendulum::CLASS_NAME = "InvertedPendulum";

InvertedPendulum::InvertedPendulum(const std::string& inName) :
  sotEntity(inName), 
  forceSIN(NULL, "InvertedPendulum("+inName+")::input(vector)::forcein"),
  stateSOUT(boost::bind(&InvertedPendulum::computeDynamics, this, _1, _2)),
  cartMass_(1.0), pendulumMass_(1.0)
{
}

InvertedPendulum::~InvertedPendulum()
{
}

