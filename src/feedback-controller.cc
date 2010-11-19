/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include "dynamic-graph/tutorial/feedback-controller.hh"
#include "constant.hh"

using namespace dynamicgraph;
using namespace dynamicgraph::tutorial;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeedbackController, "FeedbackController");

FeedbackController::FeedbackController(const std::string& inName) :
  Entity(inName),
  stateSIN(NULL, "FeedbackController("+inName+")::input(vector)::state"),
  forceSOUT(stateSIN,
	    "FeedbackController("+inName+")::output(vector)::force"),
  gain_(boost::numeric::ublas::zero_matrix<double>(4))
{
  // Register signals into the entity.
  signalRegistration (stateSIN);
  signalRegistration (forceSOUT);

  // Set signals as constant to size them
  Vector force = boost::numeric::ublas::zero_vector<double>(1);
  Vector state = boost::numeric::ublas::zero_vector<double>(4);
  forceSOUT.setConstant(force);
  stateSIN.setConstant(state);

  // Define refresh function for output signal
  boost::function2<Vector&, Vector&,const int&> ftest
    = boost::bind(&FeedbackController::computeForceFeedback,
		  this, _1, _2);

  forceSOUT.setFunction(boost::bind(&FeedbackController::computeForceFeedback,
				    this, _1, _2));
  // setGain
  addCommand(std::string("setGain"),
	     new ::dynamicgraph::command::Setter<FeedbackController, Matrix>
	     (*this, &FeedbackController::setGain));
  // getGain
  addCommand(std::string("getGain"),
	     new ::dynamicgraph::command::Getter<FeedbackController, Matrix>
	     (*this, &FeedbackController::getGain));
}

FeedbackController::~FeedbackController()
{
}

Vector& FeedbackController::computeForceFeedback(Vector& force,
						 const int& inTime)
{
  const Vector& state = stateSIN(inTime);

  if (state.size() != 4)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 4.",
					state.size());
  force = -prod(gain_,state);
  return force;
}
