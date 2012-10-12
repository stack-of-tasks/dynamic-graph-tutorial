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
  stateSIN_(NULL, "FeedbackController("+inName+")::input(vector)::state"),
  zmpSIN_ (NULL, "FeedbackController("+inName+")::input(float)::zmp"),
  controlSOUT_("FeedbackController("+inName+")::output(float)::control"),
  gains_(4)
{
  // Register signals into the entity.
  signalRegistration (stateSIN_);
  signalRegistration (zmpSIN_);
  signalRegistration (controlSOUT_);
  controlSOUT_.addDependency (stateSIN_);

  gains_.fill (0.);

  // Set signals as constant to size them
  double control = 0;
  controlSOUT_.setConstant(control);
  Vector state(2); state.fill(0.);
  stateSIN_.setConstant(state);

  controlSOUT_.setFunction(boost::bind
			   (&FeedbackController::computeControlFeedback,
			    this, _1, _2));

  std::string docstring;
  // setGain
  docstring =
    "\n"
    "    Set gains of closed loop system\n"
    "      - input\n"
    "        a vector of dimension 4\n"
    "\n";
  addCommand(std::string("setGains"),
	     new ::dynamicgraph::command::Setter<FeedbackController, Vector>
	     (*this, &FeedbackController::setGains, docstring));

  // getGain
  docstring =
    "\n"
    "    Get gains of closed loop system\n"
    "      - return a vector of dimension 4\n"
    "\n";
  addCommand(std::string("getGains"),
	     new ::dynamicgraph::command::Getter<FeedbackController, Vector>
	     (*this, &FeedbackController::getGains, docstring));
}

FeedbackController::~FeedbackController()
{
}

double& FeedbackController::computeControlFeedback(double& control,
						   const int& inTime)
{
  const Vector& state = stateSIN_ (inTime);

  if (state.size() != 4)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 2.",
					state.size());
  control = - gains_.scalarProduct (state);
  return control;
}
