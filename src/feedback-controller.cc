/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <dynamic-graph/linear-algebra.h>
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
  zmpSIN_ (NULL, "FeedbackController("+inName+")::input(double)::zmp"),
  controlSOUT_("FeedbackController("+inName+")::output(Vector)::control"),
  gains_(4)
{
  // Register signals into the entity.
  signalRegistration (stateSIN_);
  signalRegistration (zmpSIN_);
  signalRegistration (controlSOUT_);
  controlSOUT_.addDependency (stateSIN_);

  gains_.fill (0.);

  // Set signals as constant to size them
  Vector control (1); control.setZero ();
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

Vector& FeedbackController::computeControlFeedback(Vector& control,
						   const int& inTime)
{
  Vector state = stateSIN_ (inTime);

  if (state.size() < 4)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be at least 4.",
					state.size());
  state.resize (4, false);
  control.resize (1);
  control (0) = - gains_.scalarProduct (state);
  return control;
}
