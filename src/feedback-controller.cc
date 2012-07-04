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
  zmpSIN_ (NULL, "FeedbackController("+inName+")::input(vector)::zmp"),
  controlSOUT_(stateSIN_,
	    "FeedbackController("+inName+")::output(vector)::control"),
  comGain_(1.), zmpGain_ (1.)
{
  // Register signals into the entity.
  signalRegistration (stateSIN_);
  signalRegistration (zmpSIN_);
  signalRegistration (controlSOUT_);

  // Set signals as constant to size them
  Vector control (1.); control.fill (0.);
  controlSOUT_.setConstant(control);
  Vector state(2); state.fill(0.);
  stateSIN_.setConstant(state);

  // Define refresh function for output signal
  boost::function2<Vector&, Vector&,const int&> ftest
    = boost::bind(&FeedbackController::computeControlFeedback,
		  this, _1, _2);

  controlSOUT_.setFunction(boost::bind
			   (&FeedbackController::computeControlFeedback,
			    this, _1, _2));
  std::string docstring;
  // setGain
  docstring =
    "\n"
    "    Set com gain\n"
    "      - input\n"
    "        a floating point number\n"
    "\n";
  addCommand(std::string("setComGain"),
	     new ::dynamicgraph::command::Setter<FeedbackController, double>
	     (*this, &FeedbackController::setComGain, docstring));

  // getGain
  docstring =
    "\n"
    "    Get gain of controller\n"
    "      - return a floating point number\n"
    "\n";
  addCommand(std::string("getComGain"),
	     new ::dynamicgraph::command::Getter<FeedbackController, double>
	     (*this, &FeedbackController::getComGain, docstring));
  // setGain
  docstring =
    "\n"
    "    Set zmp gain\n"
    "      - input\n"
    "        a floating point number\n"
    "\n";
  addCommand(std::string("setZmpGain"),
	     new ::dynamicgraph::command::Setter<FeedbackController, double>
	     (*this, &FeedbackController::setZmpGain, docstring));

  // getGain
  docstring =
    "\n"
    "    Get gain of controller\n"
    "      - return a floating point number\n"
    "\n";
  addCommand(std::string("getZmpGain"),
	     new ::dynamicgraph::command::Getter<FeedbackController, double>
	     (*this, &FeedbackController::getZmpGain, docstring));
}

FeedbackController::~FeedbackController()
{
}

Vector& FeedbackController::computeControlFeedback(Vector& control,
						   const int& inTime)
{
  const Vector& state = stateSIN_ (inTime);
  const Vector& zmp = zmpSIN_ (inTime);

  if (state.size() != 1)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 1.",
					state.size());
  control = - comGain_ * state + zmpGain_ * zmp;
  return control;
}
