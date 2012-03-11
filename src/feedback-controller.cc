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
  zmpSIN_ (NULL, "FeedbackController("+inName+")::input(double)::zmp"),
  controlSOUT_(stateSIN_,
	    "FeedbackController("+inName+")::output(double)::force"),
  comGain_(1.), zmpGain_ (1.)
{
  // Register signals into the entity.
  signalRegistration (stateSIN_);
  signalRegistration (controlSOUT_);

  // Set signals as constant to size them
  double control = 0.;
  Vector state(2);
  state.fill(0.);
  controlSOUT_.setConstant(control);
  stateSIN_.setConstant(state);

  // Define refresh function for output signal
  boost::function2<double&, double&,const int&> ftest
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

double& FeedbackController::computeControlFeedback(double& control,
						   const int& inTime)
{
  const Vector& state = stateSIN_ (inTime);
  const double& zmp = zmpSIN_ (inTime);

  if (state.size() != 2)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 2.",
					state.size());
  double x = state (0);
  control = -comGain_ * x - zmpGain_ * zmp;
  return control;
}
