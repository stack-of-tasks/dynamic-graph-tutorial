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
  controlSOUT_(stateSIN_,
	    "FeedbackController("+inName+")::output(float)::control"),
  comGain_(0.), comDotGain_ (0.), zmpGain_ (0.)
{
  // Register signals into the entity.
  signalRegistration (stateSIN_);
  signalRegistration (zmpSIN_);
  signalRegistration (controlSOUT_);

  // Set signals as constant to size them
  double control = 0;
  controlSOUT_.setConstant(control);
  Vector state(2); state.fill(0.);
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
    "    Get com gain of controller\n"
    "      - return a floating point number\n"
    "\n";
  addCommand(std::string("getComGain"),
	     new ::dynamicgraph::command::Getter<FeedbackController, double>
	     (*this, &FeedbackController::getComGain, docstring));
  // setGain
  docstring =
    "\n"
    "    Set com velocity gain\n"
    "      - input\n"
    "        a floating point number\n"
    "\n";
  addCommand(std::string("setComDotGain"),
	     new ::dynamicgraph::command::Setter<FeedbackController, double>
	     (*this, &FeedbackController::setComDotGain, docstring));

  // getGain
  docstring =
    "\n"
    "    Get com velocity gain of controller\n"
    "      - return a floating point number\n"
    "\n";
  addCommand(std::string("getComDotGain"),
	     new ::dynamicgraph::command::Getter<FeedbackController, double>
	     (*this, &FeedbackController::getComDotGain, docstring));
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
  control = - comGain_ * state (0) - comDotGain_ * state (1) + zmpGain_ * zmp;
  return control;
}
