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
  cartMass_(1.0), pendulumMass_(1.0), pendulumLength_(1.0)
{
  // Register signals into the entity.
  signalRegistration (stateSIN);
  signalRegistration (forceSOUT);

  // Set signals as constant to size them
  Vector force = Vector(1); force.setZero();
  Vector state = Vector(4); state.setZero();
  forceSOUT.setConstant(force);
  stateSIN.setConstant(state);

  // Define refresh function for output signal
  boost::function2<Vector&, Vector&,const int&> ftest
    = boost::bind(&FeedbackController::computeForceFeedback,
		  this, _1, _2);

  forceSOUT.setFunction(boost::bind(&FeedbackController::computeForceFeedback,
				    this, _1, _2));
  // setCartMass
  addCommand(std::string("setCartMass"),
	     new ::dynamicgraph::command::Setter<FeedbackController, double>
	     (*this, &FeedbackController::setCartMass));
  // getCartMass
  addCommand(std::string("getCartMass"),
	     new ::dynamicgraph::command::Getter<FeedbackController, double>
	     (*this, &FeedbackController::getCartMass));
  // setPendulumMass
  addCommand(std::string("setPendulumMass"),
	     new ::dynamicgraph::command::Setter<FeedbackController, double>
	     (*this, &FeedbackController::setPendulumMass));
  // getPendulumMass
  addCommand(std::string("getPendulumMass"),
	     new ::dynamicgraph::command::Getter<FeedbackController, double>
	     (*this, &FeedbackController::getPendulumMass));
  // setPendulumLength
  addCommand(std::string("setPendulumLength"),
	     new ::dynamicgraph::command::Setter<FeedbackController, double>
	     (*this, &FeedbackController::setPendulumLength));
  // getPendulumLength
  addCommand(std::string("getPendulumLength"),
	     new ::dynamicgraph::command::Getter<FeedbackController, double>
	     (*this, &FeedbackController::getPendulumLength));
}

FeedbackController::~FeedbackController()
{
}

FeedbackController::Vector&
FeedbackController::computeForceFeedback(Vector& force,
					 const int& inTime)
{
  const Vector& state = stateSIN(inTime);

    if (state.size() != 4)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 4.",
					state.size());
    double m = pendulumMass_;
    double M = cartMass_;
    double l = pendulumLength_;
    double g = Constant::gravity;
    double T = 1.; // length of integration interval
    unsigned nbStep = 50;

    // Linearized system about the origin
    Matrix A = Matrix(4,4); A.setZero();
    A(0,2) = 1.;
    A(1,3) = 1.;
    A(2,1) = m*g/M;
    A(3,1) = (m+M)*g/(M*l);

    Matrix B = Matrix(4,1); B.setZero();
    B(2,0) = 1/M;
    B(3,0) = 1./(M*l);

    Matrix Q = Matrix(4,4); Q.setZero();
    Q(0,0) = 1/(l*l);
    Q(1,1) = 1;

    Matrix invR = Matrix(1,1);
    invR(0,0) = l*l*(M+m)*(M+m)/(T*T*T*T);
    
    Matrix S = Matrix(4,4); S.setZero();
    S(0,0) = 1./(l*l);
    S(1,1) = 1.;
    S(2,2) = T*T/(l*l);
    S(3,3) = T*T;

    // Integration of Ricatti equation
    Matrix P = S;
    double dt = T/nbStep;
    for (unsigned i = 0; i<nbStep; i++) {
      P += dt*ricatti(P, A, B, invR, Q);
    }
    
    force = -invR*B.transpose()*P*inState;
    return force;
}

FeedbackController::Matrix
FeedbackController::ricatti(const Matrix& P, const Matrix& A, const Matrix& B,
			    const Matrix& invR, const Matrix& Q)
{
  Matrix dotP = -P*A - A.transpose()*P + P*B*invR*B.transpose()*P - Q;
  return dotP;
}
