/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#ifndef DG_TUTORIAL_FEEDBACK_CONTROLLER_HH
#define DG_TUTORIAL_FEEDBACK_CONTROLLER_HH

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>

#include "dynamic-graph/tutorial/api.hh"

namespace dynamicgraph {
  namespace tutorial {
    /**
       \brief Feedback controller for an inverted pendulum
       
       This class implements a feedback control for the inverted pendulum
       represented by class InvertedPendulum
    */
    class DG_TUTORIAL_EXPORT FeedbackController : public Entity
    {
    public:
      typedef boost::numeric::ublas::vector<double> Vector;
      typedef boost::numeric::ublas::matrix<double> Matrix;
      typedef boost::numeric::ublas::zero_vector<double> ZeroVector;
      typedef boost::numeric::ublas::zero_matrix<double> ZeroMatrix;

      /**
	 \brief Constructor by name
      */
      FeedbackController(const std::string& inName);

      ~FeedbackController();

      /// Each entity should provide the name of the class it belongs to
      virtual const std::string& getClassName (void) const {
	return CLASS_NAME;
      }

      /**
	  \name Parameters
	  @{
      */
      /**
	 \brief Set the mass of the cart
      */
      void setCartMass (const double& inMass) {
	cartMass_ = inMass;
      }

      /**
	 \brief Get the mass of the cart
      */
      double getCartMass () const {
	return cartMass_;
      }

      /**
	 \brief Set the mass of the cart
      */
      void setPendulumMass (const double& inMass) {
	pendulumMass_ = inMass;
      }

      /**
	 \brief Get the mass of the pendulum
      */
      double getPendulumMass () const {
	return pendulumMass_;
      }

      /**
	 \brief Set the length of the cart
      */
      void setPendulumLength (const double& inLength) {
	pendulumLength_ = inLength;
      }

      /**
	 \brief Get the length of the pendulum
      */
      double getPendulumLength () const {
	return pendulumLength_;
      }

      /**
	 @}
      */

    protected:
      /*
	\brief Class name
      */
      static const std::string CLASS_NAME;

    private:
      /**
	 Compute the control law
      */
      Vector& computeForceFeedback(Vector& force, const int& inTime);

      Matrix ricatti(const Matrix& P, const Matrix& A, const Matrix& B,
		     const Matrix& R, const Matrix& Q);
      /**
	 \brief State of the inverted pendulum
      */
      Signal< Vector, int> stateSIN;
      /**
	 \brief Force computed by the control law
      */
      Signal< Vector, int > forceSOUT;

      /// \brief Mass of the cart
      double cartMass_;
      /// \brief Mass of the pendulum
      double pendulumMass_;
      /// \brief Length of the pendulum
      double pendulumLength_;
    };
  } // namespace tutorial
} // namespace dynamicgraph

#endif //DG_TUTORIAL_FEEDBACK_CONTROLLER_HH
