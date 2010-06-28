/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#ifndef DG_TUTORIAL_INVERTED_PENDULUM_HH
#define DG_TUTORIAL_INVERTED_PENDULUM_HH

#include <boost/numeric/ublas/vector.hpp>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>

#include "dynamic-graph/tutorial/api.hh"

namespace dynamicgraph {
  namespace tutorial {

    /**
       \brief Inverted Pendulum on a cart

       This class represents the classical inverted pendulum on a cart.
       The equation of motion is:

       \f{eqnarray*}{
       \left ( M + m \right ) \ddot x - m l \ddot \theta \cos \theta + m l \dot \theta^2 \sin \theta &=& F\\
       m l (-g \sin \theta - \ddot x \cos \theta + l \ddot \theta) &=& 0 
       \f}
    */

    class DG_TUTORIAL_EXPORT InvertedPendulum : public Entity
    {
    public:
      typedef boost::numeric::ublas::vector<double> Vector;
      /**
	 \brief Constructor by name
      */
      InvertedPendulum(const std::string& inName);

      ~InvertedPendulum();

      virtual const std::string& getClassName (void) const {
	return CLASS_NAME;
      }

      void incr();
      /**
	  \name Signals
	  @{
      */
      /**
	 \brief Input of the inverted pendulum
      */
      SignalPtr< Vector, int > forceSIN;
      /**
	 \brief State of the inverted pendulum
      */
      Signal< Vector, int> stateSOUT;

      /**
	 @}
      */

      /**
	  \name Parameters
	  @{
      */
      /**
	 \brief Set the mass of the cart
      */
      void setCartMass (double inMass) {
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
      void setPendulumMass (double inMass) {
	pendulumMass_ = inMass;
      }

      /**
	 \brief Get the mass of the pendulum
      */
      double getPendulumMass () const {
	return pendulumMass_;
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
	 \name Parameters
	 @{
      */
      /// \brief Mass of the cart
      double cartMass_;
      /// \brief Mass of the pendulum
      double pendulumMass_;
      /**
	 @}
      */
      /**
	 \brief State of the system
      */
      Vector state_;

      /**
	 \brief Compute the evolution of the state of the pendulum
      */
      Vector computeDynamics(const Vector& inState, const Vector& inControl);
    };
  };
};
#endif
