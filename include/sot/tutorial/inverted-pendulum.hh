/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#ifndef SOT_TUTORIAL_INVERTED_PENDULUM_HH
#define SOT_TUTORIAL_INVERTED_PENDULUM_HH

#include <boost/numeric/ublas/vector.hpp>
#include <sot/sotEntity.h>
#include <sot/sotSignalTimeDependant.h>

#include "sot/tutorial/api.hh"

/**
  \brief namespace sot
 */

namespace sot {
  namespace tutorial {

    /**
       \brief Main class of package sot-tutorial
    */

    class SOT_TUTORIAL_EXPORT InvertedPendulum : public sotEntity
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

      /**
	  \name Signals
	  @{
      */
      /**
	 \brief Input of the inverted pendulum
      */
      sotSignalTimeDependant< Vector, int > forceSIN;
      /**
	 \brief State of the inverted pendulum
      */
      sotSignalTimeDependant< Vector, int> stateSOUT;

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
      Vector& computeDynamics(Vector& outState, int time);
    };
  };
};
#endif
