/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#ifndef SOT_TUTORIAL_INVERTED_PENDULUM_HH
#define SOT_TUTORIAL_INVERTED_PENDULUM_HH

#include "sot/tutorial/api.hh"

#include <sot/sotEntity.h>

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
      sotSignalPtr< Vector, int > forceSIN;
      /**
	 \brief State of the inverted pendulum
      */
      sotSignalPtr< Vector, int> stateSOUT;

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
      virtual void commandLine( const std::string& cmdLine,
				std::istringstream& cmdArgs,
				std::ostream& os );

    protected:
      /*
	\brief Class name
      */
      static const std::string& CLASS_NAME;

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
    };
  };
};
#endif
