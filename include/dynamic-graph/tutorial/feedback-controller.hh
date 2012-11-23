/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#ifndef DYNAMIC_GRAPH_TUTORIAL_FEEDBACK_CONTROLLER_HH
#define DYNAMIC_GRAPH_TUTORIAL_FEEDBACK_CONTROLLER_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

namespace dynamicgraph {
  namespace tutorial {
    /**
       \brief Feedback controller for a table cart

       This class implements a feedback control for the table cart
       represented by class TableCart
    */
    class FeedbackController : public Entity
    {
      DYNAMIC_GRAPH_ENTITY_DECL ();
    public:
      /**
	 \brief Constructor by name
      */
      FeedbackController(const std::string& inName);

      ~FeedbackController();

      /// \name Gains
      /// @{

      /// Set pole of feedback controller
      void setGains (const Vector& inGains) {
	gains_ = inGains;
      }

      /// Get gain relative to center of mass
      Vector getGains () const {
	return gains_;
      }

      /// @}

    private:
      /**
	 Compute the control law
      */
      Vector& computeControlFeedback(Vector& control, const int& inTime);

      /// State of the table cart
      SignalPtr < Vector, int> stateSIN_;
      /// ZMP
      SignalPtr < double, int> zmpSIN_;
      /// Control computed by the control law
      SignalTimeDependent < Vector, int > controlSOUT_;

      /// Gain of the controller relative to center of mass
      Vector gains_;
    };
  } // namespace tutorial
} // namespace dynamicgraph

#endif //DYNAMIC_GRAPH_TUTORIAL_FEEDBACK_CONTROLLER_HH
