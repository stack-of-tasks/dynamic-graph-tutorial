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

      /// Set gain relative to center of mass
      void setComGain (const double& inGain) {
	comGain_ = inGain;
      }

      /// Get gain relative to center of mass
      double getComGain () const {
	return comGain_;
      }

      /// Set gain relative to ZMP
      void setZmpGain (const double& inGain) {
	zmpGain_ = inGain;
      }

      /// Get gain relative to ZMP
      double getZmpGain () const {
	return zmpGain_;
      }

      /// @}

    private:
      /**
	 Compute the control law
      */
      Vector& computeControlFeedback(Vector& control, const int& inTime);

      /// State of the table cart
      SignalPtr < ::dynamicgraph::Vector, int> stateSIN_;
      /// ZMP
      SignalPtr < ::dynamicgraph::Vector, int> zmpSIN_;
      /// Control computed by the control law
      SignalTimeDependent < ::dynamicgraph::Vector, int > controlSOUT_;

      /// Gain of the controller relative to center of mass
      double comGain_;
      /// Gain of the controller relative to center of pressure
      double zmpGain_;
    };
  } // namespace tutorial
} // namespace dynamicgraph

#endif //DYNAMIC_GRAPH_TUTORIAL_FEEDBACK_CONTROLLER_HH
