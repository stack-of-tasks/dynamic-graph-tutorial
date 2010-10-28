/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#ifndef DG_TUTORIAL_CONSTANT_HH
#define DG_TUTORIAL_CONSTANT_HH

namespace dynamicgraph {
  namespace tutorial {
    class Constant
    {
    public:
      static const double gravity;
    };

const double Constant::gravity = 9.81;
  } // namespace tutorial
} // namespace dynamicgraph

#endif // DG_TUTORIAL_CONSTANT_HH
