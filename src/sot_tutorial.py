"""
    Copyright 2010 CNRS

    Author: Florent Lamiraux
"""
import sot_tutorial_wrap as stw

class InvertedPendulum:
    """
    Constructor: create and store a pointer to an C++ object InvertedPendulum
    """
    def __init__(self, name):
        self.object = stw.createInvertedPendulum(name)

    """
    Destructor: delete the C++ object allocated at creation
    """
    def __del__(self):
        stw.deleteInvertedPendulum(self.object)

    def getCartMass(self):
        return stw.invertedPendulumGetCartMass(self.object)


