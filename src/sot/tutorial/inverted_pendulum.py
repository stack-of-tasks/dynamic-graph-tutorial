"""
    Copyright 2010 CNRS

    Author: Florent Lamiraux
"""
import dynamic_graph.tutorial.wrap as stw

class InvertedPendulum:
    """
    This class binds dynamicgraph::tutorial::InvertedPendulum C++ class
    """
    def __init__(self, name):
        """
        Constructor: create and store a pointer to a C++ object InvertedPendulum
        """
        self.object = stw.createInvertedPendulum(name)

    def __del__(self):
        """
        Destructor: delete the C++ object allocated at creation
        """
        stw.deleteInvertedPendulum(self.object)
    
    def getCartMass(self):
        """
        Get mass of the cart
        """
        return stw.invertedPendulumGetCartMass(self.object)

    def setCartMass(self, mass):
        """
        Set mass of the cart
        """
        return stw.invertedPendulumSetCartMass(self.object, mass)

    def getPendulumMass(self):
        """
        Get mass of the pendulum
        """
        return stw.invertedPendulumGetPendulumMass(self.object)

    def setPendulumMass(self, mass):
        """
        Set mass of the pendulum
        """
        return stw.invertedPendulumSetPendulumMass(self.object, mass)
