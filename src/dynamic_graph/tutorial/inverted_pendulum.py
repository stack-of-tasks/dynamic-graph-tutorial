"""
    Copyright 2010 CNRS

    Author: Florent Lamiraux
"""
import wrap as stw

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
        if hasattr(self, "object"):
            stw.deleteInvertedPendulum(self.object)
    
    @property
    def cart_mass(self):
        """
        Get mass of the cart
        """
        return stw.invertedPendulumGetCartMass(self.object)

    @cart_mass.setter
    def cart_mass(self, mass):
        """
        Set mass of the cart
        """
        return stw.invertedPendulumSetCartMass(self.object, mass)

    @property
    def pendulum_mass(self):
        """
        Get mass of the pendulum
        """
        return stw.invertedPendulumGetPendulumMass(self.object)

    @cart_mass.setter
    def pendulum_mass(self, mass):
        """
        Set mass of the pendulum
        """
        return stw.invertedPendulumSetPendulumMass(self.object, mass)
