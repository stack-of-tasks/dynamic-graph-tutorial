"""
    Copyright 2010 CNRS

    Author: Florent Lamiraux
"""
import wrap
import dynamic_graph.entity as dge

class InvertedPendulum (dge.Entity):
    """
    This class binds dynamicgraph::tutorial::InvertedPendulum C++ class
    """
    def __init__(self, name):
        """
        Constructor: if not called by a child class, create and store a pointer
        to a C++ InvertedPendulum object.
        """
        if not self.object :
            self.object = wrap.createInvertedPendulum(name)
        # Call parent constructor
        dge.Entity.__init__(self, name)

    @property
    def cart_mass(self):
        """
        Get mass of the cart
        """
        return wrap.invertedPendulumGetCartMass(self.object)

    @cart_mass.setter
    def cart_mass(self, mass):
        """
        Set mass of the cart
        """
        return wrap.invertedPendulumSetCartMass(self.object, mass)

    @property
    def pendulum_mass(self):
        """
        Get mass of the pendulum
        """
        return wrap.invertedPendulumGetPendulumMass(self.object)

    @pendulum_mass.setter
    def pendulum_mass(self, mass):
        """
        Set mass of the pendulum
        """
        return wrap.invertedPendulumSetPendulumMass(self.object, mass)

    @property
    def pendulum_length(self):
        """
        Get length of the pendulum
        """
        return wrap.invertedPendulumGetPendulumLength(self.object)

    @pendulum_length.setter
    def pendulum_length(self, length):
        """
        Set length of the pendulum
        """
        return wrap.invertedPendulumSetPendulumLength(self.object, length)

    def incr(self, timeStep):
        """
        Increment time by one time step and re-compute state accordingly
        """
        return wrap.invertedPendulumIncr(self.object, timeStep)

    def print_state(self) :
        return wrap.invertedPendulumPrintState(self.object)
