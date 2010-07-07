"""
    Copyright 2010 CNRS

    Author: Florent Lamiraux
"""

import re
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

    @property
    def state(self):
        """
        Read value of state signal and parse the string into a tuple
        """
        s = self.signal("state")
        stringValue = s.value
        vec = filter(lambda a:a is not "",
                     re.split("[^eE0-9.+\-]", stringValue))
        if int(vec[0]) != len(vec)-1 :
            raise RuntimeError("Size of state vector is inconsistent.")
        return map (float, vec[1:])

    @state.setter
    def state(self, value):
        """
        Set the value of the state signal from a given input vector
        """
        if len(value) is not 4:
            raise RuntimeError("Size of state should be 4.")



        stringValue = "[" + str(len(value)) + "]("
        for x in value :
            try :
                stringValue += str(float(x)) + ","
            except exc :
                raise RunTimeError("Input should be a vector of float.")
        stringValue = stringValue[0:-1] + ")"
        print ("stringValue = %s" % stringValue)
        s = self.signal("state")
        s.value = stringValue

    def incr(self, timeStep):
        """
        Increment time by one time step and re-compute state accordingly
        """
        return wrap.invertedPendulumIncr(self.object, timeStep)

    def print_state(self) :
        return wrap.invertedPendulumPrintState(self.object)
