"""
    Copyright 2010 CNRS

    Author: Florent Lamiraux
"""
import dynamic_graph as dg
import wrap

def initEntity(self, name):
    """
    Common constructor of Entity classes
    """
    dg.entity.Entity.__init__(self, self.class_name, name)

entityTypeList = dg.factory_get_entity_class_list()
for e in entityTypeList:
    class metacls(type):
        def __new__(mcs, name, bases, dict):
            return type.__new__(mcs, name, bases, dict)

    # Create new class
    a = metacls(e, (dg.entity.Entity,), {})
    # Store class name in class member
    a.class_name = e
    print ("new class %s"%e)
    # set class constructor
    setattr(a, '__init__', initEntity)
    # Store new class in dictionary with class name
    dg.__dict__[e] = a
