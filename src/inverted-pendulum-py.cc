/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <Python.h>
#include "dynamic-graph/tutorial/inverted-pendulum.hh"

using dynamicgraph::tutorial::InvertedPendulum;

namespace invertedPendulum {

  static void destroy (void* self);
  static PyObject* error;

  /**
     \brief Create an instance of InvertedPendulum
  */
  static PyObject*
  create(PyObject* self, PyObject* args)
  {
    char *name = NULL;

    if (!PyArg_ParseTuple(args, "s", &name))
      return NULL;

    InvertedPendulum* invertedPendulum = NULL;
    try {
      invertedPendulum = new InvertedPendulum(std::string(name));
    } catch (dynamicgraph::ExceptionFactory& exc) {
      PyErr_SetString(error, exc.getStringMessage().c_str());
      return NULL;
    }

    // Return the pointer
    return PyCObject_FromVoidPtr((void*)invertedPendulum, destroy);
  }

  /**
     \brief Destroy an instance of InvertedPendulum
  */
  static void
  destroy (void* self)
  {
    InvertedPendulum* invertedPendulum = (InvertedPendulum*)self;
    delete invertedPendulum;
  }

  /**
     \brief Binding for InvertedPendulum::getCartMass
  */
  static PyObject*
  getCartMass (PyObject* self, PyObject* args)
  {
    InvertedPendulum* invertedPendulum = NULL;
    PyObject* object = NULL;
    void* pointer = NULL;

    if (!PyArg_ParseTuple(args, "O", &object))
      return NULL;
    if (!PyCObject_Check(object))
      return NULL;

    pointer = PyCObject_AsVoidPtr(object);
    invertedPendulum = (InvertedPendulum*)pointer;
    double result = invertedPendulum->getCartMass();

    return Py_BuildValue("f", result);
  }

  /**
     \brief Binding for InvertedPendulum::setCartMass
  */
  static PyObject*
  setCartMass (PyObject* self, PyObject* args)
  {
    InvertedPendulum* invertedPendulum = NULL;
    PyObject* object = NULL;
    void* pointer = NULL;
    double value = 0;

    if (!PyArg_ParseTuple(args, "Od", &object, &value))
      return NULL;
    if (!PyCObject_Check(object))
      return NULL;

    pointer = PyCObject_AsVoidPtr(object);

    invertedPendulum = (InvertedPendulum*)pointer;
    invertedPendulum->setCartMass(value);

    return Py_BuildValue("");
  }

  /**
     \brief Binding for InvertedPendulum::getPendulumMass
  */
  static PyObject*
  getPendulumMass (PyObject* self, PyObject* args)
  {
    InvertedPendulum* invertedPendulum = NULL;
    PyObject* object = NULL;
    void* pointer = NULL;

    if (!PyArg_ParseTuple(args, "O", &object))
      return NULL;
    if (!PyCObject_Check(object))
      return NULL;

    pointer = PyCObject_AsVoidPtr(object);
    invertedPendulum = (InvertedPendulum*)pointer;
    double result = invertedPendulum->getPendulumMass();

    return Py_BuildValue("f", result);
  }

  /**
     \brief Binding for InvertedPendulum::setPendulumMass
  */
  static PyObject*
  setPendulumMass (PyObject* self, PyObject* args)
  {
    InvertedPendulum* invertedPendulum = NULL;
    PyObject* object = NULL;
    void* pointer = NULL;
    double value;

    if (!PyArg_ParseTuple(args, "Od", &object, &value))
      return NULL;
    if (!PyCObject_Check(object))
      return NULL;

    pointer = PyCObject_AsVoidPtr(object);

    invertedPendulum = (InvertedPendulum*)pointer;
    invertedPendulum->setPendulumMass(value);

    return Py_BuildValue("");
  }

  /**
     \brief Binding for InvertedPendulum::getPendulumLength
  */
  static PyObject*
  getPendulumLength (PyObject* self, PyObject* args)
  {
    InvertedPendulum* invertedPendulum = NULL;
    PyObject* object = NULL;
    void* pointer = NULL;

    if (!PyArg_ParseTuple(args, "O", &object))
      return NULL;
    if (!PyCObject_Check(object))
      return NULL;

    pointer = PyCObject_AsVoidPtr(object);
    invertedPendulum = (InvertedPendulum*)pointer;
    double result = invertedPendulum->getPendulumLength();

    return Py_BuildValue("f", result);
  }

  /**
     \brief Binding for InvertedPendulum::setPendulumLength
  */
  static PyObject*
  setPendulumLength (PyObject* self, PyObject* args)
  {
    InvertedPendulum* invertedPendulum = NULL;
    PyObject* object = NULL;
    void* pointer = NULL;
    double value;

    if (!PyArg_ParseTuple(args, "Od", &object, &value))
      return NULL;
    if (!PyCObject_Check(object))
      return NULL;

    pointer = PyCObject_AsVoidPtr(object);

    invertedPendulum = (InvertedPendulum*)pointer;
    invertedPendulum->setPendulumLength(value);

    return Py_BuildValue("");
  }

  /**
     \brief Binding for InvertedPendulum::incr
  */
  static PyObject*
  incr (PyObject* self, PyObject* args)
  {
    InvertedPendulum* invertedPendulum = NULL;
    PyObject* object = NULL;
    void* pointer = NULL;
    double timeStep;

    if (!PyArg_ParseTuple(args, "Od", &object, &timeStep))
      return NULL;
    if (!PyCObject_Check(object))
      return NULL;

    pointer = PyCObject_AsVoidPtr(object);

    invertedPendulum = (InvertedPendulum*)pointer;
    try {
      invertedPendulum->incr(timeStep);
    } catch (dynamicgraph::ExceptionAbstract& exc) {
      PyErr_SetString(error, exc.getStringMessage().c_str());
      return NULL;
    }

    return Py_BuildValue("");
  }

  /**
     \brief Print value of state signal
  */
  static PyObject*
  printState (PyObject* self, PyObject* args)
  {
    InvertedPendulum* invertedPendulum = NULL;
    PyObject* object = NULL;
    void* pointer = NULL;

    if (!PyArg_ParseTuple(args, "O", &object))
      return NULL;
    if (!PyCObject_Check(object))
      return NULL;

    pointer = PyCObject_AsVoidPtr(object);

    invertedPendulum = (InvertedPendulum*)pointer;
    try {
      std::cout << "calling InvertedPendulum->state()" << std::endl;
      std::cout << "state = " << invertedPendulum->state() << std::endl;
    } catch (dynamicgraph::ExceptionAbstract& exc) {
      PyErr_SetString(error, exc.getStringMessage().c_str());
      return NULL;
    }

    return Py_BuildValue("");
  }
}

/**
   \brief List of python functions
*/
static PyMethodDef dynamicGraphTutorialMethods[] = {
  {"createInvertedPendulum",  invertedPendulum::create, METH_VARARGS,
     "Create an instance of InvertedPendulum."},
  {"invertedPendulumGetCartMass", invertedPendulum::getCartMass, METH_VARARGS,
   "Get mass of the cart of inverted pendulum."},
  {"invertedPendulumSetCartMass", invertedPendulum::setCartMass, METH_VARARGS,
   "Set mass of the cart of inverted pendulum."},
  {"invertedPendulumGetPendulumMass", invertedPendulum::getPendulumMass,
   METH_VARARGS, "Get mass of the pendulum."},
  {"invertedPendulumSetPendulumMass", invertedPendulum::setPendulumMass,
   METH_VARARGS, "Set mass of the pendulum."},
  {"invertedPendulumGetPendulumLength", invertedPendulum::getPendulumLength,
   METH_VARARGS, "Get length of the pendulum."},
  {"invertedPendulumSetPendulumLength", invertedPendulum::setPendulumLength,
   METH_VARARGS, "Set length of the pendulum."},
  {"invertedPendulumIncr", invertedPendulum::incr,
   METH_VARARGS, "Increment time by time step and recompute state."},
  {"invertedPendulumPrintState", invertedPendulum::printState,
   METH_VARARGS, "Print value of state signal."},
  {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC
initwrap(void)
{
    PyObject *m;

    m = Py_InitModule("wrap", dynamicGraphTutorialMethods);
    if (m == NULL)
        return;

    invertedPendulum::error =
      PyErr_NewException("dynamic_graph.tutorial.wrap.error",
			 NULL, NULL);
    Py_INCREF(invertedPendulum::error);
    PyModule_AddObject(m, "error", invertedPendulum::error);
}
