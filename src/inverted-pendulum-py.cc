/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <Python.h>
#include "dynamic-graph/tutorial/inverted-pendulum.hh"

using sot::tutorial::InvertedPendulum;

namespace invertedPendulum {

  /**
     \brief Create an instance of InvertedPendulum
  */
  static PyObject*
  create(PyObject* self, PyObject* args)
  {
    char *name = NULL;

    if (!PyArg_ParseTuple(args, "s", &name))
      return NULL;

    InvertedPendulum* obj = new InvertedPendulum(std::string(name));

    // Return the pointer as an integer
    return Py_BuildValue("i", (unsigned int)obj);
  }

  /**
     \brief Deallocate an instance of InvertedPendulum
  */
  static PyObject*
  destroy (PyObject* self, PyObject* args)
  {
    InvertedPendulum* obj = NULL;
    unsigned int pointer;

    if (!PyArg_ParseTuple(args, "I", &pointer))
      return NULL;

    obj = (InvertedPendulum*)pointer;
    delete obj;

    return Py_BuildValue("");
  }

  /**
     \brief Binding for InvertedPendulum::getCartMass
  */
  static PyObject*
  getCartMass (PyObject* self, PyObject* args)
  {
    InvertedPendulum* obj = NULL;
    unsigned int pointer;

    if (!PyArg_ParseTuple(args, "I", &pointer))
      return NULL;

    obj = (InvertedPendulum*)pointer;
    double result = obj->getCartMass();

    return Py_BuildValue("f", result);
  }

  /**
     \brief Binding for InvertedPendulum::setCartMass
  */
  static PyObject*
  setCartMass (PyObject* self, PyObject* args)
  {
    InvertedPendulum* obj = NULL;
    unsigned int pointer;
    double value;

    if (!PyArg_ParseTuple(args, "Id", &pointer, &value))
      return NULL;

    obj = (InvertedPendulum*)pointer;
    obj->setCartMass(value);

    return Py_BuildValue("");
  }

  /**
     \brief Binding for InvertedPendulum::getPendulumMass
  */
  static PyObject*
  getPendulumMass (PyObject* self, PyObject* args)
  {
    InvertedPendulum* obj = NULL;
    unsigned int pointer;

    if (!PyArg_ParseTuple(args, "I", &pointer))
      return NULL;

    obj = (InvertedPendulum*)pointer;
    double result = obj->getPendulumMass();

    return Py_BuildValue("f", result);
  }

  /**
     \brief Binding for InvertedPendulum::setPendulumMass
  */
  static PyObject*
  setPendulumMass (PyObject* self, PyObject* args)
  {
    InvertedPendulum* obj = NULL;
    unsigned int pointer;
    double value;

    if (!PyArg_ParseTuple(args, "Id", &pointer, &value))
      return NULL;

    obj = (InvertedPendulum*)pointer;
    obj->setPendulumMass(value);

    return Py_BuildValue("");
  }
}

/**
   \brief List of python functions
*/
static PyMethodDef sotTutorialMethods[] = {
  {"createInvertedPendulum",  invertedPendulum::create, METH_VARARGS,
     "Create an instance of InvertedPendulum."},
  {"deleteInvertedPendulum",  invertedPendulum::destroy, METH_VARARGS,
     "Destroy an instance of InvertedPendulum."},
  {"invertedPendulumGetCartMass", invertedPendulum::getCartMass, METH_VARARGS,
   "Get mass of the cart of inverted pendulum."},
  {"invertedPendulumSetCartMass", invertedPendulum::setCartMass, METH_VARARGS,
   "Set mass of the cart of inverted pendulum."},
  {"invertedPendulumGetPendulumMass", invertedPendulum::getPendulumMass,
   METH_VARARGS, "Get mass of the pendulum."},
  {"invertedPendulumSetPendulumMass", invertedPendulum::setPendulumMass,
   METH_VARARGS, "Set mass of the pendulum."},
  {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC
initsot_tutorial_wrap(void)
{
    PyObject *m;

    m = Py_InitModule("sot_tutorial_wrap", sotTutorialMethods);
    if (m == NULL)
        return;
}
