/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <Python.h>
#include "sot/tutorial/inverted-pendulum.hh"

using sot::tutorial::InvertedPendulum;

namespace invertedPendulum {

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
}

static PyMethodDef sotTutorialMethods[] = {
  {"createInvertedPendulum",  invertedPendulum::create, METH_VARARGS,
     "Create an instance of InvertedPendulum."},
  {"deleteInvertedPendulum",  invertedPendulum::destroy, METH_VARARGS,
     "Destroy an instance of InvertedPendulum."},
  {"invertedPendulumGetCartMass", invertedPendulum::getCartMass, METH_VARARGS,
   "Get mass of the cart of inverted pendulum."},
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
