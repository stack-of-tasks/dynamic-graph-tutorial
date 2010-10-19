/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <Python.h>

/**
   \brief List of python functions
*/
static PyMethodDef dynamicGraphTutorialMethods[] = {
  {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC
initwrap(void)
{
    PyObject *m;

    m = Py_InitModule("wrap", dynamicGraphTutorialMethods);
    if (m == NULL)
        return;
}
