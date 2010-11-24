/**
\page dg_tutorial_inverted_pendulum_python Python module

\section dg_tutorial_inverted_pendulum_python_intro Introduction

This page explains how to embed new C++ classes into python langage. We explain
how to access our new class InvertedPendulum from a python interpreter.

\section dg_tutorial_inverted_pendulum_python_implementation Implementation

We create a python module (called <c>dynamic_graph.tutorial</c>)
containing the library where C++ code of the class is
compiled. Details about compilation and installation can be found in
file <c>src/CMakeLists.txt</c>.

File <c>src/inverted-pendulum-py.cc</c> defines an empty sub-module
called wrap. This file contains almost no specific information about our
application and can be reused almost as such for other applications.

File <c>src/dynamic_graph/tutorial/__init__.py</c> is the entry point of the
module.

The following commands import shared libraries containing the code of
dynamic-graph and of <c>InvertedPendulum</c> class
\code
import dynamic_graph as dg
import wrap
\endcode


\section dg_tutorial_inverted_pendulum_python_bindings Bindings python

The following line will automatically create a python class binding <c>InvertedPendulum</c> and <c>FeedbackController</c>:
\code
dg.entity.updateEntityClasses(globals())
\endcode

For each new entity class, a python class is automatically generated.
At creation of the first instance of a class, for each command, a method with
the same name and same number of arguments is added.
\code
InvertedPendulum('')
FeedbackController('')
\endcode
Creating one instance of each class at initialization of the module trigger  automatic documentation using <c>sphinx-build</c>.

\sa <a href="../sphinx-html/index.html">documentation of python bindings</a>,
<c>src/simu.py</c> for an example python script.
 */
