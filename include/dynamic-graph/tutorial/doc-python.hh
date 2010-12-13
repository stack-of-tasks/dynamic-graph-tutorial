/**
\page dg_tutorial_inverted_pendulum_python Python module

\section dg_tutorial_inverted_pendulum_python_intro Introduction

Generating python bindings for new Entity classes is straightforward. We only need to add the following lines into file <c>src/CMakeLists.txt</c>:

<code>
SET(NEW_ENTITY_CLASS
  InvertedPendulum
  FeedbackController
  )

DYNAMIC_GRAPH_PYTHON_MODULE("tutorial")
</code>
\sa <a href="../sphinx-html/index.html">documentation of python bindings</a>,
<c>src/simu.py</c> for an example python script.
 */
