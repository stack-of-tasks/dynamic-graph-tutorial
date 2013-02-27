from math import sqrt
import numpy as np
import matplotlib.pyplot as pl
from math import pi, sin, cos
from dynamic_graph import plug
from dynamic_graph.tutorial import FeedbackController
from dynamic_graph.sot.dynamics import flexibility_f, flexibility_h
from dynamic_graph.sot.core import Kalman, Multiply_double_vector, \
    Add_of_vector
from dynamic_graph.sot.tools import Oscillator
from dynamic_graph.tutorial.simulator import Simulator

th0 = .05

simulator = Simulator (name = "robot", m = 57., zeta = .8077, kth = 510,
                       kdth = 20., Iyy = 6.9417085929720743)

# gains
alpha = (4., 4., 4., 4.,)


gains = simulator.computeGains (alpha)
timeStep = 0.005
# Initial state
x0 = (0.,th0, 0., 0., simulator.kth)
P0 = ((.02, 0., 0., 0., 0.,),
      (0., .02, 0., 0., 0.,),
      (0., .0, .03, 0., 0.,),
      (0., 0., 0., .03, 0.,),
      (0., 0., 0., 0., 1e-3,))
Q = ((0., 0., 0., 0., 0.,),
     (0., 0., 0., 0., 0.,),
     (0., 0., 0., 0., 0.,),
     (0., 0., 0., .02, 0.,),
     (0., 0., 0., 0., 0.,),)
R = ((.01, 0.),(0., 1.,),)

simulator.robot.state.value = x0 [:4]

K = FeedbackController("K")
K.setGains (gains)
#K.setGains ((0.,0.,0.,0.,))
print ("gains: {0}".format (K.getGains ()))

ekf = Kalman ('ekf')
ekf.setInitialState (x0)
ekf.setInitialVariance (P0)
f = flexibility_f ('f')
h = flexibility_h ('h')
f.setTimePeriod (timeStep)
h.setTimePeriod (timeStep)
mult = Multiply_double_vector ('mult')
mult.sin2.value = (0.,)
sinus = Oscillator ('sinus')
sinus.setTimePeriod (timeStep)
sinus.magnitude.value = 10.
sinus.omega.value = 10.
add = Add_of_vector ('sum')
plug (sinus.sout, mult.sin1)
plug (K.control, add.sin1)
plug (mult.sout, add.sin2)
plug (ekf.x_est, K.state)
plug (add.sout, f.control)
plug (add.sout, simulator.robot.control)
plug (ekf.x_est, f.state)
plug (f.newState, h.state)
plug (f.jacobian, ekf.F)
plug (h.jacobian, ekf.H)
f.cosineFoot.value = 1.
f.nbSupport.value = 1
ekf.Q.value = Q
ekf.R.value = R
plug (f.newState, ekf.x_pred)
plug (h.observation, ekf.y_pred)
plug (simulator.robot.output, ekf.y)
#robot.control.value = 0.

timeSteps = []
states = []
controls = []
outputs = []
stateEstimations = []

def play (nbSteps):
    # Loop over time and compute discretized state
    for x in xrange(nbSteps) :
        t = x*timeStep
        timeSteps.append(t)
        states.append(simulator.robot.state.value)
        controls.append (K.control.value)
        outputs.append (simulator.robot.output.value)
        stateEstimations.append (ekf.x_est.value)
        simulator.robot.incr(timeStep)

    # Convert into numpy array
    x = np.array(timeSteps)
    y1 = np.array (states).transpose ()
    y2 = np.array (outputs).transpose ()
    y4 = np.array (controls)
    y5 = np.array (stateEstimations).transpose ()

    fig  = pl.figure ()
    ax1 = fig.add_subplot (211)
    ax2 = fig.add_subplot (212)

    # plot configuration variables
    ax1.plot (x,y1[0])
    ax1.plot (x,y1[1])
    ax1.plot (x,y5[0])
    ax1.plot (x,y5[1])

    # plot velocity variables
    ax2.plot(x,y5 [4])

    leg = ax1.legend(("xi", "theta", "xi_est",
                      "theta_est",
                      ))
    leg = ax2.legend(("kth",))

    pl.show()

if __name__ == '__main__' :
    #play(1000)
    pass

