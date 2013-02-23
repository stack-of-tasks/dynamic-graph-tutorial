from math import sqrt
import numpy as np
import matplotlib.pyplot as pl
from math import pi, sin, cos
from dynamic_graph import plug
from dynamic_graph.tutorial import TableCart, FeedbackController
from dynamic_graph.sot.dynamics import flexibility_f, flexibility_h
from dynamic_graph.sot.core import Kalman, Multiply_double_vector, \
    Add_of_vector
from dynamic_graph.sot.tools import Oscillator

zeta = .80
th0 = .05
g = 9.81
m = 56.
kth = 425.
kdth = 0.0
# define inverted pendulum
robot = TableCart ("robot")
robot.setCartMass (m)
robot.setCartHeight (zeta)
robot.setStiffness (kth)
robot.setViscosity (kdth)

# gains
alpha1 = 2.
alpha2 = 4.
alpha3 = 6.
alpha4 = 8.

# Get coefficients of characteristic polynomial
a0 = alpha1*alpha2*alpha3*alpha4
a1 = alpha2*alpha3*alpha4 + alpha1*alpha3*alpha4+alpha1*alpha2*alpha4+alpha1*alpha2*alpha3
a2 = alpha3*alpha4 + alpha2*alpha4 + alpha2*alpha3 + alpha1*alpha4 + alpha1*alpha3 + alpha1*alpha2
a3 = alpha1 + alpha2 + alpha3 + alpha4

# Compute feedback gains
mu = kth/(m*zeta) - g
L = np.matrix ([[0,0,zeta,1],
               [zeta,1,kdth/(m*zeta),0],
               [kdth/(m*zeta),0,mu,-g/zeta],
               [mu,-g/zeta,0,0]])
v = np.matrix ([[a3*zeta-kdth/(m*zeta)],
                [a2*zeta - mu],
                [a1*zeta],
                [a0*zeta]])

gains = np.linalg.solve (L, v)
gains = tuple ([gains.item (i,0) for i in range (4)])

timeStep = 0.001
# Initial state
x0 = (0.,th0, 0., 0., 450.)
P0 = ((.02, 0., 0., 0., 0.,),
      (0., .02, 0., 0., 0.,),
      (0., .0, .03, 0., 0.,),
      (0., 0., 0., .03, 0.,),
      (0., 0., 0., 0., 100,))
Q = ((0., 0., 0., 0., 0.,),
     (0., 0., 0., 0., 0.,),
     (0., 0., 0., 0., 0.,),
     (0., 0., 0., .02, 0.,),
     (0., 0., 0., 0., 0.,),)
R = ((.01, 0.),(0., 1.,),)

robot.state.value = x0 [:4]

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
mult.sin2.value = (1.,)
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
plug (add.sout, robot.control)
plug (ekf.x_est, f.state)
plug (f.newState, h.state)
plug (f.jacobian, ekf.F)
plug (h.jacobian, ekf.H)
ekf.Q.value = Q
ekf.R.value = R
plug (f.newState, ekf.x_pred)
plug (h.observation, ekf.y_pred)
plug (robot.output, ekf.y)
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
        states.append(robot.state.value)
        controls.append (K.control.value)
        outputs.append (robot.output.value)
        stateEstimations.append (ekf.x_est.value)
        robot.incr(timeStep)

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

