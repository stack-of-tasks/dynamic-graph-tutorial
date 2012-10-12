from math import sqrt
import numpy as np
import matplotlib.pyplot as pl
from math import pi, sin, cos
from dynamic_graph import plug
from dynamic_graph.tutorial import TableCart, FeedbackController

zeta = .80
th0 = .05
g = 9.81
m = 56.
kth = 850.
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
A = np.matrix ([[0,0,zeta,1],
               [zeta,1,kdth/(m*zeta),0],
               [kdth/(m*zeta),0,mu,-g/zeta],
               [mu,-g/zeta,0,0]])
v = np.matrix ([[a3*zeta-kdth/(m*zeta)],
                [a2*zeta - mu],
                [a1*zeta],
                [a0*zeta]])

gains = np.linalg.solve (A, v)
gains = tuple ([gains.item (i,0) for i in range (4)])

# Initial state
robot.state.value = (0.,th0, 0., 0.,)

K = FeedbackController("K")
K.setGains (gains)
#K.setGains ((0.,0.,0.,0.,))
print ("gains: {0}".format (K.getGains ()))

plug(robot.state, K.state)
plug(K.control, robot.control)
#robot.control.value = 0.

timeStep = 0.001

def play (nbSteps):
    timeSteps = []
    states = []
    controls = []
    zmps = []

    # Loop over time and compute discretized state
    for x in xrange(nbSteps) :
        t = x*timeStep
        timeSteps.append(t)
        states.append(robot.state.value)
        controls.append (K.control.value)
        zmps.append (robot.zmp.value)
        robot.incr(timeStep)

    # Convert into numpy array
    x = np.array(timeSteps)
    y1 = np.array (states).transpose ()
    y2 = np.array (zmps)
    y4 = np.array (controls)

    fig  = pl.figure ()
    ax1 = fig.add_subplot (211)
    ax2 = fig.add_subplot (212)

    # plot configuration variables
    ax1.plot (x,y1[0])
    ax1.plot (x,y1[1])
    ax1.plot (x,y2)

    # plot velocity variables
    ax2.plot(x,y4)

    leg = ax1.legend(("xi", "theta", "zmp"))
    leg = ax2.legend(("u",))

    pl.show()

if __name__ == '__main__' :
    play(20000)

