import numpy as np
import matplotlib.pyplot as pl
from math import pi, sin
from dynamic_graph import plug
from dynamic_graph.tutorial import TableCart, FeedbackController

# define inverted pendulum
robot = TableCart ("robot")
robot.setCartMass (56.0)
robot.setCartHeight (0.80)

K = FeedbackController("K")
K.setComGain (.01)
K.setZmpGain (.01)

plug(robot.state, K.state)
plug(robot.zmp, K.zmp)
plug(K.control, robot.control)

timeStep = 0.001

def play (nbSteps):
    timeSteps = []
    states = []
    forces = []
    zmps = []
    controls = []

    # Loop over time and compute discretized state
    for x in xrange(nbSteps) :
        t = x*timeStep
        if t <= 1.:
            robot.force.value = (sin (t/pi),)
        timeSteps.append(t)
        states.append(robot.state.value)
        forces.append(robot.force.value)
        zmps.append(robot.zmp.value)
        controls.append (K.control.value)
        robot.incr(timeStep)

    # Convert into numpy array
    x = np.array(timeSteps)
    y1 = np.array (states).transpose ()
    y2 = np.array (forces).transpose ()
    y3 = np.array (zmps).transpose ()
    y4 = np.array (controls).transpose ()

    fig  = pl.figure ()
    ax1 = fig.add_subplot (121)
    ax2 = fig.add_subplot (122)

    # plot configuration variables
    ax1.plot (x,y1[0])
    ax1.plot (x,y3[0])

    # plot velocity variables
    ax2.plot(x,y2[0])
    ax2.plot(x,y4[0])

    leg = ax1.legend(("x", "zmp"))
    leg = ax2.legend(("f", "dx"))

    pl.show()

if __name__ == '__main__' :
    play(100)

