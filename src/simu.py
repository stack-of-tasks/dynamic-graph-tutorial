import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.tutorial as dgt
import dynamic_graph.signal_base as dgsb

# define inverted pendulum
a = dgt.InvertedPendulum("IP")
a.setCartMass(1.0)
a.setPendulumMass(1.0)
a.setPendulumLength(1.0)

b = dgt.FeedbackController("K")

# plug signals
dg.plug('IP.state', 'K.state')
dg.plug('K.force', 'IP.force')

timeStep = 0.001

# Set value of state signal
a.signal('state').value = '[4](0.0,0.1,0.0,0.0)'

gain = ((0.0,27.0,0.001,0.001,),)
b.setGain(gain,)

def play (nbSteps):
    timeSteps = []
    values = []

    # Set value of state signal
    a.signal('state').value = '[4](0.0,0.1,0.0,0.0)'
    # Loop over time and compute discretized state values
    for x in xrange(nbSteps) :
        t = x*timeStep
        timeSteps.append(t)
        values.append(dgsb.stringToTuple(a.signal('state').value))
        a.incr(timeStep)

    # Convert into numpy array
    x = np.array(timeSteps)
    y = np.array(values).transpose()

    fig  = pl.figure()
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)

    # plot configuration variables
    ax1.plot(x,y[0])
    ax1.plot(x,y[1])

    # plot velocity variables
    ax2.plot(x,y[2])
    ax2.plot(x,y[3])

    leg = ax1.legend(("x", "theta"))
    leg = ax2.legend(("dx", "dtheta"))

    pl.show()

