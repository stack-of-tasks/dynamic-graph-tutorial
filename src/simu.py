import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.tutorial as dgt
import dynamic_graph.signal_base as dgsb

def A(ip, dt):
    jac = []
    for i in range(4):
        x = [0,0,0,0]
        x[i] = dt
        ip.signal('state').value = dgsb.tupleToString(x)
        ip.incr(dt)
        y = dgsb.stringToTuple(ip.signal('state').value)
        b = np.array(y)
        a = np.array(x)
        deriv = (b-a)/(dt*dt)
        jac.append(deriv.tolist())
    return np.array(jac).transpose()

def B(ip, dt):
    jac = []
    x = [0,0,0,0]
    for i in range(1):
        ip.signal('state').value = dgsb.tupleToString(x)
        f = [0]
        f[i] = dt
        ip.signal('force').value = dgsb.tupleToString(f)
        ip.incr(dt)
        y = dgsb.stringToTuple(ip.signal('state').value)
        b = np.array(y)
        a = np.array(x)
        deriv = (b-a)/(dt*dt)
        jac.append(deriv.tolist())
    return np.array(jac).transpose()

# define inverted pendulum
a = dgt.InvertedPendulum("IP")
a.setCartMass(1.0)
a.setPendulumMass(1.0)
a.setPendulumLength(1.0)

# Set value of state signal
a.signal('state').value = '[4](0.0,0.01,0.0,0.0)'

timeStep = 0.01
timeSteps = []
values = []

# Loop over time and compute discretized state values
for x in xrange(10000) :
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

