import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph.tutorial.inverted_pendulum as ip

# define inverted pendulum
a = ip.InvertedPendulum("IP")
a.cart_mass = 1.0
a.pendulum_mass = 0.1
a.pendulum_length = 0.2

# Set value of state signal
a.state = [0.0, 0.01, 0.0, 0.0]
