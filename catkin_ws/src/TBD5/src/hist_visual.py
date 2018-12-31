import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(-np.pi, np.pi, 100) # test data (will be angles)

y=np.random.random_integers(1, 100, 100) # test data (will be ranges)

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, 'r-')

counter=0
while counter<1000:
    line1.set_ydata(np.random.random_integers(1, 100, 100))
    fig.canvas.draw()
    counter=counter+1
