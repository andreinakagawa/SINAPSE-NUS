import matplotlib.pyplot as plt
import numpy as np
import time

# create the figure
fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(np.random.random((50,50)))
plt.show(block=False)

# draw some data in loop
for i in range(100):
    # wait for a second
    #time.sleep(1)
    # replace the image contents
    im.set_array(np.random.random((50,50)))
    # redraw the figure
    #fig.canvas.draw()
    plt.pause(0.00001)
    print i

raw_input()