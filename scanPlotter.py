

import pandas as pd
import matplotlib.pyplot as plt
import time

# Enable interactive mode
plt.ion()

fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-')  # Empty initial line

ax.set_xlabel("Sample (index)")
ax.set_ylabel("Distance (m)")
ax.set_title("Lidar Ranges")

while True:
    # Read CSV every iteration
    df = pd.read_csv("scan.csv", header=None, names=["index", "distance"])

    # Update line data
    line.set_xdata(df["index"])
    line.set_ydata(df["distance"])

    # Update axes limits if needed
    ax.relim()
    ax.autoscale_view()

    # Draw updated plot
    fig.canvas.draw()
    fig.canvas.flush_events()

    time.sleep(0.1)  # Update every 0.1 seconds
