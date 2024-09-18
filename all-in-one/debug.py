import pysee
import numpy as np

# define the camera and algorithm parameters
see = pysee.init("../config.json")

# load the point cloud
points = np.loadtxt("../test.txt")[:, 0:3].tolist()

# set the current view, position and orientation(pointing direction)
current_v = [-1.25516047, 0.16405614, 0.90033961, 0.81845733, -0.11847473, -0.56222001]

# used for returning the next view
next_v = [0, 0, 0, 0, 0, 0]

# search for the next view
see.search_nbv_once(points, current_v, next_v)

print(f"Next view: {next_v}")
