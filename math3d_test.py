import math3d as m3d
import numpy as np

tf = m3d.Transform()
tf.pos = np.array([10,20,30])
tf.orient.rotate_zb(np.deg2rad(-90))
tf.orient.rotate_xb(np.deg2rad(-180))

print(tf)