"""
Test DMP (Dynamic Movement Primitives) offline (without simulation or real world experiment, just get the path)
Author: Sha
Date: June, 2020
"""
import numpy as np
import matplotlib.pyplot as plt
import os
import pydmps
import pydmps.dmp_discrete

# desired trajectory, you can compute it in the produce_demon_traj.py file
cwd = os.path.dirname(os.path.realpath(__file__))
y_des = np.loadtxt(cwd+'/ee_traj.txt').T
y_des -= y_des[:, 0][:, None]

# test normal, slow, fast run
# n_dmp: dimension of the dmp, here, we use the end_effector's traj in Cartesian space
# with Quaternion orientation
dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500, ay=np.ones(7) * 20.0)
y_ = dmp.imitate_path(y_des=y_des, plot=False)
y_track_normal, _, _ = dmp.rollout(tau=1)
y_track_slow, _, _ = dmp.rollout(tau=0.1)
y_track_fast, _, _ = dmp.rollout(tau=4)

# test spatial scaling
y_track = []
dmp.reset_state()
for t in range(dmp.timesteps):
    y, _, _ = dmp.step()
    y_track.append(np.copy(y))
    # move the target slightly every time step
    dmp.goal += np.array([0, -0.002, 0, 0, 0, 0, 0])
y_track = np.array(y_track)

plt.figure(1, figsize=(6, 6))

plt.plot(y_track_normal[:, 1], y_track_normal[:, 2], "b", lw=2)
plt.plot(y_track_slow[:, 1], y_track_slow[:, 2], "--r", lw=2)
plt.plot(y_track_fast[:, 1], y_track_fast[:, 2], "y", lw=2)
plt.plot(y_track[:, 1], y_track[:, 2], "*g", lw=2)
plt.legend(['Normal', 'Slow', 'Fast', 'moving_goal'])
plt.title("DMP system - draw half circle")
plt.axis("equal")
plt.xlim([-1, 1])
plt.ylim([-1, 0.5])

plt.figure(2)
plt.subplot(3, 1, 1)
plt.title("DMP system - draw half circle")
plt.plot(y_track_normal[:, 1:3])
plt.ylabel('Normal')
plt.subplot(3, 1, 2)
plt.plot(y_track_slow[:, 1:3])
plt.ylabel('Slow')
plt.subplot(3, 1, 3)
plt.plot(y_track_fast[:, 1:3])
plt.ylabel('Fast')
plt.show()
