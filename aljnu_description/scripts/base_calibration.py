import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis
from shapely.geometry import Polygon
import numpy as np

np.set_printoptions(suppress=True)

l = 930
w = 720
p = 0.001 * np.array(
    [
        [375.6, 350.05, -700.1],
        [358.8, -363.92, -695.1],
        # [359.54, -309.85, -696.86],
        # [362.8, -163.65, -697.15],
        # [393.04, 203.9, -699.2],
        [-561.69, -352.84, -692.13],
        [-561.95, 359.95, -691.92],
    ]
)

pxy = p[:, :2]
poly = Polygon(pxy)
px = poly.centroid.x
py = poly.centroid.y
pz = np.average(p[:, 2])

P_basefootprint_2_basearm = np.array([px, py, pz])
H_basefootprint_2_basearm = np.eye(4)
H_basefootprint_2_basearm[:3, 3] = P_basefootprint_2_basearm
print("H_basefootprint_2_basearm")
print(H_basefootprint_2_basearm)


wheel_vertical_offset = -0.0702
wheel_radius = 1.6459e-01
zoffset = wheel_vertical_offset - wheel_radius
P_basefootprint_2_baselink = np.array([0, 0, zoffset])
H_basefootprint_2_baselink = np.eye(4)
H_basefootprint_2_baselink[:3, 3] = P_basefootprint_2_baselink
print("H_basefootprint_2_baselink")
print(H_basefootprint_2_baselink)


H_basearm_2_baselink = H_basefootprint_2_baselink @ np.linalg.inv(H_basefootprint_2_basearm)
print("H_basearm_2_baselink")
print(H_basearm_2_baselink)

ax = make_3d_axis(ax_s=1, unit="m", n_ticks=6)
plot_transform(ax=ax, name="base_arm")
for i in range(p.shape[0]):
    ax.scatter(p[i, 0], p[i, 1], p[i, 2], s=10, c="r")
    ax.text(p[i, 0], p[i, 1], p[i, 2], f"p{i}", fontsize=12)
ax.set_xlabel("X-axis (m)")
ax.set_ylabel("Y-axis (m)")
ax.set_zlabel("Z-axis (m)")
plt.show()
