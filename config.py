import pybullet_data
import os
import math
import pybullet as p

PLANE_URDF = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
REPO       = os.path.abspath(os.path.dirname(__file__))
URDF_DIR   = os.path.join(REPO, "urdf")
MESHES_DIR = os.path.join(REPO, "meshes")
UR5_URDF   = os.path.join(URDF_DIR, "ur5_suction.urdf")


TIMESTEP   = 1.0/1000.0      # 1 kHz fixed step
SUBSTEPS   = 4
SOLVER_IT  = 220
SPEEDUP    = 2.0
FORCE_ARM  = 550
KP         = 0.55
KV         = 1.0


# With suction_tcp as the actual EE link, no offset needed.
TCP_OFFSET = [0.0, 0.0, 0.0]
DOWN       = p.getQuaternionFromEuler([math.pi, 0.0, 0.0])


bin_center_xy = (0.50, -0.40)
bin_inner = (0.40, 0.30)
bin_height = 0.20