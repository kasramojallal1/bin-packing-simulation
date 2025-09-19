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


# ---------- Conveyor settings ----------
# Center of belt (x,y,z), z is the *center* height of the belt body
BELT_CENTER = (0.55, 0.32, 0.015)    # near where you were staging the box
BELT_SIZE   = (0.50, 0.20, 0.03)     # Lx, Ly, thickness

# Direction the belt moves boxes (unit-ish in XY). (-1,0) = push toward -X.
BELT_DIR    = (-1.0, 0.0)
BELT_SPEED  = 0.25                   # m/s along BELT_DIR

# Where you want the box to stop for picking (XY only)
PICKUP_XY   = (0.35, 0.32)



# With suction_tcp as the actual EE link, no offset needed.
TCP_OFFSET = [0.0, 0.0, 0.0]
DOWN       = p.getQuaternionFromEuler([math.pi, 0.0, 0.0])


bin_center_xy = (0.50, -0.40)
bin_inner = (0.350, 0.250)
bin_height = 0.20