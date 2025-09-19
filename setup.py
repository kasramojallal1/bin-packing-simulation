import pybullet as p

import numpy as np
import time
import os

import config

# def setup_visuals():
#     p.resetDebugVisualizerCamera(1, 40, -35, [0.35,0.0,0.25])


def setup_visuals():
    # Hide the whole GUI chrome (Explorer/Params/status bar)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Turn off the buffer preview panels on the left
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    # (optional) if you created any user debug sliders/text, clear them:
    # p.removeAllUserDebugItems()

    p.resetDebugVisualizerCamera(1, 50, -35, [0.45, -0.10, 0.15])

    p.setGravity(0, 0, -30.81)
    p.setRealTimeSimulation(10)  # <- syncs physics to wall time

def apply_fast_stable_physics():
    p.setTimeStep(config.TIMESTEP)
    p.setPhysicsEngineParameter(
        fixedTimeStep=config.TIMESTEP,
        numSubSteps=config.SUBSTEPS,
        numSolverIterations=config.SOLVER_IT,
        warmStartingFactor=0.85,
        restitutionVelocityThreshold=0.05,
        enableConeFriction=1,
    )

def ensure_ground():
    if config.PLANE_URDF and os.path.exists(config.PLANE_URDF):
        p.loadURDF(config.PLANE_URDF); return
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[10,10,0.005])
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[10,10,0.005], rgbaColor=[0.85,0.92,1.0,1.0])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                      basePosition=[0,0,-0.005])


def setup_box(size_in, staging_pos):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s / 2 for s in size_in])
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s / 2 for s in size_in], rgbaColor=[0.10, 0.60, 0.90, 1.0])
    incoming_id = p.createMultiBody(baseMass=0.25, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                    basePosition=staging_pos)
    p.changeDynamics(incoming_id, -1, lateralFriction=1.2, rollingFriction=0.002, spinningFriction=0.002,
                     restitution=0.0)

    return incoming_id


def make_demo_state_and_instruction():
    bin_xyz  = {"Lx":0.35, "Ly":0.25, "Lz":0.20}
    placed = [
        {"id":"p0", "pos":[0.00, 0.00, 0.00], "size":[0.10, 0.10, 0.10]},
        {"id":"p1", "pos":[0.10, 0.00, 0.00], "size":[0.10, 0.10, 0.10]},
    ]
    incoming = {"size":[0.10, 0.10, 0.10]}
    anchors = [{"id":"r0_a0", "pos":[0.10, 0.10, 0.00], "surface":"floor"}]
    instruction = {
        "rotation_index": 0,
        "anchor_id": "r0_a0",
        "path": [
            [0.005, 0.105, 0.10]
        ]
    }
    state = {"bin":bin_xyz, "placed":placed, "anchors":anchors, "incoming":incoming}
    return state, instruction


def bin_local_to_world(bin_center_xy, inner_LWH, p_local):
    L, W, H = inner_LWH
    cx, cy = bin_center_xy
    origin = np.array([cx - L/2.0, cy - W/2.0, 0.0])
    return (origin + np.array(p_local)).tolist()

def spawn_boxes_in_bin(bin_center_xy, inner_LWH, placed, color=[0.95,0.55,0.20,1.0]):
    ids = []
    for rec in placed:
        L,W,H = rec["size"]
        pmin_w = bin_local_to_world(bin_center_xy, inner_LWH, rec["pos"])
        center = [pmin_w[0] + L/2.0, pmin_w[1] + W/2.0, pmin_w[2] + H/2.0]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[L/2, W/2, H/2])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[L/2, W/2, H/2], rgbaColor=color)
        bid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                basePosition=center)
        p.changeDynamics(bid,-1,lateralFriction=1.2,rollingFriction=0.002,spinningFriction=0.002,restitution=0.0)
        ids.append(bid)
    return ids

def create_bin(center_xy=(0.70, 0.00),
               inner=(0.35, 0.25), height=0.20,
               wall=0.012, floor_thickness=0.02,
               rgba=(0.85, 0.85, 0.90, 1.0)):
    cx, cy = center_xy
    L, W   = inner
    H      = height
    t      = wall

    floor_half = [(L + 2*t)/2, (W + 2*t)/2, floor_thickness/2]
    floor_col  = p.createCollisionShape(p.GEOM_BOX, halfExtents=floor_half)
    floor_vis  = p.createVisualShape(p.GEOM_BOX, halfExtents=floor_half, rgbaColor=rgba)
    floor_id   = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_col,
                                   baseVisualShapeIndex=floor_vis,
                                   basePosition=[cx, cy, floor_thickness/2])

    long_half  = [L/2, t/2, H/2]
    long_col   = p.createCollisionShape(p.GEOM_BOX, halfExtents=long_half)
    long_vis   = p.createVisualShape(p.GEOM_BOX, halfExtents=long_half, rgbaColor=rgba)
    yoff       = W/2 + t/2
    wall_y_pos = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=long_col,
                                   baseVisualShapeIndex=long_vis,
                                   basePosition=[cx, cy + yoff, H/2])
    wall_y_neg = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=long_col,
                                   baseVisualShapeIndex=long_vis,
                                   basePosition=[cx, cy - yoff, H/2])

    short_half = [t/2, W/2, H/2]
    short_col  = p.createCollisionShape(p.GEOM_BOX, halfExtents=short_half)
    short_vis  = p.createVisualShape(p.GEOM_BOX, halfExtents=short_half, rgbaColor=rgba)
    xoff       = L/2 + t/2
    wall_x_pos = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=short_col,
                                   baseVisualShapeIndex=short_vis,
                                   basePosition=[cx + xoff, cy, H/2])
    wall_x_neg = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=short_col,
                                   baseVisualShapeIndex=short_vis,
                                   basePosition=[cx - xoff, cy, H/2])

    for b in (floor_id, wall_y_pos, wall_y_neg, wall_x_pos, wall_x_neg):
        p.changeDynamics(b, -1, lateralFriction=1.2, restitution=0.0)

    return {"floor": floor_id, "walls": [wall_y_pos, wall_y_neg, wall_x_pos, wall_x_neg]}