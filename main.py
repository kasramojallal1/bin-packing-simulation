import os, time
import numpy as np
import pybullet as p
import pybullet_data

import setup
import config
import functions

def ee_link_index(rid):
    # Prefer wrist_3_link, then suction_tcp/tool0/ee_link, else last
    prefs = [b"wrist_3_link", b"suction_tcp", b"tool0", b"ee_link"]
    best = p.getNumJoints(rid) - 1
    for j in range(p.getNumJoints(rid)):
        if any(tok in p.getJointInfo(rid, j)[12] for tok in prefs):
            best = j
    return best


def arm_joint_indices(rid):
    return [j for j in range(p.getNumJoints(rid))
            if p.getJointInfo(rid,j)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)]


def tcp_above_box_center(bin_center_xy, inner_LWH, box_min_local, box_size, clearance=0.015):
    L,W,H = box_size
    pmin_w = np.array(setup.bin_local_to_world(bin_center_xy, inner_LWH, box_min_local))
    center = pmin_w + np.array([L/2, W/2, H/2])
    top    = center + np.array([0,0,H/2])
    tcp    = (top + np.array([0,0,clearance])).tolist()
    return tcp, config.DOWN


def place_by_instruction(rid, joints, ee,
                         bin_center_xy, inner_LWH, bin_height,
                         box_size, instruction,
                         hover_Z, hover_margin,
                         approach_clear, release_clear,
                         descend_steps,
                         grasp_constraint_id):
    """
    - 'instruction["path"]' : list of bin-local MIN-CORNER waypoints [[x,y,z], ...]
    - Motion policy: up to SAFE hover -> lateral @ hover to each XY -> vertical descend.
      For intermediate waypoints we descend to 'approach_clear' above the box top,
      for the final waypoint we descend to 'release_clear' and release the grasp.
    """
    assert "path" in instruction and len(instruction["path"]) > 0, "instruction.path missing/empty"

    L, W, Hbin = inner_LWH
    bx, by, bz = box_size

    #  chooses the larger of the two, so the TCP always clears the bin rim (even with tall boxes).
    safe_hover = max(hover_Z, bin_height + bz + hover_margin)


    # It converts a bin-local min-corner waypoint into world coordinates, then uses the box size to compute the box’s center and the top-surface point directly above that center.
    def world_top_from_min(min_local):
        # min_local is bin-local min-corner of the *placed* box
        pmin_w = setup.bin_local_to_world(bin_center_xy, inner_LWH, min_local)
        center = [pmin_w[0] + bx/2.0, pmin_w[1] + by/2.0, pmin_w[2] + bz/2.0]
        top    = [center[0], center[1], center[2] + bz/2.0]
        return center, top

    # 0) go straight up to safe hover from current pose
    Lp, _ = p.getLinkState(rid, ee, True)[4:6]
    functions.goto_tcp(rid, joints, ee, [Lp[0], Lp[1], safe_hover], config.DOWN, steps=180)

    path = instruction["path"]
    for i, min_local in enumerate(path):
        _, top = world_top_from_min(min_local)
        print(top)

        # 1) lateral move at SAFE hover to XY of this waypoint
        functions.goto_tcp(rid, joints, ee, [top[0], top[1], safe_hover], config.DOWN, steps=200)

        # 2) vertical descend
        if i < len(path) - 1:
            # intermediate waypoint: stop a bit above the box top, then go back to hover
            pre_z = top[2] + approach_clear
            functions.goto_tcp(rid, joints, ee, [top[0], top[1], pre_z], config.DOWN, steps=descend_steps)
            functions.goto_tcp(rid, joints, ee, [top[0], top[1], safe_hover], config.DOWN, steps=120)
        else:
            # FINAL waypoint: descend closer and release
            pre_z   = top[2] + approach_clear
            final_z = top[2] + release_clear
            functions.goto_tcp(rid, joints, ee, [top[0], top[1], pre_z],   config.DOWN, steps=descend_steps)
            functions.goto_tcp(rid, joints, ee, [top[0], top[1], final_z], config.DOWN, steps=descend_steps)

            # release grasp and let it settle a few frames
            if grasp_constraint_id is not None:
                p.removeConstraint(grasp_constraint_id)
            for _ in range(45):
                p.stepSimulation()

            # exit vertically to hover
            functions.goto_tcp(rid, joints, ee, [top[0], top[1], safe_hover], config.DOWN, steps=200)




# ---------- main ----------
def main():
    p.connect(p.GUI)
    setup.setup_visuals()

    # search paths
    if config.PLANE_URDF:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

    for d in [config.REPO, config.URDF_DIR, config.MESHES_DIR, os.path.join(config.MESHES_DIR,"ur5")]:
        p.setAdditionalSearchPath(d)

    plane_id = p.loadURDF(config.PLANE_URDF)
    # remove texture and set a nicer color
    p.changeVisualShape(plane_id, -1, textureUniqueId=-1,
                        rgbaColor=[0.18, 0.20, 0.22, 1.0])  # pick your color

    setup.apply_fast_stable_physics()
    setup.ensure_ground()

    setup.create_bin(center_xy=config.bin_center_xy, inner=config.bin_inner, height=config.bin_height, wall=0.012)

    bin_state, instructions = setup.make_demo_state_and_instruction()
    inner_LWH = (bin_state["bin"]["Lx"], bin_state["bin"]["Ly"], bin_state["bin"]["Lz"])

    setup.spawn_boxes_in_bin(config.bin_center_xy, inner_LWH, bin_state["placed"])

    # Robot
    ur5_id = p.loadURDF(config.UR5_URDF, [0,0,0], [0,0,0,1], useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
    ee  = ee_link_index(ur5_id)       # suction_tcp
    arm = arm_joint_indices(ur5_id)

    # Incoming box at staging area (dynamic)
    incoming_box_size = bin_state["incoming"]["size"]
    incoming_box_position = [0.42, 0.32, incoming_box_size[2]/2]
    incoming_box_id = setup.setup_box(incoming_box_size, incoming_box_position)


    # Pick (top-down)
    hover_h = 0.20

    # 👉 “Where exactly is my UR5’s end-effector link (ee) in the world right now?”
    Lp, _ = p.getLinkState(ur5_id, ee, True)[4:6]

    #  👉 “Take the UR5’s tool center point (TCP) straight up/down to hover_h above the ground, without changing XY, pointing the gripper downward, in a smooth 200-step trajectory.”
    functions.goto_tcp(ur5_id, arm, ee, [Lp[0], Lp[1], hover_h], config.DOWN)

    top = functions.body_top_center(incoming_box_id)
    # 👉 “Slide the TCP over the XY center of the target box, but stay high enough above it (at hover height).”
    functions.goto_tcp(ur5_id, arm, ee, [top[0], top[1], hover_h], config.DOWN)


    # 👉 “the XY center of the box, Z slightly above the box top (at least 6 cm).”
    pre = [top[0], top[1], max(top[2] + 0.015, 0.06)]

    # 👉 “Come straight down from hover height until you’re just above the box’s top surface, ready to grab.”
    functions.goto_tcp(ur5_id, arm, ee, pre, config.DOWN)

    # 👉 “Stick the incoming box to the robot’s tool.”
    grasp_constraint = functions.attach_vacuum(ur5_id, ee, incoming_box_id)

    # 👉 “Now that I’ve grabbed the box, go straight up to a safe hover height with it.”
    functions.goto_tcp(ur5_id, arm, ee, [pre[0], pre[1], hover_h], config.DOWN)


    place_by_instruction(
        ur5_id, arm, ee,
        bin_center_xy=config.bin_center_xy,
        inner_LWH=inner_LWH,
        bin_height=config.bin_height,
        box_size=incoming_box_size,
        instruction=instructions,  # ← your LLM-produced instruction dict
        hover_Z=hover_h,
        hover_margin=0.15,
        approach_clear=0.020,
        release_clear=0.001,
        descend_steps=420,
        grasp_constraint_id=grasp_constraint
    )

    while p.isConnected():
        p.stepSimulation()
        if config.SPEEDUP>0: time.sleep(config.TIMESTEP/config.SPEEDUP)

if __name__ == "__main__":
    try:
        main()
    finally:
        if p.isConnected(): p.disconnect()
