import pybullet as p

import numpy as np
import time

import config


def attach_vacuum(rid, link, obj_id):
    # Fixed constraint between suction_tcp (parent) and object base
    Lp, Lo = p.getLinkState(rid, link, True)[4:6]
    Op, Oo = p.getBasePositionAndOrientation(obj_id)
    inv_Op, inv_Oo = p.invertTransform(Op, Oo)
    child_p, child_o = p.multiplyTransforms(inv_Op, inv_Oo, Lp, Lo)
    c = p.createConstraint(rid, link, obj_id, -1, p.JOINT_FIXED, [0,0,0],
                           parentFramePosition=[0,0,0], parentFrameOrientation=[0,0,0,1],
                           childFramePosition=child_p, childFrameOrientation=child_o)
    return c


def body_top_center(bid):
    a,b = p.getAABB(bid)
    a,b = np.array(a), np.array(b)
    xy = 0.5*(a[:2]+b[:2]); z=b[2]
    return np.array([xy[0], xy[1], z], float)


def ik(rid, link, pos, orn):
    return np.array(p.calculateInverseKinematics(
        rid, link, pos, orn, maxNumIterations=150, residualThreshold=1e-4))


def get_q(rid, joints):
    return np.array([p.getJointState(rid, j)[0] for j in joints])


def min_jerk(n):
    t = np.linspace(0, 1, n)
    return 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5


def follow_jtraj(rid, joints, q_start, q_goal, steps):
    al = min_jerk(steps).reshape(-1, 1)
    for k in range(steps):
        qk = (1 - al[k]) * q_start + al[k] * q_goal
        for i, j in enumerate(joints):
            p.setJointMotorControl2(rid, j, p.POSITION_CONTROL,
                                    targetPosition=float(qk[i]),
                                    force=config.FORCE_ARM, positionGain=config.KP, velocityGain=config.KV)
        p.stepSimulation()
        if config.SPEEDUP > 0: time.sleep(config.TIMESTEP / config.SPEEDUP)


def goto_tcp(rid, joints, link, tcp_pos, tcp_orn, steps=180):
    # TCP is a real link now (suction_tcp), so solve IK directly to tcp_pos,tcp_orn
    q_goal = ik(rid, link, tcp_pos, tcp_orn)
    if q_goal is None:
        return False
    q_start = get_q(rid, joints)
    follow_jtraj(rid, joints, q_start, q_goal[:len(joints)], steps)
    return True

# ---------- small utils ----------
def R_of(q):
    return np.array(p.getMatrixFromQuaternion(q)).reshape(3,3)