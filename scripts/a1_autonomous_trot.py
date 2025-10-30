import os
import numpy as np
import mujoco as mj
import mujoco.viewer as mjv

MODEL_PATH = "mujoco_menagerie/unitree_a1/scene.xml"
assert os.path.exists(MODEL_PATH), f"Missing model: {MODEL_PATH}"

# --- Load model and setup data ---
model = mj.MjModel.from_xml_path(MODEL_PATH)
data  = mj.MjData(model)
model.opt.gravity[:] = [0, 0, -9.81]

# Standing pose (FR, FL, RR, RL)
stand_angles = np.array([
    0.0, -0.7,  1.4,
    0.0, -0.7,  1.4,
    0.0,  0.7, -1.4,
    0.0,  0.7, -1.4
], dtype=float)

# Set base pose and joints
data.qpos[:] = 0.0
data.qvel[:] = 0.0
data.qpos[0:3] = [0, 0, 0.35]
data.qpos[3:7] = [1, 0, 0, 0]
data.qpos[7:19] = stand_angles
mj.mj_forward(model, data)

KP, KD = 150.0, 8.0
target_q = stand_angles.copy()

def apply_pd_hold():
    q_err  = target_q - data.qpos[7:19]
    qd_err = -data.qvel[6:]
    data.ctrl[:] = KP * q_err + KD * qd_err

# --- Gait setup ---
cmd_vel = {"x": 0.5, "y": 0.0, "yaw": 0.0}  # Walk forward

leg_phase = np.array([0.0, np.pi, np.pi, 0.0], dtype=float)
BASE_FREQ = 1.5
MAX_AMP_THIGH = 0.35
MAX_AMP_CALF  = 0.35
TURN_BIAS_HIP = 0.20
phase = 0.0

def step_gait(dt):
    global phase, target_q
    speed_scale = np.clip(abs(cmd_vel["x"]), 0.0, 1.0)
    freq = BASE_FREQ * (0.2 + 0.8 * speed_scale)
    phase += 2.0 * np.pi * freq * dt

    A_thigh = MAX_AMP_THIGH * speed_scale
    A_calf  = MAX_AMP_CALF  * speed_scale
    yaw_scale = np.clip(cmd_vel["yaw"], -1.0, 1.0)

    tq = stand_angles.copy()

    hip_bias = TURN_BIAS_HIP * yaw_scale
    offsets = [(-hip_bias, 0), (hip_bias, 3), (-hip_bias, 6), (hip_bias, 9)]

    for i, (bias, idx) in enumerate(offsets):
        s = np.sin(phase + leg_phase[i])
        tq[idx + 0] += bias
        tq[idx + 1] +=  A_thigh * s
        tq[idx + 2] += -A_calf  * s

    target_q[:] = tq

# --- Viewer loop ---
with mjv.launch_passive(model, data) as viewer:
    start_time = data.time
    last_t = data.time

    while viewer.is_running():
        t = data.time
        dt = max(1e-3, t - last_t)
        last_t = t

        # Only start gait after 2 seconds
        if t - start_time > 2.0:
            step_gait(dt)
        else:
            target_q[:] = stand_angles  # just hold still

        apply_pd_hold()
        mj.mj_step(model, data)

        dog_pos = data.xpos[model.body("trunk").id]
        viewer.cam.lookat[:] = dog_pos
        viewer.cam.distance  = 2.0
        viewer.cam.elevation = -15
        viewer.cam.azimuth   = 90

        viewer.sync()

