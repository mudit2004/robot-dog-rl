import mujoco
import mujoco.viewer
import numpy as np
import os

MODEL_PATH = "mujoco_menagerie/unitree_a1/scene.xml"
assert os.path.exists(MODEL_PATH), f"❌ Missing model: {MODEL_PATH}"

# --- Load model ---
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# --- Initialize joint angles (stable standing pose) ---
# 12 joints in order: FL, FR, RL, RR (hip, thigh, calf)
stand_angles = np.array([
    0.0, -0.7, 1.4,   # FL
    0.0, -0.7, 1.4,   # FR
    0.0, -0.7, 1.4,   # RL
    0.0, -0.7, 1.4    # RR
])

# Apply starting configuration
data.qpos[7:19] = stand_angles  # Skip 7 floating base DOFs
data.qvel[:] = 0.0

# --- PD gains ---
Kp, Kd = 60.0, 2.0
target_q = np.copy(data.qpos[7:19])

print("✅ Robot initialized in stand pose.")

# --- Viewer loop ---
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Compute joint-level control
        q_err = target_q - data.qpos[7:19]
        qd_err = -data.qvel[6:]  # Skip base DOFs
        data.ctrl[:] = Kp * q_err + Kd * qd_err

        mujoco.mj_step(model, data)
        viewer.sync()

