import mujoco
import mujoco.viewer
import numpy as np
import os

MODEL_PATH = "mujoco_menagerie/unitree_a1/scene.xml"
assert os.path.exists(MODEL_PATH), f"❌ Missing model: {MODEL_PATH}"

# --- Load model ---
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
model.opt.gravity[:] = [0, 0, -9.81]
data = mujoco.MjData(model)

# --- Initialize joint angles (stable standing pose) ---
# 12 joints in order: FL, FR, RL, RR (hip, thigh, calf)
stand_angles = np.array([
    0.0, -0.7, 1.4,   # FL
    0.0, -0.7, 1.4,   # FR
    0.0,  0.7, -1.4,  # RL (reverse sign)
    0.0,  0.7, -1.4   # RR (reverse sign)
])

# Apply starting configuration
data.qpos[7:19] = 0.0
data.qvel[:] = 0.0

# --- Disable gravity temporarily ---
model.opt.gravity[:] = [0, 0, 0]

# --- Smooth transition to standing pose ---
for step in range(500):
    alpha = step / 500.0
    data.qpos[7:19] = alpha * stand_angles
    mujoco.mj_step(model, data)

# --- Now enable gravity ---
model.opt.gravity[:] = [0, 0, -9.81]

# --- Let it settle gently ---
for _ in range(300):
    mujoco.mj_step(model, data)

# --- PD gains ---
Kp, Kd = 150.0, 8.0
target_q = np.copy(stand_angles)


print("✅ Robot initialized in stand pose.")

# --- Viewer loop ---
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # --- PD control ---
        q_err = target_q - data.qpos[7:19]
        qd_err = -data.qvel[6:]
        data.ctrl[:] = Kp * q_err + Kd * qd_err

        mujoco.mj_step(model, data)

        # --- 🟢 Camera follow mode ---
        # Get the dog's current base position
        dog_pos = data.xpos[model.body('trunk').id]

        # Slightly offset the camera behind and above the dog
        viewer.cam.lookat[:] = dog_pos
        viewer.cam.distance = 2      # zoom out distance
        viewer.cam.elevation = -15     # look down a little
        viewer.cam.azimuth = 90        # rotate around yaw

        viewer.sync()

