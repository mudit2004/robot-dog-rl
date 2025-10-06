import mujoco
import mujoco.viewer
import numpy as np
import os

MODEL_PATH = "mujoco_menagerie/unitree_a1/scene.xml"
assert os.path.exists(MODEL_PATH), f"❌ Missing model: {MODEL_PATH}"

# --- Load model and data ---
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# --- Define a "standing" target pose ---
target_qpos = np.copy(data.qpos)
for i in range(model.nu):
    name = model.joint(model.actuator(i).trnid[0]).name
    if "hip_joint" in name:
        target_qpos[7 + i] = 0.0
    elif "thigh_joint" in name:
        target_qpos[7 + i] = -0.7
    elif "calf_joint" in name:
        target_qpos[7 + i] = 1.4

Kp, Kd = 60.0, 3.0
move_offset = np.zeros(3)  # [x, y, z]

print("\n🦿 Controls:")
print("  ↑ / ↓   → Move forward / backward")
print("  ← / →   → Move left / right")
print("  PgUp / PgDn → Move up / down")
print("  R       → Reset pose")
print("  ESC     → Quit\n")

# --- Key handling inside MuJoCo viewer ---
def handle_keyboard(viewer, key):
    global move_offset

    if key == mujoco.viewer.KEY_ESCAPE:
        viewer.close()
    elif key == ord("r"):
        print("🔄 Resetting pose...")
        data.qpos[:] = target_qpos
        data.qvel[:] = 0.0
        move_offset[:] = 0
    elif key == mujoco.viewer.KEY_UP:
        move_offset[0] += 0.05
    elif key == mujoco.viewer.KEY_DOWN:
        move_offset[0] -= 0.05
    elif key == mujoco.viewer.KEY_LEFT:
        move_offset[1] += 0.05
    elif key == mujoco.viewer.KEY_RIGHT:
        move_offset[1] -= 0.05
    elif key == mujoco.viewer.KEY_PAGE_UP:
        move_offset[2] += 0.05
    elif key == mujoco.viewer.KEY_PAGE_DOWN:
        move_offset[2] -= 0.05

# --- Main viewer loop ---
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.user_key_callback = handle_keyboard

    while viewer.is_running():
        data.qpos[0:3] = move_offset

        # PD control for standing
        qpos_err = target_qpos[7:] - data.qpos[7:]
        qvel_err = -data.qvel[6:]
        data.ctrl[:] = Kp * qpos_err + Kd * qvel_err

        mujoco.mj_step(model, data)
        viewer.sync()

