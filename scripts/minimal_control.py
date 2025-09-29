import mujoco
import mujoco.viewer
import numpy as np
import os

MODEL_PATH = "mujoco_menagerie/unitree_a1/scene.xml"
assert os.path.exists(MODEL_PATH), f"❌ Missing model: {MODEL_PATH}"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# === Joint groups (note: these are joints, not actuators) ===
hip_joints   = [model.joint("FL_hip_joint").id, model.joint("FR_hip_joint").id,
                model.joint("RL_hip_joint").id, model.joint("RR_hip_joint").id]
thigh_joints = [model.joint("FL_thigh_joint").id, model.joint("FR_thigh_joint").id,
                model.joint("RL_thigh_joint").id, model.joint("RR_thigh_joint").id]
calf_joints  = [model.joint("FL_calf_joint").id, model.joint("FR_calf_joint").id,
                model.joint("RL_calf_joint").id, model.joint("RR_calf_joint").id]

# === Map actuators to joints ===
actuator_ids = [model.actuator(i).trnid[0] for i in range(model.nu)]

# === Target "standing" pose only for actuated joints ===
target_qpos = np.zeros(model.nu)
for i, j in enumerate(actuator_ids):
    name = model.joint(j).name
    if "hip" in name:
        target_qpos[i] = 0.0
    elif "thigh" in name:
        target_qpos[i] = 0.8
    elif "calf" in name:
        target_qpos[i] = -1.5

# === PD gains ===
Kp, Kd = 50, 2

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Compute control only for actuated joints
        qpos_err = target_qpos - data.qpos[actuator_ids]
        qvel_err = -data.qvel[actuator_ids]
        data.ctrl[:] = Kp * qpos_err + Kd * qvel_err

        mujoco.mj_step(model, data)
        viewer.sync()

