import time
import numpy as np
import mujoco
import mujoco.viewer

# Load the Unitree A1 model
model = mujoco.MjModel.from_xml_path("models/a1.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Running minimal control example... Press Ctrl+C to exit.")

    t0 = time.time()
    while viewer.is_running():
        # Time since start
        t = time.time() - t0

        # Example: sinusoidal control for joint index 0 (front-left hip)
        joint_id = 0  
        data.ctrl[joint_id] = 0.5 * np.sin(2 * np.pi * 0.5 * t)

        # Step physics forward
        mujoco.mj_step(model, data)

        # Sync viewer at ~60Hz
        viewer.sync()
