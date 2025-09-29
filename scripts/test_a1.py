import mujoco
import mujoco.viewer
import os

# Path to the Unitree A1 Mujoco model XML
MODEL_PATH = os.path.join("mujoco_menagerie", "unitree_a1", "scene.xml")

# Load the model
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Open the interactive viewer
print("Launching Mujoco viewer... Close the window to stop.")
mujoco.viewer.launch(model, data)
