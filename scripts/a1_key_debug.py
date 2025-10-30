import mujoco
import mujoco.viewer as mjv

model = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody>
    <body name="box" pos="0 0 0.1">
      <geom type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
""")
data = mujoco.MjData(model)

def handle_keyboard(viewer, key):
    print(f"KEY PRESSED: {key}")

with mjv.launch_passive(model, data) as viewer:
    viewer.user_key_callback = handle_keyboard
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

