# --- Block 1: Imports & constants ---
import os
import numpy as np
import mujoco as mj
import mujoco.viewer as mjv  # use a single consistent alias
import glfw  # for KEY_KP_* constants (e.g., glfw.KEY_KP_8 = 328)


MODEL_PATH = "mujoco_menagerie/unitree_a1/scene.xml"

# PD gains for holding a pose
KP = 150.0
KD = 8.0

###########################################################

# --- Block 2: Load model and spawn in a stable standing pose ---

assert os.path.exists(MODEL_PATH), f"Missing model: {MODEL_PATH}"
model = mj.MjModel.from_xml_path(MODEL_PATH)
data  = mj.MjData(model)

# Gravity on from the start
model.opt.gravity[:] = [0, 0, -9.81]

# IMPORTANT: order in qpos is FR, FL, RR, RL (hip, thigh, calf)
stand_angles = np.array([
    0.0, -0.7,  1.4,    # FR
    0.0, -0.7,  1.4,    # FL
    0.0,  0.7, -1.4,    # RR
    0.0,  0.7, -1.4     # RL
], dtype=float)

# Set base pose and joint angles directly
data.qpos[:] = 0.0
data.qvel[:] = 0.0
data.qpos[0:3] = [0, 0, 0.35]     # trunk above ground
data.qpos[3:7] = [1, 0, 0, 0]     # neutral quaternion
data.qpos[7:19] = stand_angles    # legs in standing pose
mj.mj_forward(model, data)        # update derived quantities

target_q = stand_angles.copy()

def apply_pd_hold():
	q_err  = target_q - data.qpos[7:19]
	qd_err = -data.qvel[6:]               # 12 leg dofs start at qvel index 6
	data.ctrl[:] = KP * q_err + KD * qd_err

print("Dog spawned directly in stable standing pose.")

###########################################################

# --- Block 3: Keyboard input for base motion control ---
# Put this BEFORE entering the viewer loop and set the callback on the viewer

cmd_vel = {"x": 0.0, "y": 0.0, "yaw": 0.0}

print("\nNumpad Controls:")
print("  [8] / [2] : Move forward / backward")
print("  [4] / [6] : Strafe left / right")
print("  [7] / [9] : Turn left / right")
print("  [5]       : Stop")
print("  R         : Reset pose")
print("  ESC       : Quit\n")


def handle_keyboard(view, key):
	print(f"Key pressed: {key}") 
	if key == glfw.KEY_ESCAPE:
		view.close()
	elif key == ord("r"):
		print("Resetting pose...")
		data.qpos[:] = 0
		data.qvel[:] = 0
		data.qpos[0:3] = [0, 0, 0.35]
		data.qpos[3:7] = [1, 0, 0, 0]
		data.qpos[7:19] = stand_angles
		mj.mj_forward(model, data)
	elif key == ord("8"):
 		cmd_vel["x"] += 0.05
 		print(f"Forward speed: {cmd_vel['x']:.2f}")
	elif key == ord("2"):
	    	cmd_vel["x"] -= 0.05
	    	print(f"Backward speed: {cmd_vel['x']:.2f}")
	elif key == ord("4"):
	    	cmd_vel["y"] += 0.05
	    	print(f"Move left: {cmd_vel['y']:.2f}")
	elif key == ord("6"):
	    	cmd_vel["y"] -= 0.05
	    	print(f"Move right: {cmd_vel['y']:.2f}")
	elif key == ord("7"):
	    	cmd_vel["yaw"] += 0.05
	    	print(f"Turn left: {cmd_vel['yaw']:.2f}")
	elif key == ord("9"):
	    	cmd_vel["yaw"] -= 0.05
	    	print(f"Turn right: {cmd_vel['yaw']:.2f}")
	elif key == ord("5"):
	    	cmd_vel["x"] = cmd_vel["y"] = cmd_vel["yaw"] = 0.0
	    	print("Stop all motion")
	else:
		print(f"Unhandled key: {key}")


# --- Block 4: Simple gait generator driven by cmd_vel ---

# Leg order in qpos[7:19] is: FR, FL, RR, RL  (each: hip, thigh, calf)
# Trot phasing: FR & RL in phase; FL & RR opposite phase
leg_phase = np.array([0.0, np.pi, np.pi, 0.0], dtype=float)

# Base stride parameters
BASE_FREQ = 1.5            # Hz at |cmd_vel| = 1.0
MAX_AMP_THIGH = 0.35       # rad
MAX_AMP_CALF  = 0.35       # rad
TURN_BIAS_HIP = 0.20       # rad max hip abduction for yaw

phase = 0.0

def step_gait(dt):
	global phase, target_q
	# frequency scales with forward command magnitude
	speed_scale = np.clip(abs(cmd_vel["x"]), 0.0, 1.0)
	freq = BASE_FREQ * (0.2 + 0.8 * speed_scale)   # keep some motion at low cmd
	phase += 2.0 * np.pi * freq * dt

	# amplitudes
	A_thigh = MAX_AMP_THIGH * np.clip(abs(cmd_vel["x"]), 0.0, 1.0)
	A_calf  = MAX_AMP_CALF  * np.clip(abs(cmd_vel["x"]), 0.0, 1.0)

	# turn bias on hip abduction: left vs right legs get opposite signs
	yaw_scale = np.clip(cmd_vel["yaw"], -1.0, 1.0)
	hip_bias_FR = -TURN_BIAS_HIP * yaw_scale
	hip_bias_FL =  TURN_BIAS_HIP * yaw_scale
	hip_bias_RR = -TURN_BIAS_HIP * yaw_scale
	hip_bias_RL =  TURN_BIAS_HIP * yaw_scale

	# start from standing pose each frame
	tq = stand_angles.copy()

	# Build sinusoidal offsets for thigh/calf per leg with trot phasing
	# FR indices in qpos[7:19]: 0,1,2
	s_FR = np.sin(phase + leg_phase[0])
	tq[0] += hip_bias_FR
	tq[1] +=  A_thigh * s_FR
	tq[2] += -A_calf  * s_FR

	# FL: 3,4,5
	s_FL = np.sin(phase + leg_phase[1])
	tq[3] += hip_bias_FL
	tq[4] +=  A_thigh * s_FL
	tq[5] += -A_calf  * s_FL

	# RR: 6,7,8
	s_RR = np.sin(phase + leg_phase[2])
	tq[6] += hip_bias_RR
	tq[7] +=  A_thigh * s_RR
	tq[8] += -A_calf  * s_RR

	# RL: 9,10,11
	s_RL = np.sin(phase + leg_phase[3])
	tq[9]  += hip_bias_RL
	tq[10] +=  A_thigh * s_RL
	tq[11] += -A_calf  * s_RL

	# apply to current target
	target_q[:] = tq


# --- One viewer loop with callback attached ---
with mjv.launch_passive(model, data) as view:
	view._user_callback_key = handle_keyboard

	# last sim time to compute dt
	last_t = data.time

	while view.is_running():
		t = data.time
		dt = max(1e-3, t - last_t)  # avoid zero dt
		last_t = t

		step_gait(dt)       # <-- use cmd_vel to update target_q
		apply_pd_hold()     # PD tracks target_q
		mj.mj_step(model, data)

		# camera follow
		dog_pos = data.xpos[model.body('trunk').id]
		view.cam.lookat[:] = dog_pos
		view.cam.distance  = 2.0
		view.cam.elevation = -15
		view.cam.azimuth   = 90

		view.sync()

