import argparse
from pyexpat import model

import mujoco
import mujoco.viewer
import numpy as np
from pathlib import Path

from part_a_model import WheelEncoder
from track_a_estimation import PoseEKF
from track_a_sensors import CalibratedMarkerCamera, Lidar, build_room, evaluation_pose

SCENE_PATH = Path(__file__).resolve().parent/ "project_scene.xml"


def run(args):

    #setup mujoco environment
    rng=np.random.default_rng(args.seed)
    model = build_room()
    data=mujoco.MjData(model)
    camera = CalibratedMarkerCamera()
    lidar = Lidar(model)

    left_encoder = WheelEncoder(args.seed)
    right_encoder = WheelEncoder(args.seed)
    left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_left") 
    right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_right")
    left_qpos = model.jnt_qposadr[left_id]
    right_qpos = model.jnt_qposadr[right_id]
    previous_left = data.qpos[left_qpos]
    previous_right = data.qpos[right_qpos]

    ekf = PoseEKF()

    frames = []
    with mujoco.viewer.launch_passive(model, data) as viewer: #laod the veiw for project_scene.xml
        with mujoco.Renderer(model) as renderer:              #used to capture images from the camera
                        while viewer.is_running():
                            mujoco.mj_step(model, data)

                            #Part A encoder
                            current_left = data.qpos[left_qpos]
                            current_right = data.qpos[right_qpos]
                            left_rotation = current_left - previous_left
                            right_rotation = current_right - previous_right
                            previous_left = current_left
                            previous_right = current_right
                            dl = left_encoder.read(left_rotation)
                            dr = right_encoder.read(right_rotation)
                            

                            #Part B sensors
                            renderer.update_scene(data, camera="navigation")
                            image = renderer.render()
                            camera_pose_estimates, corners, ids = camera.observe(image) #needs to be checked
                            lidar_readings = lidar.read(model, data, rng)

                            #Part C Control
                            ekf.predict([dl, dr])
                            corrected = ekf.correct(camera_pose_estimates)
                                #viewer.sync() #can be used to update viewer after mujoco.mj_step(model, data)

                                # currently has lidar and camera readings working in the loop. control and model
                                # need to be implemented next
                            

                                

    
def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--seed',type=int,default=0)
    p.add_argument( '--scene', type=str, default='scene_turtlebot3_waffle_pi.xml')
    return p.parse_args()
if __name__=='__main__':
    args = parser()
    run(args)