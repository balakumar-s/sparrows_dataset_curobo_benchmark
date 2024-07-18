# Third Party
import pickle
from tqdm import tqdm

import numpy as np
import torch

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.util.logger import setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig


def benchmark_curobo(
    test_id, motion_gen, obs_num, data, plan_pose_target=True, write_usd=False, robot_data=None
):
    world_file = "data/sparrows_comparison/" + str(obs_num) + "obs/world_" + str(test_id) + ".yml"

    world_dict = load_yaml(world_file)
    world = WorldConfig.from_dict(world_dict).get_obb_world()
    motion_gen.update_world(world)
    motion_gen.reset()

    start_state_tensor = torch.zeros(7, device=motion_gen.tensor_args.device)
    goal_state_tensor = torch.zeros(7, device=motion_gen.tensor_args.device)

    for i in range(7):
        start_state_tensor[i] = data["initial"]["qpos"][i]
        goal_state_tensor[i] = data["initial"]["qgoal"][i]
    start_state = JointState.from_position(start_state_tensor.view(1, -1))
    goal_state = JointState.from_position(goal_state_tensor.view(1, -1))
    goal_pose = motion_gen.compute_kinematics(goal_state).ee_pose.clone()

    plan_config = MotionGenPlanConfig(timeout=0.5)
    if plan_pose_target:
        result = motion_gen.plan_single(start_state, goal_pose, plan_config)
    else:
        result = motion_gen.plan_single_js(start_state, goal_state, plan_config)

    if_success = result.success.cpu().numpy()
    if if_success:
        q = result.optimized_plan.position.cpu().numpy()
        dt = result.optimized_dt.cpu().numpy()
    else:
        if plan_pose_target:
            if not result.valid_query:
                print(result.status)
        q = None
        dt = 0.1
    solve_time = result.total_time

    if if_success and write_usd:
        usd_helper = UsdHelper()
        usd_helper.write_trajectory_animation_with_robot_usd(
            robot_data,
            world,
            start_state,
            result.optimized_plan,
            save_path="log/sparrows/obs_" + str(obs_num) + "_p_" + str(test_id) + ".usd",
            visualize_robot_spheres=False,
            flatten_usd=True,
            goal_pose=goal_pose,
            goal_color=[0.0,1.0,0.0,1.0],
        )

    result_dict = {
        "success": if_success,
        "trajectory": q,
        "dt": dt,
        "solve_time": solve_time,
        "qpos": start_state_tensor.cpu().numpy(),
        "qgoal": goal_state_tensor.cpu().numpy(),
    }

    return result_dict


if __name__ == "__main__":

    setup_curobo_logger("error")
    robot_file = "kinova_gen3.yml"
    robot_data = load_yaml(join_path(get_robot_configs_path(), robot_file))
    robot_data["robot_cfg"]["kinematics"]["ee_link"] = "bracelet_link"
    robot_data["robot_cfg"]["kinematics"]["collision_link_names"] = [
        "base_link",
        "shoulder_link",
        "half_arm_1_link",
        "half_arm_2_link",
        "forearm_link",
        "spherical_wrist_1_link",
        "spherical_wrist_2_link",
        "bracelet_link",
    ]
    robot_data["robot_cfg"]["kinematics"]["mesh_link_names"] = robot_data["robot_cfg"][
        "kinematics"
    ]["collision_link_names"]
    robot_data["robot_cfg"]["kinematics"]["collision_sphere_buffer"] = 0.0
    for obs_num in [10, 20, 40]:
        datadir = (
            "data/no_filter_planning_results/planning_results_pi_6/3d7links"
            + str(obs_num)
            + "obs/"
        )
        filename = (
            "armtd_1branched_t0.5_stats_3d7links100trials"
            + str(obs_num)
            + "obs150steps_0.5limit.pkl"
        )

        with open(datadir + filename, "rb") as f:
            data = pickle.load(f)

        time_list = []
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_data,
            "collision_table.yml",
            interpolation_dt=0.01,
            use_cuda_graph=True,
            collision_cache={"obb": obs_num},
            maximum_trajectory_dt=0.25,
            cspace_threshold=0.05,
        )

        motion_gen = MotionGen(motion_gen_config)
        motion_gen.warmup(warmup_js_trajopt=True)
        all_results = []
        success = 0
        for i in tqdm(range(100), leave=False):
            result_dict = benchmark_curobo(
                test_id=i,
                motion_gen=motion_gen,
                data=data[i],
                obs_num=obs_num,
                robot_data=robot_data,
            )
            if result_dict["success"]:
                success += 1
                time_list.append(result_dict["solve_time"])
            all_results.append(result_dict)

        print(
            "Success: ",
            str(success) + " Obs: " + str(obs_num) + " Time: " + str(np.mean(time_list)),
        )
