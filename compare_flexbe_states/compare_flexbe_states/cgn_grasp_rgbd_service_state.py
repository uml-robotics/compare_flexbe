#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from contact_graspnet_ros2.srv import GetGrasps as SrvType
from contact_graspnet_ros2.msg import Grasps

import subprocess, os
<<<<<<< local_main_cgn
<<<<<<< HEAD
import numpy as np
=======
>>>>>>> legacy-feature-cgn
=======
import numpy as np
>>>>>>> feature/cgn



class CGNGraspRGBDServiceState(EventState):
    """
<<<<<<< local_main_cgn
<<<<<<< HEAD
    Calls the Contact-GraspNet `get_grasps_rgbd` service using only a precomputed
=======
    Calls the Contact-GraspNet `get_grasps` service using only a precomputed
>>>>>>> legacy-feature-cgn
=======
    Calls the Contact-GraspNet `get_grasps_rgbd` service using only a precomputed
>>>>>>> feature/cgn
    scene name (RGBD pipeline).

    Assumes the server:
      - Expects `scene_name` to select test_data/<scene_name>.npy
      - Ignores `points` and `mask` fields (or they can be left empty)

    This matches your `grasp_executor_rgbd_server.py`, which uses:
        self.scene_name = request.scene_name
        np_path = f"test_data/{scene_name}.npy"
    and loads 'results/predictions_<scene_name>.npz'. :contentReference[oaicite:1]{index=1}

    ># scene_name              string                 CGN scene name (without .npy)
    <# grasp_target_poses      geometry_msgs/Pose[]   Grasp poses
    <# grasp_scores            float[]                Grasp scores
    <# grasp_samples           geometry_msgs/Point[]  Contact points
    <# grasp_object_ids        int[]                  Object IDs per grasp

    <= done                    Service call succeeded
    <= failed                  Service call failed or no grasps received
    """

    def __init__(self,
                 service_timeout: float = 10.0,
<<<<<<< local_main_cgn
<<<<<<< HEAD
                 service_name: str = '/get_grasps_rgbd'):
=======
                 service_name: str = '/get_grasps'):
>>>>>>> legacy-feature-cgn
=======
                 service_name: str = '/get_grasps_rgbd'):
>>>>>>> feature/cgn
        super().__init__(
            outcomes=['done', 'failed'],
            input_keys=['scene_name'],
            output_keys=['grasp_target_poses', 'grasp_scores', 'grasp_samples', 'grasp_object_ids']
        )

        self._service_timeout = float(service_timeout)
        self._service_name = str(service_name)

        self._srv = ProxyServiceCaller({self._service_name: SrvType})
        self._res = None
        self._had_error = False

    def on_enter(self, userdata):
        self._res = None
        self._had_error = False

        try:
            scene_name = str(userdata.scene_name)
        except Exception:
            scene_name = "scene_from_ucn"

        Logger.loginfo(f"[CGNGraspRGBDServiceState] Requesting grasps for scene '{scene_name}'")

        request = SrvType.Request()
        # Leave points/mask empty; server loads data from scene_name internally.
        request.points = []
        request.mask = []
        request.scene_name = scene_name

        # Wait for service to be available
        deadline = time.time() + self._service_timeout
        while time.time() < deadline:
            if self._srv.is_available(self._service_name):
                break
            time.sleep(0.2)

        if not self._srv.is_available(self._service_name):
            Logger.logerr(f"[CGNGraspRGBDServiceState] Service '{self._service_name}' not available after "
                          f"{self._service_timeout}s.")
            self._had_error = True
            return


        # Call service
        try:
            self._res = self._srv.call(self._service_name, request)

<<<<<<< local_main_cgn
<<<<<<< HEAD
=======
>>>>>>> feature/cgn
            Logger.loginfo(f"[CGNGraspRGBDServiceState] Called service '{self._service_name}'.")

            # cmd = [
            #     "python3",
            #     "/home/csrobot/graspnet_ws/src/contact_graspnet_ros2/contact_graspnet/result_plotter.py",
            # ]
            # subprocess.check_call(cmd)
            # Logger.loginfo(f"[CGNGraspRGBDServiceState] Plotted grasp poses in 3d open view using result_plotter'.")
            
<<<<<<< local_main_cgn
=======
            cmd = [
                "python3",
                "/home/csrobot/graspnet_ws/src/contact_graspnet_ros2/contact_graspnet/result_plotter.py",
            ]
            subprocess.check_call(cmd)

            Logger.loginfo(f"[CGNGraspRGBDServiceState] Called service '{self._service_name}'.")
>>>>>>> legacy-feature-cgn
=======
>>>>>>> feature/cgn
        except Exception as e:
            Logger.logerr(f"[CGNGraspRGBDServiceState] Service call failed: {e}")
            self._had_error = True
            self._res = None

    def execute(self, userdata):
        if self._had_error or self._res is None:
            return 'failed'

        try:
<<<<<<< local_main_cgn
<<<<<<< HEAD
            # time.sleep(15)
=======
>>>>>>> legacy-feature-cgn
=======
            # time.sleep(15)
>>>>>>> feature/cgn
            grasps = self._res.grasps  # type: Grasps
            userdata.grasp_target_poses = list(grasps.poses)
            userdata.grasp_scores = list(grasps.scores)
            userdata.grasp_samples = list(grasps.samples)
            userdata.grasp_object_ids = list(grasps.object_ids)
<<<<<<< local_main_cgn
<<<<<<< HEAD
            # Logger.loginfo(f"[CGNGraspRGBDServiceState] grasps.object_ids = {grasps.object_ids}, userdata.grasp_object_ids {userdata.grasp_object_ids}.")

            Logger.loginfo(f"[CGNGraspRGBDServiceState] Received {len(grasps.poses)} grasp poses for grasp_object_ids: {np.unique(userdata.grasp_object_ids)}.")
=======

            Logger.loginfo(f"[CGNGraspRGBDServiceState] Received {len(grasps.poses)} grasp poses.")
>>>>>>> legacy-feature-cgn
=======
            # Logger.loginfo(f"[CGNGraspRGBDServiceState] grasps.object_ids = {grasps.object_ids}, userdata.grasp_object_ids {userdata.grasp_object_ids}.")

            Logger.loginfo(f"[CGNGraspRGBDServiceState] Received {len(grasps.poses)} grasp poses for grasp_object_ids: {np.unique(userdata.grasp_object_ids)}.")
>>>>>>> feature/cgn
        except Exception as e:
            Logger.logerr(f"[CGNGraspRGBDServiceState] Failed to copy result to userdata: {e}")
            return 'failed'

        if len(userdata.grasp_target_poses) == 0:
<<<<<<< local_main_cgn
<<<<<<< HEAD

=======
>>>>>>> legacy-feature-cgn
=======

>>>>>>> feature/cgn
            Logger.logwarn("[CGNGraspRGBDServiceState] No grasps in response.")
            return 'failed'

        return 'done'

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
