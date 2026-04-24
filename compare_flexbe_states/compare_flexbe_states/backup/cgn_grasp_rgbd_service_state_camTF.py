#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from geometry_msgs.msg import Pose
from contact_graspnet_ros2.srv import GetGrasps


class CGNGraspRGBDServiceState(EventState):
    """
    Call Contact-GraspNet RGBD grasp server and output a list of grasp poses.

    ># scene_name          string      Name of the scene (e.g. 'scene_from_ucn')
    <# grasp_target_poses  Pose[]      Grasps, transformed into base frame
    <# grasp_scores        float[]     Corresponding scores

    <= done                        Grasp request succeeded (may still be empty)
    <= failed                          Service call failed or exception
    """

    def __init__(self,
                 service_name='/get_grasps',
                 service_timeout=10.0,
                 # Base -> camera transform (match your static_transform_publisher)
                 base_to_cam_xyz=(0.0, 0.2, 1.25),
                 base_to_cam_rpy=(0.0, 1.2, 0.0),
                 apply_cam_to_base_transform=True):
        super(CGNGraspRGBDServiceState, self).__init__(
            outcomes=['done', 'failed'],
            input_keys=['scene_name'],
            output_keys=['grasp_target_poses', 'grasp_scores'])

        self._service_name = service_name
        self._timeout = service_timeout
        self._srv = ProxyServiceCaller({self._service_name: GetGrasps})

        self._res = None
        self._failed = False

        # --- Camera -> base transform setup ---
        self._apply_tf = apply_cam_to_base_transform
        self._T_base_cam = np.eye(4)
        self._tft = None

        if self._apply_tf:
            try:
                import tf_transformations as tft
                self._tft = tft

                tx, ty, tz = base_to_cam_xyz
                rr, rp, ry = base_to_cam_rpy

                T_trans = tft.translation_matrix((tx, ty, tz))
                T_rot = tft.euler_matrix(rr, rp, ry)
                self._T_base_cam = np.dot(T_trans, T_rot)

                Logger.loginfo(
                    "[CGNGraspRGBDServiceState] Using base->camera "
                    f"xyz={base_to_cam_xyz}, rpy={base_to_cam_rpy}"
                )
            except Exception as e:
                Logger.logwarn(
                    "[CGNGraspRGBDServiceState] Could not import "
                    f"tf_transformations or build transform ({e}). "
                    "Grasps will remain in camera frame."
                )
                self._apply_tf = False

    # ------------- helper functions -------------

    def _pose_to_matrix(self, pose: Pose) -> np.ndarray:
        """Convert geometry_msgs/Pose (camera frame) to 4x4 transform."""
        if self._tft is None:
            # Fallback: pure translation, identity rotation
            T = np.eye(4)
            T[0, 3] = pose.position.x
            T[1, 3] = pose.position.y
            T[2, 3] = pose.position.z
            return T

        q = [pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w]
        T = self._tft.quaternion_matrix(q)
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        return T

    def _matrix_to_pose(self, T: np.ndarray) -> Pose:
        """Convert 4x4 transform (base frame) back to geometry_msgs/Pose."""
        pose = Pose()
        pose.position.x = float(T[0, 3])
        pose.position.y = float(T[1, 3])
        pose.position.z = float(T[2, 3])

        if self._tft is not None:
            q = self._tft.quaternion_from_matrix(T)
            pose.orientation.x = float(q[0])
            pose.orientation.y = float(q[1])
            pose.orientation.z = float(q[2])
            pose.orientation.w = float(q[3])
        else:
            # Identity orientation if tf_transformations not available
            pose.orientation.w = 1.0

        return pose

    def _transform_grasps_cam_to_base(self, poses_cam):
        """Apply base->camera transform to a list of Pose in camera frame."""
        if not self._apply_tf:
            return list(poses_cam)

        poses_base = []
        for p_cam in poses_cam:
            T_cam_grasp = self._pose_to_matrix(p_cam)
            T_base_grasp = np.dot(self._T_base_cam, T_cam_grasp)
            poses_base.append(self._matrix_to_pose(T_base_grasp))
        return poses_base

    # ------------- FlexBE callbacks -------------

    def on_enter(self, userdata):
        self._res = None
        self._failed = False

        scene_name = userdata.scene_name if 'scene_name' in userdata else ''
        if not scene_name:
            Logger.logwarn(
                "[CGNGraspRGBDServiceState] No scene_name provided in userdata, "
                "using default 'scene_from_ucn'."
            )
            scene_name = 'scene_from_ucn'

        Logger.loginfo(
            f"[CGNGraspRGBDServiceState] Requesting grasps for scene "
            f"'{scene_name}'"
        )

        try:
            req = GetGrasps.Request()
            # RGBD/UCN pipeline passes all geometry via 'scene_name';
            # points/mask are not used in the current server implementation.
            req.scene_name = scene_name

            # Async call – result handled in execute()
            self._srv.call(self._service_name, req)
        except Exception as e:
            Logger.logerr(
                f"[CGNGraspRGBDServiceState] Failed to call service "
                f"'{self._service_name}': {e}"
            )
            self._failed = True

    def execute(self, userdata):
        if self._failed:
            return 'failed'

        # wait for service response
        if not self._srv.done(self._service_name):
            return None

        try:
            self._res = self._srv.result(self._service_name)
        except Exception as e:
            Logger.logerr(
                f"[CGNGraspRGBDServiceState] Error getting service result: {e}"
            )
            return 'failed'

        if self._res is None or len(self._res.grasps.poses) == 0:
            Logger.logwarn(
                "[CGNGraspRGBDServiceState] No grasps in response."
            )
            userdata.grasp_target_poses = []
            userdata.grasp_scores = []
            return 'finished'

        # Transform from camera frame to base frame
        poses_cam = self._res.grasps.poses
        poses_base = self._transform_grasps_cam_to_base(poses_cam)

        userdata.grasp_target_poses = poses_base
        userdata.grasp_scores = list(self._res.grasps.scores)

        Logger.loginfo(
            f"[CGNGraspRGBDServiceState] Received "
            f"{len(poses_base)} grasps (after transform)."
        )

        return 'done'

    def on_exit(self, userdata):
        pass

    def on_stop(self):
        pass

    def on_start(self):
        pass
