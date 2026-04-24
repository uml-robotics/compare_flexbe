#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CGNGraspServiceState
--------------------
FlexBE state that calls a Contact-GraspNet ROS 2 service to compute grasp poses
from a PointCloud2. This version is backward-compatible with older behaviors
that pass a `field_names` kwarg, and is robust to PointCloud2 field layouts:
it auto-detects XYZ when needed and ignores extra fields (rgb/rgba/intensity).
It also performs a short service discovery wait to avoid transient availability
races from the GUI.

Inputs (userdata):
  - cloud_in: sensor_msgs/PointCloud2 (required)
  - indices:  list[int] (optional, ignored if cloud is already filtered)
  - scene_id: int (optional, ignored when use_scene_id=False)

Outputs (userdata):
  - grasp_target_poses: list[geometry_msgs.msg.Pose]
  - grasp_scores:       list[float]
  - grasp_samples:      list[geometry_msgs.msg.Point]
  - grasp_object_ids:   list[int]

Parameters:
  - service_name   (str):  fully-qualified service name (default: '/get_grasps')
  - use_scene_id   (bool): if True, ignore cloud and send scene_id to the server
  - service_timeout(float): seconds to wait for discovery before failing
  - field_names    (list): optional explicit field names to read from PointCloud2

Requires the service type:
  contact_graspnet_ros2/srv/GetGrasps
"""
from __future__ import annotations
import time
from typing import List, Iterable, Optional

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import Pose, Point

# Import the service/message types from your package
from contact_graspnet_ros2.srv import GetGrasps
from contact_graspnet_ros2.msg import Grasps

import struct
import math



class CGNGraspServiceStateV2(EventState):
    """Call Contact-GraspNet service with either live PointCloud2 or a scene_id.

    <= done    successfully received grasps
    <= failed  service unavailable / call failed / bad input

    -- service_name     string   Service name to call (FQN recommended)
    -- use_scene_id     bool     If True, send scene_id; otherwise send points
    -- service_timeout  float    Seconds to wait for service discovery
    -- field_names      list[str]Optional: fields to read (default auto-detect XYZ)

    ># cloud_in          PointCloud2  Input cloud (ignored if use_scene_id=True)
    ># indices           int[]        Optional indices (already filtered cloud OK)
    ># scene_id          int          Scene id (only when use_scene_id=True)

    #> grasp_target_poses Pose[]      Output poses
    #> grasp_scores        float[]     Output scores
    #> grasp_samples       Point[]     Output contact samples
    #> grasp_object_ids    int[]       Output object ids
    """

    def __init__(self,
                 service_name: str = '/get_grasps',
                 use_scene_id: bool = False,
                 service_timeout: float = 5.0,
                 field_names: Optional[list] = None,
                 z_min: float = 0.25):
        super().__init__(outcomes=['done', 'failed'],
                         input_keys=['cloud_in', 'indices', 'scene_id'],
                         output_keys=['grasp_target_poses',
                                      'grasp_scores',
                                      'grasp_samples',
                                      'grasp_object_ids'])
        self._service_name = service_name
        self._use_scene_id = bool(use_scene_id)
        self._service_timeout = float(service_timeout)
        # Back-compat: allow behaviors to pass field_names; default to auto
        self._fields = list(field_names) if field_names is not None else None

        # Proxy for ROS 2 services in FlexBE
        self._srv = ProxyServiceCaller({self._service_name: GetGrasps})

        self._had_error = False
        self._start_time = 0.0

    # ---------------------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------------------
    @staticmethod
    def _ensure_int_list(seq: Iterable) -> list[int]:
        return [int(v) for v in seq]

    @staticmethod
    def _pc2_detect_xyz_fields(cloud: PointCloud2) -> list[str]:
        """Return a suitable [x,y,z]-like triplet from available fields."""
        available = {f.name for f in cloud.fields}
        candidates = [
            ['x', 'y', 'z'],
            ['X', 'Y', 'Z'],
            ['pos_x', 'pos_y', 'pos_z'],
        ]
        for names in candidates:
            if set(names).issubset(available):
                return names
        raise RuntimeError(
            f"Cloud lacks xyz fields; available={sorted(list(available))}")

    def _finite3(a, b, c):
        return math.isfinite(a) and math.isfinite(b) and math.isfinite(c)

    def _pc2_to_xyz_flat(self, cloud: PointCloud2) -> list[float]:
        # Normal path: use declared fields
        if cloud.fields and len(cloud.fields) > 0:
            fields = None
            if self._fields is not None:
                available = {f.name for f in cloud.fields}
                if not set(self._fields).issubset(available):
                    Logger.logwarn(f"[CGNGraspServiceState] Provided field_names {self._fields} "
                                   f"not all present; available={sorted(list(available))}. "
                                   f"Falling back to auto-detect.")
                    fields = self._pc2_detect_xyz_fields(cloud)
                else:
                    fields = self._fields
            else:
                fields = self._pc2_detect_xyz_fields(cloud)

            pts_iter = pc2.read_points(cloud, field_names=fields, skip_nans=True)
            flat = []
            for t in pts_iter:
                flat.extend((float(t[0]), float(t[1]), float(t[2])))
            return flat

        # Fallback path: no fields declared — try raw float32 XYZ at offsets 0,4,8
        Logger.logwarn("[CGNGraspServiceState] Cloud has no fields; attempting raw parse as float32 XYZ")
        step = cloud.point_step or 0
        if step < 12:
            raise RuntimeError(f"Cannot parse cloud without fields and point_step<{12}: point_step={step}")

        count = int(cloud.width) * int(cloud.height)
        buf = cloud.data  # bytes
        endian = "<" if not cloud.is_bigendian else ">"
        flat = []
        for i in range(count):
            off = i * step
            if off + 12 > len(buf):
                break
            x, y, z = struct.unpack(endian + "fff", buf[off:off+12])
            if _finite3(x, y, z):
                flat.extend((float(x), float(y), float(z)))
        if not flat:
            raise RuntimeError("Raw parse produced zero XYZ points; upstream cloud is malformed.")
        return flat

    # ---------------------------------------------------------------------
    def on_enter(self, userdata):
        self._had_error = False
        self._start_time = time.time()

        # Wait briefly for discovery (prevents transient GUI timing races)
        deadline = time.time() + self._service_timeout
        while time.time() < deadline:
            if self._srv.is_available(self._service_name):
                break
            time.sleep(0.2)

        if not self._srv.is_available(self._service_name):
            Logger.logerr(f"[CGNGraspServiceState] Service '{self._service_name}' "
                          f"not available after {self._service_timeout}s.")
            self._had_error = True
            return

        # Build request
        req = GetGrasps.Request()

        try:
            if self._use_scene_id:
                if 'scene_id' not in userdata or userdata.scene_id is None:
                    raise ValueError("use_scene_id=True but userdata.scene_id is missing")
                req.scene_id = int(userdata.scene_id)
                req.points = []
                req.mask = []
                Logger.loginfo(f"[CGNGraspServiceState] Requesting grasps for scene_id={req.scene_id}")
            else:
                if 'cloud_in' not in userdata or userdata.cloud_in is None:
                    raise ValueError("use_scene_id=False but userdata.cloud_in is missing")
                cloud = userdata.cloud_in
                Logger.loginfo(
                    f"[CGNGraspServiceState] Cloud meta: frame={cloud.header.frame_id}, "
                    f"size={cloud.width}x{cloud.height}, fields={len(cloud.fields)}, "
                    f"point_step={cloud.point_step}, is_bigendian={cloud.is_bigendian}"
                )
                if not isinstance(cloud, PointCloud2):
                    raise TypeError("userdata.cloud_in must be a sensor_msgs/PointCloud2")

                flat_xyz = self._pc2_to_xyz_flat(cloud)
                req.points = [float(v) for v in flat_xyz]

                # Optional indices -> if present and intended as a sparse selection,
                # you can convert to a dense 0/1 mask here. For typical pre-filtered
                # clouds, just send an all-ones mask of the same length.
                n_pts = len(req.points) // 3
                req.mask = [1] * n_pts

                Logger.loginfo(
                    f"[CGNGraspServiceState] Sending {n_pts} points to '{self._service_name}'")

            # Perform the service call
            res = self._srv.call(self._service_name, req)
            if res is None:
                raise RuntimeError("Service returned None")

            # Extract and validate response
            if not isinstance(res.grasps, Grasps):
                raise TypeError("Response.grasps has unexpected type")

            grasps_msg = res.grasps
            # Ensure lists exist even if empty
            userdata.grasp_target_poses = list(grasps_msg.poses)
            userdata.grasp_scores = list(grasps_msg.scores)
            userdata.grasp_samples = list(grasps_msg.samples)
            userdata.grasp_object_ids = list(grasps_msg.object_ids)

            Logger.loginfo(
                f"[CGNGraspServiceState] Received {len(userdata.grasp_target_poses)} grasps")

        except Exception as e:
            Logger.logerr(f"[CGNGraspServiceState] Request failed: {e}")
            self._had_error = True

    # ---------------------------------------------------------------------
    def execute(self, userdata):
        if self._had_error:
            return 'failed'
        return 'done'

    # ---------------------------------------------------------------------
    def on_exit(self, userdata):
        pass

    def on_stop(self):
        pass

    def on_start(self):
        pass
