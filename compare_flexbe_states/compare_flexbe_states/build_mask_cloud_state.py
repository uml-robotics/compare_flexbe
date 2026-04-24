#!/usr/bin/env python3
import os
import numpy as np

from flexbe_core import EventState, Logger
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class BuildMaskedCloudsState(EventState):
    """
    Loads im_label.npy from result_dir and slices the input PointCloud2 into per-instance clouds.
    Assumes point cloud is organized and aligned with the RGB-D used by the segmenter.

    ># cloud_in         sensor_msgs/PointCloud2
    ># result_dir       str  (from UnseenObjSegmentationServiceState)
    <# instance_clouds  list[PointCloud2]  per-instance segmented clouds
    <# instance_ids     list[int]          matching instance ids

    <= finished
    <= failed
    """
    def __init__(self):
        super().__init__(outcomes=['finished', 'failed'],
                         input_keys=['cloud_in', 'result_dir'],
                         output_keys=['instance_clouds', 'instance_ids'])

    def on_enter(self, userdata):
        self._ok = True
        try:
            label_path = os.path.join(userdata.result_dir, 'im_label.npy')
            self._labels = np.load(label_path)      # H x W, ints
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to load labels: {e}")
            self._ok = False
            return

        try:
            # Extract the organized points
            # NOTE: This requires cloud_in to be organized and aligned with im_label.npy.
            pts = np.asarray(list(pc2.read_points(userdata.cloud_in, field_names=("x", "y", "z"), skip_nans=False)))
            # Reshape to H x W x 3
            H, W = self._labels.shape
            self._points = pts.reshape(H, W, 3)
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to map cloud to image grid: {e}")
            self._ok = False

    def execute(self, userdata):
        if not self._ok:
            return 'failed'

        try:
            instance_ids = sorted(list(set(self._labels.flatten())))

            instance_clouds = []
            for iid in instance_ids:
                mask = (self._labels == iid)
                if mask.sum() == 0:
                    continue
                pts_i = self._points[mask]  # N_i x 3

                # Build a PointCloud2 from xyz
                fields = [
                    pc2.PointField('x', 0,  pc2.PointField.FLOAT32, 1),
                    pc2.PointField('y', 4,  pc2.PointField.FLOAT32, 1),
                    pc2.PointField('z', 8,  pc2.PointField.FLOAT32, 1),
                ]
                header = userdata.cloud_in.header
                pc2_msg = pc2.create_cloud(header, fields, pts_i.tolist())
                instance_clouds.append(pc2_msg)

            userdata.instance_clouds = instance_clouds
            userdata.instance_ids = instance_ids
            Logger.loginfo(f"[{type(self).__name__}] Built {len(instance_clouds)} instance clouds.")
            return 'finished'

        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Error building instance clouds: {e}")
            return 'failed'
