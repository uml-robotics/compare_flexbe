#!/usr/bin/env python3
import json, os
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from unseen_obj_clst_ros2.srv import SegImage      # existing
from unseen_obj_clst_ros2.srv import SegCloud      # new (you’ll add)
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo

class UnseenObjSegmentationServiceState(EventState):
    """
    If userdata.cloud_in is present -> call SegCloud (point cloud mode);
    else -> call SegImage (image mode).
    Outputs are identical in both modes:
      seg_json (dict), result_dir (str), instance_ids (list), classes (list), bboxes (list), message (str)
    """
    def __init__(self,
                 image_service='run_segmentation',
                 cloud_service='run_segmentation_cloud',
                 service_timeout=5.0,
                 default_image_name='000000'):
        super().__init__(
            outcomes=['finished','failed'],
            input_keys=['image_name','cloud_in','camera_info'],
            output_keys=['seg_json','result_dir','instance_ids','classes','bboxes','message']
        )
        self._timeout = float(service_timeout)
        self._default_image_name = default_image_name
        self._image_srv_name = image_service
        self._cloud_srv_name = cloud_service

        self._srv = ProxyServiceCaller({
            self._image_srv_name: SegImage,
            self._cloud_srv_name: SegCloud
        })
        self._res = None
        self._err = False
        self._mode = 'cloud'  # image' or 'cloud'

    def on_enter(self, userdata):
        self._res, self._err = None, False
        cloud_present = isinstance(getattr(userdata, 'cloud_in', None), PointCloud2)
        self._mode = 'cloud' if cloud_present else 'image'

        srv_name = self._cloud_srv_name if self._mode == 'cloud' else self._image_srv_name
        if not self._srv.is_available(srv_name, timeout=self._timeout):
            Logger.logerr(f"[{type(self).__name__}] Service '{srv_name}' not available after {self._timeout}s.")
            self._err = True
            return

        try:
            if self._mode == 'cloud':
                req = SegCloud.Request()
                req.cloud = userdata.cloud_in
                if isinstance(getattr(userdata, 'camera_info', None), CameraInfo):
                    req.cam_info = userdata.camera_info
                self._res = self._srv.call(srv_name, req)
            else:
                im_name = getattr(userdata, 'image_name', None) or self._default_image_name
                req = SegImage.Request()
                req.image_name = im_name
                self._image_name = im_name
                self._res = self._srv.call(srv_name, req)
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Service call failed: {e}")
            self._err = True

    def execute(self, userdata):
        if self._err or self._res is None:
            return 'failed'
        if not self._res.success:
            userdata.message = self._res.log_output or "Segmentation failed."
            return 'failed'

        try:
            seg_json = json.loads(self._res.json_result)
            classes = seg_json.get('classes', [])
            instance_ids = seg_json.get('instance_ids', [])
            bboxes = seg_json.get('bboxes', [])

            # Prefer absolute path if server included it (recommended)
            result_dir = seg_json.get('result_dir', '')
            if not result_dir:
                base_output_dir = seg_json.get('base_output_dir', "/root/graspnet_ws/src/unseen_obj_clst_ros2/compare_UnseenObjectClustering/output/inference_results")
                image_name = getattr(self, '_image_name', '000000')  # image mode only; cloud mode server should set result_dir
                result_dir = os.path.join(base_output_dir, f"segmentation_{image_name}")

            userdata.seg_json = seg_json
            userdata.result_dir = result_dir
            userdata.classes = classes
            userdata.instance_ids = instance_ids
            userdata.bboxes = bboxes
            userdata.message = self._res.log_output or ""
            return 'finished'

        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Parse error: {e}")
            return 'failed'
