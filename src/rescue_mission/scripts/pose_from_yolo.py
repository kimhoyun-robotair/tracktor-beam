#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from tf_transformations import quaternion_matrix

class YoloPadTracker(Node):
    def __init__(self):
        super().__init__('yolo_pad_tracker')

        # ───── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter('pad_class_id', 0)           # YOLO에서 pad label id
        self.declare_parameter('camera_q_bxyz', [0., 0., 0., 1.])  # Body←Cam 회전 (x,y,z,w)  *주의*
        self.declare_parameter('camera_t_b',    [0., 0., 0.])      # Cam 기준점 Body 좌표 [m]
        self.declare_parameter('conf_th',       0.6)
        self.pad_class_id  = self.get_parameter('pad_class_id').value
        q_bcam = self.get_parameter('camera_q_bxyz').value
        # 회전: Cam→Body  (우리가 원식에 쓰는 R_cb)
        self.R_cb = quaternion_matrix([q_bcam[0], q_bcam[1], q_bcam[2], q_bcam[3]])[:3,:3]
        self.t_cb = np.array(self.get_parameter('camera_t_b').value).reshape(3,1)
        self.conf_th       = self.get_parameter('conf_th').value

        # ───── I/O 토픽 ──────────────────────────────────────────────
        self.sub_det  = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.detection_cb, 10)
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_cb, 1)
        self.sub_pos  = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.pos_cb, 10)
        self.sub_att  = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.att_cb, 10)

        self.pub_pose = self.create_publisher(
            PoseStamped, '/target_pose', 10)

        # ───── 내부 상태 ─────────────────────────────────────────────
        self.K = None               # 3×3
        self.alt = None             # z (NED, 음수)
        self.pv_n = None            # 기체 위치 (NED) 3×1
        self.R_bn = np.eye(3)       # Body->NED 회전 (초기값 단위행렬)

    # --------------- Callbacks -----------------
    def info_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3,3)

    def pos_cb(self, msg: VehicleLocalPosition):
        self.alt = msg.z
        self.pv_n = np.array([[msg.x],[msg.y],[msg.z]])

    def att_cb(self, msg: VehicleAttitude):
        q = msg.q  # w,x,y,z  (PX4 순서)
        self.R_bn = quaternion_matrix([q[1], q[2], q[3], q[0]])[:3,:3]  # (x,y,z,w)

    # --------------- Detection -----------------
    def detection_cb(self, msg: Detection2DArray):
        if self.K is None or self.alt is None or self.pv_n is None:
            return

        bbox, conf = self.select_pad_bbox(msg)
        if bbox is None or conf < self.conf_th:
            return

        # 1. 픽셀 → 카메라 좌표 (방향벡터)
        u, v = bbox.center.x, bbox.center.y
        fx, fy, cx, cy = self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]
        dir_c = np.array([[ (u-cx)/fx ],
                          [ (v-cy)/fy ],
                          [ 1.0 ]])
        dir_c = dir_c / np.linalg.norm(dir_c, axis=0)  # 정규화

        # 2. 카메라 → Body
        dir_b = self.R_cb @ dir_c
        p_cam_b = self.t_cb                 # Cam 광학중심 in Body

        # 3. Body → NED
        dir_n = self.R_bn @ dir_b
        p_cam_n = self.pv_n + self.R_bn @ p_cam_b

        # 4. 스케일 λ (지면을 z=0 기준)
        if abs(dir_n[2,0]) < 1e-3:  # 매우 수평이면 신뢰도↓
            self.get_logger().warn('dir_n.z too small; skipping frame')
            return
        lam = -p_cam_n[2,0] / dir_n[2,0]   # z_pad_n=0
        if lam < 0:
            return
        p_pad_n = p_cam_n + lam * dir_n    # 3×1

        # 5. PoseStamped 발행 (orientation 없음 → identity)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "ned"       # PrecisionLand 내부 TF와 일관될 것
        pose.pose.position.x = float(p_pad_n[0])
        pose.pose.position.y = float(p_pad_n[1])
        pose.pose.position.z = float(p_pad_n[2])
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

    # ---------------- util ---------------------
    def select_pad_bbox(self, det_arr: Detection2DArray):
        best_bbox = None
        best_score = 0.0
        for det in det_arr.detections:
            res = det.results[0]           # 결과 1개 가정
            if res.id != self.pad_class_id:
                continue
            if res.score > best_score:
                best_score = res.score
                best_bbox  = det.bbox
        return best_bbox, best_score


def main():
    rclpy.init()
    node = YoloPadTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
