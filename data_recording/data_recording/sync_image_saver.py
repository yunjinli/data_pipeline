#!/usr/bin/env python3
# sync_image_saver.py
# Subscribes to two Image topics (cam0 & cam1), checks sync, stores pairs and saves them on shutdown.

import os
import csv
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from message_filters import Subscriber, ApproximateTimeSynchronizer

class SyncImageSaver(Node):
    def __init__(self):
        super().__init__('sync_image_saver')

        # Parameters (cam0/cam1 naming)
        self.declare_parameter('cam0_topic', '/cam0/image_raw')
        self.declare_parameter('cam1_topic', '/cam1/image_raw')
        self.declare_parameter('output_dir', '/tmp/paired_images')
        self.declare_parameter('queue_size', 20)
        self.declare_parameter('slop', 0.03)              # seconds tolerance
        self.declare_parameter('save_format', 'png')      # png/jpg
        self.declare_parameter('log_every', 50)           # print stats every N pairs

        cam0_topic   = self.get_parameter('cam0_topic').get_parameter_value().string_value
        cam1_topic   = self.get_parameter('cam1_topic').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        queue_size   = int(self.get_parameter('queue_size').value)
        slop         = float(self.get_parameter('slop').value)
        self.save_format = self.get_parameter('save_format').get_parameter_value().string_value.lower()
        self.log_every  = int(self.get_parameter('log_every').value)

        os.makedirs(self.output_dir, exist_ok=True)

        # QoS tuned for camera streams
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, queue_size)
        )

        # Subscribers via message_filters
        self.bridge = CvBridge()
        self.sub_cam0 = Subscriber(self, Image, cam0_topic, qos_profile=qos)
        self.sub_cam1 = Subscriber(self, Image, cam1_topic, qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_cam0, self.sub_cam1],
            queue_size=queue_size, slop=slop, allow_headerless=False
        )
        self.sync.registerCallback(self.synced_cb)

        # Storage for pairs (timestamps + images)
        self.pairs = []  # dicts: {t0, t1, dt, frame_id0, frame_id1, img0, img1}
        self.count = 0

        self.get_logger().info(
            f"Listening:\n  cam0: {cam0_topic}\n  cam1: {cam1_topic}\n"
            f"queue_size={queue_size}, slop={slop}s\nOutput dir: {self.output_dir}"
        )

        # Catch SIGINT/SIGTERM for saving on exit
        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def synced_cb(self, msg0: Image, msg1: Image):
        t0 = msg0.header.stamp.sec + msg0.header.stamp.nanosec * 1e-9
        t1 = msg1.header.stamp.sec + msg1.header.stamp.nanosec * 1e-9
        dt = abs(t0 - t1)

        try:
            img0 = self.bridge.imgmsg_to_cv2(msg0, desired_encoding='passthrough')
            img1 = self.bridge.imgmsg_to_cv2(msg1, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        self.pairs.append({
            't0': t0, 't1': t1, 'dt': dt,
            'frame_id0': msg0.header.frame_id,
            'frame_id1': msg1.header.frame_id,
            'img0': img0, 'img1': img1
        })
        self.count += 1

        if self.count % self.log_every == 0:
            dts = [p['dt'] for p in self.pairs[-self.log_every:]]
            self.get_logger().info(
                f"Pairs: {self.count} | dt mean={np.mean(dts):.4f}s "
                f"median={np.median(dts):.4f}s max={np.max(dts):.4f}s"
            )

    def _signal_handler(self, signum, _frame):
        self.get_logger().info(f"Signal {signum} received: saving {len(self.pairs)} pairs…")
        try:
            self.save_all()
        finally:
            self.get_logger().info("Done. Shutting down.")
            rclpy.shutdown()

    def save_all(self):
        if not self.pairs:
            self.get_logger().warn("No pairs to save.")
            return

        cam0_dir = os.path.join(self.output_dir, 'cam0')
        cam1_dir = os.path.join(self.output_dir, 'cam1')
        os.makedirs(cam0_dir, exist_ok=True)
        os.makedirs(cam1_dir, exist_ok=True)

        manifest_path = os.path.join(self.output_dir, 'pairs_manifest.csv')
        with open(manifest_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 'cam0_path', 'cam1_path',
                             't_cam0', 't_cam1', 'dt', 'frame_id_cam0', 'frame_id_cam1'])
            for i, p in enumerate(self.pairs):
                stem = f"{i:06d}"
                p0 = os.path.join(cam0_dir, f"{stem}.{self.save_format}")
                p1 = os.path.join(cam1_dir, f"{stem}.{self.save_format}")
                self._imwrite(p0, p['img0'])
                self._imwrite(p1, p['img1'])
                writer.writerow([i, p0, p1,
                                 f"{p['t0']:.9f}", f"{p['t1']:.9f}", f"{p['dt']:.9f}",
                                 p.get('frame_id0',''), p.get('frame_id1','')])

        dts = np.array([p['dt'] for p in self.pairs], dtype=np.float64)
        self.get_logger().info(
            f"Saved {len(self.pairs)} pairs.\n"
            f"Sync stats: mean={dts.mean():.6f}s, median={np.median(dts):.6f}s, max={dts.max():.6f}s\n"
            f"Manifest: {manifest_path}"
        )

    def _imwrite(self, path, img):
        # Handle 16-bit and float images
        if img.dtype in (np.float32, np.float64):
            finite = np.isfinite(img)
            if finite.any():
                mn, mx = float(img[finite].min()), float(img[finite].max())
                if mx > mn:
                    img = np.clip((img - mn) / (mx - mn) * 255.0, 0, 255).astype(np.uint8)
                else:
                    img = np.zeros_like(img, dtype=np.uint8)
            else:
                img = np.zeros_like(img, dtype=np.uint8)
        if img.dtype == np.uint16 and self.save_format == 'jpg':
            img = (img / 256).astype(np.uint8)  # JPEG is 8-bit
        cv.imwrite(path, img)

def main():
    rclpy.init()
    node = SyncImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: saving…")
        node.save_all()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
