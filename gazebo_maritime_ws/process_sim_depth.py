import threading
import time
import sys

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from pymavlink import mavutil
from serial import SerialException

def hfov_to_focal_lengths(hfov,w,h):
    """
    
    """
    cx = w / 2
    cy = h / 2
    fx = w / (2*np.tan(hfov/2))
    vfov = 2 * np.arctan((h / w) * np.tan(hfov/2))
    fy = h / (2*np.tan(vfov/2))
    print(f"fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}")
    return fx, fy, cx, cy

class GzDepthLidar:
    def __init__(self, topic, fx, fy, cx, cy):
        self.topic = topic
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)

        self.node = Node()
        self.lock = threading.Lock()
        self.latest_depth = None

        # Depth processing settings
        self.use_median = False
        self.use_gaussian = False
        self.use_bilateral = False
        self.patch_size = 5
        self.sigma_bilateral = 50.0

        # Sampling / pseudo-lidar settings
        self.row_step = 2
        self.col_step = 2
        self.fov_deg = 90.0
        self.num_pts = 72
        self.target_quantile = 0.01

        # Vertical mask in camera Y, meters
        # Old code used -750 to +750 mm
        self.y_floor = -0.75
        self.y_ceil = 0.0

        # Range limits, meters
        # Old code used 300..10000 mm
        self.min_supported_dist = 0.3
        self.max_supported_dist = 10.0

        self.last_depths_sent = None

        self._init_plot()

        ok = self.node.subscribe(Image, self.topic, self._cb)
        if not ok:
            raise RuntimeError(f"Failed to subscribe to {self.topic}")
        print(f"Subscribed to {self.topic}")

    def _init_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(1, 3, figsize=(15, 5))

        self.drawn, = self.ax[0].plot([], [], 'ro', markersize=4)
        self.drawn_smooth, = self.ax[2].plot([], [], 'ro', markersize=4)

        self.heatmap = self.ax[1].imshow(
            np.zeros((480, 640), dtype=np.float32),
            cmap="plasma_r",
            vmin=self.min_supported_dist,
            vmax=min(5.0, self.max_supported_dist),
        )

        for i in range(4):
            radius = (i + 1)
            self.ax[0].add_patch(
                patches.Circle((0.0, 0.0), radius=radius, fill=False, edgecolor='black', linewidth=1)
            )
            self.ax[2].add_patch(
                patches.Circle((0.0, 0.0), radius=radius, fill=False, edgecolor='black', linewidth=1)
            )

        self.ax[0].set_xlabel("X position (m)")
        self.ax[0].set_ylabel("Z position (m)")
        self.ax[0].set_title("Obstacle Distances")
        self.ax[0].set_xlim(-5.0, 5.0)
        self.ax[0].set_ylim(0.0, 10.0)

        self.ax[2].set_xlabel("Smoothed X position (m)")
        self.ax[2].set_ylabel("Smoothed Z position (m)")
        self.ax[2].set_title("Smoothed Obstacle Distances")
        self.ax[2].set_xlim(-5.0, 5.0)
        self.ax[2].set_ylim(0.0, 10.0)

        self.ax[1].set_title("Processed Depth")
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _decode_depth(self, msg: Image) -> np.ndarray:
        width = msg.width
        height = msg.height
        step = msg.step
        raw = msg.data

        expected_f32 = width * height * 4
        expected_u16 = width * height * 2

        if len(raw) == expected_f32:
            return np.frombuffer(raw, dtype=np.float32).reshape((height, width))

        if len(raw) == expected_u16:
            depth = np.frombuffer(raw, dtype=np.uint16).reshape((height, width)).astype(np.float32)
            return depth * 0.001  # assume mm -> m

        if step == width * 4:
            return np.frombuffer(raw, dtype=np.float32).reshape((height, width))

        if step == width * 2:
            depth = np.frombuffer(raw, dtype=np.uint16).reshape((height, width)).astype(np.float32)
            return depth * 0.001

        raise ValueError(
            f"Unsupported image layout: width={width}, height={height}, "
            f"step={step}, bytes={len(raw)}, pixel_format_type={msg.pixel_format_type}"
        )

    def _process_depth(self, depth: np.ndarray) -> np.ndarray:
        depth = depth.astype(np.float32, copy=True)
        invalid = ~np.isfinite(depth) | (depth <= 0.0)

        if self.use_median:
            temp = depth.copy()
            temp[invalid] = 0.0
            depth = cv2.medianBlur(temp, self.patch_size)

        if self.use_gaussian:
            temp = depth.copy()
            temp[invalid] = 0.0
            depth = cv2.GaussianBlur(temp, (self.patch_size, self.patch_size), 0)

        if self.use_bilateral:
            temp = depth.copy()
            temp[invalid] = 1e4
            depth = cv2.bilateralFilter(
                temp, self.patch_size, self.sigma_bilateral, self.sigma_bilateral
            )
            depth[invalid] = 0.0

        return depth

    def _depth_to_points(self, depth: np.ndarray) -> np.ndarray:
        h, w = depth.shape
        points = np.full((h, w, 3), np.nan, dtype=np.float32)

        rows = np.arange(0, h, self.row_step)
        cols = np.arange(0, w, self.col_step)
        cc, rr = np.meshgrid(cols, rows)

        z = depth[rr, cc]
        valid = np.isfinite(z) & (z > 0.0)

        x = (cc - self.cx) * z / self.fx
        y = (rr - self.cy) * z / self.fy

        points[rr[valid], cc[valid], 0] = x[valid]
        points[rr[valid], cc[valid], 1] = y[valid]
        points[rr[valid], cc[valid], 2] = z[valid]
        return points

    def _depth_for_heatmap(self, depth: np.ndarray) -> np.ndarray:
        sampled = depth[::self.row_step, ::self.col_step].copy()
        invalid = ~np.isfinite(sampled) | (sampled <= 0.0)
        sampled[invalid] = self.max_supported_dist
        return sampled

    def _pseudo_lidar_from_points(self, points: np.ndarray):
        mask = ~np.isnan(points).any(axis=2)
        pts = points[mask]
        if pts.shape[0] == 0:
            return None, None

        # Vertical band filter
        y_valid = (pts[:, 1] > self.y_floor) & (pts[:, 1] < self.y_ceil)
        pts = pts[y_valid]
        if pts.shape[0] == 0:
            return None, None

        # yaw = atan2(x, z), same idea as old code
        yaws = np.degrees(np.atan2(pts[:, 0], pts[:, 2]))
        if yaws.size == 0:
            return None, None

        angle_offset = self.fov_deg / 2.0
        increment_f = -self.fov_deg / self.num_pts
        bin_width = self.fov_deg / self.num_pts

        bin_indices = np.floor((angle_offset - yaws) / bin_width).astype(int)
        bin_indices = np.clip(bin_indices, 0, self.num_pts - 1)

        depth_xz = np.hypot(pts[:, 0], pts[:, 2])

        bucketed = [[] for _ in range(self.num_pts)]
        for b, d in zip(bin_indices, depth_xz):
            if self.min_supported_dist <= d <= self.max_supported_dist:
                bucketed[b].append(d)

        depths_passed = np.full(self.num_pts, self.max_supported_dist + 1.0, dtype=np.float32)
        for i in range(self.num_pts):
            if bucketed[i]:
                depths_passed[i] = np.quantile(bucketed[i], self.target_quantile)

        angles_passed = np.array(
            [angle_offset + increment_f * i for i in range(self.num_pts)],
            dtype=np.float32
        )
        return depths_passed, angles_passed

    def _update_plot(self, heatmap_depth, depths_passed, angles_passed):
        self.heatmap.set_data(heatmap_depth)
        self.heatmap.set_clim(self.min_supported_dist, min(10.0, self.max_supported_dist))

        valid = depths_passed <= self.max_supported_dist
        ang_rad = np.radians(angles_passed[valid])

        z_obs = depths_passed[valid] * np.cos(ang_rad)
        x_obs = depths_passed[valid] * np.sin(ang_rad)

        self.drawn.set_xdata(x_obs)
        self.drawn.set_ydata(z_obs)

        if self.last_depths_sent is not None:
            avg_depths = 0.5 * (self.last_depths_sent + depths_passed)
            valid2 = avg_depths <= self.max_supported_dist
            ang2 = np.radians(angles_passed[valid2])

            z_s = avg_depths[valid2] * np.cos(ang2)
            x_s = avg_depths[valid2] * np.sin(ang2)

            self.drawn_smooth.set_xdata(x_s)
            self.drawn_smooth.set_ydata(z_s)

        self.last_depths_sent = depths_passed.copy()
        #print(self.last_depths_sent)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _cb(self, msg: Image):
        try:
            depth = self._decode_depth(msg)
            depth = self._process_depth(depth)
            with self.lock:
                self.latest_depth = depth
        except Exception as e:
            print(f"Callback error: {e}")

    def run(self):
        with self.lock:
            depth = None if self.latest_depth is None else self.latest_depth.copy()

        if depth is not None:
            points = self._depth_to_points(depth)
            heatmap = self._depth_for_heatmap(depth)
            depths_passed, angles_passed = self._pseudo_lidar_from_points(points)

            if depths_passed is not None:
                self._update_plot(heatmap, depths_passed, angles_passed)

                center = depth[depth.shape[0] // 2, depth.shape[1] // 2]
                #print(
                #    f"center={center:.3f} m | "
                #    f"min wedge depth={np.min(depths_passed):.3f} m"
                #)

            return depths_passed, angles_passed
        return None, None
    
    def cleanup(self):
        plt.close(self.fig)
        cv2.destroyAllWindows()

class DepthBroadcast():
    """
    Class to send depth values to the 
    """
    def __init__(self, hfov, serial_port_rx='udpout:192.168.2.1:14550',serial_port_tx='udpout:127.0.0.1:14560'):
        self.serial_port_rx = serial_port_rx
        self.serial_port_tx = serial_port_tx
        self.min_supported_dist = 0.3 * 100
        self.max_supported_dist = 10.0 * 100
        num_pts = 72
        self.fov_deg = np.rad2deg(hfov)
        self.increment_f = self.fov_deg / num_pts
        try:
            #self.connection = mavutil.mavlink_connection(serial_port, baud=baudrate)
            self.connection_rx = mavutil.mavlink_connection(serial_port_rx)
            ret = self.connection_rx.wait_heartbeat(timeout=5)
            if ret is None:
               print(f"Failed to receive heartbeat from serial port {serial_port_rx}")
               self.connection_rx = None
            else:
                print(f"con: {self.connection_rx}")
                print("Heartbeat Received From Boat Connection")

                # Initialize the link to transmit data
                try:
                    self.connection_tx = mavutil.mavlink_connection(
                        serial_port_tx,
                        source_system=200,
                        source_component=196,
                        force_mavlink2=True,
                    )
                except (SerialException, TimeoutError):
                    print(f"\nFailed to initialize the connection at {serial_port_tx}!\n")
                    self.connection_tx = None                    

        except (SerialException, TimeoutError):
           print(f"\nFailed to initialize the connection at {serial_port_rx}!\n")
           self.connection_rx = None
        
    
    def send_depth(self,depths):
        depths_passed_int = [max(int(self.min_supported_dist),min(int(self.max_supported_dist),int(x * 100))) for x in depths] # convert m to cm
        frame = mavutil.mavlink.MAV_FRAME_BODY_FRD
        angle_offset = float(self.fov_deg / 2)
        if self.connection_tx is not None: # Send to boat if connection is alive
            print(f"sent depths of length {len(depths_passed_int)}")
            self.connection_tx.mav.obstacle_distance_send( 
                int(time.time() * 1e6), # time in us
                0,                      # sensor type
                depths_passed_int, 
                1,  # incrememnt (unused if given increment_f)
                int(self.min_supported_dist), 
                int(self.max_supported_dist),
                self.increment_f,
                angle_offset,
                frame
            )
            self.connection_tx.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )

if __name__ == "__main__":
    # From gazebo sensor parameters:
    hfov = 1.047
    w = 640
    h = 480

    fx, fy, cx, cy = hfov_to_focal_lengths(hfov=hfov,w=w,h=h)
    viewer = GzDepthLidar(
        topic="/depth_camera/depth_image",
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
    )
    #depth_sender = DepthBroadcast(hfov, serial_port_rx='udp:127.0.0.1:14552' ,serial_port_tx="udpout:127.0.0.1:14560")

    while True:
        try:
            depths, angles = viewer.run()
            #if depths is not None:
            #    print(f"depths is not none")
            #    depth_sender.send_depth(depths)
            #if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            #    break

            time.sleep(0.01)
        except KeyboardInterrupt:
            viewer.cleanup()
            sys.exit()    