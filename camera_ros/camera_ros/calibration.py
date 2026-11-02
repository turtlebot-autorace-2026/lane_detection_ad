import rclpy, cv2, os, glob, pickle, time
from datetime import datetime
import numpy as np
from rclpy.node import Node

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        self.image_save_interval = 1.0  # seconds
        self.last_save_time = time.time()
        self.image_counter = 0
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.save_dir = os.path.join(BASE_DIR, 'calibration_images', session)
        os.makedirs(self.save_dir, exist_ok=False)
        self.get_logger().info(f"New calibration images: {self.save_dir}")
        self.get_logger().info("Use a 7x9 inner-corner checkerboard (18.2 mm squares). Capture at least 20 varied views, then press 'q' in the video window to calibrate.")
        self.stop_requested = False

        # OpenCV camera
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            self.stop_requested = True
            return
        # Use the same sensor mode as publisher; do not calibrate resized frames.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    def capture_loop(self):
        while rclpy.ok() and not self.stop_requested:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to grab frame from camera")
                continue
            if frame.shape[:2] != (720, 1280):
                self.get_logger().error(f"Expected 1280x720, got {frame.shape[1]}x{frame.shape[0]}. Calibration cancelled.")
                self.stop_requested = False
                break

            cv2.imshow('Live Capture - Press q to stop', frame)
            key = cv2.waitKey(1) & 0xFF

            if time.time() - self.last_save_time >= self.image_save_interval:
                filename = os.path.join(self.save_dir, f'calib_{self.image_counter:03d}.jpg')
                if not cv2.imwrite(filename, frame):
                    raise RuntimeError(f"Failed to save calibration image: {filename}")
                self.get_logger().info(f"Saved ({self.image_counter + 1}): {filename}")
                self.last_save_time = time.time()
                self.image_counter += 1

            if key == ord('q'):
                self.get_logger().info("Stopping capture and calibrating...")
                self.stop_requested = True

        self.cap.release()
        cv2.destroyAllWindows()


def calibrate_camera(image_folder):
    CHECKERBOARD = (7, 9)
    square_size = 18.2  # mm
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objpoints = []
    imgpoints = []
    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * square_size
    images = sorted(glob.glob(os.path.join(image_folder, '*.jpg')))
    gray = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None: continue
        if img.shape[:2] != (720, 1280):
            raise RuntimeError(f"Calibration image must be 1280x720: {fname}")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('Calibration Result', img)
            cv2.waitKey(200)
    cv2.destroyAllWindows()

    print(f"Valid checkerboard views: {len(objpoints)}/{len(images)}")
    if gray is None or len(objpoints) < 10:
        print("Calibration cancelled: Need at least 10 valid checkerboard views. Previous calibration is unchanged.")
        return

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    print("\nCamera matrix:\n", mtx)
    print("\nDistortion coefficients:\n", dist)
    print(f"\nRMS reprojection error: {ret:.4f} pixels")
    if not np.isfinite(ret) or not np.all(np.isfinite(mtx)) or not np.all(np.isfinite(dist)):
        raise RuntimeError("Invalid calibration result; previous calibration is unchanged.")

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    calib_file = os.path.join(BASE_DIR, 'camera_calibration.pkl')
    with open(calib_file + '.tmp', 'wb') as f:
        pickle.dump({
            'camera_matrix': mtx,
            'dist_coeffs': dist,
            'rvecs': rvecs,
            'tvecs': tvecs,
            'image_size': (1280, 720),
            'rms_reprojection_error': ret,
            'valid_image_count': len(objpoints),
            'image_directory': image_folder,
        }, f)
    os.replace(calib_file + '.tmp', calib_file)
    print(f"\nCalibration data saved to {os.path.join(BASE_DIR, 'camera_calibration.pkl')}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        node.capture_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    if node.stop_requested and node.image_counter > 0:
        calibrate_camera(node.save_dir)


if __name__ == '__main__':
    main()

