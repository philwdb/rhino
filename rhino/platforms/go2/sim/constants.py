from pathlib import Path

VIDEO_WIDTH = 320
VIDEO_HEIGHT = 240
VIDEO_CAMERA_FOV = 45    # degrees, head_camera
DEPTH_CAMERA_FOV = 160   # degrees, lidar_* cameras

MAX_RANGE = 3.0          # metres
MIN_RANGE = 0.2          # metres
MAX_HEIGHT = 1.2         # metres

VIDEO_FPS = 20
LIDAR_FPS = 2
ODOM_FREQUENCY = 50
LIDAR_VOXEL_SIZE = 0.05  # metres, for point cloud downsampling

ASSETS_PATH = Path(__file__).parent.parent / "assets"
LAUNCHER_PATH = Path(__file__).parent / "mujoco_process.py"
