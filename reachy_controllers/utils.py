"""Utils for camera publisher node."""
from reachy_pyluos_hal.config import get_reachy_config


camera_config = {
    'C1': {
        'left': {'video_port': '/dev/video4', 'rotation': '90'},
        'right': {'video_port': '/dev/video0', 'rotation': '270'},
    },
    'C2': {
        'left': {'video_port': '/dev/video0', 'rotation': '270'},
        'right': {'video_port': '/dev/video4', 'rotation': '90'},
    }
}


def get_camera_config():
    """Run model identification checks on the cameras."""
    config = get_reachy_config()
    try:
        camera_config = config['camera_config']
    except KeyError:
        return 'C1'

    if camera_config not in ['C1', 'C2']:
        print("Invalid camera config in Reachy's configuration file")
        return

    return camera_config
