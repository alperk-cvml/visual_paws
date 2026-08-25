# Videos Directory

Place pre-recorded video files here for use with the `video_camera_sim` node.

The Docker container mounts this directory at `/videos` (read-only).

## Usage

Set the `VIDEO_PATH` environment variable to the path inside the container:

```bash
# In .env or docker-compose override:
VIDEO_PATH=/videos/my_recording.mp4
```

Or pass it as a launch argument:

```bash
ros2 launch video_camera_sim video_camera.launch.py video_path:=/videos/my_recording.mp4
```

## Supported Formats

Any format supported by OpenCV VideoCapture: MP4 (H.264), AVI, MKV, etc.
