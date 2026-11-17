"""Node-level tests for the Metric3D V2 depth node.

These need rclpy and torch, so they run inside the monocular_depth container:

    docker exec monocular_depth bash -lc \
      "cd /work/ws/src/monocular_depth && python3 -m pytest test -q"

The model itself is stubbed out — what is worth pinning here is the part that
is easy to get wrong and silent when it is: metric scale comes from the camera
focal length, and depth must never be published without one.
"""

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
torch = pytest.importorskip('torch')

from cv_bridge import CvBridge  # noqa: E402
from sensor_msgs.msg import CameraInfo  # noqa: E402

from monocular_depth.depth_node import DepthNode, _CANONICAL_INPUT_SIZE  # noqa: E402


class _StubModel:
    """Stands in for Metric3D: returns a canonical-space depth of 1 everywhere.

    With a uniform prediction the de-canonicalisation factor is the only thing
    left in the output, which is exactly what these tests are about.
    """

    def inference(self, payload):
        del payload
        return torch.ones(1, 1, *_CANONICAL_INPUT_SIZE), None, None


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(DepthNode, '_load_model', lambda self: _StubModel())
    rclpy.init()
    node = DepthNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _camera_info(width=640, height=480, fx=500.0):
    info = CameraInfo()
    info.width = width
    info.height = height
    info.k = [fx, 0.0, width / 2, 0.0, fx, height / 2, 0.0, 0.0, 1.0]
    return info


def _image_msg(bridge, width=640, height=480):
    msg = bridge.cv2_to_imgmsg(
        np.zeros((height, width, 3), dtype=np.uint8), encoding='bgr8'
    )
    msg.header.frame_id = 'camera_link_optical'
    msg.header.stamp.sec = 1234
    msg.header.stamp.nanosec = 5678
    return msg


def test_depth_scales_linearly_with_focal_length(node):
    """The whole reason Metric3D replaced DA-V2: scale comes from the camera."""
    bgr = np.zeros((480, 640, 3), dtype=np.uint8)

    single = node._infer(bgr, fx=500.0)
    double = node._infer(bgr, fx=1000.0)

    assert np.allclose(double, 2.0 * single, rtol=1e-4)
    # A canonical prediction of 1 m becomes (fx * resize_scale) / 1000 m.
    resize_scale = min(_CANONICAL_INPUT_SIZE[0] / 480, _CANONICAL_INPUT_SIZE[1] / 640)
    assert np.allclose(single, (500.0 * resize_scale) / 1000.0, rtol=1e-4)


def test_depth_is_returned_at_the_source_resolution(node):
    """The published CameraInfo describes the RGB frame, so depth must match it."""
    for height, width in ((480, 640), (720, 1280), (600, 600)):
        depth = node._infer(np.zeros((height, width, 3), dtype=np.uint8), fx=500.0)
        assert depth.shape == (height, width)
        assert depth.dtype == np.float32


def test_frames_are_dropped_until_camera_info_arrives(node):
    """Without a focal length there is no metre — publishing anything would be
    a guess at an arbitrary scale, which is the failure DA-V2 exhibited."""
    published = []
    node.depth_pub.publish = published.append

    node._image_callback(_image_msg(node.bridge))
    assert published == []

    node._camera_info_callback(_camera_info())
    node.last_inference_time = 0.0  # bypass the rate limiter for this call
    node._image_callback(_image_msg(node.bridge))
    assert len(published) == 1


def test_published_depth_carries_the_source_header(node):
    """Point clouds inherit this header; restamping or reframing breaks TF."""
    published = []
    node.depth_pub.publish = published.append
    node._camera_info_callback(_camera_info())

    msg = _image_msg(node.bridge)
    node._image_callback(msg)

    depth_msg = published[0]
    assert depth_msg.header.frame_id == msg.header.frame_id
    assert depth_msg.header.stamp == msg.header.stamp
    assert depth_msg.encoding == '32FC1'
    assert (depth_msg.height, depth_msg.width) == (msg.height, msg.width)


def test_camera_info_is_republished_with_the_depth_timestamp(node):
    """depth_image_proc synchronises depth against this CameraInfo."""
    infos = []
    node.depth_pub.publish = lambda _msg: None
    node.camera_info_pub.publish = infos.append
    node._camera_info_callback(_camera_info())

    msg = _image_msg(node.bridge)
    node._image_callback(msg)

    assert infos[0].header.stamp == msg.header.stamp
    assert infos[0].header.frame_id == msg.header.frame_id


def test_resolution_mismatch_is_reported(node):
    """A CameraInfo calibrated at another resolution scales every depth value
    by the same wrong factor — silence here would look like a model problem."""
    warnings = []
    node.get_logger().warn = lambda text, **_kwargs: warnings.append(text)
    node.depth_pub.publish = lambda _msg: None
    node._camera_info_callback(_camera_info(width=1280, height=720))

    node._image_callback(_image_msg(node.bridge, width=640, height=480))

    assert any('does not describe this frame' in w for w in warnings)


def test_bridge_round_trip_is_metric_float32(node):
    """32FC1 in metres is the interface contract with depth_image_proc."""
    bridge = CvBridge()
    node._camera_info_callback(_camera_info())
    published = []
    node.depth_pub.publish = published.append

    node._image_callback(_image_msg(node.bridge))

    depth = bridge.imgmsg_to_cv2(published[0], desired_encoding='32FC1')
    assert depth.dtype == np.float32
    assert np.all(np.isfinite(depth))
    assert np.all(depth >= 0.0)
