import importlib.util
import pathlib
from types import SimpleNamespace
from unittest import mock

from sensor_msgs.msg import JointState


def load_adapter():
    path = (
        pathlib.Path(__file__).resolve().parents[1]
        / "scripts"
        / "front_piper_joint_state_adapter.py"
    )
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def info(self, _msg):
        pass

    def warn(self, _msg):
        pass


def test_front_piper_joint_state_adapter_strips_prefix():
    module = load_adapter()
    publisher = FakePublisher()

    with mock.patch.object(module.Node, "__init__", return_value=None):
        node = module.FrontPiperJointStateAdapter.__new__(module.FrontPiperJointStateAdapter)
        node.declare_parameter = mock.Mock()
        node.get_parameter = lambda name: SimpleNamespace(
            value={
                "input_topic": "/joint_states",
                "output_topic": "/front_piper/feedback/joint_states",
                "source_prefix": "front_piper_",
                "output_joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            }[name]
        )
        node.create_publisher = mock.Mock(return_value=publisher)
        node.create_subscription = mock.Mock()
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=3_000_000_000))
        node.get_logger = lambda: FakeLogger()
        module.FrontPiperJointStateAdapter.__init__(node)

    msg = JointState()
    msg.name = [
        "front_piper_joint1",
        "front_piper_joint2",
        "front_piper_joint3",
        "front_piper_joint4",
        "front_piper_joint5",
        "front_piper_joint6",
        "rear_piper_joint1",
    ]
    msg.position = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 99.0]
    msg.velocity = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 9.9]
    msg.effort = [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 9.9]

    node._callback(msg)

    assert len(publisher.messages) == 1
    out = publisher.messages[0]
    assert out.name == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    assert list(out.position) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert list(out.velocity) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert list(out.effort) == [1.1, 1.2, 1.3, 1.4, 1.5, 1.6]
