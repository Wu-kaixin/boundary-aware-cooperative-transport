from types import SimpleNamespace

from src.optitrack.natnet_adapter import NatNetAdapter


class FakeLogger:
    def warning(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass

    def debug(self, *args, **kwargs):
        pass


class FakeNatNetClient:
    last_instance = None

    def __init__(self):
        FakeNatNetClient.last_instance = self
        self.new_frame_with_data_listener = None
        self.command_port = None
        self.data_port = None

    def set_client_address(self, value):
        self.client_address = value

    def set_server_address(self, value):
        self.server_address = value

    def set_use_multicast(self, value):
        self.use_multicast = value

    def set_print_level(self, value):
        self.print_level = value

    def run(self, stream_type):
        self.stream_type = stream_type
        return True

    def connected(self):
        return True


def make_adapter() -> NatNetAdapter:
    adapter = NatNetAdapter.__new__(NatNetAdapter)
    adapter.config = {
        "client_ip": "127.0.0.1",
        "server_ip": "127.0.0.1",
        "command_port": 1510,
        "data_port": 1511,
        "connection_type": "Unicast",
        "stream_type": "d",
        "connect_check_timeout_s": 0.01,
    }
    adapter.logger = FakeLogger()
    adapter.frame_callback = None
    adapter.client = None
    adapter.latest_bodies = []
    import threading

    adapter.lock = threading.Lock()
    adapter.client_cls = FakeNatNetClient
    adapter.sdk_available = True
    return adapter


def test_start_uses_full_frame_callback():
    adapter = make_adapter()

    adapter.start()

    client = FakeNatNetClient.last_instance
    assert client.new_frame_with_data_listener == adapter._receive_frame_with_data


def test_full_frame_callback_is_the_only_latest_bodies_writer():
    adapter = make_adapter()
    rigid_body = SimpleNamespace(
        id_num=2,
        pos=(4.0, 5.0, 6.0),
        rot=(0.1, 0.2, 0.3, 0.4),
        tracking_valid=False,
    )
    mocap_data = SimpleNamespace(
        rigid_body_data=SimpleNamespace(rigid_body_list=[rigid_body])
    )

    adapter._receive_frame_with_data({"timestamp": 123.0, "mocap_data": mocap_data})

    assert len(adapter.latest_bodies) == 1
    body = adapter.latest_bodies[0]
    assert body.rigid_body_id == 2
    assert body.name == "2"
    assert body.position == (4.0, 5.0, 6.0)
    assert body.quaternion == (0.1, 0.2, 0.3, 0.4)
    assert body.tracked is False
    assert body.timestamp == 123.0

