"""
test_pupil_driver.py
--------------------
Unit tests for PupilDriver — no live Pupil Capture instance required.

The ZMQ sockets are replaced with lightweight fakes so the tests run
entirely offline.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch, call
import msgpack
import pytest

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pack(obj) -> bytes:
    return msgpack.packb(obj, use_bin_type=True)


def _make_mock_ctx(sub_port: int = 50021):
    """Return a mock zmq.Context whose sockets behave like Pupil Capture."""
    ctx = MagicMock()

    req_socket = MagicMock()
    sub_socket = MagicMock()

    # REQ socket: respond to "SUB_PORT" with the port number
    req_socket.recv_string.return_value = str(sub_port)

    ctx.socket.side_effect = [req_socket, sub_socket]
    return ctx, req_socket, sub_socket


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestPupilDriverConnect:
    def test_connect_negotiates_sub_port(self):
        from pupil_ros2_driver.pupil_driver import PupilDriver

        ctx, req, sub = _make_mock_ctx(sub_port=50021)
        with patch("pupil_ros2_driver.pupil_driver.zmq.Context") as MockCtx:
            MockCtx.instance.return_value = ctx
            driver = PupilDriver(host="localhost", req_port=50020)
            driver.connect()

        req.send_string.assert_called_once_with("SUB_PORT")
        sub.connect.assert_called_once_with("tcp://localhost:50021")
        assert driver.is_connected

    def test_disconnect_closes_sockets(self):
        from pupil_ros2_driver.pupil_driver import PupilDriver

        ctx, req, sub = _make_mock_ctx()
        with patch("pupil_ros2_driver.pupil_driver.zmq.Context") as MockCtx:
            MockCtx.instance.return_value = ctx
            driver = PupilDriver()
            driver.connect()
            driver.disconnect()

        sub.close.assert_called_once()
        req.close.assert_called_once()
        ctx.term.assert_called_once()
        assert not driver.is_connected

    def test_reconnect_after_disconnect(self):
        """Disconnect then connect again should succeed without raising."""
        from pupil_ros2_driver.pupil_driver import PupilDriver

        ctx, req, sub = _make_mock_ctx()
        with patch("pupil_ros2_driver.pupil_driver.zmq.Context") as MockCtx:
            MockCtx.instance.return_value = ctx
            driver = PupilDriver()
            driver.connect()
            driver.disconnect()
            assert not driver.is_connected

        # Reconnect with a fresh mock
        ctx2, req2, sub2 = _make_mock_ctx()
        with patch("pupil_ros2_driver.pupil_driver.zmq.Context") as MockCtx2:
            MockCtx2.instance.return_value = ctx2
            driver.connect()
            assert driver.is_connected


class TestPupilDriverSubscribe:
    def _connected_driver(self):
        from pupil_ros2_driver.pupil_driver import PupilDriver
        ctx, req, sub = _make_mock_ctx()
        with patch("pupil_ros2_driver.pupil_driver.zmq.Context") as MockCtx:
            MockCtx.instance.return_value = ctx
            driver = PupilDriver()
            driver.connect()
        return driver, sub

    def test_subscribe_sets_socket_option(self):
        driver, sub = self._connected_driver()
        driver.subscribe(["frame.world", "gaze"])
        assert sub.setsockopt_string.call_count == 2

    def test_unsubscribe_sets_socket_option(self):
        driver, sub = self._connected_driver()
        driver.subscribe(["gaze"])
        driver.unsubscribe(["gaze"])
        # subscribe + unsubscribe = 2 calls
        assert sub.setsockopt_string.call_count == 2

    def test_subscribe_not_connected_raises(self):
        from pupil_ros2_driver.pupil_driver import PupilDriver
        driver = PupilDriver()
        with pytest.raises(RuntimeError):
            driver.subscribe(["gaze"])


class TestPupilDriverRecv:
    def _connected_driver_with_sub(self):
        from pupil_ros2_driver.pupil_driver import PupilDriver
        import zmq

        ctx, req, sub = _make_mock_ctx()
        with patch("pupil_ros2_driver.pupil_driver.zmq.Context") as MockCtx:
            MockCtx.instance.return_value = ctx
            driver = PupilDriver()
            driver.connect()
        return driver, sub

    def test_recv_returns_topic_and_payload(self):
        driver, sub = self._connected_driver_with_sub()
        payload = {"timestamp": 1.23, "confidence": 0.9}
        sub.recv_multipart.return_value = [b"gaze", _pack(payload)]

        result = driver.recv()
        assert result is not None
        topic, data = result
        assert topic == "gaze"
        assert data["confidence"] == pytest.approx(0.9)

    def test_recv_returns_none_on_timeout(self):
        import zmq
        driver, sub = self._connected_driver_with_sub()
        sub.recv_multipart.side_effect = zmq.Again

        result = driver.recv()
        assert result is None

    def test_recv_not_connected_raises(self):
        from pupil_ros2_driver.pupil_driver import PupilDriver
        driver = PupilDriver()
        with pytest.raises(RuntimeError):
            driver.recv()


class TestPupilDriverNotification:
    def test_send_notification_encodes_payload(self):
        from pupil_ros2_driver.pupil_driver import PupilDriver

        ctx, req, sub = _make_mock_ctx()
        req.recv_string.side_effect = ["50021", "OK"]
        with patch("pupil_ros2_driver.pupil_driver.zmq.Context") as MockCtx:
            MockCtx.instance.return_value = ctx
            driver = PupilDriver()
            driver.connect()
            ack = driver.send_notification({"subject": "recording.start"})

        assert ack == "OK"
        # The second send_multipart call should be the notification
        assert req.send_multipart.called
