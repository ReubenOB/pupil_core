"""
pupil_driver.py
---------------
Low-level Python interface to the Pupil Core Network API.

Pupil Capture / Pupil Service expose a ZeroMQ-based API that lets external
processes subscribe to sensor data and send remote-procedure commands.

Default ports (configurable in Pupil Capture → Network API plugin):
  - IPC Backend (REQ/REP)  : tcp://localhost:50020
  - Subscriber socket port : returned by the REQ socket on connection

Protocol overview
-----------------
1.  Connect a REQ socket to port 50020.
2.  Send "SUB_PORT"  → receive the subscriber port number as a string.
3.  Connect a SUB socket to that port and subscribe to topics of interest.
4.  Each message is a two-frame ZMQ multipart message:
      frame[0]  – topic bytes  (e.g. b"frame.world")
      frame[1]  – msgpack-serialised payload dict

Useful topics
-------------
  frame.world      – world camera JPEG/raw frames
  frame.eye.0      – eye-0 camera frames (left  eye by convention)
  frame.eye.1      – eye-1 camera frames (right eye by convention)
  pupil.0          – pupil datum from eye 0
  pupil.1          – pupil datum from eye 1
  gaze             – gaze datum (post-mapping)
  notify           – plugin / system notifications
"""

from __future__ import annotations

import logging
from typing import Callable, Dict, List, Optional

import msgpack
import zmq

logger = logging.getLogger(__name__)

# Default Pupil Capture / Service network ports
DEFAULT_PUPIL_HOST = "localhost"
DEFAULT_PUPIL_REQ_PORT = 50020


class PupilDriver:
    """
    Thin wrapper around the Pupil Core ZeroMQ Network API.

    Usage::

        driver = PupilDriver()
        driver.connect()
        driver.subscribe(["frame.world", "frame.eye.0", "frame.eye.1", "gaze"])

        while running:
            topic, payload = driver.recv()
            # handle topic / payload …

        driver.disconnect()

    The class is intentionally *not* thread-safe.  Callers that need
    concurrent access should use a single driver in a dedicated thread and
    communicate the received messages via queues.
    """

    def __init__(
        self,
        host: str = DEFAULT_PUPIL_HOST,
        req_port: int = DEFAULT_PUPIL_REQ_PORT,
        recv_timeout_ms: int = 1000,
    ) -> None:
        self._host = host
        self._req_port = req_port
        self._recv_timeout_ms = recv_timeout_ms

        self._ctx: Optional[zmq.Context] = None
        self._req: Optional[zmq.Socket] = None
        self._sub: Optional[zmq.Socket] = None
        self._sub_port: Optional[int] = None
        self._connected: bool = False

    # ------------------------------------------------------------------
    # Connection helpers
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to Pupil Capture / Service and negotiate the sub port."""
        self._ctx = zmq.Context.instance()

        # REQ socket — used to query the subscriber port
        self._req = self._ctx.socket(zmq.REQ)
        self._req.connect(f"tcp://{self._host}:{self._req_port}")

        self._sub_port = self._request_sub_port()
        logger.info("Pupil sub port: %s", self._sub_port)

        # SUB socket — receives sensor data
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.set(zmq.RCVTIMEO, self._recv_timeout_ms)
        self._sub.connect(f"tcp://{self._host}:{self._sub_port}")

        self._connected = True
        logger.info(
            "Connected to Pupil Capture at %s:%s (sub port %s)",
            self._host,
            self._req_port,
            self._sub_port,
        )

    def disconnect(self) -> None:
        """Close ZMQ sockets and terminate the context."""
        if self._sub is not None:
            self._sub.close()
            self._sub = None
        if self._req is not None:
            self._req.close()
            self._req = None
        if self._ctx is not None:
            self._ctx.term()
            self._ctx = None
        self._connected = False
        logger.info("Disconnected from Pupil Capture")

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Subscription management
    # ------------------------------------------------------------------

    def subscribe(self, topics: List[str]) -> None:
        """Subscribe to one or more Pupil topic strings."""
        if self._sub is None:
            raise RuntimeError("Not connected — call connect() first")
        for topic in topics:
            self._sub.setsockopt_string(zmq.SUBSCRIBE, topic)
            logger.debug("Subscribed to topic: %s", topic)

    def unsubscribe(self, topics: List[str]) -> None:
        """Unsubscribe from one or more Pupil topic strings."""
        if self._sub is None:
            return
        for topic in topics:
            self._sub.setsockopt_string(zmq.UNSUBSCRIBE, topic)
            logger.debug("Unsubscribed from topic: %s", topic)

    # ------------------------------------------------------------------
    # Receiving data
    # ------------------------------------------------------------------

    def recv(self) -> Optional[tuple[str, Dict]]:
        """
        Receive and deserialise one message from the subscriber socket.

        Returns ``(topic, payload_dict)`` or ``None`` if no message was
        available within the configured timeout window.
        """
        if self._sub is None:
            raise RuntimeError("Not connected — call connect() first")
        try:
            raw_topic, raw_payload = self._sub.recv_multipart()
            topic = raw_topic.decode("utf-8", errors="replace")
            payload = msgpack.unpackb(raw_payload, raw=False)
            return topic, payload
        except zmq.Again:
            # Timeout — no message available
            return None
        except Exception as exc:  # noqa: BLE001
            logger.warning("Error receiving message: %s", exc)
            return None

    # ------------------------------------------------------------------
    # Remote procedure calls
    # ------------------------------------------------------------------

    def send_notification(self, notification: Dict) -> str:
        """
        Send a notification dict to Pupil Capture via the REQ socket.

        Returns the acknowledgement string from Pupil Capture.
        """
        if self._req is None:
            raise RuntimeError("Not connected — call connect() first")
        payload = msgpack.packb(notification, use_bin_type=True)
        self._req.send_multipart([b"notify.", payload])
        return self._req.recv_string()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _request_sub_port(self) -> int:
        """Ask Pupil Capture for the subscriber port number."""
        if self._req is None:
            raise RuntimeError("REQ socket not initialised")
        self._req.send_string("SUB_PORT")
        return int(self._req.recv_string())
