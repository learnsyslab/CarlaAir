"""Minimal synchronous client for CarlaAir's AirSim RPC server.

The ``airsim`` pip package depends on msgpack-rpc-python and tornado 4, which
do not run on recent Python versions (e.g. Python 3.12 of ROS 2 Jazzy). The
AirSim server speaks plain msgpack-RPC over TCP, so a small client is enough.
Structured arguments such as image requests are sent as dicts with AirSim's
field names, e.g. ``{"camera_name": "0", "image_type": 0,
"pixels_as_float": False, "compress": False}``.

One instance is not thread-safe; give each thread its own client.
"""

from __future__ import annotations

import socket

import msgpack

_REQUEST = 0
_RESPONSE = 1


class AirSimRpcError(RuntimeError):
    """The AirSim server reported an error for a call."""


class AirSimRpc:
    def __init__(self, host: str = "127.0.0.1", port: int = 41451, timeout: float = 10.0):
        self._socket = socket.create_connection((host, port), timeout=timeout)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._unpacker = msgpack.Unpacker(raw=False, strict_map_key=False)
        self._message_id = 0

    def call(self, method: str, *params):
        """Call ``method`` and return its result."""
        self._message_id += 1
        self._socket.sendall(
            msgpack.packb([_REQUEST, self._message_id, method, list(params)], use_bin_type=True)
        )
        while True:
            for message in self._unpacker:
                kind, message_id, error, result = message
                if kind != _RESPONSE or message_id != self._message_id:
                    continue
                if error is not None:
                    raise AirSimRpcError(f"{method}: {error}")
                return result
            data = self._socket.recv(1 << 22)
            if not data:
                raise ConnectionError("AirSim closed the connection")
            self._unpacker.feed(data)

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "AirSimRpc":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
