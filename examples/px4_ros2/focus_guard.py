#!/usr/bin/env python3
"""Keep keyboard focus away from the CarlaAir window (view-only window).

CarlaAir starts its own drone control on the first WASD/Space/Shift key typed
into its window. With a PX4 vehicle that control fights PX4's flight modes and
can abort the engine. This guard watches the active X11 window and, whenever
it belongs to the engine process, asks the window manager to activate the
previously active window again, so the window stays visible but never keeps
keyboard focus.

Usage: focus_guard.py ENGINE_PID   (exits when the engine exits)
"""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import sys
import time

xlib = ctypes.cdll.LoadLibrary(ctypes.util.find_library("X11") or "libX11.so.6")
Window = ctypes.c_ulong
Atom = ctypes.c_ulong
xlib.XOpenDisplay.restype = ctypes.c_void_p
xlib.XOpenDisplay.argtypes = [ctypes.c_char_p]
xlib.XDefaultRootWindow.restype = Window
xlib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
xlib.XInternAtom.restype = Atom
xlib.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
xlib.XGetWindowProperty.argtypes = [
    ctypes.c_void_p, Window, Atom, ctypes.c_long, ctypes.c_long, ctypes.c_int, Atom,
    ctypes.POINTER(Atom), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_ulong),
    ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_void_p)]
xlib.XFree.argtypes = [ctypes.c_void_p]
xlib.XSendEvent.argtypes = [ctypes.c_void_p, Window, ctypes.c_int, ctypes.c_long, ctypes.c_void_p]
xlib.XFlush.argtypes = [ctypes.c_void_p]
xlib.XSetErrorHandler.argtypes = [ctypes.c_void_p]

ANY_PROPERTY_TYPE = 0
CLIENT_MESSAGE = 33
SUBSTRUCTURE_MASK = (1 << 19) | (1 << 20)  # SubstructureNotify | SubstructureRedirect


class ClientMessageEvent(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("serial", ctypes.c_ulong), ("send_event", ctypes.c_int),
                ("display", ctypes.c_void_p), ("window", Window), ("message_type", Atom),
                ("format", ctypes.c_int), ("data", ctypes.c_long * 5)]


class XEvent(ctypes.Union):
    _fields_ = [("xclient", ClientMessageEvent), ("pad", ctypes.c_long * 24)]


# Windows can vanish between two calls; ignore the resulting X errors.
_ignore_errors = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p)(lambda *_: 0)


def first_long(display, window: int, atom: int) -> int | None:
    actual_type, actual_format = Atom(), ctypes.c_int()
    count, remaining, data = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.c_void_p()
    status = xlib.XGetWindowProperty(display, window, atom, 0, 1, False, ANY_PROPERTY_TYPE,
                                     ctypes.byref(actual_type), ctypes.byref(actual_format),
                                     ctypes.byref(count), ctypes.byref(remaining), ctypes.byref(data))
    if status != 0 or not data.value:
        return None
    try:
        return ctypes.cast(data, ctypes.POINTER(ctypes.c_ulong))[0] if count.value else None
    finally:
        xlib.XFree(data)


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__.strip().splitlines()[-1].strip(), file=sys.stderr)
        return 2
    engine_pid = int(sys.argv[1])
    display = xlib.XOpenDisplay(None)
    if not display:
        print("focus_guard: cannot open the X display", file=sys.stderr)
        return 1
    xlib.XSetErrorHandler(_ignore_errors)
    root = xlib.XDefaultRootWindow(display)
    active_atom = xlib.XInternAtom(display, b"_NET_ACTIVE_WINDOW", False)
    pid_atom = xlib.XInternAtom(display, b"_NET_WM_PID", False)

    previous = None
    while os.path.exists(f"/proc/{engine_pid}"):
        active = first_long(display, root, active_atom)
        if active:
            if first_long(display, active, pid_atom) == engine_pid:
                if previous:
                    event = XEvent()
                    event.xclient.type = CLIENT_MESSAGE
                    event.xclient.window = previous
                    event.xclient.message_type = active_atom
                    event.xclient.format = 32
                    event.xclient.data[0] = 2  # source: pager, honoured by window managers
                    xlib.XSendEvent(display, root, False, SUBSTRUCTURE_MASK, ctypes.byref(event))
                    xlib.XFlush(display)
            else:
                previous = active
        time.sleep(0.1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
