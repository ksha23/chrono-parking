"""Mouse and keyboard for the viewer, polled from the operating system."""

import sys


class MouseKeys:
    """Polls the mouse and keyboard on macOS (Quartz + the Objective-C runtime through ctypes).
    PyChrono's Irrlicht bindings cannot deliver window events to Python, so this is how the
    drag-the-target mode gets its input."""
    KEYS = dict(space=49, enter=36, q=12, e=14, r=15, left=123, right=124)

    def __init__(self):
        import ctypes
        import ctypes.util
        if sys.platform != "darwin":
            raise RuntimeError("mouse/keyboard polling is only implemented for macOS")
        self.ct = ctypes
        cg = ctypes.CDLL("/System/Library/Frameworks/ApplicationServices.framework/ApplicationServices")
        cg.CGEventSourceButtonState.restype = ctypes.c_bool
        cg.CGEventSourceButtonState.argtypes = [ctypes.c_int32, ctypes.c_uint32]
        cg.CGEventSourceKeyState.restype = ctypes.c_bool
        cg.CGEventSourceKeyState.argtypes = [ctypes.c_int32, ctypes.c_uint16]
        self.cg = cg
        objc = ctypes.CDLL(ctypes.util.find_library("objc"))
        objc.objc_getClass.restype = ctypes.c_void_p
        objc.objc_getClass.argtypes = [ctypes.c_char_p]
        objc.sel_registerName.restype = ctypes.c_void_p
        objc.sel_registerName.argtypes = [ctypes.c_char_p]
        self.objc = objc

        class Point(ctypes.Structure):
            _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]
        self.Point = Point
        self.app = self._msg(objc.objc_getClass(b"NSApplication"), b"sharedApplication")
        self.window = None

    def _msg(self, obj, sel, restype=None, *args, argtypes=()):
        ct = self.ct
        fn = ct.cast(self.objc.objc_msgSend, ct.CFUNCTYPE(restype or ct.c_void_p, ct.c_void_p, ct.c_void_p, *argtypes))
        return fn(obj, self.objc.sel_registerName(sel), *args)

    def poll(self, height):
        """Mouse position in window pixels (origin top left), buttons, held keys, focus."""
        ct = self.ct
        if self.window is None:
            wins = self._msg(self.app, b"windows")
            if wins and self._msg(wins, b"count", ct.c_ulong):
                self.window = self._msg(wins, b"objectAtIndex:", None, 0, argtypes=(ct.c_ulong,))
        if not self.window:
            return None
        p = self._msg(self.window, b"mouseLocationOutsideOfEventStream", self.Point)
        return dict(x=p.x, y=height - p.y,
                    left=bool(self.cg.CGEventSourceButtonState(0, 0)),
                    right=bool(self.cg.CGEventSourceButtonState(0, 1)),
                    keys={k for k, code in self.KEYS.items() if self.cg.CGEventSourceKeyState(0, code)},
                    focus=bool(self._msg(self.app, b"isActive", ct.c_bool)))
