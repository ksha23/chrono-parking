"""Drawing helpers of the viewer that do not need the window: the pixel font, the colour scale,
resampling of pictures."""

import ctypes

import numpy as np


_GLYPHS = {
    "A": "0E11111F111111", "B": "1E11111E11111E", "C": "0E11101010110E", "D": "1E11111111111E",
    "E": "1F10101E10101F", "F": "1F10101E101010", "G": "0E11101711110F", "H": "1111111F111111",
    "I": "0E04040404040E", "J": "0702020202120C", "K": "11121418141211", "L": "1010101010101F",
    "M": "111B1515111111", "N": "11191513111111", "O": "0E11111111110E", "P": "1E11111E101010",
    "Q": "0E11111115120D", "R": "1E11111E141211", "S": "0F10100E01011E", "T": "1F040404040404",
    "U": "1111111111110E", "V": "11111111110A04", "W": "1111111515150A", "X": "11110A040A1111",
    "Y": "11110A04040404", "Z": "1F01020408101F", "0": "0E11131519110E", "1": "040C040404040E",
    "2": "0E11010204081F", "3": "1E01010E01011E", "4": "02060A121F0202", "5": "1F101E0101110E",
    "6": "0608101E11110E", "7": "1F010204080808", "8": "0E11110E11110E", "9": "0E11110F01020C",
    ".": "00000000000C0C", ":": "000C0C000C0C00", "-": "0000001F000000", "+": "0004041F040400",
    "/": "01010204081010", "(": "02040808080402", ")": "08040202020408", ",": "000000000C0408",
    "<": "02040810080402", ">": "08040201020408", "=": "00001F001F0000", "%": "18190204081303",
    "'": "04040800000000", "!": "04040404040004", "?": "0E110102040004",
}
_RUNS = {}
for _ch, _hx in _GLYPHS.items():
    _runs = []
    for _row in range(7):
        _bits = int(_hx[2 * _row:2 * _row + 2], 16)
        _col = 0
        while _col < 5:
            if _bits & (16 >> _col):
                _c0 = _col
                while _col < 5 and _bits & (16 >> _col):
                    _col += 1
                _runs.append((_row, _c0, _col))
            else:
                _col += 1
    _RUNS[_ch] = _runs
# the same glyphs as bitmaps, 7 rows of 5 pixels and one of spacing
_BITS = {_ch: np.array([[(int(_hx[2 * _r:2 * _r + 2], 16) >> (4 - _c)) & 1 if _c < 5 else 0 for _c in range(6)]
                        for _r in range(7)], dtype=bool) for _ch, _hx in _GLYPHS.items()}
_BLANK = np.zeros((7, 6), dtype=bool)


def _ramp(anchors, n=256):
    t = np.linspace(0.0, len(anchors) - 1.0, n)
    a = np.array(anchors, dtype=float)
    return np.stack([np.interp(t, np.arange(len(a)), a[:, k]) for k in range(3)], axis=1).astype(np.uint8)


# colour scale of the sensor pictures: blue (far, low) through green and yellow to red (near, high)
RAMP = _ramp([(46, 58, 150), (40, 130, 235), (30, 205, 200), (100, 235, 100), (230, 228, 50), (250, 140, 30),
              (222, 44, 32)])


def resample(img, w, h):
    """An image (rows, columns, 3) as a picture of exactly w x h pixels: averaged down by a whole
    factor first, then interpolated. Irrlicht draws a scaled image by picking the nearest pixel,
    which breaks up thin lines, so every picture is made at the size it is drawn at."""
    f = min(img.shape[0] // h, img.shape[1] // w)
    if f > 1:
        total = np.zeros((img.shape[0] // f, img.shape[1] // f, 3), np.uint16)
        for i in range(f):
            for j in range(f):
                total += img[i:total.shape[0] * f:f, j:total.shape[1] * f:f]
        img = total // (f * f)
    H, W = img.shape[:2]
    if (H, W) == (h, w):
        return img.astype(np.uint8)
    y = np.clip((np.arange(h) + 0.5) * H / h - 0.5, 0.0, H - 1.0)
    x = np.clip((np.arange(w) + 0.5) * W / w - 0.5, 0.0, W - 1.0)
    y0, x0 = np.minimum(y.astype(int), H - 2), np.minimum(x.astype(int), W - 2)
    fy, fx = (y - y0).astype(np.float32)[:, None, None], (x - x0).astype(np.float32)[None, :, None]
    rows = img[y0] * (1.0 - fy) + img[y0 + 1] * fy
    return (rows[:, x0] * (1.0 - fx) + rows[:, x0 + 1] * fx + 0.5).astype(np.uint8)


class _IrrString(ctypes.Structure):
    """Memory layout of irr::core::string<char>, the type Irrlicht names a texture with."""
    _fields_ = [("array", ctypes.c_char_p), ("allocated", ctypes.c_uint32), ("used", ctypes.c_uint32),
                ("allocator", ctypes.c_void_p)]
