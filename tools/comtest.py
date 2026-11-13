"""Find which part of SetCommState a COM port rejects (Windows only).

Usage:  python comtest.py COM1
"""
import ctypes
import sys
from ctypes import wintypes

k32 = ctypes.WinDLL("kernel32", use_last_error=True)


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wintypes.DWORD),
        ("BaudRate", wintypes.DWORD),
        ("flags", wintypes.DWORD),
        ("wReserved", wintypes.WORD),
        ("XonLim", wintypes.WORD),
        ("XoffLim", wintypes.WORD),
        ("ByteSize", ctypes.c_ubyte),
        ("Parity", ctypes.c_ubyte),
        ("StopBits", ctypes.c_ubyte),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", wintypes.WORD),
    ]


class COMMPROP(ctypes.Structure):
    _fields_ = [
        ("wPacketLength", wintypes.WORD), ("wPacketVersion", wintypes.WORD),
        ("dwServiceMask", wintypes.DWORD), ("dwReserved1", wintypes.DWORD),
        ("dwMaxTxQueue", wintypes.DWORD), ("dwMaxRxQueue", wintypes.DWORD),
        ("dwMaxBaud", wintypes.DWORD), ("dwProvSubType", wintypes.DWORD),
        ("dwProvCapabilities", wintypes.DWORD), ("dwSettableParams", wintypes.DWORD),
        ("dwSettableBaud", wintypes.DWORD), ("wSettableData", wintypes.WORD),
        ("wSettableStopParity", wintypes.WORD), ("dwCurrentTxQueue", wintypes.DWORD),
        ("dwCurrentRxQueue", wintypes.DWORD), ("dwProvSpec1", wintypes.DWORD),
        ("dwProvSpec2", wintypes.DWORD), ("wcProvChar", wintypes.WCHAR * 1),
    ]


def err():
    e = ctypes.get_last_error()
    return f"FAILED (error {e}: {ctypes.FormatError(e).strip()})"


def dump(d):
    f = d.flags
    return (f"Baud={d.BaudRate} Byte={d.ByteSize} Par={d.Parity} Stop={d.StopBits} "
            f"flags=0x{f:08X} (DTR={(f >> 4) & 3} RTS={(f >> 12) & 3} OutX={(f >> 8) & 1} "
            f"InX={(f >> 9) & 1}) XonLim={d.XonLim} XoffLim={d.XoffLim} "
            f"Xon={d.XonChar.hex()} Xoff={d.XoffChar.hex()} Err={d.ErrorChar.hex()} "
            f"Eof={d.EofChar.hex()} Evt={d.EvtChar.hex()}")


port = sys.argv[1] if len(sys.argv) > 1 else "COM1"
k32.CreateFileW.restype = wintypes.HANDLE
h = k32.CreateFileW("\\\\.\\" + port, 0xC0000000, 0, None, 3, 0, None)
if h in (None, wintypes.HANDLE(-1).value):
    print("open:", err())
    sys.exit(1)
print("open: ok")

cp = COMMPROP()
cp.wPacketLength = ctypes.sizeof(COMMPROP)
if k32.GetCommProperties(h, ctypes.byref(cp)):
    print(f"props: MaxTx={cp.dwMaxTxQueue} MaxRx={cp.dwMaxRxQueue} MaxBaud=0x{cp.dwMaxBaud:X} "
          f"Caps=0x{cp.dwProvCapabilities:X} Params=0x{cp.dwSettableParams:X} "
          f"Bauds=0x{cp.dwSettableBaud:X} Data=0x{cp.wSettableData:X} "
          f"StopPar=0x{cp.wSettableStopParity:X} CurTx={cp.dwCurrentTxQueue} CurRx={cp.dwCurrentRxQueue}")
else:
    print("GetCommProperties:", err())

print("SetupComm(4096,4096):", "ok" if k32.SetupComm(h, 4096, 4096) else err())

d = DCB()
d.DCBlength = ctypes.sizeof(DCB)
if not k32.GetCommState(h, ctypes.byref(d)):
    print("GetCommState:", err())
    sys.exit(1)
print("current:", dump(d))
base = DCB.from_buffer_copy(d)


def attempt(name, change):
    t = DCB.from_buffer_copy(base)
    change(t)
    ok = k32.SetCommState(h, ctypes.byref(t))
    print(f"{name:28s}", "ok" if ok else err())


def set_flags(t, mask, value):
    t.flags = (t.flags & ~mask) | value


attempt("unchanged state", lambda t: None)
attempt("baud 9600", lambda t: setattr(t, "BaudRate", 9600))
attempt("8N1", lambda t: (setattr(t, "ByteSize", 8), setattr(t, "Parity", 0), setattr(t, "StopBits", 0)))
attempt("fBinary", lambda t: set_flags(t, 1, 1))
attempt("DTR enable", lambda t: set_flags(t, 3 << 4, 1 << 4))
attempt("RTS enable", lambda t: set_flags(t, 3 << 12, 1 << 12))
attempt("no XON/XOFF, no CTS/DSR", lambda t: set_flags(t, (1 << 8) | (1 << 9) | (1 << 2) | (1 << 3), 0))
attempt("Xon=11 Xoff=13", lambda t: (setattr(t, "XonChar", b"\x11"), setattr(t, "XoffChar", b"\x13")))
attempt("ErrorChar off, Abort off", lambda t: set_flags(t, (1 << 10) | (1 << 11) | (1 << 14), 0))


def pyserial_like(t):
    t.BaudRate = 9600
    t.ByteSize, t.Parity, t.StopBits = 8, 0, 0
    set_flags(t, 0xFFFFFFFF, 1 | (1 << 4) | (1 << 12))
    t.XonChar, t.XoffChar = b"\x11", b"\x13"


attempt("everything pyserial sets", pyserial_like)
k32.CloseHandle(h)
