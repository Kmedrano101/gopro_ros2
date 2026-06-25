#!/usr/bin/env python3
"""Query connected GoPro(s) over USB for serial number, model, and firmware.

Useful for finding the values needed in config/gopro_cameras.yaml (`serial`,
and the `ip` derived from it -- see README "Find Camera Serial & IP").

Usage:
    python3 check_gopro.py              # auto-detect a single connected camera
    python3 check_gopro.py 252          # target the camera whose serial ends in "252"
    python3 check_gopro.py 252 106      # query multiple cameras, one at a time

Requires: pip install open-gopro
"""
import asyncio
import sys
from open_gopro import WiredGoPro


async def check_camera(serial=None):
    async with WiredGoPro(serial) as gopro:
        info = await gopro.http_command.get_camera_info()
        label = serial or "auto-detected"
        print(f"--- camera ({label}) ---")
        print(info.data)


async def main(serials):
    if not serials:
        # No serial given: connects to whichever camera answers mDNS first.
        # With multiple cameras connected, pass each one's serial suffix
        # explicitly (as shown above) to query them individually.
        await check_camera()
        return
    for serial in serials:
        await check_camera(serial)


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1:]))
