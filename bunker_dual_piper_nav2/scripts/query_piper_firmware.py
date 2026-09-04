#!/usr/bin/env python3
"""Read PiPER firmware versions without issuing arm motion commands."""

import sys
import time

from piper_sdk import C_PiperInterface_V2


def query(port: str) -> str:
    piper = C_PiperInterface_V2(port)
    piper.ConnectPort()
    piper.SearchPiperFirmwareVersion()
    time.sleep(0.10)
    result = piper.GetPiperFirmwareVersion()
    if result == -0x4AF:
        piper.SearchPiperFirmwareVersion()
        time.sleep(0.25)
        result = piper.GetPiperFirmwareVersion()
    return str(result)


def main() -> int:
    ports = sys.argv[1:]
    if not ports:
        print("Usage: python3 query_piper_firmware.py <front_can> <rear_can>")
        print("Example: python3 query_piper_firmware.py can0 can1")
        return 2

    failed = False
    for port in ports:
        try:
            print(f"{port}: {query(port)}")
        except Exception as error:
            failed = True
            print(f"{port}: ERROR: {error}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

