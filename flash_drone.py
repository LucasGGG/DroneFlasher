#!/usr/bin/env python3
"""
Drone Flash Tool
STM32G431CBU6  ->  hex firmware via dfu-util
ESP32 WROOM-32 ->  ELRS firmware via esptool.py
Betaflight CLI ->  dump restore via pyserial
"""

import subprocess
import sys
import time
import os
import glob
import argparse
import threading

# ── dependency check ──────────────────────────────────────────────────────────

def check_dep(cmd, install_hint):
    try:
        subprocess.run([cmd, "--version"], capture_output=True, check=True)
        return True
    except (FileNotFoundError, subprocess.CalledProcessError):
        print(f"[!] '{cmd}' not found. {install_hint}")
        return False

def ensure_deps():
    ok = True
    ok &= check_dep("dfu-util",  "brew install dfu-util")
    ok &= check_dep("esptool.py", "pip install esptool")
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        print("[!] pyserial not found. pip install pyserial")
        ok = False
    return ok

# ── helpers ───────────────────────────────────────────────────────────────────

def run(cmd, label=""):
    label = label or " ".join(cmd)
    print(f"\n>>> {label}")
    result = subprocess.run(cmd, text=True)
    return result.returncode == 0

def detect_dfu():
    """Return True if any STM32 DFU device is found."""
    r = subprocess.run(["dfu-util", "-l"], capture_output=True, text=True)
    return "Found DFU" in r.stdout or "STM" in r.stdout or "0483" in r.stdout

def list_serial_ports():
    from serial.tools import list_ports
    return [p.device for p in list_ports.comports()]

def pick_port(hint=""):
    ports = list_serial_ports()
    if not ports:
        return None
    if hint:
        matches = [p for p in ports if hint.lower() in p.lower()]
        if matches:
            return matches[0]
    print("\nAvailable serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p}")
    try:
        idx = int(input("Select port number: ").strip())
        return ports[idx]
    except (ValueError, IndexError):
        return ports[0]

def wait_for_dfu(timeout=30):
    print("[*] Waiting for STM32 DFU device", end="", flush=True)
    for _ in range(timeout):
        if detect_dfu():
            print(" found!")
            return True
        print(".", end="", flush=True)
        time.sleep(1)
    print(" timeout!")
    return False

def wait_for_serial(timeout=20, hint=""):
    print("[*] Waiting for serial port", end="", flush=True)
    for _ in range(timeout):
        port = pick_port(hint) if hint else None
        ports = list_serial_ports()
        if ports:
            print(f" {ports[0]}")
            return port or ports[0]
        print(".", end="", flush=True)
        time.sleep(1)
    print(" timeout!")
    return None

# ── STM32 flash ───────────────────────────────────────────────────────────────

def flash_stm32(hex_file, alt=0):
    """Flash hex to STM32 via dfu-util."""
    print("\n" + "="*60)
    print("STEP 1 — STM32 Firmware Flash")
    print("="*60)

    if not os.path.isfile(hex_file):
        print(f"[!] Hex file not found: {hex_file}")
        return False

    print(f"[*] Hex: {hex_file}")
    print("[*] Put flight controller in DFU mode (hold BOOT, plug USB)")

    if not wait_for_dfu():
        return False

    # dfu-util needs .bin; convert hex -> bin via objcopy if needed
    target = hex_file
    if hex_file.endswith(".hex"):
        bin_file = hex_file.replace(".hex", ".bin")
        print(f"[*] Converting hex -> bin: {bin_file}")
        r = subprocess.run(
            ["objcopy", "--input-target=ihex", "--output-target=binary",
             hex_file, bin_file],
            capture_output=True, text=True
        )
        if r.returncode != 0:
            # Try dfu-util direct hex support
            target = hex_file
            bin_file = None
        else:
            target = bin_file

    cmd = [
        "dfu-util",
        "--device", "0483:df11",
        "--alt", str(alt),
        "--dfuse-address", "0x08000000:leave",
        "--download", target,
    ]
    ok = run(cmd, f"dfu-util -> {os.path.basename(target)}")

    if ok:
        print("[+] STM32 flash complete. Controller rebooting...")
        time.sleep(3)
    else:
        print("[-] STM32 flash failed.")
    return ok

# ── Betaflight dump restore ───────────────────────────────────────────────────

def send_dump(dump_file, port=None, baud=115200):
    """Send Betaflight CLI dump over serial."""
    print("\n" + "="*60)
    print("STEP 2 — Betaflight CLI Dump Restore")
    print("="*60)

    if not os.path.isfile(dump_file):
        print(f"[!] Dump file not found: {dump_file}")
        return False

    import serial as ser_mod

    if not port:
        port = wait_for_serial(hint="usb")
        if not port:
            print("[!] No serial port detected.")
            return False

    print(f"[*] Port: {port}  Baud: {baud}")
    print(f"[*] Dump: {dump_file}")

    with open(dump_file, "r", encoding="utf-8", errors="ignore") as f:
        commands = [l.rstrip() for l in f if l.strip() and not l.startswith("#")]

    try:
        s = ser_mod.Serial(port, baud, timeout=2)
        time.sleep(1)

        # Enter CLI mode
        s.write(b"#\r\n")
        time.sleep(0.5)
        s.read_all()

        print(f"[*] Sending {len(commands)} commands...")
        for i, cmd in enumerate(commands):
            s.write((cmd + "\r\n").encode())
            time.sleep(0.05)
            if i % 50 == 0 and i > 0:
                print(f"    {i}/{len(commands)}", end="\r")
                time.sleep(0.2)

        # Save & reboot
        s.write(b"save\r\n")
        time.sleep(2)
        s.close()
        print("\n[+] Dump applied and saved.")
        return True

    except Exception as e:
        print(f"[-] Serial error: {e}")
        return False

# ── ELRS ESP32 flash ──────────────────────────────────────────────────────────

def flash_elrs(bin_file, port=None, baud=921600):
    """Flash ELRS firmware to ESP32 via esptool."""
    print("\n" + "="*60)
    print("STEP 3 — ELRS / ESP32 Flash")
    print("="*60)

    if not os.path.isfile(bin_file):
        print(f"[!] ELRS bin not found: {bin_file}")
        return False

    if not port:
        print("[*] Plug in ESP32 DevKit")
        port = wait_for_serial(hint="usbserial")
        if not port:
            print("[!] ESP32 not detected.")
            return False

    print(f"[*] Port: {port}")
    print(f"[*] Firmware: {bin_file}")

    cmd = [
        "esptool.py",
        "--chip", "esp32",
        "--port", port,
        "--baud", str(baud),
        "write_flash",
        "-z",
        "--flash_mode", "dio",
        "--flash_freq", "80m",
        "--flash_size", "detect",
        "0x00000", bin_file,
    ]
    ok = run(cmd, f"esptool.py -> {os.path.basename(bin_file)}")
    if ok:
        print("[+] ELRS flash complete.")
    else:
        print("[-] ELRS flash failed.")
    return ok

# ── main ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Drone flash tool: STM32 hex → dump → ELRS ESP32",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Full pipeline
  python flash_drone.py --hex fc_firmware.hex --dump bf_dump.txt --elrs elrs_rx.bin

  # Only flash firmware
  python flash_drone.py --hex fc_firmware.hex

  # Firmware + dump, no ELRS
  python flash_drone.py --hex fc_firmware.hex --dump bf_dump.txt

  # Only ELRS
  python flash_drone.py --elrs elrs_rx.bin --elrs-port /dev/cu.usbserial-0001
""",
    )
    p.add_argument("--hex",        metavar="FILE",   help="STM32 .hex firmware file")
    p.add_argument("--dump",       metavar="FILE",   help="Betaflight CLI dump .txt")
    p.add_argument("--elrs",       metavar="FILE",   help="ELRS .bin firmware for ESP32")
    p.add_argument("--fc-port",    metavar="PORT",   help="Serial port for FC (auto-detect if omitted)")
    p.add_argument("--elrs-port",  metavar="PORT",   help="Serial port for ESP32 (auto-detect if omitted)")
    p.add_argument("--baud",       type=int, default=115200, help="FC serial baud (default 115200)")
    p.add_argument("--elrs-baud",  type=int, default=921600, help="ELRS flash baud (default 921600)")
    p.add_argument("--dfu-alt",    type=int, default=0,      help="DFU alt setting (default 0)")
    return p.parse_args()

def main():
    args = parse_args()

    if not any([args.hex, args.dump, args.elrs]):
        print("[!] Nothing to do. Specify --hex, --dump, or --elrs (or all).")
        print("    Run with --help for examples.")
        sys.exit(1)

    if not ensure_deps():
        sys.exit(1)

    results = {}

    # Step 1: Flash STM32
    if args.hex:
        ok = flash_stm32(args.hex, alt=args.dfu_alt)
        results["STM32 flash"] = ok
        if not ok:
            print("\n[!] STM32 flash failed — aborting dump step.")
            args.dump = None

    # Step 2: Dump
    if args.dump:
        ok = send_dump(args.dump, port=args.fc_port, baud=args.baud)
        results["Dump restore"] = ok
        if not ok:
            print("\n[!] Dump failed.")

    # Step 3: ELRS
    if args.elrs:
        ok = flash_elrs(args.elrs, port=args.elrs_port, baud=args.elrs_baud)
        results["ELRS flash"] = ok

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    for step, ok in results.items():
        status = "[+] OK" if ok else "[-] FAILED"
        print(f"  {status}  {step}")

    all_ok = all(results.values())
    print("\nDone." if all_ok else "\nCompleted with errors.")
    sys.exit(0 if all_ok else 1)

if __name__ == "__main__":
    main()
