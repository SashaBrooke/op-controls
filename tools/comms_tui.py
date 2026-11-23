#!/usr/bin/env python3
import curses
import struct
import sys
import threading
import time
from collections import deque

import serial
import intcomm_pb2 as pb

# --- COBS & CRC-16 Framing ---

def crc16_ccitt(data: bytes, init: int = 0x0000) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)

def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        if code == 0 or idx + code > len(data) + 1:
            raise ValueError("Invalid COBS frame")
        idx += 1
        for _ in range(1, code):
            out.append(data[idx])
            idx += 1
        if code < 0xFF and idx < len(data):
            out.append(0)
    return bytes(out)

def frame_packet(pkt: pb.IntcommPacket) -> bytes:
    payload = pkt.SerializeToString()
    crc = crc16_ccitt(payload)
    # Note: '<H' is little-endian; change to '>H' if packet.c packs CRC big-endian
    raw = payload + struct.pack("<H", crc)
    return cobs_encode(raw) + b"\x00"

def unframe_packet(frame: bytes) -> pb.IntcommPacket:
    raw = cobs_decode(frame)
    if len(raw) < 3:
        raise ValueError("Frame too short")
    payload, rx_crc = raw[:-2], struct.unpack("<H", raw[-2:])[0]
    calc_crc = crc16_ccitt(payload)
    if rx_crc != calc_crc:
        raise ValueError(f"CRC mismatch: rx=0x{rx_crc:04X} calc=0x{calc_crc:04X}")
    pkt = pb.IntcommPacket()
    pkt.ParseFromString(payload)
    return pkt

# --- TUI Application ---

class CommsTUI:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.dtr = True
        self.ser.rts = True

        self.running = True
        self.lock = threading.Lock()
        self.packet_id = 1

        self.logs = deque(maxlen=200)
        self.latest_axis = "No AxisInfo stream received yet"
        self.latest_pid = "No PidInfo stream received yet"
        self.stream_counts = {"axis": 0, "pid": 0, "crc_err": 0}

    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        with self.lock:
            self.logs.append(f"[{ts}] {msg}")

    def send_command(self, cmd: pb.Command):
        cmd.packet_id = self.packet_id
        self.packet_id += 1
        pkt = pb.IntcommPacket(command=cmd)
        wire = frame_packet(pkt)
        self.ser.write(wire)
        self.ser.flush()
        self.log(f"TX Command (id={cmd.packet_id}, type={cmd.WhichOneof('cmd')})")

    def rx_worker(self):
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                for b in chunk:
                    if b == 0x00:
                        if buf:
                            self._handle_frame(bytes(buf))
                            buf.clear()
                    else:
                        buf.append(b)
            except Exception as e:
                self.log(f"RX Serial Error: {e}")
                time.sleep(0.2)

    def _handle_frame(self, frame: bytes):
        try:
            pkt = unframe_packet(frame)
        except Exception as e:
            with self.lock:
                self.stream_counts["crc_err"] += 1
            self.log(f"RX Drop: {e}")
            return

        which = pkt.WhichOneof("payload")
        if which == "ack":
            ack = pkt.ack
            code_name = pb.AckCode.Name(ack.code)
            resp_type = ack.WhichOneof("response_data")
            extra = ""
            if resp_type == "config_data":
                p = ack.config_data.pan_pid
                extra = f" | Config Kp={p.kp:.3f} Ki={p.ki:.3f} Kd={p.kd:.3f}"
            elif resp_type == "state_data":
                s = ack.state_data
                mode_name = pb.GimbalMode.Name(s.mode)
                extra = (f" | Mode={mode_name} Angle={s.pan_current_angle:.1f} "
                         f"Setpt={s.pan_setpoint:.1f} Rate={s.stream_rate_hz}Hz")
            self.log(f"RX ACK (id={ack.packet_id}): {code_name}{extra}")

        elif which == "stream":
            st = pkt.stream
            stype = st.WhichOneof("data")
            with self.lock:
                if stype == "axis_info":
                    a = st.axis_info
                    self.stream_counts["axis"] += 1
                    self.latest_axis = (
                        f"Axis={pb.Axis.Name(a.axis)} | Angle={a.angle_deg:7.2f} deg | "
                        f"Dir={pb.MotorDirection.Name(a.motor_dir)} | PWM={a.motor_pwm}"
                    )
                elif stype == "pid_info":
                    p = st.pid_info
                    self.stream_counts["pid"] += 1
                    self.latest_pid = (
                        f"Setpt={p.setpoint:6.1f} Meas={p.measurement:6.1f} | "
                        f"P={p.p_term:6.2f} I={p.i_term:6.2f} D={p.d_term:6.2f} | Out={p.output:6.2f}"
                    )

    def prompt_input(self, stdscr, prompt: str) -> str:
        h, w = stdscr.getmaxyx()
        curses.echo()
        curses.curs_set(1)
        stdscr.addstr(h - 2, 0, " " * (w - 1))
        stdscr.addstr(h - 2, 0, prompt, curses.A_BOLD)
        stdscr.refresh()
        raw = stdscr.getstr(h - 2, len(prompt), 40).decode("utf-8").strip()
        curses.noecho()
        curses.curs_set(0)
        return raw

    def run(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        rx_thread = threading.Thread(target=self.rx_worker, daemon=True)
        rx_thread.start()
        self.log(f"Connected to {self.ser.port}")

        while self.running:
            self.draw(stdscr)
            ch = stdscr.getch()
            if ch == -1:
                time.sleep(0.05)
                continue

            if ch in (ord("q"), ord("Q")):
                self.running = False
            elif ch == ord("1"):
                self.send_command(pb.Command(system_action=pb.ACTION_GIMBAL_ARM))
            elif ch == ord("2"):
                self.send_command(pb.Command(system_action=pb.ACTION_GIMBAL_FREE))
            elif ch == ord("3"):
                self.send_command(pb.Command(system_action=pb.ACTION_GIMBAL_READ))
            elif ch == ord("4"):
                self.send_command(pb.Command(system_action=pb.ACTION_CONFIG_READ))
            elif ch == ord("5"):
                self.send_command(pb.Command(system_action=pb.ACTION_CONFIG_SAVE))
            elif ch == ord("6"):
                val = self.prompt_input(stdscr, "Pan Setpoint (0-4095): ")
                if val:
                    try:
                        cmd = pb.Command(setpoint_command=pb.SetpointCommand(
                            axis=pb.AXIS_PAN, setpoint=float(val)))
                        self.send_command(cmd)
                    except ValueError:
                        self.log("Invalid float value")
            elif ch == ord("7"):
                val = self.prompt_input(stdscr, "Stream [on/off] & Rate Hz (e.g. 'on 50' or 'off'): ")
                if val:
                    parts = val.split()
                    enable = parts[0].lower() in ("on", "1", "true")
                    rate = int(parts[1]) if len(parts) > 1 else 20
                    cmd = pb.Command(stream_command=pb.StreamCommand(
                        global_streaming=enable,
                        stream_rate_hz=rate,
                        pan_pos_stream=enable,
                        pan_pid_stream=enable,
                    ))
                    self.send_command(cmd)
            elif ch == ord("8"):
                val = self.prompt_input(stdscr, "Set Pan PID 'Kp Ki Kd' (e.g. '2.5 0.1 0.05'): ")
                if val:
                    try:
                        kp, ki, kd = map(float, val.split())
                        cmd = pb.Command(pid_set_command=pb.PidSetCommand(
                            axis=pb.AXIS_PAN,
                            params=pb.PidParams(kp=kp, ki=ki, kd=kd)
                        ))
                        self.send_command(cmd)
                    except ValueError:
                        self.log("Enter 3 numbers separated by spaces")
            elif ch == ord("9"):
                cmd = pb.Command(pid_revert_command=pb.PidRevertCommand(axis=pb.AXIS_PAN))
                self.send_command(cmd)

        self.ser.close()

    def draw(self, stdscr):
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        stdscr.addstr(0, 0, "=== OHPOSSUM CONTROLS - USB CDC PROTOCOL TUI ==="[:w-1], curses.A_BOLD)
        menu = (
            "[1] Arm  [2] Free  [3] Read State  [4] Read Config  [5] Save Config\n"
            "[6] Setpoint  [7] Stream On/Off + Hz  [8] Set PID  [9] Revert PID  [Q] Quit"
        )
        stdscr.addstr(2, 0, menu)

        with self.lock:
            counts = dict(self.stream_counts)
            axis_str = self.latest_axis
            pid_str = self.latest_pid
            log_lines = list(self.logs)

        stdscr.addstr(5, 0, f"--- LIVE TELEMETRY (Axis pkts: {counts['axis']} | PID pkts: {counts['pid']} | CRC Err: {counts['crc_err']}) ---"[:w-1], curses.A_BOLD)
        stdscr.addstr(6, 2, axis_str[:w-3])
        stdscr.addstr(7, 2, pid_str[:w-3])

        stdscr.addstr(9, 0, "--- COMMAND & ACK LOG ---"[:w-1], curses.A_BOLD)
        avail_rows = max(1, h - 11)
        for idx, line in enumerate(log_lines[-avail_rows:]):
            if 10 + idx < h - 1:
                stdscr.addstr(10 + idx, 0, line[:w-1])

        stdscr.refresh()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: uv run comms_tui.py <COM_PORT> [BAUD]")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    tui = CommsTUI(port, baud)
    try:
        curses.wrapper(tui.run)
    finally:
        tui.running = False
        if getattr(tui, "ser", None) and tui.ser.is_open:
            tui.ser.close()
