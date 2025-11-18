#!/usr/bin/env python3


# for python2.7 compatibility
from __future__ import print_function

import sys
import argparse
import binascii
import socket
import struct
import json
import zlib
import base64
import time
import array
import os
import tkinter as tk
import threading
import re

from sys import platform as _platform

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)

try:
    import serial
except ImportError as e:
    print("Failed to import serial: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyserial")
    print("")
    sys.exit(1)

# Example hardcoded listener topics list
COMMON_TOPICS = [
    "sensor_gyro",
    "sensor_accel",
    "sensor_gps_0",
    "sensor_gps_1",
    "estimator_status_flags",
    "vehicle_global_position",
    "sensor_septentrio",
    "sensor_gps_heading_0",
    "sensor_water_speed_generic",
    "nmea_engine",
]

class MavlinkSerialPort():
    '''an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets'''
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def write(self, b):
        '''write some bytes'''
        self.debug("sending '%s' (0x%02x) of len %u\n" % (b, ord(b[0]), len(b)), 2)
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0]*(70-len(buf)))
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0,
                                             0,
                                             n,
                                             buf)
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        '''read some bytes into self.buf'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(str(chr(x)) for x in data)

    def read(self, n):
        '''read some bytes'''
        if len(self.buf) == 0:
            self._recv()
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            if self._debug >= 2:
                for b in ret:
                    self.debug("read 0x%x" % ord(b), 2)
            return ret
        return ''

# Define time to use time.time() by default
def _time():
    return time.time()

# Detect python version
if sys.version_info[0] < 3:
    runningPython3 = False
else:
    runningPython3 = True
    if sys.version_info[1] >=3:
        # redefine to use monotonic time when available
        def _time():
            try:
                return time.monotonic()
            except Exception:
                return time.time()

def list_listener_topics(mav_serialport, timeout=3.0):
    """
    Sends the 'listener' command via MAVLink SERIAL_CONTROL
    and returns a list of available topics.
    """
    print("Requesting listener topics...")

    # Wake up shell
    mav_serialport.write('\n')
    time.sleep(0.3)

    # Send the listener command (no args)
    mav_serialport.write('listener\n')
    time.sleep(0.3)

    output = ''
    start_time = time.time()

    while time.time() - start_time < timeout:
        mav_serialport._recv()  # Pull in SERIAL_CONTROL messages
        chunk = mav_serialport.read(1024)
        if chunk:
            output += chunk
        time.sleep(0.05)

    # Extract topics from output
    topics = []
    match = re.search(r"Available topics:\s*(.*)", output, re.S)
    if match:
        topics_block = match.group(1)
        # Split on whitespace and filter out empty lines
        topics = [t.strip() for t in topics_block.split() if t.strip()]

    if topics:
        print(f"Found {len(topics)} topics:")
        for t in topics:
            print(f" - {t}")
    else:
        print("[!] No topics found.")

    return topics

def run_listener(mav_serialport, topic, timeout=3.0):
    """
    Run a PX4 'listener <topic>' command via MAVLink shell and return the output.

    Parameters:
        mav_serialport: Instance of MavlinkSerialPort
        topic (str): uORB topic to listen to (e.g., 'sensor_gyro', 'sensor_gps_0')
        timeout (float): Seconds to wait for output

    Returns:
        str: Output from the listener command
    """
    if not topic or not isinstance(topic, str):
        raise ValueError("Topic must be a non-empty string")

    command = f'listener {topic}'
    print(f"Running listener command: {command}")

    # Wake up NSH
    mav_serialport.write('\n')
    time.sleep(0.3)

    # Send the listener command
    mav_serialport.write(command + '\n')
    time.sleep(0.3)

    # Collect response
    output = ''
    start_time = time.time()

    while time.time() - start_time < timeout:
        mav_serialport._recv()
        chunk = mav_serialport.read(1024)
        if chunk:
            output += chunk
        time.sleep(0.05)

    output = output.strip()

    if output:
        print(f"==== listener {topic} Output ====")
        print(output)
    else:
        print(f"[!] No response from listener {topic} within timeout.")

    return output


def run_ver_mcu_command(mav_serialport, timeout=3.0):
    """
    Send 'ver mcu' command via MAVLink SERIAL_CONTROL and return the output.
    """
    print("Sending 'ver mcu' command...")

    # Wake up the shell
    mav_serialport.write('\n')
    time.sleep(0.3)

    # Send the command
    mav_serialport.write('ver mcu\n')
    time.sleep(0.3)

    # Read loop to capture shell output
    output = ''
    start_time = time.time()

    while time.time() - start_time < timeout:
        mav_serialport._recv()  # Pull in SERIAL_CONTROL messages
        chunk = mav_serialport.read(1024)
        if chunk:
            output += chunk
        time.sleep(0.05)

    output = output.strip()

    if output:
        print("==== ver mcu Output ====")
        print(output)
    else:
        print("[!] No response received within timeout.")

    return output

def run_shell_command(mav_serialport, command, timeout=3.0):
    """
    Send a shell command via MAVLink SERIAL_CONTROL and return the output.
    
    Parameters:
        mav_serialport: Instance of MavlinkSerialPort
        command (str): The shell command to send (e.g., 'ver mcu', 'listener sensor_gyro')
        timeout (float): How long to wait for the output before timing out

    Returns:
        str: Collected output from the shell
    """
    print(f"Sending shell command: '{command}'")

    # Wake up the shell
    mav_serialport.write('\n')
    time.sleep(0.3)

    # Send the command
    mav_serialport.write(command.strip() + '\n')
    time.sleep(0.3)

    # Collect output
    output = ''
    start_time = time.time()

    while time.time() - start_time < timeout:
        mav_serialport._recv()  # Pull in any SERIAL_CONTROL messages
        chunk = mav_serialport.read(1024)
        if chunk:
            output += chunk
        time.sleep(0.05)

    output = output.strip()

    if output:
        print(f"==== {command} Output ====")
        print(output)
    else:
        print("[!] No response received within timeout.")

    return output




def start_listener_ui(mav_serialport):
    root = tk.Tk()
    root.title("PX4 Listener Topic Selector")
    root.geometry("900x600")

    # Dropdown for topics
    tk.Label(root, text="Select uORB Topic to Listen:", font=("Arial", 14)).pack(pady=10)
    selected_topic = tk.StringVar(value=COMMON_TOPICS[0])
    topic_menu = tk.OptionMenu(root, selected_topic, *COMMON_TOPICS)
    topic_menu.config(font=("Arial", 12))
    topic_menu.pack()

    # Text widget for output display
    text_widget = tk.Text(root, wrap="word", font=("Courier", 12))
    text_widget.pack(fill="both", expand=True, pady=10, padx=10)

    running = [False]

    def update_output():
        last_output = ""
        while running[0]:
            output = run_listener(mav_serialport, selected_topic.get(), timeout=1.0)
            if output and output != last_output:
                text_widget.delete("1.0", tk.END)
                text_widget.insert(tk.END, output)
                last_output = output
            time.sleep(0.5)

    def start_stream():
        if running[0]:
            return  # already running
        running[0] = True
        threading.Thread(target=update_output, daemon=True).start()
        start_button.config(state="disabled")
        stop_button.config(state="normal")

    def stop_stream():
        running[0] = False
        start_button.config(state="normal")
        stop_button.config(state="disabled")

    # Start / Stop buttons
    button_frame = tk.Frame(root)
    button_frame.pack(pady=5)
    start_button = tk.Button(button_frame, text="Start Streaming", command=start_stream, font=("Arial", 12))
    start_button.pack(side="left", padx=5)
    stop_button = tk.Button(button_frame, text="Stop Streaming", command=stop_stream, state="disabled", font=("Arial", 12))
    stop_button.pack(side="left", padx=5)

    root.protocol("WM_DELETE_WINDOW", lambda: (stop_stream(), root.destroy()))
    root.mainloop()



def create_listener_window(mav_serialport):
    mav_serialport = MavlinkSerialPort("udp:0.0.0.0:14550", baudrate=57600, devnum=10)
    # Create a new top-level window (child of root)
    window = tk.Toplevel()
    window.title("New PX4 Listener")
    window.geometry("800x600")

    # You can reuse your listener UI code here but use `window` as parent
    # For example, topic dropdown, text widget, start/stop buttons, etc.

    # Sample UI parts:
    selected_topic = tk.StringVar(value=COMMON_TOPICS[0])
    topic_menu = tk.OptionMenu(window, selected_topic, *COMMON_TOPICS)
    topic_menu.pack()

    text_widget = tk.Text(window, wrap="word", font=("Courier", 12))
    text_widget.pack(fill="both", expand=True)

    running = [False]

    def update_loop():
        last_output = ""
        while running[0]:
            raw_output = run_listener(mav_serialport, selected_topic.get(), timeout=1.0)
            filtered_output = extract_topic_data(raw_output, selected_topic.get())
            if filtered_output and filtered_output != last_output:
                text_widget.delete("1.0", tk.END)
                text_widget.insert(tk.END, filtered_output)
                last_output = filtered_output
            time.sleep(0.5)

    def start_stream():
        if running[0]:
            return
        running[0] = True
        threading.Thread(target=update_loop, daemon=True).start()

    def stop_stream():
        running[0] = False

    start_button = tk.Button(window, text="Start Streaming", command=start_stream)
    start_button.pack(side="left", padx=10, pady=10)

    stop_button = tk.Button(window, text="Stop Streaming", command=stop_stream)
    stop_button.pack(side="left", padx=10, pady=10)

    window.protocol("WM_DELETE_WINDOW", lambda: (stop_stream(), window.destroy()))

def main_ui(mav_serialport):
    root = tk.Tk()
    root.title("PX4 Main Control")
    root.geometry("400x200")

    open_listener_button = tk.Button(root, text="Open New Listener Window",
                                     command=lambda: create_listener_window(mav_serialport))
    open_listener_button.pack(pady=50)

    root.mainloop()

def extract_topic_data(raw_output, topic):
    """
    Extract all blocks of data related to the topic from raw listener output.

    Returns a cleaned string with only that topic's info.
    """

    # Regex pattern to match blocks starting with the topic name, up to the next empty line or nsh> prompt
    # The re.DOTALL makes '.' match newlines
    # This pattern assumes each block starts with the topic name alone on a line
    pattern = rf"({re.escape(topic)}.*?)(?=\n\S|\n\n|nsh>)"

    matches = re.findall(pattern, raw_output, flags=re.DOTALL)

    if not matches:
        return f"[No {topic} data found]"

    # Join all matched blocks with a separator if multiple instances
    return "\n\n".join(match.strip() for match in matches)

if __name__ == "__main__":
    mav_serialport = MavlinkSerialPort("udp:0.0.0.0:14550", 57600, devnum=10)
    time.sleep(0.5)
    run_ver_mcu_command(mav_serialport)
    time.sleep(0.5)
    topics = list_listener_topics(mav_serialport)
    print(topics)
    time.sleep(0.5)
    #start_listener_ui(mav_serialport)
    main_ui(mav_serialport)
    time.sleep(0.5)
    mav_serialport.close()
