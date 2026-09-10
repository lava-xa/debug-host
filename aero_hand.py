#!/usr/bin/env python3
# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time 
import struct
import threading
from serial import Serial, SerialTimeoutException
from typing import Iterator

from aero_open_sdk.aero_hand_constants import AeroHandConstants
from aero_open_sdk.joints_to_actuations import MOTOR_PULLEY_RADIUS, JointsToActuationsModel
from aero_open_sdk.actuations_to_joints import ActuationsToJointsModelCompact

## Setup Modes
HOMING_MODE = 0x01
SET_ID_MODE = 0x02
TRIM_MODE = 0x03

## Command Modes
CTRL_POS = 0x11
CTRL_TOR = 0x12
CTRL_SPE = 0x14

## Request Modes
GET_ALL = 0x21
GET_POS = 0x22
GET_VEL = 0x23
GET_CURR = 0x24
GET_TEMP = 0x25
GET_CAPS = 0x26

## Setting Modes
SET_SPE = 0x31
SET_TOR = 0x32

_UINT16_MAX = 65535

_RAD_TO_DEG = 180.0 / 3.141592653589793
_DEG_TO_RAD = 3.141592653589793 / 180.0

class AeroHand:
    def __init__(self, port=None, baudrate=921600):
        ## Connect to serial port
        if port is None:
            print("No port specified. Attempting to auto-detect Aero Hand serial port...")
            port = self._detect_port()
        self._serial_lock = threading.RLock()
        self.ser = Serial(
            port=None,
            baudrate=baudrate,
            timeout=0.1,
            write_timeout=0.25,
        )
        # Avoid leaving the ESP32-S3 USB serial interface in reset/boot mode.
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.port = port
        self.ser.open()

        # Opening the native USB CDC port can reset the ESP32.  Give the
        # firmware time to reach its 16-byte host protocol loop.
        time.sleep(0.8)

        ## Clean Buffers before starting
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        aero_hand_constants = AeroHandConstants()

        self.joint_names = aero_hand_constants.joint_names
        self.joint_lower_limits = aero_hand_constants.joint_lower_limits
        self.joint_upper_limits = aero_hand_constants.joint_upper_limits

        self.actuation_names = aero_hand_constants.actuation_names
        self.actuation_lower_limits = aero_hand_constants.actuation_lower_limits
        self.actuation_upper_limits = aero_hand_constants.actuation_upper_limits

        self.joints_to_actuations_model = JointsToActuationsModel()
        self.actuations_to_joints_model = ActuationsToJointsModelCompact()

    def _detect_port(self):

        base_path = '/dev/serial/by-id/'
        esp_32_prefix = 'usb-Espressif_USB_JTAG_serial_debug_unit_'
        
        if not os.path.exists(base_path):
            raise RuntimeError(
                "Could not find /dev/serial/by-id/.\n"
                "  → No serial-by-id symlinks found.\n"
                "  → Is this running on Linux? Is the Aero Hand connected?"
                "If running on Windows, please refer to the documentation to specify the port manually."
            )
        
        detected_ports = [d for d in os.listdir(base_path) if esp_32_prefix in d]

        if len(detected_ports) == 0:
            raise RuntimeError("No Aero Hand serial port detected. Check connection and try again.")
        elif len(detected_ports) > 1:
            raise RuntimeError("Multiple Aero Hand serial ports detected. Please specify the port manually.")
        else:
            return os.path.join(base_path, detected_ports[0])

    def create_trajectory(self, trajectory: list[tuple[list[float], float]]) -> Iterator[list[float]]:
        rate = 100  # Hz

        def _interp_keypoints(start, end, t):
            return [start[i] + t * (end[i] - start[i]) for i in range(len(start))]

        for i in range(1, len(trajectory)):
            prev_keypoint, _ = trajectory[i - 1]
            curr_keypoint, duration = trajectory[i]

            num_steps = int(duration * rate)

            for step in range(1, num_steps + 1):
                t = step / num_steps
                yield _interp_keypoints(prev_keypoint, curr_keypoint, t)

    def run_trajectory(self, trajectory: list):
        ## Linerly interpolate between trajectory points
        interpolated_traj = self.create_trajectory(trajectory)
        for waypoint in interpolated_traj:
            self.set_joint_positions(waypoint)
            time.sleep(0.01)
        return
    
    def convert_seven_joints_to_sixteen(self, positions: list) -> list:
        return [
            positions[0], positions[1], positions[2], positions[2],
            positions[3], positions[3], positions[3],
            positions[4], positions[4], positions[4],
            positions[5], positions[5], positions[5],
            positions[6], positions[6], positions[6],
        ]

    def set_joint_positions(self, positions: list):
        """
        Set the joint positions of the Aero Hand.

        Args:
            positions (list): A list of 16 joint positions. (degrees)
        """
        assert len(positions) in (16, 7), "Expected 16 or 7 Joint Positions"
        if len(positions) == 7:
            positions = self.convert_seven_joints_to_sixteen(positions)
        ## Clamp the positions to the joint limits.
        positions = [
            max(
                self.joint_lower_limits[i],
                min(positions[i], self.joint_upper_limits[i]),
            )
            for i in range(16)
        ]

        ## Convert to actuations
        actuations = self.joints_to_actuations_model.hand_actuations(positions)

        ## Normalize actuation to uint16 range. (0-65535)
        actuations = [
            (actuations[i] - self.actuation_lower_limits[i])
            / (self.actuation_upper_limits[i] - self.actuation_lower_limits[i])
            * _UINT16_MAX
            for i in range(7)
        ]
        try:
            self._send_data(CTRL_POS, [int(a) for a in actuations])
        except SerialTimeoutException as e:
            print(f"Serial Timeout while sending joint positions: {e}")
            return

    def tendon_to_actuations(self, tendon_extension: float) -> float:
        """
        Convert tendon extension (mm) to actuator actuations (degrees).
        Args:
            tendon_extension (float): Tendon extension in mm.
        Returns:
            float: actuator actuations in degrees.
        """

        return (tendon_extension / MOTOR_PULLEY_RADIUS) * _RAD_TO_DEG
    
    def actuations_to_tendon(self, actuation: float) -> float:
        """
        Convert actuator actuations (degrees) to tendon extension (mm).
        Args:
            actuation (float): actuator actuations in degrees.
        Returns:
            float: Tendon extension in mm.
        """

        return (actuation * MOTOR_PULLEY_RADIUS) * _DEG_TO_RAD

    def set_actuations(self, actuations: list):
        """
        This function is used to set the actuations of the hand directly.
        Use this with caution as Thumb actuations are not independent i.e. setting one
        actuation requires changes in other actuations. We use the joint to 
        actuations model to handle this. But this function give you direct access.
        If the actuations are not coupled correctly, it will cause Thumb tendons to
        derail.
        Args:
            actuations (list): A list of 7 actuations in degrees
            actuator actuations sequence being:
            (thumb_cmc_abd_act, thumb_cmc_flex_act, thumb_tendon, index_tendon, middle_tendon, ring_tendon, pinky_tendon)
        """
        assert len(actuations) == 7, "Expected 7 Actuations"

        ## Clamp the actuations to the limits.
        actuations = [
            max(
                self.actuation_lower_limits[i],
                min(actuations[i], self.actuation_upper_limits[i]),
            )
            for i in range(7)
        ]

        ## Normalize actuation to uint16 range. (0-65535)
        actuations = [
            (actuations[i] - self.actuation_lower_limits[i])
            / (self.actuation_upper_limits[i] - self.actuation_lower_limits[i])
            * _UINT16_MAX
            for i in range(7)
        ]

        try:
            self._send_data(CTRL_POS, [int(a) for a in actuations])
        except SerialTimeoutException as e:
            print(f"Error while writing to serial port: {e}")
            return

    def _wait_for_ack(self, opcode: int, timeout_s: float) -> bytes:
        frame = self._read_frame(opcode, timeout_s)
        return frame[2:]

    def _read_frame(self, opcode: int, timeout_s: float) -> bytes:
        """Read one matching 16-byte frame, skipping ESP32 boot text/noise."""
        deadline = time.monotonic() + timeout_s
        buffered = bytearray()
        marker = bytes((opcode & 0xFF, 0x00))
        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            chunk = self.ser.read(max(16, waiting))
            if not chunk:
                continue
            buffered.extend(chunk)

            frame_start = buffered.find(marker)
            if frame_start < 0:
                if len(buffered) > 1:
                    del buffered[:-1]
                continue
            if len(buffered) - frame_start >= 16:
                return bytes(buffered[frame_start:frame_start + 16])
            if frame_start:
                del buffered[:frame_start]

        raise TimeoutError(
            f"Response (opcode 0x{opcode:02X}) not received within {timeout_s}s"
        )
    
    def set_id(self, id: int, current_limit: int):
        """This fn is used by the GUI to set actuator IDs and current limits for the first time."""
        if not (0 <= id <= 253):
            raise ValueError("new_id must be 0..253")
        if not (0 <= current_limit <= 1023):
            raise ValueError("current_limit must be in between 0..1023")
        
        with self._serial_lock:
            self.ser.reset_input_buffer()
            payload = [0] * 7
            payload[0] = id & 0xFF   # stored in low byte of word0
            payload[1] = current_limit & 0x03FF
            self._send_data(SET_ID_MODE, payload)
            payload = self._wait_for_ack(SET_ID_MODE, 5.0)
        old_id, new_id, cur_limit = struct.unpack_from("<HHH", payload, 0)
        return {"Old_id": old_id, "New_id": new_id, "Current_limit": cur_limit}
    
    def set_speed(self, id: int, speed: int):
        """ 
        Set the speed of a specific actuator.This speed setting is max by default when the motor moves.
        This is different from speed control mode. It only affect the dynamic of motion execution during position control.
        Args:
            id (int): Actuator ID (0..6)
            speed (int): Speed value (0..32766)
        """
        if not (0 <= id <= 6):
            raise ValueError("id must be 0..6")
        if not (0 <= speed <= 32766):
            raise ValueError("speed must be in range 0..32766")
        with self._serial_lock:
            self.ser.reset_input_buffer()
            payload = [0] * 7
            payload[0] = id & 0xFFFF
            payload[1] = speed & 0xFFFF
            self._send_data(SET_SPE, payload)
            payload = self._wait_for_ack(SET_SPE, 2.0)
        id, speed_val = struct.unpack_from("<HH", payload, 0)
        return {"Servo ID": id, "Speed": speed_val}

    def set_torque(self, id: int, torque: int):
        """ 
         Set the torque of a specific actuator. This torque setting is max by default when the motor moves.
         This is different from torque control mode. It only affect the dynamic of motion execution during position control.
         Args:
            id (int): Actuator ID (0..6)
            torque (int): Torque value (0..1000)
        """
        if not (0 <= id <= 6):
            raise ValueError("id must be 0..6")
        if not (0 <= torque <= 1000):
            raise ValueError("torque must be in range 0..1000")
        with self._serial_lock:
            self.ser.reset_input_buffer()
            payload = [0] * 7
            payload[0] = id & 0xFFFF
            payload[1] = torque & 0xFFFF
            self._send_data(SET_TOR, payload)
            payload = self._wait_for_ack(SET_TOR, 2.0)
        id, torque_val = struct.unpack_from("<HH", payload, 0)
        return {"Servo ID": id, "Torque": torque_val}

    def trim_servo(self, id: int, degrees: int):
        """This fn is used by the GUI to fine tune the actuator positions."""
        if not (0 <= id <= 6):
            raise ValueError("id must be 0..6")
        if not (-360 <= degrees <= 360):
            raise ValueError("degrees out of range")
        
        with self._serial_lock:
            self.ser.reset_input_buffer()
            payload = [0] * 7
            payload[0] = id & 0xFFFF
            payload[1] = degrees & 0xFFFF
            self._send_data(TRIM_MODE, payload)
            payload = self._wait_for_ack(TRIM_MODE, 2.0)
        id, extend = struct.unpack_from("<HH", payload, 0)
        return {"Servo ID": id, "Extend Count": extend}
    
    def ctrl_speeds(self, speeds: list[int]):
        """Control all seven motor speeds with signed values.

        Positive values rotate counterclockwise, negative values rotate
        clockwise, and zero stops the corresponding motor.
        """
        if len(speeds) != 7:
            raise ValueError("speeds must contain exactly 7 values")
        if not all(-32766 <= speed <= 32766 for speed in speeds):
            raise ValueError("all speeds must be in range -32766..32766")
        payload = [speed & 0xFFFF for speed in speeds]
        self._send_data(CTRL_SPE, payload)

    def ctrl_torque(self, torques: list[int]):
        """
        Control all seven motor torques with signed values.

        Positive values apply counterclockwise torque, negative values apply
        clockwise torque, and zero stops applying torque.
        """
        if len(torques) != 7:
            raise ValueError("torques must contain exactly 7 values")
        if not all(-1000 <= torque <= 1000 for torque in torques):
            raise ValueError("all torques must be in range -1000..1000")
        payload = [torque & 0xFFFF for torque in torques]
        self._send_data(CTRL_TOR, payload)

    def _send_data(self, header: int, payload: list[int] = [0] * 7):
        assert self.ser is not None, "Serial port is not initialized"
        assert len(payload) == 7, "Payload must be a list of 7 integers in Range 0-65535"
        assert all(0 <= v <= 65535 for v in payload), "Payload values must be in Range 0-65535"
        msg = struct.pack("<2B7H", header & 0xFF, 0x00, *(v & 0xFFFF for v in payload))
        with self._serial_lock:
            self.ser.write(msg)
            self.ser.flush()

    def _request_values(self, opcode: int, format_string: str, timeout_s: float = 1.0):
        """Send a request and return the seven values from its matching reply."""
        with self._serial_lock:
            self.ser.reset_input_buffer()
            self._send_data(opcode)
            frame = self._read_frame(opcode, timeout_s)
        return struct.unpack(format_string, frame)[2:]

    def get_capabilities(self, timeout_s: float = 2.0) -> dict:
        """Probe the ESP32 host protocol without moving any motors."""
        values = self._request_values(GET_CAPS, "<2B7H", timeout_s)
        return {
            "protocol_version": values[0],
            "flags": values[1],
            "single_motor_speed": bool(values[1] & 0x0001),
            "signed_batch_control": bool(values[1] & 0x0002),
        }

    def send_homing(self, timeout_s: float = 175.0):
        with self._serial_lock:
            self.ser.reset_input_buffer()
            self._send_data(HOMING_MODE)
            payload = self._wait_for_ack(HOMING_MODE, timeout_s)
        if all(b == 0 for b in payload):
            return True
        else:
            raise ValueError(f"Unexpected HOMING payload: {payload.hex()}")

    def get_forward_kinematics(self):
        raise NotImplementedError("This method is not yet implemented")

    def get_joint_positions(self):
        raise NotImplementedError("This method is not yet implemented")
    
    def get_joint_positions_compact(self):
        """
        Get the joint positions from the hand in the compact 7 joint representation.
        Returns:
            list: A list of 7 joint positions. (degrees)
        """
        actuations = self.get_actuations()
        ## If there was an error getting actuations, return None
        if actuations is None:
            return None
        ## Convert to radians
        actuations = [act * _DEG_TO_RAD for act in actuations]

        ## Get Joint Positions
        joint_positions = self.actuations_to_joints_model.hand_joints(actuations)

        ## Convert to degrees
        joint_positions = [pos * _RAD_TO_DEG for pos in joint_positions]

        return joint_positions

    def get_actuations(self):
        """
        Get the actuation values from the hand.
        Returns:
            list: A list of 7 actuations. (degrees)
        """
        positions_uint16 = self._request_values(GET_POS, "<2B7H")
        ## Convert to degrees
        positions = [
            self.actuation_lower_limits[i]
            + (positions_uint16[i] / _UINT16_MAX)
            * (self.actuation_upper_limits[i] - self.actuation_lower_limits[i])
            for i in range(7)
        ]
        return positions

    def get_actuator_currents(self):
        """
        Get the actuator currents from the hand.
        Returns:
            list: A list of 7 actuator currents. (mA)
        """
        ## Convert to mA using the conversion factor 1 unit = 6.5 mA as per Feetech documentation
        values = self._request_values(GET_CURR, "<2B7h")
        currents_mA = [val * 6.5 for val in values]
        return currents_mA

    def get_actuator_temperatures(self):
        """
        Get the actuator temperatures from the hand.
        Returns:
            list: A list of 7 actuator temperatures. (Degree Celsius)
        """
        ## Temperatures are in degree Celsius directly
        values = self._request_values(GET_TEMP, "<2B7H")
        temperatures = [float(val) for val in values]
        return temperatures

    def get_actuator_speeds(self):
        """
        Get the actuator speeds from the hand.
        Returns:
            list: A list of 7 actuator speeds. (RPM)
        """
        ## Convert to RPM using the conversion factor 1 unit = 0.732 RPM as per Feetech documentation
        values = self._request_values(GET_VEL, "<2B7h")
        speeds_rpm = [val * 0.732 for val in values]
        return speeds_rpm

    def close(self):
        with self._serial_lock:
            self.ser.close()
