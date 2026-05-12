import socket
import struct
import time

class LIMITS:
    # Joint limits in degrees
    # Note: these limits are more conservative than in the Dobot documentation
    J1MIN = -90
    J1MAX = 90
    J2MIN = 0
    J2MAX = 70
    J3MIN = -15
    J3MAX = 40
    J4MIN = -90
    J4MAX = 90

class DobotClient:
    """
    Python API for Dobot Magician.
    """

    def __init__(self):
        self._sock = socket.socket()

        while True:
            try:
                time.sleep(1)
                with open("/tmp/DOBOT_MAGICIAN_PORT", "r") as f:
                    port = int(f.read())
                self._sock.connect(("localhost", port))
                break
            except:
                pass

    def start_homing(self):
        """
        Start homing procedure
        Input: none
        Output: none
        """
        self._sock.send(struct.pack("<1B4f", 0, 0.0, 0.0, 0.0, 0.0))

    def get_joint_state(self):
        """
        Get joint state
        Input: none
        Output: tuple (4 joint angles in degrees)
        """
        self._sock.send(struct.pack("<1B4f", 1, 0.0, 0.0, 0.0, 0.0))
        data = self._sock.recv(struct.calcsize("<4f"))
        response = struct.unpack("<4f", data)
        return response

    def is_goal_valid(self, j1, j2, j3, j4):
        """
        Check if point-to-point (PTP) goal for joints is valid
        Input: 4 floats (goal joint angles in degrees)
        Output: boolean (True if goal is valid)
        """
        self._sock.send(struct.pack("<1B4f", 2, j1, j2, j3, j4))
        data = self._sock.recv(struct.calcsize("<1B"))
        (response,) = struct.unpack("<1B", data)
        return bool(response)

    # Set point-to-point (PTP) goal for joints
    def set_joint_ptp(self, j1, j2, j3, j4):
        """
        Set point-to-point (PTP) goal for joints
        Input: 4 floats (goal joint angles in degrees)
        Output: none
        Note: goal will be clipped to joint limits 
        """
        self._sock.send(struct.pack("<1B4f", 3, j1, j2, j3, j4))

    # Set suction cup on/off
    def set_suction_cup(self, enable):
        """
        Set suction cup on/off
        Input: boolean (True to set suction cup on)
        Output: none
        """
        if enable:
            self._sock.send(struct.pack("<1B4f", 4, 0.0, 0.0, 0.0, 0.0))
        else:
            self._sock.send(struct.pack("<1B4f", 5, 0.0, 0.0, 0.0, 0.0))

    # Stop current action
    def stop_current_action(self):
        """
        Stop current action immediately
        Input: none
        Output: none
        """
        self._sock.send(struct.pack("<1B4f", 6, 0.0, 0.0, 0.0, 0.0))
