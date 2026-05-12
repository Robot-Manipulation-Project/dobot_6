import serial
import struct
import subprocess
import sys

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

# Helper function that clips joint values to the given limits
def clip(value, min_value, max_value):
    if value < min_value:
        return min_value
    elif value > max_value:
        return max_value
    else:
        return value


class DobotDriver:
    """
    Implementation of Dobot Magician USB driver based on the communication
    protocol v.1.1.5. Six basic commands are implemented:
        Start homing procedure
        Get joint state
        Check if point-to-point goal for joints is valid
        Set point-to-point goal for joints (goal will be clipped to joint limits)
        Set suction cup on/off
        Stop current action
    """

    def __init__(self):
        # Detect USB port connected to the Dobot and create serial connection (blocking read)
        for i in range(8):
            try:
                output = subprocess.run(
                    f"udevadm info --name=/dev/ttyACM{i}",
                    shell=True,
                    capture_output=True,
                )
                if output.stdout:
                    print(f"Dobot Magician found at /dev/ttyACM{i}")
                    self._usb = serial.Serial(f"/dev/ttyACM{i}", 115200)
                    # Set joint velocities and accelerations
                    self._send_command(80, 1, 1, "<8f", (30, 30, 30, 200, 200, 200, 200, 200), 8)
                    return None
            except:
                pass

        print("Dobot Magician was not found at /dev/ttyACM[0-7]")
        sys.exit(1)

    def _send_command(
        self,
        command_id,
        ctrl_rw,
        ctrl_isqueued,
        params_struct,
        params_values,
        response_size,
    ):
        """
        Send command to Dobot Magician with the fields:
            command_id : index of command
            ctrl_rw, ctrl_isqueued : control bits
            params_struct, params_values : C structure and values of parameters
            response_size : byte size of expected response
        """
        # Create command packet
        ctrl = ctrl_rw + (ctrl_isqueued << 1)
        params = struct.pack(params_struct, *params_values)
        params_size = struct.calcsize(params_struct)
        checksum = -(command_id + ctrl + sum(params)) % 256
        packet = struct.pack(
            f"<{params_size + 6}B",
            0xAA,
            0xAA,
            params_size + 2,
            command_id,
            ctrl,
            *params,
            checksum,
        )

        # Send command packet to Dobot, receive and verify response packet
        self._usb.write(packet)
        response = self._usb.read(response_size + 6)
        assert len(response) == (
            response_size + 6
        ), f"Response byte size is {len(response)} but expected {response_size + 6}"
        return response

    # Parse response packet
    @staticmethod
    def _parse_response(params_struct, response):
        # Verify checksum and byte size of response parameters
        checksum = -sum(response[3:-1]) % 256
        params_size = struct.calcsize(params_struct)
        assert (
            checksum == response[-1]
        ), f"Checksum is {checksum} but expected {response[-1]}"
        assert params_size == (
            response[2] - 2
        ), f"Response parameters byte size is {response[2] - 2} but expected {params_size}"

        # Unpack and return the response parameters
        params = struct.unpack(params_struct, response[5:-1])
        return params

    # Start homing procedure using the command sequence:
    # 1. Start homing procedure
    # 2. Set custom home joint state (0, 0, 0, 0)
    def start_homing(self):
        self._send_command(31, 1, 1, "<L", (0,), 8)
        self._send_command(84, 1, 1, "<B4f", (4, 0, 0, 0, 0), 8)

    # Get joint state
    # Note: j3 - j2 below is because j3actual = j3dobot - j2 in the Dobot internal representation
    def get_joint_state(self):
        response = self._send_command(10, 0, 0, "", (), 32)
        _, _, _, _, j1, j2, j3, j4 = self._parse_response("<8f", response)
        return j1, j2, j3 - j2, j4

    # Check if point-to-point (PTP) goal for joints is valid
    def is_goal_valid(self, j1, j2, j3, j4):
        if (
            j1 < J1MIN
            or j1 > J1MAX
            or j2 < J2MIN
            or j2 > J2MAX
            or j3 < J3MIN
            or j3 > J3MAX
            or j4 < J4MIN
            or j4 > J4MAX
        ):
            return False
        else:
            return True

    # Set point-to-point (PTP) goal for joints
    # Note: j2 + j3 below is because j3dobot = j2 + j3actual in the Dobot internal representation
    def set_joint_ptp(self, j1, j2, j3, j4):
        j1 = clip(j1, J1MIN, J1MAX) 
        j2 = clip(j2, J2MIN, J2MAX) 
        j3 = clip(j3, J3MIN, J3MAX) 
        j4 = clip(j4, J4MIN, J4MAX) 
        self._send_command(84, 1, 1, "<B4f", (4, j1, j2, j2 + j3, j4), 8)

    # Set suction cup on/off (1/0)
    def set_suction_cup(self, enable):
        self._send_command(62, 1, 1, "<2B", (1, 1 if enable else 0), 8)
        
    # Stop current action using the command sequence:
    # 1. Force stop any queue command
    # 2. Clear command queue
    # 3. Start executing commands from queue
    def stop_current_action(self):
        self._send_command(242, 1, 0, "", (), 0)
        self._send_command(245, 1, 0, "", (), 0)
        self._send_command(240, 1, 0, "", (), 0)

    # Send command, parse and return response
    def _iocommand(self, id, rw, queue, cmd_struct, cmd_params, res_size, res_struct):
        response = self._send_command(id, rw, queue, cmd_struct, cmd_params, res_size)
        return self._parse_response(res_struct, response)
