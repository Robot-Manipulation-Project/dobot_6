"""
Implementation of the Dobot Magician server using selectors based on sockets.
Custom protocol:
    Start homing procedure
        command: 0, 0., 0., 0., 0.
        response: none
    Get joint state
        command: 1, 0., 0., 0., 0.
        response: j1, j2, j3, j4
    Check if point-to-point goal for joints is valid
        command: 2, j1, j2, j3, j4
        response: bool (True if goal is valid)
    Set point-to-point goal for joints (goal will be clipped to joint limits)
        command: 3, j1, j2, j3, j4
        response: none
    Set suction cup on
        command: 4, 0., 0., 0., 0.
        response: none
    Set suction cup off
        command: 5, 0., 0., 0., 0.
        response: none
    Stop current action
        command: 6, 0., 0., 0., 0.
        response: none
"""
import selectors
import socket
import struct

from dobot_driver import DobotDriver


sel = selectors.DefaultSelector()

dobot = DobotDriver()

def accept(sock, mask):
    # accept socket connections
    conn, addr = sock.accept()
    conn.setblocking(True)
    sel.register(conn, selectors.EVENT_READ, read)


def read(conn, mask):
    # process commands according to the custom protocol
    data = conn.recv(struct.calcsize("<1B4f"))
    if data:
        id, j1, j2, j3, j4 = struct.unpack("<1B4f", data)
        if id == 0:
            dobot.start_homing()
        elif id == 1:
            response = dobot.get_joint_state()
            conn.send(struct.pack("<4f", *response))
        elif id == 2:
            is_goal_valid = dobot.is_goal_valid(j1, j2, j3, j4)
            conn.send(struct.pack("<1B", is_goal_valid))
        elif id == 3:
            dobot.set_joint_ptp(j1, j2, j3, j4)
        elif id == 4:
            dobot.set_suction_cup(True)
        elif id == 5:
            dobot.set_suction_cup(False)
        elif id == 6:
            dobot.stop_current_action()
    else:
        sel.unregister(conn)
        conn.close()


# Create socket and listen for events (save port in a temp file)
sock = socket.socket()
sock.bind(("localhost", 0))
port = sock.getsockname()[1]
with open("/tmp/DOBOT_MAGICIAN_PORT", "w") as f:
    f.write(str(port))
sock.listen(1)
sock.setblocking(False)

sel.register(sock, selectors.EVENT_READ, accept)

while True:
    events = sel.select()
    for key, mask in events:
        callback = key.data
        callback(key.fileobj, mask)
