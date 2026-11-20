import json
import socket
import sys
import time

import pygame

HOST = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 5005
RATE_HZ = 20

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    sys.exit('No joystick found. Plug it in and try again.')
joy = pygame.joystick.Joystick(0)
joy.init()
print(f'Joystick: {joy.get_name()}  ->  {HOST}:{PORT}')

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

telemetry, last_telemetry = None, 0.0
period = 1.0 / RATE_HZ
try:
    while True:
        t0 = time.monotonic()
        pygame.event.pump()
        axes = [round(joy.get_axis(i), 3) for i in range(joy.get_numaxes())]
        buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
        sock.sendto(json.dumps({'axes': axes, 'buttons': buttons}).encode(), (HOST, PORT))

        while True:  # read everything that arrived
            try:
                data, _ = sock.recvfrom(2048)
                telemetry, last_telemetry = json.loads(data.decode()), time.monotonic()
            except BlockingIOError:
                break
            except ConnectionResetError:  # Windows "port unreachable" quirk
                break

        if telemetry and time.monotonic() - last_telemetry < 1.0:
            line = (f"roll {telemetry['roll']:7.1f}  pitch {telemetry['pitch']:7.1f}  "
                    f"yaw {telemetry['yaw']:7.1f}  depth {telemetry['depth']:5.2f} m")
        else:
            line = 'waiting for telemetry from the ROV...'
        print('\r' + line.ljust(70), end='', flush=True)

        time.sleep(max(0.0, period - (time.monotonic() - t0)))
except KeyboardInterrupt:
    print('\nBye')