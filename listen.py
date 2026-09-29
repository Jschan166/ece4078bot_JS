"""
listen.py - PiBot robot-side server (runs ON the Raspberry Pi).

Changes compared with the original lab version (search for [FIX] / [NEW]):

  [FIX 1] Reliable encoder counts.
          Old: when a move ended, the PID thread reset the counters to 0 at the
          same moment the wheel server was reading them for the reply, so a
          move that DID happen was sometimes reported as (0, 0).
          New: counters are reset at the START of each move (never at stop),
          and the reply is sent after the wheels have braked to a stop, so the
          counts include the full movement (also the coasting after the stop).

  [FIX 2] Precise move duration.
          Old: the duration was checked every 20 ms, so a 0.146 s turn pulse
          could be up to ~14% too long.  New: checked every 2 ms.

  [FIX 3] Encoder-target moves (mode 2) cannot finish instantly on stale
          counts from the previous move.

  [NEW]   Start "kick" (80% power for 0.05 s at every start) is now adjustable:
              python listen.py --kick_pwm 80 --kick_time 0.05   (default = original)
              python listen.py --kick_time 0                     (no kick)
          If you change it, redo the turn/straight calibration.

Everything else (protocol, ports, PID, pins) is unchanged, so the PC-side
botconnect.py works exactly as before.
"""
import socket
import struct
import io
import threading
import argparse
import time
from time import monotonic
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# Network Configuration
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# PID Constants (default values, will be overridden by client)
use_PID = 1
KP, KI, KD = 0, 0, 0
MAX_CORRECTION = 30  # Maximum PWM correction value

# Global variables
running = True
left_pwm, right_pwm = 0, 0
left_count, right_count = 0, 0
prev_left_state, prev_right_state = None, None
MIN_PWM_THRESHOLD = 15
current_movement, prev_movement = 'stop', 'stop'

# [NEW] Adjustable start kick and stop settling (set from command line in __main__)
KICK_PWM = 80          # % duty cycle of the start kick (original: 80)
KICK_TIME = 0.05       # seconds (original: 0.05). 0 disables the kick.
STOP_SETTLE_TIME = 0.12  # [FIX 1] wait for the brake before reporting counts
TIMING_POLL = 0.002    # [FIX 2] 2 ms duration resolution (original: 20 ms)

# [FIX 1] Counter access is shared by 3 threads (encoder callbacks, PID, server)
count_lock = threading.Lock()


def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4, GPIO.OUT)

    # This prevents slight motor jerk when connection is established
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB, GPIO.LOW)

    # Encoder setup and interrupt (both activated and deactivated)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)

    # Initialize PWM
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 500)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 500)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)


def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)

    # Check for actual state change. Without this, false positive happens due to electrical noise
    if (prev_left_state is not None and current_state != prev_left_state):
        with count_lock:
            left_count += 1
        prev_left_state = current_state
    elif prev_left_state is None:
        prev_left_state = current_state  # First reading


def right_encoder_callback(channel):
    global right_count, prev_right_state
    current_state = GPIO.input(RIGHT_ENCODER)

    if (prev_right_state is not None and current_state != prev_right_state):
        with count_lock:
            right_count += 1
        prev_right_state = current_state
    elif prev_right_state is None:
        prev_right_state = current_state


def reset_encoder():
    global left_count, right_count
    with count_lock:
        left_count, right_count = 0, 0


def read_counts():
    """[FIX 1] Consistent snapshot of both counters."""
    with count_lock:
        return left_count, right_count


def set_motors(left, right):
    global prev_movement, current_movement

    # Pre-Start Kick (Motor Priming), to overcome static friction when movement starts
    if prev_movement == 'stop' and KICK_TIME > 0 and KICK_PWM > 0:
        if current_movement == 'forward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        elif current_movement == 'backward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        elif current_movement == 'clockwise':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        elif current_movement == 'anticlockwise':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)

        left_motor_pwm.ChangeDutyCycle(KICK_PWM)
        right_motor_pwm.ChangeDutyCycle(KICK_PWM)
        time.sleep(KICK_TIME)

    # Set the desired PWM
    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        # Active braking
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(100)

    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)


def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0  # Zero means stop
    elif abs(pwm_value) < min_threshold:
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value


def pid_control():
    '''This function sets the motor pwm using pid control'''
    global left_pwm, right_pwm, use_PID, KP, KI, KD, prev_movement, current_movement

    integral = 0
    last_error = 0
    last_time = monotonic()
    while running:
        current_time = monotonic()
        dt = current_time - last_time
        last_time = current_time

        prev_movement = current_movement
        if (left_pwm > 0 and right_pwm > 0): current_movement = 'forward'
        elif (left_pwm < 0 and right_pwm < 0): current_movement = 'backward'
        elif (left_pwm == 0 and right_pwm == 0): current_movement = 'stop'
        elif (left_pwm > 0 and right_pwm < 0): current_movement = 'clockwise'
        else: current_movement = 'anticlockwise'

        if current_movement == 'stop':
            # [FIX 1] Brake, but DO NOT reset the counters here: the wheel
            # server still needs them for the reply of the move that just ended.
            integral = 0
            last_error = 0
            set_motors(0, 0)
            time.sleep(0.005)
            continue

        # [FIX 1] New movement starting (or direction change): fresh counters,
        # so the PID and the reply only cover THIS movement.
        if prev_movement != current_movement:
            reset_encoder()
            integral = 0
            last_error = 0

        if not use_PID:
            target_left_pwm = left_pwm
            target_right_pwm = right_pwm
        else:
            lc, rc = read_counts()
            error = lc - rc
            proportional = KP * error
            integral += KI * error * dt
            integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))  # Anti-windup
            derivative = KD * (error - last_error) / dt if dt > 0 else 0
            correction = proportional + integral + derivative
            correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
            last_error = error

            if current_movement == 'forward':
                target_left_pwm = left_pwm - correction
                target_right_pwm = right_pwm + correction
            elif current_movement == 'backward':
                target_left_pwm = left_pwm + correction
                target_right_pwm = right_pwm - correction
            elif current_movement == 'clockwise':
                target_left_pwm = left_pwm - correction
                target_right_pwm = right_pwm - correction
            else:  # anticlockwise
                target_left_pwm = left_pwm + correction
                target_right_pwm = right_pwm + correction

        final_left_pwm = apply_min_threshold(target_left_pwm, MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(target_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)

        if target_left_pwm != 0 and args.verbose:
            lc, rc = read_counts()
            print(f"L/R PWM: ({target_left_pwm:.2f},{target_right_pwm:.2f}), L/R Enc: ({lc}, {rc})")

        time.sleep(0.01)


def camera_stream_server():
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (480, 360)})
    picam2.configure(camera_config)
    picam2.start()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    print(f"Camera stream server started on port {CAMERA_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Camera stream client connected")
            while running:
                ready = client_socket.recv(1)
                if not ready: break

                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)
                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size) + jpeg_data)
                except:
                    print("Camera stream client disconnected")
                    break

        except Exception as e:
            print(f"Camera stream server error: {str(e)}")

        if 'client_socket' in locals() and client_socket:
            client_socket.close()

    server_socket.close()
    picam2.stop()


def pid_config_server():
    global use_PID, KP, KI, KD

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    print(f"PID config server started on port {PID_CONFIG_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"PID config client connected")

            try:
                data = client_socket.recv(16)
                if data and len(data) == 16:
                    use_PID, KP, KI, KD = struct.unpack("!ffff", data)
                    if use_PID: print(f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}")
                    else: print("The robot is not using PID.")
                    response = struct.pack("!i", 1)
                else:
                    response = struct.pack("!i", 0)

                client_socket.sendall(response)

            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    client_socket.sendall(struct.pack("!i", 0))
                except:
                    pass

            client_socket.close()

        except Exception as e:
            print(f"PID config server error: {str(e)}")

    server_socket.close()


def recv_exact(sock, size):
    """Read exactly `size` bytes (a single recv() may return fewer)."""
    data = b''
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def stop_and_read_counts():
    """[FIX 1] Stop, let the brake finish, then return the counts of the move."""
    global left_pwm, right_pwm
    left_pwm = 0
    right_pwm = 0
    time.sleep(STOP_SETTLE_TIME)
    return read_counts()


def wheel_server():
    global left_pwm, right_pwm, running

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Wheel client connected")

            while running:
                try:
                    move_mode = recv_exact(client_socket, 1)
                    if not move_mode:
                        print("Wheel client disconnected")
                        break
                    move_mode = struct.unpack("!B", move_mode)[0]

                    if move_mode == 0:
                        data = recv_exact(client_socket, 8)
                        if not data:
                            print("Wheel client sending error")
                            break
                        left_speed, right_speed = struct.unpack("!ff", data)
                        print(f"Received Mode 0 with L/R speed: {left_speed:.4f}, {right_speed:.4f}")
                        left_pwm, right_pwm = left_speed * 100, right_speed * 100
                        counts = read_counts()

                    elif move_mode == 1:
                        data = recv_exact(client_socket, 12)
                        if not data:
                            print("Wheel client sending error")
                            break
                        left_speed, right_speed, duration = struct.unpack("!fff", data)
                        print(f"Received Mode 1 with L/R speed: {left_speed:.4f}, {right_speed:.4f}, Duration: {duration:.3f}s")

                        reset_encoder()  # [FIX 3] never report the previous move's counts
                        autonomous_start_time = monotonic()
                        left_pwm, right_pwm = left_speed * 100, right_speed * 100

                        # [FIX 2] 2 ms resolution instead of 20 ms
                        while running:
                            remaining = duration - (monotonic() - autonomous_start_time)
                            if remaining <= 0:
                                break
                            time.sleep(min(TIMING_POLL, remaining))

                        elapsed = monotonic() - autonomous_start_time
                        counts = stop_and_read_counts()
                        print(f"Timed movement completed after {elapsed:.3f}s, L/R enc: {counts}")

                    elif move_mode == 2:
                        data = recv_exact(client_socket, 16)
                        if not data:
                            print("Wheel client sending error")
                            break
                        left_speed, right_speed, target_left_enc, target_right_enc = struct.unpack("!ffii", data)
                        print(f"Received Mode 2 with L/R speed: {left_speed:.4f}, {right_speed:.4f}, L/R enc: {target_left_enc}, {target_right_enc}")

                        reset_encoder()  # [FIX 3] start from zero, not stale counts
                        autonomous_start_time = monotonic()
                        left_pwm, right_pwm = left_speed * 100, right_speed * 100

                        while running:
                            lc, rc = read_counts()
                            if lc >= target_left_enc and rc >= target_right_enc:
                                break
                            if monotonic() - autonomous_start_time >= 8:  # safety after 8s
                                print("Encoder-based movement failed (timeout).")
                                break
                            time.sleep(TIMING_POLL)

                        counts = stop_and_read_counts()
                        print(f"Encoder-based movement completed, L/R enc: {counts}")

                    else:
                        print(f"Unknown move mode {move_mode}")
                        break

                    # Send encoder counts back as acknowledgement for all modes
                    client_socket.sendall(struct.pack("!ii", int(counts[0]), int(counts[1])))

                except Exception as e:
                    print(f"Wheel client disconnected ({e})")
                    break

        except Exception as e:
            print(f"Wheel server error: {str(e)}")

        # Safety: stop the wheels whenever the client goes away
        left_pwm, right_pwm = 0, 0
        if 'client_socket' in locals() and client_socket:
            client_socket.close()

    server_socket.close()


def main():
    try:
        setup_gpio()

        pid_thread = threading.Thread(target=pid_control)
        pid_thread.daemon = True
        pid_thread.start()

        camera_thread = threading.Thread(target=camera_stream_server)
        camera_thread.daemon = True
        camera_thread.start()

        pid_config_thread = threading.Thread(target=pid_config_server)
        pid_config_thread.daemon = True
        pid_config_thread.start()

        wheel_server()

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--kick_pwm', type=float, default=80.0,
                        help="start kick duty cycle in %% (original 80)")
    parser.add_argument('--kick_time', type=float, default=0.05,
                        help="start kick duration in s (original 0.05, 0 = off)")
    parser.add_argument('--stop_settle', type=float, default=0.12,
                        help="wait after braking before reporting counts (s)")
    args = parser.parse_args()
    KICK_PWM = args.kick_pwm
    KICK_TIME = args.kick_time
    STOP_SETTLE_TIME = args.stop_settle
    print(f"Start kick: {KICK_PWM:.0f}% for {KICK_TIME:.3f}s | stop settle {STOP_SETTLE_TIME:.2f}s")
    main()
