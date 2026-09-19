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

# INCREASED to 35: Ensures motors have enough torque to move without needing a violent 80% kick
MIN_PWM_THRESHOLD = 35 
current_movement, prev_movement = 'stop', 'stop'


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
    
    # Initialize PWM (frequency: 100Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)


def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)
    if (prev_left_state is not None and current_state != prev_left_state):       
        left_count += 1
        prev_left_state = current_state 
    elif prev_left_state is None:
        prev_left_state = current_state


def right_encoder_callback(channel):
    global right_count, prev_right_state
    current_state = GPIO.input(RIGHT_ENCODER)
    if (prev_right_state is not None and current_state != prev_right_state): 
        right_count += 1
        prev_right_state = current_state
    elif prev_right_state is None:
        prev_right_state = current_state


def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0


def set_motors(left, right):
    global prev_movement, current_movement
    
    # THE MOTOR KICK BLOCK HAS BEEN COMPLETELY REMOVED HERE
    # This stops the robot from overshooting tap-turns blindly.
    
    # Set the desired PWM directly
    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
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
    global left_pwm, right_pwm, left_count, right_count, use_PID, KP, KI, KD, prev_movement, current_movement
    
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
        
        if not use_PID:
            target_left_pwm = left_pwm
            target_right_pwm = right_pwm
        else:
            if current_movement != 'stop':
                error = left_count - right_count
                proportional = KP * error
                integral += KI * error * dt
                integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))
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
                elif current_movement == 'anticlockwise':
                    target_left_pwm = left_pwm + correction
                    target_right_pwm = right_pwm + correction
            else:
                integral = 0
                last_error = 0
                reset_encoder()
                set_motors(0,0)
                time.sleep(0.005) 
                continue
            
        final_left_pwm = apply_min_threshold(target_left_pwm, MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(target_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)
        
        if target_left_pwm != 0 and args.verbose:
            print(f"L/R PWM: ({target_left_pwm:.2f},{target_right_pwm:.2f}), L/R Enc: ({left_count}, {right_count})")
        
        time.sleep(0.01)


def camera_stream_server():
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (480,360)})
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
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
                except:
                    pass
                    
            client_socket.close()
                    
        except Exception as e:
            print(f"PID config server error: {str(e)}")
    
    server_socket.close()
    

def wheel_server():
    global left_pwm, right_pwm, running, left_count, right_count
    
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
                    move_mode = client_socket.recv(1)
                    if not move_mode or len(move_mode) != 1:
                        print("Wheel client sending error")
                        break
                    move_mode = struct.unpack("!B", move_mode)[0]
                    
                    if move_mode == 0:
                        data = client_socket.recv(8)
                        if not data or len(data) != 8:
                            print("Wheel client sending error")
                            break
                        
                        left_speed, right_speed = struct.unpack("!ff", data)
                        left_pwm, right_pwm = left_speed*100, right_speed*100
                    
                    elif move_mode == 1:
                        data = client_socket.recv(12)
                        if not data or len(data) != 12:
                            print("Wheel client sending error")
                            break
                        
                        left_speed, right_speed, duration = struct.unpack("!fff", data)
                        left_pwm, right_pwm = left_speed*100, right_speed*100
                        
                        autonomous_start_time = monotonic()
                        while running:
                            elapsed_time = monotonic() - autonomous_start_time
                            if elapsed_time >= duration:
                                left_pwm = 0
                                right_pwm = 0
                                break
                            time.sleep(0.02)
                    
                    elif move_mode == 2:
                        data = client_socket.recv(16)
                        if not data or len(data) != 16:
                            print("Wheel client sending error")
                            break
                        
                        left_speed, right_speed, target_left_enc, target_right_enc = struct.unpack("!ffii", data)
                        left_pwm, right_pwm = left_speed*100, right_speed*100
                        
                        reset_encoder()
                        
                        autonomous_start_time = monotonic()
                        while running:
                            elapsed_time = monotonic() - autonomous_start_time
                            if left_count >= target_left_enc and right_count >= target_right_enc:
                                left_pwm = 0
                                right_pwm = 0
                                break
                            elif elapsed_time >= 8: 
                                left_pwm = 0
                                right_pwm = 0
                                break
                            time.sleep(0.02)
                        
                    response = struct.pack("!ii", left_count, right_count)
                    client_socket.sendall(response)
                    
                except Exception as e:
                    print(f"Wheel client disconnected")
                    break
                    
        except Exception as e:
            print(f"Wheel server error: {str(e)}")
        
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
    args = parser.parse_args()
    main()
