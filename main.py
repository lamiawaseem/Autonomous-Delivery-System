from hcrs04 import HCSR04
from machine import Pin, ADC, I2C, Timer, PWM
from imu import MPU6050
from time import sleep, ticks_ms, ticks_diff
import stepper
from servo import Servo 

# === Constants ===
SAMPLES = 4
THRESHOLD = 10
OBJECT_THRESHOLD = 5
IR = 6000
SPEED = 50  # Motor speed in percent
FIND_TAPE_INITIAL = False  # Flag for initial tape search

# === Initialize Components ===
# Ultrasonic sensors (assume: sensor1=left, sensor2=right, sensor3=front)
left_sensor = HCSR04(trigger_pin=16, echo_pin=17)
right_sensor = HCSR04(trigger_pin=5, echo_pin=6)
front_sensor = HCSR04(trigger_pin=26, echo_pin=27)


reed_switch = Pin(1, Pin.IN, Pin.PULL_DOWN)
line_sen = Pin(15, Pin.IN)
ir = ADC(28)
led = Pin("LED", Pin.OUT)
limit_switch = Pin(0, Pin.IN, Pin.PULL_UP)


# === L298N Motor Driver ===
# Motor A (Right motor)
motor_a_in1 = Pin(11, Pin.OUT)
motor_a_in2 = Pin(10, Pin.OUT)
motor_a_en = PWM(Pin(12))
motor_a_en.freq(1000)
motor_a_correction = 1.0


# Motor B (Left motor)
motor_b_in3 = Pin(9, Pin.OUT)
motor_b_in4 = Pin(8, Pin.OUT)
motor_b_en = PWM(Pin(7))
motor_b_en.freq(1000)
motor_b_correction = 1.0


# === Stepper Motor ===
IN1 = 1
IN2 = 2
IN3 = 3
IN4 = 4
stepper_motor = stepper.HalfStepMotor.frompins(IN1, IN2, IN3, IN4)

sg90 = Servo(Pin(21))

# === MPU6050 Setup ===
i2c = I2C(1, scl=Pin(3), sda=Pin(2))
imu = MPU6050(i2c)


# ==============================================
# Gyroscope Initialization and Callback
# ==============================================
def imu_init():
    """
    -Calibrates the gyroscope by taking samples and computing the average bias for each axis.
   
    Returns:
    -list: A list containing the average bias values for the x, y, and z axes.
    """
    samples = 500  # Number of readings to take
    bias = [0, 0, 0]  # makes list

    # Collects readings over multiple samples
    for _ in range(samples):
        gx, gy, gz = imu.gyro.xyz  # Get gyroscope data for x, y, z axes
        bias[0] += gx  
        bias[1] += gy  
        bias[2] += gz  
        sleep(0.01)    
    bias = [b / samples for b in bias]  # Divide by total number of samples
    print("Gyro Bias:", bias)
    return bias


def angular_position_callback(timer):
    """
    -Computes the change in position and angular position of the robot using the gyroscope's z-axis angular velocity
    
    Returns: None
    """
    global angular_position, last_update_time
    now = ticks_ms()
    dt = ticks_diff(now, last_update_time) / 1000.0  # seconds
    last_update_time = now


    # Read z-axis angular velocity and subtract bias
    _, _, gz = imu.gyro.xyz
    gz -= gyro_bias[2]
    angular_position += gz * dt
    # Keep angular_position in [0, 360)
    angular_position %= 360


angular_timer = Timer(0)
angular_timer.init(frequency=100, mode=Timer.PERIODIC, callback=angular_position_callback)


# ==============================================
# Motor Control Functions
# ==============================================
def right_motor(direction="stop", speed=0):
    """
    -Controls the right motor's direction and speed according to the specified direction (forward, backward, or stop).
    -The speed is also adjusted by a correction factor.
    
    Returns: None
    """
    adjusted = int(speed * motor_a_correction)
    #moves robot in specified direction
    if direction == "forward":
        motor_a_in1.value(1)
        motor_a_in2.value(0)
    elif direction == "backward":
        motor_a_in1.value(0)
        motor_a_in2.value(1)
    else:
        motor_a_in1.value(0)
        motor_a_in2.value(0)
    motor_a_en.duty_u16(int(adjusted * 65535 / 100)) #speed


def left_motor(direction="stop", speed=0):
    """
    -Controls the left motor's direction and speed according to the specified direction (forward, backward, or stop).
    -The speed is also adjusted by a correction factor.
    
    Returns: None
    """
    adjusted = int(speed * motor_b_correction)
    #moves robot in specified direction
    if direction == "forward":
        motor_b_in3.value(1)
        motor_b_in4.value(0)
    elif direction == "backward":
        motor_b_in3.value(0)
        motor_b_in4.value(1)
    else:
        motor_b_in3.value(0)
        motor_b_in4.value(0)
    motor_b_en.duty_u16(int(adjusted * 65535 / 100)) #speed


def stop_motors():
    """
    -Stops both motors
    
    Returns: None
    """
    right_motor("stop", 0)
    left_motor("stop", 0)


def drive_forward(speed):
    """
    -moves both motors forward at specified speed
    
    Returns: None
    """
    right_motor("forward", speed)
    left_motor("forward", speed)


def drive_backward(speed):
    """
    -moves both motors backward at specified speed
    
    Returns: None
    """
    right_motor("backward", speed)
    left_motor("backward", speed)

def angle_difference(target, current):
    """
    -Computes the smallest difference between angles
    
    Returns: None
    """
    return ((target - current + 180) % 360) - 180


def turn_left(speed, angle=90):
    """
    -Turns the robot left by a specified angle using gyroscope data.
        
    Returns: None
    """
    start = angular_position
    target = (start + angle) % 360
    # For left turn: right motor backward, left motor forward.
    right_motor("backward", speed)
    left_motor("forward", speed)
    # Use a tolerance of 2 degrees.
    while abs(angle_difference(target, angular_position)) > 2:
        sleep(0.01)
    stop_motors()
    print(f"Turned left {angle}°")


def turn_right(speed, angle=90):
    """
    -Turns the robot right by a specified angle using gyroscope data.
        
    Returns: None
    """
    # Record the current angular position
    start = angular_position
    # Calculate the target angular position
    target = (start - angle) % 360
    # For right turn: right motor forward, left motor backward.
    right_motor("forward", speed)
    left_motor("backward", speed)
    #turns the robot to targeted angular position within a 2 degree difference 
    while abs(angle_difference(target, angular_position)) > 2:
        sleep(0.01)
    stop_motors()
    print(f"Turned right {angle}°")


def turn_180(speed):
    """
    -Changes robots direction by rotating it 180 degrees
        
    Returns: None
    """
    turn_left(speed, 180)


# ==============================================
# Sensor Functions
# ==============================================
def get_average_distance(sensor, samples):
    """
    -Reads the distance from the specified sensor multiple times and computes the average distance
    - If the sensor fails to provide a valid reading, an error message is displayed.
        
    Returns: 
    -float: The average distance measured by the sensor.
    """
    #list to hold distance readings
    readings = []
    #gets multiple distance redings and puts them in list
    for _ in range(samples):
        try:
            readings.append(sensor.distance_cm())
        #reading fails
        except OSError as ex:
            print("Sensor error:", ex)
        sleep(0.02)
    # Returns the average of all readings or None if fails
    return sum(readings) / len(readings) if readings else None

def check_ultrasonic_sensors():
    """
    -Checks the readings of three ultrasonic sensors (left, right, and front) and prints the current distance
    -If any sensor detects an object below the threshold, it returns True to indicate an obstacle detection
    
    Returns: 
    -boolean: True if any ultrasonic sensor detects an object, otherwise False
    """
    #Gets average distance readings from each ultrasonic sensor
    left_distance = get_average_distance(left_sensor, SAMPLES)
    right_distance = get_average_distance(right_sensor, SAMPLES)
    front_distance = get_average_distance(front_sensor, SAMPLES)
    #prints valid readings or error statement id there are none
    if left_distance is not None and right_distance is not None and front_distance is not None:
        print(f"S1: {left_distance:.1f} cm, S2: {right_distance:.1f} cm, S3: {front_distance:.1f} cm")
    else:
        print("Ultrasonic sensor error.")
    #obstacle detected
    triggered = False
    #Checks if the sensors detect an object closer than the threshold
    if left_distance is not None and left_distance < THRESHOLD:
        print("PROX S1")
        triggered = True
    if right_distance is not None and right_distance < THRESHOLD:
        print("PROX S2")
        triggered = True
    if front_distance is not None and front_distance < THRESHOLD:
        print("PROX S3")
        triggered = True
    return triggered


def check_reed_switch():
    """
    -Checks if a magnetic field is detected
        
    Returns: 
    -boolean: True if the reed switch is triggered, otherwise False
    """
    return reed_switch.value() == 1


def check_line_sensor():
    """
    -Checks if black tape is detected
        
    Returns: 
    -boolean: True if the infrared sensor is triggered, otherwise False
    """
    return line_sen.value() == 0


def check_ir_photodiode():
    """
    -Checks if the IR photodiode detects a signal
        
    Returns: 
    -boolean: True if the IR photodiode detects a signal, otherwise False
    """
    return ir.read_u16() > IR


# ==============================================
# Forklift Functions (using Stepper Motor)
# ==============================================
def forklift_up():
    """
    -moves the forklift upwards using stepper motor and checks that it has been secured  
      
    Returns: 
    -boolean: True if the forklift has successfully moved up
    """
    #moves payload upward 180 degrees
    sg90.move(180)
    sleep(0.5)
    #secured
    payload_secured = True
    return payload_secured


def forklift_down():
    """
    -moves the forklift downwards using stepper motor and checks that it has been removed 

    Returns: 
    -boolean: True if the payload has been dropped
    """
    #returns to oringinal resting position
    sg90.move(0)
    sleep(0.5)
    #done
    payload_secured = False
    return payload_secured


# ==============================================
# Course Navigation Functions
# ==============================================
def start_tape():
    """
    -moves the robot forward until the line sensor detects the tape
    - then the robot stops, turns right, and returns the robot's angular position after alignment
   Returns: 
   -float: The robot's angular position after aligning with the tape.
    """
    #oves the robot forward until the line sensor detects the tape
    while not check_line_sensor():
        drive_forward(SPEED)
        sleep(0.01)
    stop_motors()
    sleep(0.5)
    #turns right
    turn_right(SPEED)  # Align to tape
    sleep(0.5)
    #updated positon
    return angular_position





def search_tape():
    """
    -Rotates the robot until the tape is found
    -If it doesn't find the tape within 2 seconds, it reverses direction and tries again.

    Returns:None
    """
    print("Searching for tape...")
    #rotates robot
    right_motor("backward", SPEED // 5)
    left_motor("forward", SPEED // 5)
    #starts time limit search to 2 seconds
    start_time = ticks_ms()
    #rotates until the tape is found
    while not check_line_sensor():
        #2 seconds have passed
        #Reverse direction and continue searching
        if ticks_diff(ticks_ms(), start_time) > 2000:
            stop_motors()
            sleep(0.5)
            right_motor("forward", SPEED // 5)
            left_motor("backward", SPEED // 5)
            # Continue searching until tape is found in the new direction
            while not check_line_sensor():
                sleep(0.1)
            break
        sleep(0.1)
    #stops once tape has been found
    stop_motors()
    print("Tape reacquired.")


def realign_on_tape(tape_angle):
    """
    -Realign robot to the recorded tape angle
    -The robot will turn either left or right
    -depending on the angular difference between its current position and the tape's recorded angle
        
    Returns: None
    """
    # Calculate the angular difference between the current position and the tape angle
    diff = angle_difference(tape_angle, angular_position)
    #determines smallest difference and turns in that direction
    #if already on tape, stops
    if diff > 0:
        turn_right(SPEED, abs(diff))
    elif diff < 0:
        turn_left(SPEED, abs(diff))
    else:
        print("Already aligned on tape.")
    stop_motors()
    sleep(0.5)


def front_clear(safe_distance):
    """
    -Checks if the front sensor detects an object within a safe distance.\

    Returns:
    - boolean: True if there is no obstacle in the front, otherwise False
    """
    front = get_average_distance(front_sensor, SAMPLES)
    return front is not None and front > safe_distance


def left_wall_detected():
    """
    -Checks if the left sensor detects a wall or obstacle.

    Returns:
    - boolean: True if the left sensor detects an object closer than the threshold
    """
    left = get_average_distance(left_sensor, SAMPLES)
    return left is not None and left < OBJECT_THRESHOLD


def right_wall_detected():
    """
    -Checks if the right sensor detects a wall or obstacle.

    Returns:
    - boolean: True if the right sensor detects an object closer than the threshold
    """
    right = get_average_distance(right_sensor, SAMPLES)
    return right is not None and right < OBJECT_THRESHOLD




def avoid_obstacle_front(speed):
    """
    Avoid a head-on obstacle using the original sensor bindings.
   
    Logic:
      1. Stop and reverse until the front sensor (sensor3) clears a safe distance.
      2. Then choose the more open side (comparing left and right sensors)
         and turn 90° away from the obstacle.
      3. Drive forward briefly to clear the obstacle.
    """
    #checks if front is clear otherwise stops
    if front_clear(2):
        print("Front clear, checking sides.")
    elif not front_clear(2):
        print("Avoiding head-on obstacle.")
        stop_motors()
        sleep(0.5)
    # Choose the more open side by comparing left and right sensor readings
        left_distance = get_average_distance(left_sensor, SAMPLES) or 999
        right_distance = get_average_distance(right_sensor, SAMPLES) or 999
        #Turns 90 degrees away from the obstacle based on the side with more space
        if left_distance > right_distance:
            print("Turning left to avoid head-on obstacle.")
            turn_left(speed, 90)
        else:
            print("Turning right to avoid head-on obstacle.")
            turn_right(speed, 90)
        sleep(0.5)
        #moves forward till it has cleared the obstacle
        drive_forward(speed)
        sleep(0.5)
        stop_motors()
        print("Head-on obstacle avoided.")




def avoid_obstacle_side(speed):
    """
    Avoid a side obstacle (object on the tape) using the "second binding."
   
    Logic:
      1. Read the left and right side sensors and decide which side is “wall‐contact.”
         (For example, if the right sensor reads very low (< OBJECT_THRESHOLD), then the wall is on the right.)
      2. Immediately perform a 90° turn in the opposite direction.
         (If the wall is on the right, the robot turns left.)
      3. Drive forward slowly while monitoring the sensor that was reading the wall.
      4. When that sensor reading rises above OBJECT_THRESHOLD, drive forward a little extra,
         then perform a "rebound" turn in the same direction as the sensor that originally saw the wall.
      5. Repeat until the tape (line sensor) is detected.
    """
    print("Starting side obstacle avoidance routine on tape.")
   
    # Step 3: Loop until the tape is detected.
    while not check_line_sensor():
        if not front_clear(2): # if the front is not clear, we break
            break
        else:
            #gets sensor readings
            left_distance = get_average_distance(left_sensor, SAMPLES)
            right_distance = get_average_distance(right_sensor, SAMPLES)
            #if no reading, assign high value
            if left_distance is None:
                left_distance = 999
            if right_distance is None:
                right_distance = 999
   
            #determines which side has the wall
            if right_distance > left_distance and left_distance < OBJECT_THRESHOLD: # if left is small, we assume wall on left.
                chosen_sensor = left_sensor  
                turn = "left"
            elif left_distance > right_distance and right_distance < OBJECT_THRESHOLD: # if right is small, we assume wall on right.
                chosen_sensor = right_sensor  
                turn = "right"

            #Gets the distance from the chosen sensor
            distance = get_average_distance(chosen_sensor, SAMPLES)
            print(f"Chosen sensor initial reading: {distance:.2f} cm")
            # Drive forward slowly while the chosen sensor still indicates a close wall.
            while (distance is not None) and (distance < OBJECT_THRESHOLD) and (not check_line_sensor()):
                drive_forward(speed // 2)
                sleep(0.05)
                distance = get_average_distance(chosen_sensor, SAMPLES)
                print(f"Current sensor reading: {distance:.2f} cm")
            #stopped once cleared
            stop_motors()
            sleep(0.1)
            #Moves forward to ensure the wall is cleared
            drive_forward(speed // 2)
            sleep(0.25)
            stop_motors()
            sleep(0.1)
            print(f"Wall no longer close on chosen side (reading: {distance:.2f} cm).")
       
            # Step 4: Perform rebound turn in the same direction as the sensor that originally saw the wall.
            #to find tape
            if chosen_sensor == right_sensor:
                turn_right(speed, 90)
            else:
                turn_left(speed, 90)
            sleep(0.1)
            drive_forward(speed // 2)
            sleep(0.25)
            stop_motors()
            sleep(0.1)  
        stop_motors()
        print("Tape detected. Exiting side obstacle avoidance routine.")


def avoid_obstacle_overall(speed):
    """
    -Overall obstacle avoidance process
        
    Returns: None
    """
    while not check_line_sensor():
        avoid_obstacle_front(speed)
        avoid_obstacle_side(speed)
               


def full_auto_to_payload(tape_angle, payload_secured, ALL_PAYLOADS_SECURED):
    """
    Navigate toward the payload zone using tape guidance.
   
    - The robot uses a very tight safe distance of 1.5 cm.
    - If an object is detected at 1.5 cm (front sensor), it stops and checks for a magnetic field.
        • If the reed switch is triggered, assume the payload is present and call forklift functions.
        • If not, treat it as an obstacle and call avoid_obstacle.
    - If the robot loses the tape, it calls search_tape() and then realigns to the recorded tape_angle.
    - The loop continues until the IR photodiode indicates that the payload zone is reached.
   
    Returns True when the payload zone is reached.
    """
    SAFE_DISTANCE = 1.5  # in cm
    print("Navigating toward payload zone...")
    while payload_secured == False and not ALL_PAYLOADS_SECURED:
        # Normal forward drive if tape is detected and no close front object.
        if front_clear(SAFE_DISTANCE) and check_line_sensor():
            drive_forward(SPEED)
        elif not front_clear(SAFE_DISTANCE):
            # An object is very close at 1.5 cm
            print("Close object detected in front!")
            stop_motors()
            sleep(0.25)
            #stop and check if obstacle is payload
            if check_reed_switch():
                print("Magnetic field detected - payload found!")
                drive_forward(SPEED // 4)  # Move forward slowly to avoid sudden movements.
                #if its a payload, activtes forklift and picks it up
                if check_reed_switch() and limit_switch.value() == 1:
                    print("Forklift up...")
                    forklift_up()
                    sleep(0.5)
                    #counts how many payloads have been picked up
                    payloads_picked_up += 1
            # If two payloads have been picked up, it turns right to look for another payload
            elif payloads_picked_up >= 2:
                turn_right(SPEED, 90)
                if check_reed_switch():  # Turn right to check for the next payload.
                    print("Magnetic field detected - payload found!")
                    drive_forward(SPEED // 4)
                    sleep(0.5)
                    stop_motors()
                    #picks up thirs payload if detected
                    if limit_switch.value() == 1 and check_reed_switch():
                        print("Forklift up...")
                        forklift_up()
                        sleep(0.5)
                        payloads_picked_up += 1
                    #returns to position before looking for third payload
                    else:
                        drive_backward(SPEED // 2)
                        sleep(0.5)
                        stop_motors()


                turn_180(SPEED)  # Turn 180° to face the next payload.
                #if object detected
                if check_reed_switch():
                    print("Magnetic field detected - payload found!")
                    drive_forward(SPEED // 4)
                    sleep(0.5)
                    stop_motors()
                    #if identified as payload, pick up
                    if limit_switch.value() == 1 and check_reed_switch():
                        print("Forklift up...")
                        forklift_up()
                        sleep(0.5)
                        payloads_picked_up += 1
                    #goes back and assumes all payloads have been found
                    else:
                        drive_backward(SPEED // 2)
                        sleep(0.5)
                        stop_motors()
                        ALL_PAYLOADS_SECURED = True
                        return ALL_PAYLOADS_SECURED
            # If no magnetic field is detected, assume obstacle and avoid it 
            else:  
                print("No magnetic field; treating as obstacle.")
                drive_backward(SPEED // 2)
                sleep(0.5)
                stop_motors()
                avoid_obstacle_overall(SPEED)
                realign_on_tape(tape_angle)
        elif not check_line_sensor():
            # If the tape is lost, search for it and realign.
            print("Tape lost during approach, searching...")
            search_tape()
            sleep(0.5)
            realign_on_tape(tape_angle)
        sleep(0.01)
    stop_motors()
    sleep(0.5)
    print("No more payloads detected.")
    return True


def move_forward_for_payload(speed):
    """
    -Moves the robot forward towards the payload zone

    Returns: None
    """
    right_motor("forward", speed)
    left_motor("forward", speed)
    sleep(0.01)


def move_backward_for_payload(speed):
    """
    -Moves the robot backward to reposition after detecting a payload or obstacle

    Returns: None
    """
    right_motor("backward", speed)
    left_motor("backward", speed)
    sleep(0.1)


def find_payload():
    """
    -Move towards the payload until detected by the reed switch
    
    Returns: None
    """
    while True:
        move_forward_for_payload(SPEED // 4)
        if check_reed_switch():
            print("Payload found!")
            move_forward_for_payload(SPEED // 6)
            break


def payload_evac():
    """
    -Evacuate the payload using the forklift stepper motor.
    Returns: None
    """
    #move backwards till limit switch is triggered
    while limit_switch.value() == 0:
        move_backward_for_payload(SPEED)
        sleep(0.1)
        #lift payload?
        if limit_switch.value() == 1 and check_reed_switch():
            print("Evacuating payload...")
            forklift_up()
            sleep(0.5)
    #move away
    move_backward_for_payload(SPEED)
    sleep(5)
    print("Payload evacuated")
    stop_motors()


def full_auto_dropoff(tape_angle_back):
    """
    -Navigate to the dropoff zone using tape guidance and obstacle avoidance.
    Returns: None
    """
    
    while True:
        #if no obstacle ahead move forward
        if front_clear(5):
            drive_forward(SPEED)
        #avoid obstacle if present
        elif not front_clear(5):
            print("Obstacle detected!")
            stop_motors()
            avoid_obstacle(SPEED)
            realign_on_tape(tape_angle_back)
        #find tape if necessary
        elif not check_line_sensor():
            print("Tape lost, searching...")
            search_tape()
            sleep(0.5)
            realign_on_tape(tape_angle_back)
        #otherwise move forward
        else:
            drive_forward(SPEED)
        sleep(0.01)


# ==============================================
# Main Autonomous Routine
# ==============================================
def main():
    global FIND_TAPE_INITIAL
    while True:
        #prints robots current position
        print(f"Angular Position: {angular_position:.2f}° | IR: {check_ir_photodiode()} | Line: {check_line_sensor()} | Magnet: {check_reed_switch()}")
        led.value(1 if check_ultrasonic_sensors() else 0)
        # Begin tape search and alignment
        tape_angle = start_tape()  # Drive until tape found and align
        tape_angle_back = (tape_angle + 180) % 360
        # Navigate toward payload zone using tape guidance and obstacle avoidance
        if full_auto_to_payload(tape_angle):
            print("Reached payload zone.")
            find_payload()
            # Optionally, forklift functions can be called here.
            payload_evac()
            # Navigate to dropoff zone
            full_auto_dropoff(tape_angle_back)
        sleep(0.1)


if __name__ == "__main__":
    #initiate gyroscope and main program
    gyro_bias = imu_init()
    main()
