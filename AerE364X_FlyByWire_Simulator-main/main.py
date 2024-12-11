# Put the various CircuitPython import statements here
import board
import busio
import adafruit_bno055
import time
import pwmio
import digitalio
import pulseio
import pulse_commands
# Configuration of the simulator (will have to remove these lines
# when running on real hardware) 
import dynamic_model
dynamic_model.enable_wind= False   # Set this to True to add a moment from the wind
dynamic_model.enable_vertical_motion= True

# ***NOTE*** Do not get the various files which are part of the simulator
# (adafruit_bno055.py, busio.py, board.py, etc.) confused with similarly
# named files used for actual CircuitPython hardware. Do NOT copy the
# simulator versions onto your Feather microcontroller! 


# Now create objects for the I2C bus, for the BNO055,
# PulseIn, PWMOut, and/or DigitalInOut
#
# Will only work with pins used as follows (per pin
# definitions in board.py)
#    SCL: SCL
#    SDA: SDA
#    D5: Rotational command pulse input from RC receiver
#    D6: Forward command pulse input from RC receiver (fixed at 1500us)
#    D9: PWM pulse out to left-side fan.
#        Running this fan forwards helps turn right,
#        Running backwards helps turn left 
#    D10: PWM pulse out to right-side fan
#        Running this fan forwards helps turn left
#        Running backwards helps turn right
#    D13: LED output. Use to indicate calibrationstate of BNO055
#
# You may also want variables for other values such as your chosen PWM frequency,


frqcy= 40
pulse_read_heading = pulseio.PulseIn(board.D5,maxlen=8)
pulse_read_lift= pulseio.PulseIn(board.D6,maxlen=8)
#pulse_read_Thrust= pulseio.PulseIn(board.D12,maxlen=6)
out_cmd_rotation=0 #in degrees
out_cmd_rotation_duty_cycle= 3932  #3932=1.5ms @40Hz
read_cmd_rotation=0
H_Error=0
Current_Alt=0
Previous_Alt=0
# Current_Alt=0
# out_cmd_alt=0
#Fans #actual drone right and left thrust+Lift back for heading
Right_fan=pwmio.PWMOut(board.D10,frequency= frqcy, duty_cycle=out_cmd_rotation_duty_cycle)
Left_fan=pwmio.PWMOut(board.D9,frequency= frqcy, duty_cycle=out_cmd_rotation_duty_cycle)
Lift_fan=pwmio.PWMOut(board.D11,frequency= frqcy, duty_cycle=2621)
#BNO055
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
led= digitalio.DigitalInOut(board.D13)
led.direction=digitalio.Direction.OUTPUT
led.value = False

Kp_coefficient=0.5
Kp_term=0
Kd_coefficient= 14
Kd_term=0
Ki_coefficient= 0.1
Ki_term=0

Kp_Alt_coefficient=3
Kp_Alt_term=0
Kd_Alt_coefficient= 10
Kd__Alt_term=0
Ki_Alt_coefficient= 1
Ki_Alt_term=0
# When adding integration capability
# You will need a variable to accumulate the integral term, etc. 
# You will also need to keep track of the time (from time.monotonic())
# of the previous iteration from the loop so you can accumulate the
# right amount
t_1=time.monotonic()

# Start an infinite loop here:
while True:
    
    dt= time.monotonic() - t_1
    t_1=time.monotonic()
    print(f'dt is {dt}')
    print(f't_1 is {t_1}')
    Heading_cmd_read=0
    Lift_cmd_read=0
    # Wait until pulses have been received 
    while (len(pulse_read_heading)) == 0 :
        time.sleep(0.1)
        print("no heading yet")
   
    while (len(pulse_read_lift)) == 0 :
        time.sleep(0.1)
        print("no lift yet")

    
    (Lift_cmd_read, Heading_cmd_read) = pulse_commands.get_pulse_commands ([pulse_read_lift,pulse_read_heading])

     
        
   
    # Check whether the BNO055 is calibrated
    # and turn on the LED on D13 as appopriate
    if sensor.calibrated == True:
        led.value =True
        ##print("led is on")
        # Extract the euler angles (Heading, Roll, Pitch)
        # in degrees from the IMU
    [Heading, Roll, Pitch]= sensor.euler
    Current_Alt= dynamic_model.dynamic_instance.pos[2]
        ##print (Heading)
        # Determine the commanded orientation based on from your pulse input from
        # pin D5
    read_cmd_rotation= (Heading_cmd_read-1500)*0.36 #will give an angle in degrees (-180 to 180)
    read_cmd_Altitude= (Lift_cmd_read-1000)/100 #give you altitude in m
        # Select a coefficient for the proportional term
        # It will probably have units similar to output_command_ms/degree
        # Determine the proportional term by obtaining the error (subtracting
        # the commanded and actual headings), then multiplying by the proportional
        # coefficient
        # 
        # TIP: If you just do simple subtraction you have a problem if the
        # commanded heading is (for example) +179 deg and the actual heading is
        # -179 deg. as it will try to turn the long way around. 
        # The simplest solution is to add 360+180 deg. to the error, then use the
        # Python modulus operator (%) to get the remainder when dividing by
        # 360, then subtract 180 deg., e.g. replace simple subtraction with
        #  ((Heading_command - Heading + 360 + 180) % 360  - 180)
    H_Error= ((read_cmd_rotation - Heading + 360 + 180) % 360  - 180)
    Alt_error= read_cmd_Altitude-Current_Alt
    print (f'the error is {H_Error}degrees')
    print (f'the Alt error is {Alt_error}')
    Kp_term= H_Error*Kp_coefficient
    Kp_Alt_term= Alt_error*Kp_Alt_coefficient
    Kd_term= sensor.gyro[2]*Kd_coefficient
    Kd__Alt_term= Kd_Alt_coefficient* (Current_Alt-Previous_Alt)/dt
    Ki_term= Ki_coefficient*H_Error*dt + Ki_term
    Ki_Alt_term= Ki_Alt_coefficient*Alt_error*dt + Ki_Alt_term

    if Ki_term >100:
        Ki_term = 100
    if Ki_term < -100:
        Ki_term = -100

    if Ki_Alt_term >100:
        Ki_Alt_term = 100
    if Ki_Alt_term < -100:
        Ki_Alt_term = -100
        
    # #print (f'sensor gyro is {sensor.gyro[2]}')
    # #print (f'the Kp term is {Kp_term}')
    # #print (f'the Kd term is {Kd_term}')
    # #print (f'the Ki term is {Ki_term}')
    # To start use just the proportional term to determine the output rotation
    # command, which is an offset in ms from the nominal 1.5 ms that commands
    # the fully reversing motors to not move
    out_cmd_rotation=Kp_term+Kd_term+Ki_term
    out_cmd_rotation_ms=out_cmd_rotation/0.36

    out_cmd_alt= read_cmd_Altitude
    # out_cmd_alt=Kp_Alt_term+Ki_Alt_term
    out_cmd_alt_ms=out_cmd_alt*100
    print(f'alt {Current_Alt}')
        # Bound the output rotation command so that it cannot exceed 0.5 or be less than
        # -0.5 (note this is 500)
    if out_cmd_rotation_ms >500:
        out_cmd_rotation_ms =500
    if out_cmd_rotation_ms < -500:
        out_cmd_rotation_ms = -500

    if out_cmd_alt_ms >1000:
        out_cmd_alt_ms =1000
    if out_cmd_alt_ms < 0:
        out_cmd_alt_ms = 0
        
    print(f'out cmd alt {out_cmd_alt_ms}')
        ##print(out_cmd_rotation_ms)
        # Apply the output rotation command in opposite senses to determine the duty
        # cycle for the PWM outputs to the left- and right- side fans (pins D7, D8)
    Right_fan.duty_cycle= 3932 -(out_cmd_rotation_ms*2.62) #3932=1.5ms @40Hz
    Left_fan.duty_cycle= 3932+ (out_cmd_rotation_ms*2.62) #3932=1.5ms @40Hz
    Lift_fan.duty_cycle= 2621+ (out_cmd_alt_ms*2.62) #2621=1ms @40Hz
        ##print (f'left fan is {Left_fan.duty_cycle}')
        ##print (f'right fan is {Right_fan.duty_cycle}')
        
    Previous_Alt=Current_Alt
    time.sleep(0.1)
    pass # Done with loop


# Once you have the above working, you can test it out (making sure it is saved
# as "main.py" by running it with the simulator "python simulator.py"

# You can control the commanded heading by clicking in the lower-right quadrant.
# The x-y arrow in the upper-left quadrant should show you the orientation of
# your craft. You can add debugging prints as needed. Start out with a small
# proportional coefficient and increase it so that it makes the craft rotate
# at a reasonable rate. 

# You will find that the craft is extremely under-damped. This is because there is
# little air resistance to rotation.

# To get it to rapidly stabilize at the desired orientation you will need to add
# a derivative term. You can get the z axis rotation rate from the .gyro[2]
# property of the BNO055.
#
# Then you will need to define a derivative coefficient (which may have to be
# negative) represented as command_ms / (rad/sec). You can create the derivative
# by multiplying your derivative coefficient by the gyro rate and include it in the
# output rotation command.

# Once you have that working consider the case when some breeze tends to
# make the craft spontaneously rotate (set dynamic_model.enable_wind=True,
# above). To still equilibriate to the correct orientation you will need
# to add an integral term.
#
# Each time through the loop the integral gets added to it
# the integral coefficient * error * dt
# where dt is the difference in time (as measured with time.monotonic())
# from the previous loop iteration to this loop iteration. 
#  * Remember to use the TIP above when calculating the error.
#
# Every time you update the integral term you also have to bound it,
# lest it get too big and cause instability. This limit would probably
# have units of command_ms and would be a reasonably small fraction of
# the maximum command of 0.5 ms. Remember to bound it on both
# positive and negative sides. 
