# Put the various CircuitPython import statements here
import board
import busio
import adafruit_bno055
import time
import pwmio
import digitalio
import pulseio
import pulse_commands
import adafruit_bmp3xx
import measure_servofreq_adafruit_pca9685
import adafruit_pca9685


Altitude_hold = False
Altitude_cmd = 0 #in m

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
pulse_read_Thrust= pulseio.PulseIn(board.D12,maxlen=8)

out_cmd_rotation=0 #in degrees
out_cmd_rotation_duty_cycle= 3932  #3932=1.5ms @40Hz
read_cmd_rotation=0
out_cmd_alt=0
out_cmd_servo=0
H_error=0
Current_Alt=0
Previous_Alt=0

#i2c, bmp, BNO055
i2c = busio.I2C(board.SCL, board.SDA)
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
servo_breakout = adafruit_pca9685.PCA9685(i2c)
servo_breakout.frequency= 40
sensor = adafruit_bno055.BNO055_I2C(i2c)
led= digitalio.DigitalInOut(board.D13)
led.direction=digitalio.Direction.OUTPUT
led.value = False

#measurement
servo_breakout.channels[15].duty_cycle=int(1.5e-3*40*65536)
PWMfreq = measure_servofreq_adafruit_pca9685.measure_servofreq(servo_breakout,15,board.D26)

#Fans #could left and right pigtailed
Right_fan=servo_breakout.channels[1] #lift and thrust
Left_fan=servo_breakout.channels[2] #lift and thrust
Back_fan=servo_breakout.channels[3]   #Heading fan
Right_fan.duty_cycle = int(1e-3*65536*PWMfreq) #initialize at 1ms
Left_fan.duty_cycle = int(1e-3*65536*PWMfreq) #initialize at 1ms
Back_fan.duty_cycle = int(1.5e-3*65536*PWMfreq) #initialize at 1.5ms
#servos, need to be mirrored
Left_servo_control=servo_breakout.channels[4] #convert some lift in thrust 
Right_servo_control=servo_breakout.channels[5] 
Left_servo_control.duty_cycle= int(1e-3*65536*PWMfreq)

i=0
# for i<100:
#     Alt=bmp.altitude
#     i=i+1

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
    #print(f'dt is {dt}')
    #print(f't_1 is {t_1}')

    Heading_cmd_read=0
    Lift_cmd_read=0
    Thrust_cmd_read=0


    # Wait until pulses have been received 
    while (len(pulse_read_heading)) == 0 :
        time.sleep(0.1)
        print("no heading yet")
   
    while (len(pulse_read_lift)) == 0 :
        time.sleep(0.1)
        print("no lift yet")

    #Assign a value to be used from read signals
    (Lift_cmd_read, Heading_cmd_read, Thrust_cmd_read) = pulse_commands.get_pulse_commands ([pulse_read_lift,pulse_read_heading, pulse_read_Thrust])

     
        
   
    # Check whether the BNO055 is calibrated
    # and turn on the LED on D13 as appopriate
    if sensor.calibrated == True:
        led.value =True
        ##print("led is on")
    # Extract the euler angles (Heading, Roll, Pitch)
    # in degrees from the IMU
    [Heading, Roll, Pitch]= sensor.euler
    Current_Alt= 0     ###Need to update with bmp

    # Determine the commanded orientation based on from your pulse input from
    # pin D5
    read_cmd_rotation= (Heading_cmd_read-1500)*0.36 #will give an angle in degrees (-180 to 180)
    read_cmd_Lift= (Lift_cmd_read-1000) #give you us signal
    read_cmd_Thrust= (Thrust_cmd_read-1500) ##map to servo cmd? if there is demand fro thrust move servo 15deggrees forward?
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
    #H_Error= ((read_cmd_rotation - Heading + 360 + 180) % 360  - 180)
    H_error= (read_cmd_rotation-sensor.gyro[2]*180/3.14159)
    #print (f'the error is {H_Error}degrees')
    #print (f'the Alt error is {Alt_error}')
    #Kp_term= H_Error*Kp_coefficient
    #Kd_term= sensor.gyro[2]*Kd_coefficient
    #Ki_term= Ki_coefficient*H_Error*dt + Ki_term
    # if Ki_term >100:
    #     Ki_term = 100
    # if Ki_term < -100:
    #     Ki_term = -100
    # out_cmd_rotation=Kp_term+Kd_term+Ki_term
    # out_cmd_rotation_us=out_cmd_rotation/0.36
    
    out_cmd_rotation_us= H_error/0.36
    

    if Altitude_hold == True:
        Alt_error= Altitude_cmd-Current_Alt

        Kp_Alt_term= Alt_error*Kp_Alt_coefficient
        Kd__Alt_term= Kd_Alt_coefficient* (Current_Alt-Previous_Alt)/dt
        Ki_Alt_term= Ki_Alt_coefficient*Alt_error*dt + Ki_Alt_term

        if Ki_Alt_term >100:
            Ki_Alt_term = 100
        if Ki_Alt_term < -100:
            Ki_Alt_term = -100
        
        out_cmd_alt=Kp_Alt_term+Kd__Alt_term+Ki_Alt_term
        out_cmd_alt_us=out_cmd_alt
    else:
        out_cmd_alt= read_cmd_Lift
        out_cmd_alt_us=out_cmd_alt*100

    out_cmd_servo= read_cmd_Thrust*0.09 # 0.09 deg = 1us
    # #print (f'sensor gyro is {sensor.gyro[2]}')
    # #print (f'the Kp term is {Kp_term}')
    # #print (f'the Kd term is {Kd_term}')
    # #print (f'the Ki term is {Ki_term}')

    
    #print(f'alt {Current_Alt}')

    # Bound the output rotation command so that it cannot exceed 0.5 or be less than
    # -0.5 (note this is 500)
    if out_cmd_rotation_us >500:
        out_cmd_rotation_us =500
    if out_cmd_rotation_us < -500:
        out_cmd_rotation_us = -500

    if out_cmd_alt_us >1000:
        out_cmd_alt_us =1000
    if out_cmd_alt_us < 0:
        out_cmd_alt_us = 0
        
        #servo moves 90deg over 1ms. 45deg in each direction? #limit at +1.55
    if out_cmd_servo > 250:
        out_cmd_servo= 250
    if out_cmd_servo < -250:
        out_cmd_servo= -250

    print(f'out cmd alt {out_cmd_alt_us}')
        ##print(out_cmd_rotation_ms)
        # Apply the output rotation command in opposite senses to determine the duty
        # cycle for the PWM outputs to the left- and right- side fans (pins D7, D8)
    Right_fan.duty_cycle= int((1e-3 + out_cmd_alt_us/1000)*65536*PWMfreq) #3932=1.5ms @40Hz
    Left_fan.duty_cycle= int((1e-3 + out_cmd_alt_us/1000)*65536*PWMfreq) #3932=1.5ms @40Hz
    Back_fan.duty_cycle= int((1.5e-3 + out_cmd_rotation_us/1000)*65536*PWMfreq) #2621=1ms @40Hz
    Left_servo_control.duty_cycle= int((1.5e-3 + out_cmd_servo/1000) *65536*PWMfreq)
    Right_servo_control.duty_cycle= int((1.5e-3 - out_cmd_servo/1000) *65536*PWMfreq)
        
        
    Previous_Alt=Current_Alt
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
