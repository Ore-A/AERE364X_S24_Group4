# Put the various CircuitPython import statements here
import board
import busio
import adafruit_bno055
import time
import pwmio
import digitalio
import pulseio
import adafruit_pca9685
import measure_servofreq_adafruit_pca9685
import pulse_commands


pulse_read_heading = pulseio.PulseIn(board.D5,maxlen=8)
pulse_read_Thrust= pulseio.PulseIn(board.D12,maxlen=8)

read_cmd_rotation=0    # rotation rate command in deg/s (-180 to 180) after initialization offset
H_Error=0              # error between commanded rotation rate and actual rotation rate

out_cmd_rotation_ms=0  # output signal for rotation in us (not ms)   
out_cmd_servo=0        # output signal to servos in us 





#BNO055
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
servo_breakout = adafruit_pca9685.PCA9685(i2c)
servo_breakout.frequency = int(40)

#calibrate actual frequency of breakout
TIMER = servo_breakout.channels[15]
TIMER.duty_cycle=int(3932)
PWMfreq = measure_servofreq_adafruit_pca9685.measure_servofreq(servo_breakout,15,board.D26)

print(f'measur {PWMfreq}')

#Initialize fan and servos to central position
BACK_OUT = servo_breakout.channels[11]
BACK_OUT.duty_cycle=int(1.5e-3* 65536*PWMfreq)
Left_servo_control=servo_breakout.channels[4] 
Right_servo_control=servo_breakout.channels[6] 
Left_servo_control.duty_cycle=int(1.41e-3* 65536*PWMfreq)
Right_servo_control.duty_cycle=int(1.45e-3* 65536*PWMfreq)

led= digitalio.DigitalInOut(board.D13)
led.direction=digitalio.Direction.OUTPUT
led.value = False

#wait a few seconds for initialization
time.sleep(2)



# Start an infinite loop here:
while True:
   
    
    Heading_cmd_read=0
    thrust_cmd_read=0
   

    
    (thrust_cmd_read, Heading_cmd_read) = pulse_commands.get_pulse_commands ([pulse_read_Thrust,pulse_read_heading])

    #print (Heading_cmd_read)
   
    # Check whether the BNO055 is calibrated
    # and turn on the LED on D13 as appopriate
    if sensor.calibrated == True:
        led.value =True
        
    # Extract the euler angles (Heading, Roll, Pitch)
    # in degrees from the IMU
    [Heading, Roll, Pitch]= sensor.euler
    
    #find actual command from signal sent us signal
    read_cmd_rotation= (Heading_cmd_read-1500)*0.36  
    read_cmd_Thrust= (thrust_cmd_read-1500) 
      
   
    #Error between commanded rate and meassured rate in degrees/sec
    H_Error= read_cmd_rotation-sensor.gyro[2]*180/3.14159
    print (f'the error is {H_Error}degrees')
   

    #convert back to a signal in us
    out_cmd_rotation_ms= H_Error/0.36
    out_cmd_servo=read_cmd_Thrust

    
    
    #bound the outpouts
    if out_cmd_rotation_ms >225:
        out_cmd_rotation_ms =225
    if out_cmd_rotation_ms < -225:
        out_cmd_rotation_ms = -225
    
    if out_cmd_servo > 300:
        out_cmd_servo= 300
    if out_cmd_servo < -300:
        out_cmd_servo= -300

    #print(f'out cmd serv {out_cmd_servo}')
   
    
    BACK_OUT.duty_cycle=int((1.5e-3+ out_cmd_rotation_ms/1000000 )* 65536*PWMfreq)
    Left_servo_control.duty_cycle=int((1.41e-3 + out_cmd_servo/1000000)* 65536*PWMfreq)
    Right_servo_control.duty_cycle=int((1.45e-3 - out_cmd_servo/1000000)* 65536*PWMfreq)
    #print (f'back fan is {BACK_OUT.duty_cycle}')
    
    
    
    pass # Done with loop

