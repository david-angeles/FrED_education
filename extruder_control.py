######### Program to control the FrED's DC extruder ########

import time
import math
import RPi.GPIO as GPIO
import busio
import board
import digitalio
import matplotlib.pyplot as plt
import adafruit_mcp3xxx.mcp3008 as MCP
GPIO.setmode(GPIO.BCM)
from adafruit_mcp3xxx.analog_in import AnalogIn

import FrED_functions

########## GPIO Pin Definitions ##########
HEATER_PIN = 6
DIRECTION_PIN = 16
STEP_PIN = 12
MICROSTEP_PIN_A = 17
MICROSTEP_PIN_B = 27
MICROSTEP_PIN_C = 22

########## Initialise GPIO ##########

#GPIO.setup(HEATER_PIN, GPIO.OUT)
GPIO.setup(DIRECTION_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(MICROSTEP_PIN_A, GPIO.OUT)
GPIO.setup(MICROSTEP_PIN_B, GPIO.OUT)
GPIO.setup(MICROSTEP_PIN_C, GPIO.OUT)

########## Initialise Thermistor ##########

spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
# Create the cs (chip select)
cs = digitalio.DigitalInOut(board.D8)
# Create the mcp object
mcp = MCP.MCP3008(spi, cs)
# Create analog inputs connected to the input pins on the MCP3008
channel_0 = AnalogIn(mcp, MCP.P0)


########## Initialise Heater ##########
GPIO.setup(HEATER_PIN, GPIO.OUT)
heater = GPIO.PWM(HEATER_PIN, 5)  # Hz frequency
heater.start(0)
            
# Update PWM duty cycle
#heater.ChangeDutyCycle(pwm_value)

motor_direction = 0
GPIO.output(DIRECTION_PIN, motor_direction)
#channel_0 = None

#DEFAULT_DIAMETER = 0.35
#MINIMUM_DIAMETER = 0.3
#MAXIMUM_DIAMETER = 0.6
STEPS_PER_REVOLUTION = 200
RESOLUTION = {'1': (0, 0, 0),
              '1/2': (1, 0, 0),
              '1/4': (0, 1, 0),
              '1/8': (1, 1, 0),
             '1/16': (0, 0, 1),
             '1/32': (1, 0, 1)}
FACTOR = {'1': 1,
               '1/2': 2,
               '1/4': 4,
               '1/8': 8,
               '1/16': 16,
               '1/32': 32}
DEFAULT_MICROSTEPPING = '1/4'
#DEFAULT_RPM = 0.6 #  Delay is not being used, will be removed temporarily
SAMPLE_TIME = 0.1
MAX_OUTPUT = 1
MIN_OUTPUT = 0

########### Initialing lists ##########
time_data = []
temperature_raw_data = []
temperature_data = []
temp_reference_data = []
heater_pwm_data = []
stepper_rpm_data = []
stepper_rpm_raw_data = []
stepper_rpm_ref_data = []
stepper_input_data = []
#PWM_motor_data = []
#motor_voltage_data = []

muestra = 1

tm = 0.02           # Sample time
match_time = 0.020

plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot(time_data, temperature_raw_data, label='Temperature (°C)')
line2, = ax.plot(time_data, temperature_data, label='Filtered Temperature (°C)')
ax.legend()
ax.set_title('Extruder subsystem')


def ploting ():
    # Update the plot
    line1.set_xdata(time_data)
    line1.set_ydata(temp_reference_data)
    line2.set_xdata(time_data)
    line2.set_ydata(temperature_data)
    #line2.set_ydata(motor_input_data)
    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.01)

def plotTemp ():
    plt.plot(time_data, temperature_raw_data)
    plt.show()

########### Variables ##########
previous_temp = 20
temp_reference = 90

previous_PIDerror = 0
error_sum = 0
previous_PIDtime = 0

########## starting to count time ##########
tstart = time.perf_counter()   #start internal clock
initial_time = time.time()

#GPIO.output(HEATER_PIN, 1)

try:
    #Loop Execution
    while True:

        current_time = time.perf_counter() - tstart 

        raw_temperature = FrED_functions.get_temperature(channel_0.voltage)
        temperature = FrED_functions.temp_filter (raw_temperature, previous_temp)
        previous_temp = temperature
       
        PWM_heater = 1

        delay = 1 #(60 / setpoint_rpm / Extruder.STEPS_PER_REVOLUTION /
                #Extruder.FACTOR[Extruder.DEFAULT_MICROSTEPPING])
        #GPIO.output(DIRECTION_PIN, 1)
        #GPIO.output(STEP_PIN, GPIO.HIGH)
        #time.sleep(delay)GPIO.setmode(GPIO.BCM)
        #GPIO.output(STEP_PIN, GPIO.LOW)
        #time.sleep(delay)




        PWM_heater, error_i, PIDerror = FrED_functions.extruder_PID (temp_reference, temperature, previous_PIDerror, 
                                          error_sum, current_time, previous_PIDtime)
        #PWM_heater, error_i = FrED_functions.extruder_PI (temp_reference, temp, error_sum, 
        #                                           current_time, previous_PIDtime)
        #PWM_heater, error_i = FrED_functions.STSM (rpm_reference, rpm, u, error_sum, 
        #                                           current_time, previous_PIDtime)
        previous_PIDtime = current_time
        previous_PIDerror = PIDerror
        error_sum = error_i

        if PWM_heater > 100: #Extruder.MAX_OUTPUT:   #temporaly limited the output
            PWM_heater = 100 #Extruder.MAX_OUTPUT
        elif PWM_heater < 0:
            PWM_heater = 0

        heater.ChangeDutyCycle(PWM_heater)


        ploting ()

        if current_time>=muestra:
            print(f"{current_time}\t{raw_temperature}\t{PWM_heater}")
            muestra = muestra + 1


    

        ########### save data to the lists #########
        time_data.append(round(current_time, 2))
        stepper_rpm_data.append(round(current_time, 2))
        
        temperature_raw_data.append(round(raw_temperature,2))
        temperature_data.append(round(temperature,2))
        temp_reference_data.append(round(temp_reference,2))
        heater_pwm_data.append(round(PWM_heater,2))


        ########## to have a consistent sample time ##########
        wait = max(0, match_time - current_time)
        time.sleep(wait)
        match_time = match_time + tm

except KeyboardInterrupt:
    print ("\nCode Stopped\n")
    ########## save data in a txt file ##########
    #plotTemp ()
    FrED_functions.save_data_temp(time_data, stepper_rpm_data, temp_reference_data,
                              temperature_raw_data, temperature_data, heater_pwm_data)
    print ("Data saved in FrED_data_temp.txt file\n\n")
    #plotTemp ()
    
finally:
    GPIO.cleanup()
