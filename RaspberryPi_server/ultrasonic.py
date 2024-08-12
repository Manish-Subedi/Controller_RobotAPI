#!/usr/bin/python
import RPi.GPIO as GPIO
import time
from log_config import log

# messages from different modules can be logged to separate files
# example: stm32_serial can log to stm32_serial_logs, and so on
filepath = 'log.txt'
log = log(filepath)

class UltraSonic:
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trig_pin, GPIO.LOW)
        time.sleep(1)
        
    def cleanup(self):
        GPIO.cleanup()  

    def calculate_distance(self) -> int:
        GPIO.output(self.trig_pin, GPIO.LOW)        
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, GPIO.LOW)
        start_time = time.time()       
        #wait for echo signal to start
        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start - start_time > 1: #timeout if no response
                log.logger.error("ultrasonic.py: pulse timed out, no waves received")      
        #wait for echo signal to end
        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
        #calculate pulse duration 
        pulse_duration = pulse_end - pulse_start
        #speed of sound is 343m/s, distance = (time * speed)/2
        distance = (pulse_duration * 34300) / 2        
        return distance	
