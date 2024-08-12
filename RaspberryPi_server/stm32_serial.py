#!/usr/bin/env python3

import serial
from log_config import log
from logging.handlers import RotatingFileHandler
import time

# messages from different modules can be logged to separate files
# example: stm32_serial can log to stm32_serial_logs, and so on
filepath = 'log.txt'
log = log(filepath)

class serial_comm:
    def __init__(self, serial_port='/dev/ttyS0', baudrate=9600):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ser = None
 
    def open_serial_port(self):
        try:
            # Open serial port
            self.ser = serial.Serial(self.serial_port, self.baudrate, 
                                     parity = serial.PARITY_NONE,
									 stopbits = serial.STOPBITS_ONE,
									 bytesize = serial.EIGHTBITS,
									 timeout=1, write_timeout=2,
									 xonxoff=False,
                                     rtscts=False,
                                     dsrdtr=False,
									 inter_byte_timeout=None,
                                     exclusive=False)
            log.logger.info("stm32_serial.py: Serial port opened at %s", self.serial_port)
            return True
        except serial.SerialException as e:
            log.logger.error("stm32_serial.py: Error: %s", e)

    def close_serial_port(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            log.logger.info("stm32_serial.py: Serial port closed")

    def serial_send(self, data):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
                log.logger.info("stm32_serial.py: Sent: %s", data.decode('utf-8'))
                return True 
            except Exception as e:
                log.logger.error("stm32_serial.py: An error occured:", e)
                return False

    def serial_receive(self):
        if self.ser and self.ser.is_open:
            try:
                # Read data
                if self.ser.in_waiting > 0:
                    received_data = self.ser.readline()
                    log.logger.info("stm32_serial.py: Received: %s", received_data.decode('utf-8'))
                    #if received_data == 'Q':
                    return received_data     
            except Exception as e:
                log.logger.info("stm32_serial.py: Error receiving data:", e)
                return None
'''
    def setup_logging(self):
        #configure logging to write messages to a rotating log file
        logging.basicConfig(level=logging.INFO)
        handler = RotatingFileHandler('iRobot_log.txt', maxBytes=100000, backupCount=3)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger = logging.getLogger()
        self.logger.addHandler(handler)
        
    def open_serial_port(self):
        try:
            # Open serial port
            self.ser = serial.Serial(self.serial_port, self.baudrate, 
                                     parity = serial.PARITY_NONE,
									 stopbits = serial.STOPBITS_ONE,
									 bytesize = serial.EIGHTBITS,
									 timeout=1, write_timeout=2,
									 xonxoff=False,
                                     rtscts=False,
                                     dsrdtr=False,
									 inter_byte_timeout=None,
                                     exclusive=False)
            self.logger.info("stm32_serial.py: Serial port opened at %s", self.serial_port)
            return True
        except serial.SerialException as e:
            self.logger.error("stm32_serial.py: Error: %s", e)

    def close_serial_port(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.logger.info("stm32_serial.py: Serial port closed")

    def serial_send(self, data):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
                self.logger.info("stm32_serial.py: Sent: %s", data.decode('utf-8'))
                return True 
            except Exception as e:
                self.logger.error("stm32_serial.py: An error occured:", e)
                return False

    def serial_receive(self):
        if self.ser and self.ser.is_open:
            try:
                # Read data
                if self.ser.in_waiting > 0:
                    received_data = self.ser.readline()
                    self.logger.info("stm32_serial.py: Received: %s", received_data.decode('utf-8'))
                    #if received_data == 'Q':

                    return received_data     
            except Exception as e:
                print("stm32_serial.py: Error receiving data:", e)
                return None
'''