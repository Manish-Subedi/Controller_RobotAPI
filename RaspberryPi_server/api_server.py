'''
Author     Manish Subedi
Date       2024-02-15

    The server is based on FastAPI framework 
    Handles the request from the automation tool and controls the robot
    The server is designed to be scalable and efficient
    The endpoints are secured with OAuth2 
    
'''
import time
from typing import Annotated
from pydantic import BaseModel
from fastapi import Depends, FastAPI, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from jose import JWTError, jwt #sudo apt install python3-jose
from datetime import datetime, timedelta, timezone
import threading
import RPi.GPIO as GPIO
from log_config import log

from ultrasonic import UltraSonic
from stm32_serial import serial_comm

# messages from different modules can be logged to separate files
# example: api_server can log to api_server_logs, and so on
filepath = 'log.txt'
log = log(filepath)

app = FastAPI()
serial = serial_comm()

TRIG_PIN = 23
ECHO_PIN = 24
ultrasonic = UltraSonic(TRIG_PIN, ECHO_PIN) #(trigpin, echopin)

if not serial.open_serial_port():
    print("Connection stm32 missing")
    exit()

oauth2_scheme = OAuth2PasswordBearer(tokenUrl = "http://127.0.0.1:8000/token")

#Secret key to encode and decode JWT tokens
SECRET_KEY = "skey"
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRY_MINUTES = 120

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    username: str
    password: str

def create_access_token(data: dict, expires_delta: timedelta | None = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(minutes=100)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

# variable declarations for threads handling and making decision based on ultrasonic and ir sensor readings. 
ultrasonic_thread = None

BUZZER_PIN = 16
GPIO.setup(BUZZER_PIN, GPIO.OUT)

#thread lock synchronisation mechanism
lock = threading.Lock()

# STOP signal will be sent if distance is less than MIN_DISTANCE
MIN_DISTANCE = 50
# read the last sent signal to the robot
signal = None
Start_signal = b'A'
Stop_signal = b'Z'
success = None
stop_flag = None

# call-back function for the daemon thread
def obstacle_check():
    while True:
        global success, signal, Start_signal, Stop_signal, stop_flag
        lock.acquire()
        distance = round(ultrasonic.calculate_distance(), 2)

        if distance is not None and distance <= MIN_DISTANCE and signal != Stop_signal:
            signal = Stop_signal
            log.logger.info("thread(obstacle_check): Robot Stopped, objects in collision range")
            success = serial.serial_send(signal)
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
        elif distance is not None and distance > MIN_DISTANCE and signal != Start_signal and stop_flag != True:
            signal = Start_signal
            log.logger.info("thread(obstacle_check): Robot back in motion")
            success = serial.serial_send(signal)
            GPIO.output(BUZZER_PIN, GPIO.LOW)
        lock.release()

        if stop_flag:
            return
          
        time.sleep(1)


@app.post("/token", response_model=Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    if form_data.username == "user" and form_data.password == "pass":
        access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRY_MINUTES)
        access_token = create_access_token (
            data = { "sub": form_data.username }, expires_delta = access_token_expires
        )
        return {"access_token": access_token, "token_type": "bearer"}
    else:
        raise HTTPException(
            status_code = status.HTTP_401_UNAUTHORIZED,
            detail = "Incorrect username or password",
            headers = {"WWW-Authenticate": "Bearer"}
        )
    
@app.get("/testnode")
async def read_times(token: str = Depends(oauth2_scheme)):
    return {"token_authentication successful, token": token}

@app.get("/home")
async def root():
    proximity = round(ultrasonic.calculate_distance(), 2)
    return {"Welcome to Robot Control Centre!": {
                "Proximity is (cm)" : proximity
                }
            }

# start the robot S : Start
@app.get("/start")
async def start_robot():
    global ultrasonic_thread, success, signal, Start_signal, stop_flag    
    if ultrasonic_thread is None or not ultrasonic_thread.is_alive() and signal != Start_signal:
        stop_flag = False
        signal = Start_signal
        success = serial.serial_send(signal)
        #daemon_thread = threading.Thread(daemon=True, target=obstacle_check)
        ultrasonic_thread = threading.Thread(target=obstacle_check)
        ultrasonic_thread.daemon = True
        ultrasonic_thread.start()
    if success:
        return{"start_sequence": "Successful"}
    else:
        return{"start_sequence": "Failed"}
        
# stop the robot Z
@app.get("/stop")
async def stop_robot():
    global ultrasonic_thread,success, signal, Stop_signal, stop_flag, number_of_loops
    lock.acquire()
    signal = Stop_signal
    stop_flag = True
    number_of_loops = 0
    lock.release()
    success = serial.serial_send(signal)

    if ultrasonic_thread and ultrasonic_thread.is_alive():
        ultrasonic_thread.join()  # Wait for the daemon thread to finish
    if success:
        return{"message": "Data transmission successful, Robot's stop sequence initialised"}           
    else:
        return{"message": "Failed to transmit data"}

#close serial port when server shuts down and GPIO cleanup 
@app.on_event("shutdown")
def shutdown_event():
    serial.close_serial_port()
    ultrasonic.cleanup()
    log.logger.info("Serial port closed and GPIO pins configured for HCSR04 cleaned up")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("api_server:app", host='0.0.0.0', port=8000, reload=True)
