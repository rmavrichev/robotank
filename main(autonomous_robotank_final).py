import gc
import uasyncio as asyncio
from utime import sleep, sleep_us
from machine import Pin, PWM, time_pulse_us
from random import getrandbits

# nodemcu pins from the motor shield
p5 = Pin(5, Pin.OUT)  # connected to GPIO4(D1)
p4 = Pin(4, Pin.OUT)  # connected to GPIO4(D2)
revrs_L = Pin(0, Pin.OUT, value=0)  # connected to GPIO0(D3)
revrs_R = Pin(2, Pin.OUT, value=0)  # connected to GPIO2(D4) , also connected to onboard wifi LED
motor_L = PWM(p5, freq=1000, duty=0)
motor_R = PWM(p4, freq=1000, duty=0)
speed = 1023  #TODO: variable speed

# servo SG90 connected to GPIO14(D5)
p14 =  Pin(14, Pin.OUT)
servo = PWM(p14, freq=50)
# on/off button
button =  Pin(15, Pin.IN, Pin.PULL_UP) # connected to D8 (GPIO15)
# onboard LED is connected to D0(GPIO16)
syst_led =  Pin(16, Pin.OUT)
# HC-SR04 ultrasonic sensor connected to GPIO12(D6)-trigger and GPIO13(D7)-echo
trig=Pin(12, Pin.OUT)
echo=Pin(13, Pin.IN)

#global flags and variables
run_flag = False
avoid_left = False
avoid_right = False
avoid_backward = False
pos_actual = 75
dist_cm = 50
debug = False


# callback function for start/stop button
def callback(p):
    global run_flag
    run_flag = not(run_flag)
    print('set run_flag', run_flag, p)

# sync fuctions
def stop_all_sync():
    revrs_L.value(0)
    motor_L.duty(0)
    revrs_R.value(0)
    motor_R.duty(0)
    
# async fuctions
async def stop_all():
    revrs_L.value(0)
    motor_L.duty(0)
    revrs_R.value(0)
    motor_R.duty(0)

async def forward(interval_ms):
    revrs_L.value(0)
    motor_L.duty(speed)
    revrs_R.value(0)
    motor_R.duty(speed)
    await asyncio.sleep_ms(interval_ms)

async def backward(interval_ms):
    revrs_L.value(1)
    motor_L.duty(speed)
    revrs_R.value(1)
    motor_R.duty(speed)
    await asyncio.sleep_ms(interval_ms)

async def right_rotate(interval_ms):
    revrs_L.value(0)
    motor_L.duty(speed)
    revrs_R.value(1)
    motor_R.duty(speed)
    await asyncio.sleep_ms(interval_ms)

async def left_rotate(interval_ms):
    revrs_L.value(1)
    motor_L.duty(speed)
    revrs_R.value(0)
    motor_R.duty(speed)
    await asyncio.sleep_ms(interval_ms)

async def right_turn(interval_ms):
    revrs_L.value(0)
    motor_L.duty(speed)
    revrs_R.value(0)
    motor_R.duty(0)
    await asyncio.sleep_ms(interval_ms)

async def left_turn(interval_ms):
    revrs_L.value(0)
    motor_L.duty(0)
    revrs_R.value(0)
    motor_R.duty(speed)
    await asyncio.sleep_ms(interval_ms)

async def moving(interval_ms):
    while True:
        if run_flag:
            # moving functions
            if avoid_backward :
                print('avoid_backward = %s' % avoid_backward)
                await backward(interval_ms*2)
                if bool(getrandbits(1)) :
                    await right_rotate(interval_ms+getrandbits(3)*100)
                    await stop_all()
                else:
                    await left_rotate(interval_ms+getrandbits(3)*100)
                    await stop_all()
            elif avoid_left :
                print('avoid_left = %s' % avoid_left)
                await left_turn(interval_ms)
            elif avoid_right :
                print('avoid_right = %s' % avoid_right)
                await right_turn(interval_ms)
            else:
                print('move_forward')
                await forward(interval_ms)
                
            await asyncio.sleep_ms(interval_ms)
        elif not run_flag:
            #stop all motors first
            await stop_all()
            await asyncio.sleep(0) # do nothing


async def blink_led(led, interval_ms):
    led_val = True
    while True:
        if run_flag:
            led_val = not(led_val)
            led_state = led.value(int(led_val))
            await asyncio.sleep_ms(interval_ms)
        elif not run_flag:
            await asyncio.sleep(0) # do nothing
            
async def async_measure_range():
    echo_timeout_us=500*2*30 # Timeout in microseconds to listen to echo pin.
    trig.off() # Stabilize the sensor
    sleep_us(5)
    trig.on()
    sleep_us(10) # Send a 10us pulse.
    trig.off()
    try:
        pulse_time = time_pulse_us(echo, 1, echo_timeout_us)
    except:
        pass
    dist = (pulse_time / 2) / 29.1
    return dist

async def make_decision(interval_ms, avoid_limit_cm):
    global avoid_left
    global avoid_right
    global avoid_backward
    while True:
        if run_flag:
            # make decision what to do
            if pos_actual == 45 and dist_cm < avoid_limit_cm :
                avoid_left = True
                if debug : print('avoid_left = %s' % avoid_left)
            elif pos_actual == 45 and dist_cm >= avoid_limit_cm :
                avoid_left = False
                if debug : print('avoid_left = %s' % avoid_left)
            elif pos_actual == 75 and dist_cm < avoid_limit_cm*1.25 :
                avoid_backward = True
                if debug : print('avoid_backward = %s' % avoid_backward)
            elif pos_actual == 75 and dist_cm >= avoid_limit_cm*1.25 :
                avoid_backward = False
                if debug : print('avoid_backward = %s' % avoid_backward)
            elif pos_actual == 105 and dist_cm < avoid_limit_cm :
                avoid_right = True
                if debug : print('avoid_right = %s' % avoid_right)
            elif pos_actual == 105 and dist_cm >= avoid_limit_cm :
                avoid_right = False
                if debug : print('avoid_right = %s' % avoid_right)
            # for debuging
            if debug : print('pos = %s, dist_cm = %s' % (pos_actual,dist_cm))  
            await asyncio.sleep_ms(interval_ms)
        elif not run_flag:
            await asyncio.sleep(0) # do nothing

async def radar_scan(interval_ms):
    pos_list = [45,75,105,75]
    global pos_actual
    global dist_cm
    while True:
        if run_flag:
            for pos in pos_list:
                servo.duty(pos)
                await asyncio.sleep_ms(interval_ms)
                dist_cm = await async_measure_range()
                pos_actual = pos
        elif not run_flag:
            await asyncio.sleep(0) # do nothing
    
#stop all motors first
stop_all_sync()

# move servo to initial position
print('Move sensor to initial position...')
servo.duty(75)
sleep(1) #wait 1s for servo reaching initial position
print('Waiting for start button...')

#enable gc
gc.enable()

# create callback fo button:
button.irq(trigger=Pin.IRQ_FALLING, handler=callback)

# define loop
loop = asyncio.get_event_loop()

#create looped tasks
loop.create_task(blink_led(syst_led, interval_ms=250))
loop.create_task(radar_scan(interval_ms=250))
loop.create_task(make_decision(interval_ms=250, avoid_limit_cm=15))
loop.create_task(moving(interval_ms=1000))

# loop run forever
loop.run_forever()
