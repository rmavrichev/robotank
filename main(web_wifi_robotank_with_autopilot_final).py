import gc
import uasyncio as asyncio
from network import WLAN, STA_IF
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
debug = False
auto_run_flag = False
avoid_left = False
avoid_right = False
avoid_backward = False
# initial pos and dist
pos_actual = 75
dist_cm = 50
# home wifi credentials
essid = 'home_wifi'
password = '12345678'

# callback function for start/stop button
def callback(p):
    global auto_run_flag
    auto_run_flag = not(auto_run_flag)
    print('set auto_run_flag', auto_run_flag, p)
    #stop all motors first
    stop_all_sync()

# sync fuctions
def do_connect():
    sta_if = WLAN(STA_IF)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect(essid, password)
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ifconfig())

def stop_all_sync():
    revrs_L.value(0)
    motor_L.duty(0)
    revrs_R.value(0)
    motor_R.duty(0)

def backward_sync():
    revrs_L.value(1)
    motor_L.duty(speed)
    revrs_R.value(1)
    motor_R.duty(speed)

def forward_sync():
    revrs_L.value(0)
    motor_L.duty(speed)
    revrs_R.value(0)
    motor_R.duty(speed)

def right_rotate_sync():
    revrs_L.value(0)
    motor_L.duty(speed)
    revrs_R.value(1)
    motor_R.duty(speed)

def left_rotate_sync():
    revrs_L.value(1)
    motor_L.duty(speed)
    revrs_R.value(0)
    motor_R.duty(speed)
    
def right_turn_sync():
    revrs_L.value(0)
    motor_L.duty(speed)
    revrs_R.value(0)
    motor_R.duty(0)

def left_turn_sync():
    revrs_L.value(0)
    motor_L.duty(0)
    revrs_R.value(0)
    motor_R.duty(speed)

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
        if auto_run_flag:
            # moving functions
            if avoid_backward :
                print('avoid_backward = %s' % avoid_backward)
                await backward(interval_ms+getrandbits(3)*100)
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
        elif not auto_run_flag:
            await asyncio.sleep(0) # do nothing


async def blink_led(led, interval_ms):
    led_val = True
    while True:
        if auto_run_flag:
            led_val = not(led_val)
            led_state = led.value(int(led_val))
            await asyncio.sleep_ms(interval_ms)
        elif not auto_run_flag:
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
        if auto_run_flag:
            # make decision what to do
            if pos_actual == 45 and dist_cm < avoid_limit_cm :
                avoid_left = True
                if debug : print('avoid_left = %s' % avoid_left)
            elif pos_actual == 45 and dist_cm >= avoid_limit_cm :
                avoid_left = False
                if debug : print('avoid_left = %s' % avoid_left)
            elif pos_actual == 75 and dist_cm < avoid_limit_cm*1.5 :
                avoid_backward = True
                if debug : print('avoid_backward = %s' % avoid_backward)
            elif pos_actual == 75 and dist_cm >= avoid_limit_cm*1.5 :
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
        elif not auto_run_flag:
            await asyncio.sleep(0) # do nothing

async def radar_scan(interval_ms):
    #pos_list = [75] # for debugging , keep servo in pos [75].
    pos_list = [45,75,105,75]
    global pos_actual
    global dist_cm
    while True:
        if auto_run_flag:
            for pos in pos_list:
                servo.duty(pos)
                await asyncio.sleep_ms(interval_ms)
                dist_cm = await async_measure_range()
                pos_actual = pos
        elif not auto_run_flag:
            await asyncio.sleep(0) # do nothing

async def web_page(request):
    global auto_run_flag
    motor_state="Stopped"
    if request.find('GET /?forward') > 0:
        motor_state="Going Forward"
        auto_run_flag = False
        forward_sync()
    elif request.find('GET /?left_rotate') > 0:
        motor_state="Rotate Left"
        auto_run_flag = False
        left_rotate_sync()
    elif request.find('GET /?right_rotate') > 0:
        motor_state="Rotate Right"
        auto_run_flag = False
        right_rotate_sync()
    elif request.find('GET /?left_turn') > 0:
        motor_state="Turn Left"
        auto_run_flag = False
        left_turn_sync()
    elif request.find('GET /?right_turn') > 0:
        motor_state="Turn Right"
        auto_run_flag = False
        right_turn_sync()
    elif request.find('GET /?backward') > 0:
        motor_state="Going Backward"
        auto_run_flag = False
        backward_sync()
    elif request.find('GET /?stop') > 0:
        motor_state="Stopped"
        auto_run_flag = False
        stop_all_sync()
    elif request.find('GET /?auto') > 0:
        auto_run_flag = not auto_run_flag
        if  auto_run_flag :
            motor_state="Autopilot"
        elif not auto_run_flag :
             motor_state="Stopped"
             stop_all_sync()

    html = """<html><head><title>RoboTank WEB</title> 
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,"> <style>
    html{font-family: Helvetica; display:inline-block; margin: 0px auto; text-align: center;}
    h1{color: #0F3376; padding: 2vh;}p{font-size: 1.5rem;}
    .button{display: inline-block; background-color: #33c080; border: none; 
    border-radius: 4px; color: white; text-decoration: none; font-size: 30px; width:100%}
    .button2{background-color: #4286f4; width:30%}
    .button3{background-color: #eb2b10; width:35%}
    .button4{background-color: #8386f4; width:44%}
    </style></head>
    <body> <h1>RoboTank WEB</h1> 
    <p>Status : <strong>""" + motor_state + """</strong></p>
    <p><a href='/?forward'><button class="button">Forward</button></a></p>
    <p><a href='/?left_turn'><button class="button button2">LEFT</button></a>
    <a href='/?stop'><button class="button button3">STOP</button></a>
    <a href='/?right_turn'><button class="button button2">RIGHT</button></a>
    <p><a href='/?backward'><button class="button">Backward</button></a></p>
    <p><a href='/?left_rotate'><button class="button button4">L-rotate</button></a>
    <a href='/?right_rotate'><button class="button button4">R-rotate</button></a></p>
    <p><a href='/?auto'><button class="button button3">AUTO</button></a></p>
    </body></html>"""
    return html

async def web_handler(reader, writer):
    try:
        request = str(await reader.read(1024))
        #print('request = %s' % request)
        header = """HTTP/1.1 200 OK\nContent-Type: text/html\nConnection: close\n\n"""
        response = await web_page(request)
        await writer.awrite(header)
        await writer.awrite(response)
        await writer.aclose()
        print("Finished processing request")
    except Exception as e:
        print(e)
    
    gc.collect()
    #print('mem_alloc = %s, mem_free = %s' % (gc.mem_alloc(), gc.mem_free()))
    
async def tcp_server(host, port):
    server = await asyncio.start_server(web_handler, host, port)

#stop all motors first
stop_all_sync()

# connect to wi-fi network
do_connect()

# move servo to initial position
print('Move sensor to initial position...')
servo.duty(75)
sleep(1) #wait 1s for servo reaching initial position
print('Waiting for start button...')


# create callback fo button:
button.irq(trigger=Pin.IRQ_FALLING, handler=callback)

# define loop
loop = asyncio.get_event_loop()

#create looped tasks
loop.create_task(blink_led(syst_led, interval_ms=250))
loop.create_task(radar_scan(interval_ms=250))
loop.create_task(make_decision(interval_ms=250, avoid_limit_cm=15))
loop.create_task(moving(interval_ms=1000))
loop.create_task(tcp_server('0.0.0.0', 80))
#loop.call_soon(asyncio.start_server(web_handler, '0.0.0.0', 80))

# loop run forever
loop.run_forever()
loop.close()
