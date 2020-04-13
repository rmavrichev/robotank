# RoboTank based on ESP8266 with motor shieldimport timeimport networkimport socketfrom machine import Pin, PWM, reset""" nodemcu pins from the motor shield """servo_1 = Pin(5, Pin.OUT)  # PWMA-GPIO5servo_2 = Pin(4, Pin.OUT)  # PWMB-GPIO4revrs_L = Pin(0, Pin.OUT, value=0)  # DA-GPIO0revrs_R = Pin(2, Pin.OUT, value=0)  # DB-GPIO2""" named after the L9110 h-bridge pins """motor_L = PWM(servo_1, freq=1000, duty=0)motor_R = PWM(servo_2, freq=1000, duty=0)""" TODO: variable speed """speed = 1023 '''# Wifi config # AP modeap = network.WLAN(network.AP_IF)ap.active(True)ap.config(essid='RoboTank')ap.config(authmode=3,password='12345678')'''""" function for connecting to your local WiFi network """ def do_connect():    essid = 'home_wifi'    password = '12345678'    sta_if = network.WLAN(network.STA_IF)    if not sta_if.isconnected():        print('connecting to network...')        sta_if.active(True)        sta_if.connect(essid, password)        while not sta_if.isconnected():            pass    print('network config:', sta_if.ifconfig())def stop_all():    revrs_L.value(0)    motor_L.duty(0)    revrs_R.value(0)    motor_R.duty(0)def backward():    revrs_L.value(1)    motor_L.duty(speed)    revrs_R.value(1)    motor_R.duty(speed)def forward():    revrs_L.value(0)    motor_L.duty(speed)    revrs_R.value(0)    motor_R.duty(speed)def right():    revrs_L.value(0)    motor_L.duty(speed)    revrs_R.value(1)    motor_R.duty(speed)def left():    revrs_L.value(1)    motor_L.duty(speed)    revrs_R.value(0)    motor_R.duty(speed)    def right_turn():    revrs_L.value(0)    motor_L.duty(speed)    revrs_R.value(0)    motor_R.duty(0)def left_turn():    revrs_L.value(0)    motor_L.duty(0)    revrs_R.value(0)    motor_R.duty(speed)def web_page(request):  motor_state="Stopped"  if request.find('GET /?forward') > 0:    motor_state="Going Forward"    forward()  if request.find('GET /?left') > 0:    motor_state="Rotate Left"    left()  if request.find('GET /?right') > 0:    motor_state="Rotate Right"     right()  if request.find('GET /?left_turn') > 0:    motor_state="Turn Left"    left_turn()  if request.find('GET /?right_turn') > 0:    motor_state="Turn Right"     right_turn()  if request.find('GET /?backward') > 0:    motor_state="Going Backward"    backward()  if request.find('GET /?stop') > 0:    motor_state="Stopped"    stop_all()    html = """<html><head><title>RoboTank WEB</title>   <meta name="viewport" content="width=device-width, initial-scale=1">  <link rel="icon" href="data:,"> <style>  html{font-family: Helvetica; display:inline-block; margin: 0px auto; text-align: center;}  h1{color: #0F3376; padding: 2vh;}p{font-size: 1.5rem;}  .button{display: inline-block; background-color: #33c080; border: none;   border-radius: 4px; color: white; text-decoration: none; font-size: 30px; width:100%}  .button2{background-color: #4286f4; width:30%}  .button3{background-color: #eb2b10; width:35%}  .button4{background-color: #8386f4; width:44%}  </style></head>  <body> <h1>RoboTank WEB</h1>   <p>Status : <strong>""" + motor_state + """</strong></p>  <p><a href='/?forward'><button class="button">Forward</button></a></p>  <p><a href='/?left_turn'><button class="button button2">LEFT</button></a>  <a href='/?stop'><button class="button button3">STOP</button></a>  <a href='/?right_turn'><button class="button button2">RIGHT</button></a>  <p><a href='/?backward'><button class="button">Backward</button></a></p>  <p><a href='/?left'><button class="button button4">L-rotate</button></a>  <a href='/?right'><button class="button button4">R-rotate</button></a></p>  </body></html>"""   return html#Stop all motors firststop_all()# connect to wi-fi networkdo_connect() # create socket for web srvraddr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]s = socket.socket()try:    s.bind(addr)    s.listen(1)except:    s.close()    s.bind(addr)    s.listen(1)# main loopwhile True:  conn, addr = s.accept()  print('Got a connection from %s' % str(addr))  request = conn.recv(1024)  request = str(request)  print('The Content = %s' % request)  response = web_page(request)  conn.send('HTTP/1.1 200 OK\n')  conn.send('Content-Type: text/html\n')  conn.send('Connection: close\n\n')  conn.sendall(response)  conn.close()