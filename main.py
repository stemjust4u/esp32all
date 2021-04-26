import utime, ubinascii, micropython, network, re, ujson
from lib.umqttsimple import MQTTClient
from machine import Pin, PWM
import gc
gc.collect()
micropython.alloc_emergency_exception_buf(100)

with open("stem", "rb") as f:    # Remove and over-ride MQTT/WIFI login info below
  stem = f.read().splitlines()

#==== MQTT SETUP ====#
MQTT_SERVER = '10.0.0.115'   # Over ride with MQTT/WIFI info
MQTT_USER = stem[0]         
MQTT_PASSWORD = stem[1]
WIFI_SSID = stem[2]
WIFI_PASSWORD = stem[3]
MQTT_CLIENT_ID = ubinascii.hexlify(machine.unique_id())

MQTT_SUB_TOPIC = []          # + is wildcard for that level
# Specific MQTT_SUB_TOPICS for ADC, servo, stepper are .appended below
MQTT_REGEX = rb'nred2esp/([^/]+)/([^/]+)' # rb is for raw, binary string for reg ex matching

#====== CONFIGURE MQTT AND TURN ON/OFF DEVICES ======================#
ESPID = b'/esp32A'
MQTT_PUB_TOPIC = [b'esp2nred/', ESPID] # Specific MQTT_PUB_TOPICS created at time of publishing
device = []                            # lvl2 category should be device names .appended in below
outgoingD = {}
cpuMHz = 240000000 # Can use 160000000 or 80000000 to drop power consumption by 10-20mA (almost 50%)
tracktime = False  # Flag to calculate main loop time
adcON = False             # Takes ~ 4ms to get adc values
buttonON = False
buttonpressed = False
rotaryencoderON = False  # Takes ~ 0.6ms to get rotary encoder data
stepperON = True        # Stepper needs delays < 1ms to run at fastest rpm
servoON = False
debugging = False   # Turn ON to see print messages. Print txt only is 0.13ms.  Print var with .format and int subtraction is 2.6ms

# Used to stagger timers for checking msgs, getting data, and publishing msgs
on_msgtimer_delay_ms = 200
get_datatimer_delay_ms = 200

# Period or frequency to check msgs, get data, publish msgs
on_msg_timer_ms = 600             # Takes ~ 2ms to check for msg
get_data_timer_ms = 600     #
pub_msg_timer_ms = 600            # Can take > 7ms to publish msgs

checkmsgs = False
get_data = False
sendmsgs = False

# Boot fails if pin 12 is pulled high
# Pins 34-39 are input only and do not have internal pull-up resistors. Good for ADC
# Items that are sent as part of mqtt topic will be binary (b'item)
if adcON:
    from adc import espADC
    # ADC is on pins 4,12-15,25-27,32-35,36,39
    MQTT_SUB_TOPIC.append(b'nred2esp/adcZCMD/+')
    device.append(b'adc')
    outgoingD[b'adc'] = {}
    outgoingD[b'adc']['send_always'] = False
    outgoingD[b'adc']['send'] = False         # Used to flag when to send results
    pinlist = [34]
    adc = espADC(pinlist, 3.3, 80, 2)    # Create adc object. Pass numOfChannels, vref, noiseThreshold=35, max Interval = 1
    # adcdata will be returned as a list
    if buttonON:
        buttonpin = 4 
        button = Pin(buttonpin, Pin.IN, Pin.PULL_UP)  # Create button
        def handle_interrupt(pin):    # Create handle interrupt function to update when button pressed
            global buttonpressed
            buttonpressed = True
        button.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=handle_interrupt) # link interrupt handler to function for pin falling or rising

if rotaryencoderON:
    from rotaryencoder import RotaryEncoder
    MQTT_SUB_TOPIC.append(b'nred2esp/rotencZCMD/+')
    device.append(b'rotencoder')
    outgoingD[b'rotencoder'] = {}
    outgoingD[b'rotencoder']['send_always'] = False
    outgoingD[b'rotencoder']['send'] = False         # Used to flag when to send results
    clkPin, dtPin, button = 18, 5, 4
    rotEnc1 = RotaryEncoder(clkPin, dtPin, button)

if stepperON:
    from uMstep28byjuln2003 import Stepper
    # controlsD, interval, stepresetare are updated in mqtt on_message
    MQTT_SUB_TOPIC.append(b'nred2esp/stepperZCMD/+')
    device.append(b'stepper')
    outgoingD[b'stepper'] = {}
    outgoingD[b'stepper']['send_always'] = True
    mqtt_controlsD={"delay":[0.1,0.3], "speed":[2,2], "mode":[0,0], "inverse":[False,True], "step":[2038,2038], "startstep":[0,0]}
    mqtt_stepreset = False    # used to reset steps thru nodered gui

    m1pins = [5,18,19,21]
    m2pins = [27,14,12,13]
    numbermotors = 2
    motor = Stepper(m1pins, m2pins, numbermotors)

if servoON:
    MQTT_SUB_TOPIC.append(b'nred2esp/servoZCMD/+')
    device.append(b'servo')
    outgoingD[b'servo'] = {}
    outgoingD[b'servo']['send_always'] = False
    outgoingD[b'servo']['send'] = False        
    servopins = [22, 23]
    servo = []
    for i, pin in enumerate(servopins):
        servo.append(PWM(Pin(pin),freq=50))
        servo[i].duty(75)
        # initialize to neutral position
        # 75 = 1.5mSec or neutral position

#======== MQTT FUNCTIONS =======#

def connect_wifi(WIFI_SSID, WIFI_PASSWORD):
    station = network.WLAN(network.STA_IF)

    station.active(True)
    station.connect(WIFI_SSID, WIFI_PASSWORD)

    while station.isconnected() == False:
        pass

    print('Connection successful')
    print(station.ifconfig())

def on_message(topic, msg):
    global mqtt_controlsD, mqtt_stepreset, servo
    if debugging: print("Received topic(tag): {0}".format(topic))
    msgmatch = re.match(MQTT_REGEX, topic)
    if msgmatch:
        incomingD = ujson.loads(msg.decode("utf-8", "ignore")) # decode json data
        incomingID = [msgmatch.group(0), msgmatch.group(1), msgmatch.group(2), type(incomingD)]
        if stepperON and incomingID[1] == 'stepperZCMD':
            if incomingID[2] == b'controls':
                mqtt_controlsD = incomingD
            elif incomingID[2] == b'stepreset':
                mqtt_stepreset = incomingD  # Boolean to flag for resetting the steps
        if servoON and incomingID[1] == b'servoZCMD':
            servo[int(incomingID[2])].duty(int(incomingD))
        #Uncomment prints for debugging. Will print the JSON incoming payload and unpack the converted dictionary
        #print("Received topic(tag): {0}".format(topic))
        #print("JSON payload: {0}".format(msg.decode("utf-8", "ignore")))
        #print("Unpacked dictionary (converted JSON>dictionary)")
        #for key, value in incomingD.items():
        #  print("{0}:{1}".format(key, value))

def connect_and_subscribe():
    global MQTT_CLIENT_ID, MQTT_SERVER, MQTT_SUB_TOPIC, MQTT_USER, MQTT_PASSWORD
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_SERVER, user=MQTT_USER, password=MQTT_PASSWORD)
    client.set_callback(on_message)
    client.connect()
    print('(CONNACK) Connected to {0} MQTT broker'.format(MQTT_SERVER))
    for topics in MQTT_SUB_TOPIC:
        client.subscribe(topics)
        print('Subscribed to {0}'.format(topics)) 
    return client

def restart_and_reconnect():
    print('Failed to connect to MQTT broker. Reconnecting...')
    utime.sleep_ms(5000)
    machine.reset()

try:
    mqtt_client = connect_and_subscribe()          # Connect and create the client
except OSError as e:
    restart_and_reconnect()

def is_integer(n):
    if n == None or n == "na":
        return False
    if isinstance(n, int):
        return True
    if abs(round(n) - n) == 0.5:
        return False
    else:
        return True

# MQTT setup is successful. Publish generic status confirmation easily seen on MQTT Explorer
mqtt_client.publish(b"status", b"esp32 connected, entering main loop")
led = Pin(2, Pin.OUT) #2 is the internal LED
led.value(1)
utime.sleep_ms(1000)
led.value(0)  # flash led to know main loop starting

#==== MAIN LOOP ========#
t0loop_us = utime.ticks_us()

t0onmsg_ms = utime.ticks_ms()
utime.sleep_ms(on_msgtimer_delay_ms)
t0_data_ms = utime.ticks_ms()
utime.sleep_ms(get_datatimer_delay_ms)
t0publish_ms = utime.ticks_ms()

while True:
    try:
        if utime.ticks_diff(utime.ticks_ms(), t0onmsg_ms) > on_msg_timer_ms:
            checkmsgs = True
            t0onmsg_ms = utime.ticks_ms()
        if utime.ticks_diff(utime.ticks_ms(), t0_data_ms) > get_data_timer_ms:
            get_data = True
            t0_data_ms = utime.ticks_ms()
        if utime.ticks_diff(utime.ticks_ms(), t0publish_ms) > pub_msg_timer_ms:
            sendmsgs = True
            t0publish_ms = utime.ticks_ms()

        if stepperON: motor.step(mqtt_controlsD)  # Main function to drive motors. As msg come in from nodered gui will update controls.

        if tracktime:
            t0main_us = utime.ticks_diff(utime.ticks_us(), t0loop_us)   # Monitor how long main loop is taking
            t0loop_us = utime.ticks_us()
            if debugging: print('Raw  time: {0} us'.format(t0main_us - 2600)) # Subtract 2.6ms for time it takes to print this line
        
        if checkmsgs:
            mqtt_client.check_msg()
            checkmsgs = False
            if debugging: print('checked msgs')
            
        if stepperON and mqtt_stepreset:  # If step reset trigger received from node red dashboard then reset the steps and reply with command for node red to reset gauge
            motor.resetsteps()
            mqtt_stepreset = False
            tempdevice = 'nredZCMD'
            mqtt_client.publish(tempdevice.join(MQTT_PUB_TOPIC), "resetstepgauge")
            if debugging: print('reset step gauge')
            
        if get_data:
            if stepperON: 
                outgoingD[b'stepper'] = motor.getdata() # Get steps, rpm, etc info to update the node red dashboard
                if debugging: print('got motor data')
            if adcON:
                adcdata = adc.getValue()
                if buttonpressed or adcdata is not None:         # Update if button pressed or voltage changed or time limit hit
                    outgoingD[b'adc']['send'] = True
                    if adcdata is not None:
                        for i,pin in enumerate(adcdata):
                            outgoingD[b'adc']['a' + str(i) + 'f'] = str(adcdata[i])  # Get the voltage of each channel    
                    if buttonON: outgoingD[b'adc']['buttoni'] = str(button.value())
                    buttonpressed = False
                    if debugging: print("got adc data")
            get_data = False

        if rotaryencoderON:
            clicks, buttonstate = rotEnc1.runencoder()
            if is_integer(clicks):
                outgoingD[b'rotencoder']['send'] = True
                outgoingD[b'rotencoder']['RotEnc1Ci'] = str(clicks)
                outgoingD[b'rotencoder']['RotEnc1Bi'] = str(buttonstate)
                if debugging: print("got encoder data")

        if sendmsgs:   # Send messages for all devices setup
            if tracktime: 
                main_msf = t0main_us / 1000
            if (stepperON and tracktime): outgoingD[b'stepper']['main_msf'] = main_msf
            for item in device:
                if outgoingD[item]['send_always']:
                    mqtt_client.publish(item.join(MQTT_PUB_TOPIC), ujson.dumps(outgoingD[item])) # Send motor data (steps, rpm, etc) back to node-red dashboard
                    if debugging: print('sent msg')
                else:
                    if outgoingD[item]['send']:
                        mqtt_client.publish(item.join(MQTT_PUB_TOPIC), ujson.dumps(outgoingD[item]))
                        outgoingD[item]['send'] = False
                        if debugging: print('sent msg')
            sendmsgs = False
            
    except OSError as e:
        restart_and_reconnect()