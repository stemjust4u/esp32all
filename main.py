import utime, ubinascii, micropython, network, re, ujson
from lib.umqttsimple import MQTTClient
from machine import Pin, PWM
import gc
gc.collect()
micropython.alloc_emergency_exception_buf(100)

def connect_wifi(WIFI_SSID, WIFI_PASSWORD):
    station = network.WLAN(network.STA_IF)

    station.active(True)
    station.connect(WIFI_SSID, WIFI_PASSWORD)

    while station.isconnected() == False:
        pass

    print('Connection successful')
    print(station.ifconfig())

def mqtt_setup(IPaddress):
    global MQTT_CLIENT_ID, MQTT_SERVER, MQTT_USER, MQTT_PASSWORD, MQTT_SUB_TOPIC, MQTT_REGEX
    with open("stem", "rb") as f:    # Remove and over-ride MQTT/WIFI login info below
      stem = f.read().splitlines()
    MQTT_SERVER = IPaddress   # Over ride with MQTT/WIFI info
    MQTT_USER = stem[0]         
    MQTT_PASSWORD = stem[1]
    WIFI_SSID = stem[2]
    WIFI_PASSWORD = stem[3]
    MQTT_CLIENT_ID = ubinascii.hexlify(machine.unique_id())
    MQTT_SUB_TOPIC = []
    # Specific MQTT_SUB_TOPICS for ADC, servo, stepper are .appended below
    MQTT_REGEX = rb'nred2esp/([^/]+)/([^/]+)' # b'txt' is binary format. Required for umqttsimple to save memory
                                              # r'txt' is raw format for easier reg ex matching
                                              # 'nred2esp/+' would also work but would not return groups
                                              # () group capture. Useful for getting topic lvls in on_message
                                              # [^/] match a char except /. Needed to get topic lvl2, lvl3 groups
                                              # + will match one or more. Requiring at least 1 match forces a lvl1/lvl2/lvl3 topic structure
                                              # * could also be used for last group and then a lvl1/lvl2 topic would also be matched

def mqtt_connect_subscribe():
    global MQTT_CLIENT_ID, MQTT_SERVER, MQTT_SUB_TOPIC, MQTT_USER, MQTT_PASSWORD
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_SERVER, user=MQTT_USER, password=MQTT_PASSWORD)
    client.set_callback(mqtt_on_message)
    client.connect()
    print('(CONNACK) Connected to {0} MQTT broker'.format(MQTT_SERVER))
    for topics in MQTT_SUB_TOPIC:
        client.subscribe(topics)
        print('Subscribed to {0}'.format(topics)) 
    return client

def mqtt_on_message(topic, msg):
    global MQTT_REGEX
    global stepperON, mqtt_controlsD, mqtt_stepreset
    global servoON, mqtt_servo_duty, mqtt_servoID
    msgmatch = re.match(MQTT_REGEX, topic)
    if msgmatch:
        incomingD = ujson.loads(msg.decode("utf-8", "ignore")) # decode json data
        incomingID = [msgmatch.group(0), msgmatch.group(1), msgmatch.group(2), type(incomingD)]
        if stepperON and incomingID[1] == b'stepperZCMD':
            if incomingID[2] == b'controls':
                mqtt_controlsD = incomingD
            elif incomingID[2] == b'stepreset':
                mqtt_stepreset = incomingD  # Boolean to flag for resetting the steps
        if servoON and incomingID[1] == b'servoZCMD':
            mqtt_servoID = int(incomingID[2])
            mqtt_servo_duty = int(incomingD)

def mqtt_reset():
    print('Failed to connect to MQTT broker. Reconnecting...')
    utime.sleep_ms(5000)
    machine.reset()

def create_servo(pinlist):
    global MQTT_SUB_TOPIC, device, outgoingD, mqtt_servoID, mqtt_servo_duty
    MQTT_SUB_TOPIC.append(b'nred2esp/servoZCMD/+')
    device.append(b'servo')
    outgoingD[b'servo'] = {}
    outgoingD[b'servo']['send_always'] = False
    outgoingD[b'servo']['send'] = False    
    mqtt_servoID = 0
    mqtt_servo_duty = 0
    servoArr = []
    setupinfo = True
    freq=50       # higher freq has lower duty resolution. esp32 can go from 1-40000 (40MHz crystal oscillator) 
    neutral = 75  # initialize to neutral position, 75=1.5mSec at 50Hz. (75/50=1.5ms or 1.5ms/20ms period = 7.5% duty cycle)
    for i, pin in enumerate(pinlist):
        servoArr.append(PWM(Pin(pin),freq=50))
        servoArr[i].duty(neutral)
        pinsummary.append(pin)
    if setupinfo: print('Servo:{0}'.format(servoArr))
    return servoArr 

def create_stepper(m1pins, m2pins, numbermotors):
    global MQTT_SUB_TOPIC, device, outgoingD, mqtt_controlsD, mqtt_stepreset
    from uMstep28byjuln2003 import Stepper
    MQTT_SUB_TOPIC.append(b'nred2esp/stepperZCMD/+')
    device.append(b'stepper')
    outgoingD[b'stepper'] = {}
    outgoingD[b'stepper']['data'] = {}
    outgoingD[b'stepper']['send_always'] = True
    mqtt_controlsD={"delay":[0,300], "speed":[3,3], "mode":[0,0], "inverse":[False,True], "step":[2038,2038], "startstep":[0,0]}
    mqtt_stepreset = False    # used to reset steps thru nodered gui
    for pin in m1pins:
        pinsummary.append(pin)
    if m2pins is not None:
        for pin in m2pins:
            pinsummary.append(pin)
    return Stepper(m1pins, m2pins, numbermotors, setupinfo=True)

def create_rotary_encoder(clkPin, dtPin, button_rotenc):
    global pinsummary, MQTT_SUB_TOPIC, device, outgoingD
    from rotaryencoder import RotaryEncoder
    MQTT_SUB_TOPIC.append(b'nred2esp/rotencZCMD/+')
    device.append(b'rotencoder')
    outgoingD[b'rotencoder'] = {}
    outgoingD[b'rotencoder']['data'] = {}
    outgoingD[b'rotencoder']['send_always'] = False
    outgoingD[b'rotencoder']['send'] = False         # Used to flag when to send results
    pinsummary.append(clkPin)
    pinsummary.append(dtPin)
    if button_rotenc is not None: pinsummary.append(button_rotenc)
    return RotaryEncoder(clkPin, dtPin, button_rotenc, setupinfo=True)

def create_adc(pinlist, switch, switchON=False):
    global MQTT_SUB_TOPIC, device, outgoingD, buttonADC_pressed, buttonADC
    from adc import espADC
    MQTT_SUB_TOPIC.append(b'nred2esp/adcZCMD/+')
    device.append(b'adc')
    outgoingD[b'adc'] = {}
    outgoingD[b'adc']['data'] = {}
    outgoingD[b'adc']['send_always'] = False
    outgoingD[b'adc']['send'] = False         # Used to flag when to send results
    buttonADC_pressed = False
    if switchON:
        buttonADC = Pin(switch, Pin.IN, Pin.PULL_UP)  # Create button
        def _buttonADC_ISR(pin):    # Create handle interrupt function to update when button pressed
            global buttonADC_pressed
            buttonADC_pressed = False
        buttonADC.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=_buttonADC_ISR) # link interrupt handler to function for pin falling or rising
        print('Button(ADC):{0}'.format(buttonADC))
    for pin in pinlist:
        pinsummary.append(pin)
    if switch is not None: pinsummary.append(switch)          # For adc noise, th is raw data, 1mV = 1.25 raw
    return espADC(pinlist, 3.3, 35, 5000, setupinfo=True)    # Create adc object. Pass numOfChannels, vref, noiseThreshold=35, max Interval (ms)

def create_buttonA(button_pin):
    global device, outgoingD, buttonA_pressed
    device.append(b'button')
    outgoingD[b'button'] = {}
    outgoingD[b'button']['data'] = {}
    outgoingD[b'button']['send_always'] = False
    outgoingD[b'button']['send'] = False
    buttonA_pressed = False
    buttonA = Pin(button_pin, Pin.IN, Pin.PULL_UP)  # Create button
    def _buttonA_ISR(pin):    # Create handle interrupt function to update when button pressed
        global buttonA_pressed, interrupt_pin
        buttonA_pressed = True
        interrupt_pin = pin  # Can use to return which pin thru the trigger
    buttonA.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=_buttonA_ISR) # link interrupt handler to function for pin falling or rising
    pinsummary.append(button_pin)
    print('Button(A):{0}'.format(buttonA))
    return buttonA

def main():
    global pinsummary, interrupt_pin
    global stepperON, mqtt_controlsD, mqtt_stepreset  # Stepper motor variables used in mqtt on_message
    global servoON, mqtt_servoID, mqtt_servo_duty     # Servo variables used in mqtt on_message
    global device, outgoingD                          # Containers setup in 'create' functions and used for Publishing mqtt
    global buttonADC_pressed, buttonADC               # Button specific for ADC (ie joystick setup) 
    global buttonA_pressed
    
    #===== SETUP VARIABLES ============#
    # Setup mqtt variables (topics and data containers) used in on_message, main loop, and publishing
    # Further setup of variables is completed in specific 'create_device' functions
    mqtt_setup('10.0.0.115')
    device = []    # mqtt lvl2 topic category and '.appended' in create functions
    outgoingD = {} # container used for publishing mqtt data
    
    debuggingLvl1 = False
    
    # umqttsimple requires topics to be byte format. For string.join to work on topics, all items must be the same, bytes.
    ESPID = b'/esp32A'  # Specific MQTT_PUB_TOPICS created at time of publishing using string.join (specifically lvl2.join)
    MQTT_PUB_TOPIC = [b'esp2nred/', ESPID]
  
    # Used to stagger timers for checking msgs, getting data, and publishing msgs
    on_msgtimer_delay_ms = 250
    # Period or frequency to check msgs, get data, publish msgs
    on_msg_timer_ms = 500     # Takes ~ 2ms to check for msg
    getdata_sndmsg_timer_ms = 500   # Can take > 7ms to publish msgs

    #=== SETUP DEVICES ===#
    # Boot fails if pin 12 is pulled high
    # Pins 34-39 are input only and do not have internal pull-up resistors. Good for ADC
    # Items that are sent as part of mqtt topic will be binary (b'item)
    pinsummary = []
    
    buttonA_ON = True
    if buttonA_ON:
        buttonA = create_buttonA(13)
    
    adcON = True             # Takes ~ 4ms to get adc values
    switchON = False          # Optional button (ie joystick)
    if adcON:
        adcpins = [34, 35]
        switch = 25
        adc = create_adc(adcpins, switch, switchON)
    
    rotaryencoderON = True  # Takes ~ 0.6ms to get rotary encoder data
    if rotaryencoderON:
        clkPin, dtPin, button_rotenc = 15, 4, 2
        rotEnc1 = create_rotary_encoder(clkPin, dtPin, button_rotenc)
    
    stepperON = True        # Stepper needs delays < 1ms to run at fastest rpm
    stepper_dataON = True
    track_mainloop_time = True
    if stepperON:
        m1pins = [5,18,19,21]
        m2pins = [12,14,27,26]
        numbermotors = 2
        motor = create_stepper(m1pins, m2pins, numbermotors)

    servoON = True
    if servoON:
        servopins = [22, 23]
        servo = create_servo(servopins)

    print('Pins in use:{0}'.format(sorted(pinsummary)))
    #==========#
    # Connect and create the client
    try:
        mqtt_client = mqtt_connect_subscribe()
    except OSError as e:
        mqtt_reset()
    # MQTT setup is successful, publish status msg and flash on-board led
    mqtt_client.publish(b'status'.join(MQTT_PUB_TOPIC), b'esp32 connected, entering main loop')
    # Initialize flags and timers
    checkmsgs = False
    get_data = False
    sendmsgs = False    
    t0onmsg_ms = utime.ticks_ms()
    utime.sleep_ms(on_msgtimer_delay_ms)
    t0_datapub_ms = utime.ticks_ms()
    t0loop_us = utime.ticks_us()
    
    while True:
        try:
            if utime.ticks_diff(utime.ticks_ms(), t0onmsg_ms) > on_msg_timer_ms:
                checkmsgs = True
                t0onmsg_ms = utime.ticks_ms()
            if utime.ticks_diff(utime.ticks_ms(), t0_datapub_ms) > getdata_sndmsg_timer_ms:
                get_data = True
                sendmsgs = True
                t0_datapub_ms = utime.ticks_ms()
            
            if stepperON: motor.step(mqtt_controlsD)  # Main function to drive all motors. As msg come in from nodered gui will update controls.
            
            if servoON: servo[mqtt_servoID].duty(mqtt_servo_duty) # Servo commands sent one-at-a-time
            
            if track_mainloop_time:
                tmain_us = utime.ticks_diff(utime.ticks_us(), t0loop_us)   # Monitor how long main loop is taking
                t0loop_us = utime.ticks_us()
                if debuggingLvl1: print('Raw  time: {0} us'.format(tmain_us - 2600)) # Subtract 2.6ms for time it takes to print this line
            
            if checkmsgs:
                mqtt_client.check_msg()
                checkmsgs = False
                
            if stepperON and mqtt_stepreset:  # If step reset trigger received from node red dashboard then reset the steps and reply with command for node red to reset gauge
                motor.resetsteps()
                mqtt_stepreset = False
                tempdevice = b'nredZCMD'
                mqtt_client.publish(tempdevice.join(MQTT_PUB_TOPIC), "resetstepgauge")
                
            if get_data:
                if stepper_dataON: 
                    outgoingD[b'stepper']['data'] = motor.getdata() # Get steps, rpm, etc info to update the node red dashboard
                if adcON:
                    adcdata = adc.getValue()
                    if buttonADC_pressed or adcdata is not None:         # Update if button pressed or voltage changed or time limit hit
                        outgoingD[b'adc']['send'] = True
                        if adcdata is not None:
                            for i,pin in enumerate(adcdata):
                                outgoingD[b'adc']['data']['a' + str(i) + 'f'] = str(adcdata[i])  # Get the voltage of each channel    
                        if switchON: outgoingD[b'adc']['data']['buttoni'] = str(buttonADC.value())
                        buttonADC_pressed = False
                if buttonA_ON:
                    if buttonA_pressed:
                        outgoingD[b'button']['send'] = True
                        outgoingD[b'button']['data']['buttonAi'] = str(buttonA.value())
                        buttonA_pressed = False
                get_data = False

            if rotaryencoderON:
                clicks = rotEnc1.update()
                if clicks is not None:
                    outgoingD[b'rotencoder']['send'] = True
                    outgoingD[b'rotencoder']['data']['RotEnc1Ci'] = str(clicks[0])
                    outgoingD[b'rotencoder']['data']['RotEnc1Bi'] = str(clicks[1]) # Button state
                    sendmsgs = True

            if sendmsgs:   # Send messages for all devices setup
                if (stepperON and track_mainloop_time): outgoingD[b'stepper']['data']['tmain_usi'] = tmain_us
                for item in device:
                    if outgoingD[item]['send_always']:
                        mqtt_client.publish(item.join(MQTT_PUB_TOPIC), ujson.dumps(outgoingD[item]['data'])) # Send motor data (steps, rpm, etc) back to node-red dashboard
                    else:
                        if outgoingD[item]['send']:
                            mqtt_client.publish(item.join(MQTT_PUB_TOPIC), ujson.dumps(outgoingD[item]['data']))
                            outgoingD[item]['send'] = False
                sendmsgs = False
                
        except OSError as e:
            mqtt_reset()

if __name__ == "__main__":
    # Run main loop            
    main()
