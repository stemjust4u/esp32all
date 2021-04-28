#!/usr/bin/env python3

"""
28BYJ-48 Stepper motor with ULN2003 Driver board
Motor instructions received via mqtt from a node-red server

See HARDWARE/MQTT initialize in boot.py for Wiring and TOPICS

Wiring
Motor1
IN1,2,3,4

"""
from machine import Pin
from time import sleep_us
import utime
import math

class Stepper:   # command comes from node-red GUI
    def __init__(self, m1pin, m2pin, numbermotors=1, setupinfo=False):
        self.numbermotors = numbermotors
        if self.numbermotors == 2:
            self.stepperpin = [[Pin(m1pin[0], Pin.OUT),Pin(m1pin[1], Pin.OUT),Pin(m1pin[2], Pin.OUT),Pin(m1pin[3], Pin.OUT)]
, [Pin(m2pin[0], Pin.OUT),Pin(m2pin[1], Pin.OUT),Pin(m2pin[2], Pin.OUT),Pin(m2pin[3], Pin.OUT)]]
        elif self.numbermotors == 1:
            self.stepperpin = [[Pin(m1pin[0], Pin.OUT),Pin(m1pin[1], Pin.OUT),Pin(m1pin[2], Pin.OUT),Pin(m1pin[3], Pin.OUT)], [0]]
        if setupinfo: print('Stepper - {0} motor(s) on  pins:{1} '.format(self.numbermotors, self.stepperpin))
        self.steppersteps = [0, 0]           # Keep track of how many steps each motor has taken
        self.stepperspeed = [[0,1,2,3,4], [0,1,2,3,4]]      # Will keep track of coil pulses for each speed
        self.steppercoils = [{"Half":[0,1], "Full":[0,1], "arr2":[0,1], "arr3":[0,1], "HarrOUT":[0,1], "FarrOUT":[0,1]}, {"Half":[0,1], "Full":[0,1], "arr2":[0,1], "arr3":[0,1], "HarrOUT":[0,1], "FarrOUT":[0,1]}]
        self.FULLREVOLUTION = 4076    # Steps per revolution
        # Setup and intialize motor parameters
        self.startstepping = [False,False]  # Flag send from node red gui to start stepping in incremental mode
        self.targetstep = [291,291]         # When incremental stepping started will calculate the target step to stop at
        self.rpmtime0 = [utime.ticks_us(), utime.ticks_us()]
        self.rpmsteps0 = [0, 0]
        self.rpm = [0,0]
        self.seq = [0,0]
        self.stepperstats = {}   # Container for sending stepper stats
        self.delay_us = 0   # Keep track of total delay/pause after sending pulses to motors
        for i in range(self.numbermotors):
            self.stepperspeed[i][2] = [0,0,0,0]  # speed 2 is hard coded as stop
            self.steppercoils[i]["Half"] = [
                                            [1,0,0,1],
                                            [1,0,0,0],
                                            [1,1,0,0],
                                            [0,1,0,0],
                                            [0,1,1,0],
                                            [0,0,1,0],
                                            [0,0,1,1],
                                            [0,0,0,1]
                                            ]
            self.steppercoils[i]["Full"]= [
                                            [1,0,0,1],
                                            [1,1,0,0],
                                            [0,1,1,0],
                                            [0,0,1,1]
                                            ]
        if setupinfo: print('Stepper Halfstep Seq:{0}'.format(self.steppercoils[0]["Half"]))
        if setupinfo: print('Stepper Fullstep Seq:{0}'.format(self.steppercoils[0]["Full"]))

    def step(self, controls):
        ''' LOOP THRU EACH STEPPER AND THE TWO ROTATIONS (CW/CCW) AND SEND COIL ARRAY (HIGH PULSES) '''
        self.command = controls
        
        for i in range(self.numbermotors):
            stepspeed = self.command["speed"][i]    # Speed from node red. stepspeed is a local variable for this loop
            self.delay_us = int(self.command["delay"][0])   # Motor delay from node red (usec)
            if stepspeed > 2:
                rotation = 0 if not self.command["inverse"][i] else 1
            elif stepspeed < 2:
                rotation = 1 if not self.command["inverse"][i] else 0
                
            if stepspeed == 3 or stepspeed == 1:  # Half step calculation
                self.stepperspeed[i][stepspeed] = self.steppercoils[i]["Half"][self.seq[i]]
                self.seq[i] = self._hseq_update(rotation, self.seq[i])
            if stepspeed == 4 or stepspeed == 0:  # Full step calculation
                if self.seq[i] > 3:
                    self.seq[i] = 3
                self.stepperspeed[i][stepspeed] = self.steppercoils[i]["Full"][self.seq[i]]
                self.seq[i] = self._fseq_update(rotation, self.seq[i])
                self.delay_us = self.delay_us + int(self.command["delay"][1]) # Add extra delay (updated from nodered) for full step
                
            # If mode is 1 (incremental stepping) and startstep has been flagged from node-red gui then startstepping
            if self.command["mode"][i] == 1 and stepspeed != 2 and self.command["startstep"][i] == 1:
                self.startstepping[i] = True
                self.command["startstep"][i] = 0 # startstepping triggered and targetstep calculated. So turn off this if cond
                if stepspeed > 2: # moving CW 
                    if math.fabs(self.steppersteps[i]) + self.command["step"][i] <= self.FULLREVOLUTION: # Set the target step based on node-red gui target and current step for that motor
                        self.targetstep[i] = math.fabs(self.steppersteps[i]) + self.command["step"][i]
                    else:
                        self.targetstep[i] = self.FULLREVOLUTION
                else:      # moving CCW
                    self.targetstep[i] = self.steppersteps[i] - self.command["step"][i]
                    if self.targetstep[i] < (self.FULLREVOLUTION * -1):
                        self.targetstep[i] = (self.FULLREVOLUTION * -1)

            # Mode set to 1 (incremental stepping) but haven't started stepping. Stop motor (stepspeed=2) and set the target step (based on node-red gui)
            # Will wait until startstep flag is sent from node-red GUI before starting motor
            if self.command["mode"][i] == 1 and not self.startstepping[i]:
                stepspeed = 2
                self.command["speed"][i] = 2

            # IN INCREMENT MODE1. Keep stepping until the target step is met. Then reset the startstepping/startstep(nodered) flags.
            elif self.command["mode"][i] == 1 and self.startstepping[i]:
                if math.fabs((math.fabs(self.steppersteps[i]) - math.fabs(self.targetstep[i]))) < 2: # if delta is less than 2 then target met. Can't use 0 since full step increments by 2
                    self.startstepping[i] = False

            # SEND COIL ARRAY (HIGH PULSES) TO GPIO PINS AND UPDATE STEP COUNTER
            for coil in range(4):
                self.stepperpin[i][coil].value(self.stepperspeed[i][stepspeed][coil])  # output the coil array (speed/direction) to the GPIO pins.
            self.steppersteps[i] = self._update_steps(stepspeed, self.steppersteps[i])  # update the motor step based on direction and half vs full step    
            
            # IF FULL REVOLUTION - reset the step counter
            if (math.fabs(self.steppersteps[i]) > self.FULLREVOLUTION):  # If hit full revolution reset the step counter. If want to step past full revolution would need to later add a 'not startstepping'
                self.steppersteps[i] = 0

        # DELAY FOR MOTORS TO UPDATE
        if self.delay_us > 0: sleep_us(self.delay_us)  # delay can be updated from node-red gui. Needs optimal setting for the motors. Currently one delay for all motors
        
    def getdata(self):
        ''' Publish how many steps, rpms, etc the motor is at (node red will use to update dashboard) '''
        for i in range(self.numbermotors):
            rpm = ((self.steppersteps[i]-self.rpmsteps0[i])/self.FULLREVOLUTION)/(utime.ticks_diff(utime.ticks_us(), self.rpmtime0[i])/60000000)
            if rpm >= 0:
                self.rpm[i] = rpm
            self.rpmsteps0[i] = self.steppersteps[i] # reset rpm counters for next calculation
            self.rpmtime0[i] = utime.ticks_us()
            self.stepperstats['steps' + str(i) + 'i'] = self.steppersteps[i]
            self.stepperstats['rpm' + str(i) + 'f'] = self.rpm[i]
            self.stepperstats['speed' + str(i) + 'i'] = self.command["speed"][i]
            self.stepperstats['delayf'] = self.delay_us
        return self.stepperstats

    def resetsteps(self):
        ''' Reset the step counters on all motors '''
        for i in range(self.numbermotors):
            self.steppersteps[i] = 0

    def _hseq_update(self, rot, seq):
        if rot == 0:
            if seq == 7:
                seq = 0
            else:
                seq += 1
        if rot == 1:
            if seq == 0:
                seq = 7
            else:
                seq -= 1
        return seq                    
                
    def _fseq_update(self, rot, seq):
        if rot == 0:
            if seq == 3:
                seq = 0
            else:
                seq += 1
        if rot == 1:
            if seq == 0:
                seq = 3
            else:
                seq -= 1
        return seq 
    
    def _update_steps(self, spd, stp):
        ''' Will update the motor step counter based on full vs half speed and CW vs CCW. half = 1, full = 2'''
        if spd == 4:
            stp += 2
        elif spd ==3:
            stp += 1
        elif spd == 1:
            stp -= 1
        elif spd == 0:
            stp -= 2
        else:
            stp = stp
        return stp