'''
Pass the pins the clk/dt and button are connected to. Records the position of the button
both CW and CCW. Starts at 0 and increments by 1 or -1.

Returns two integers, the position of the knob and the state of the button, 0 or 1.

'''

from machine import Pin
import ujson

class RotaryEncoder:
    def __init__(self, clkPin, dtPin, button):
        self.counter = 0
        self.clkUpdate = True
        self.buttonpressed = False
        self.clkPin = Pin(clkPin, Pin.IN, Pin.PULL_UP)  
        self.dtPin = Pin(dtPin, Pin.IN, Pin.PULL_UP)  
        self.button = Pin(button, Pin.IN, Pin.PULL_UP)  # Create button
        self.clkLastState = self.clkPin.value()
        self.clkState = self.clkPin.value()
        self.dtState = self.dtPin.value()
         
        self.button.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.button_callback) # link interrupt handler to function for pin falling or rising
    
    def runencoder(self):
        self.clkState = self.clkPin.value()
        self.dtState = self.dtPin.value()
        if self.clkState != self.clkLastState or self.buttonpressed:
            if self.clkState != self.clkLastState:
                if self.dtState != self.clkState:
                    self.counter -= 0.5
                else:
                    self.counter += 0.5
                self.clkLastState = self.clkState
            buttonstate = self.button.value()
            self.buttonpressed = False
            return self.counter, buttonstate
        else:
            return "na", "na"

    def button_callback(self, pin):
        self.buttonpressed = True



