''' esp32 ADC.  If any channel has a delta (current-previous) that is above the
noise threshold or if the max Time interval exceeded then the 
voltage from all initialized channels will be returned.
 When creating object, pass: pins, Vref, noise threshold, and max time interval
 Will return a list with the voltage value for each channel.

To find the noise threshold set noise threshold low and max time interval low.
Noise is in raw ADC

Max time interval is used to catch drift/creep that is below the noise threshold.

'''

from machine import Pin, ADC
import utime

class espADC:
    def __init__(self, pinlist, vref=3.3, noiseThreshold=35, maxInterval=1000, setupinfo=False):
        self.vref = vref
        self.numOfChannels = len(pinlist)
        if setupinfo: print("ADC setting up {0} channels. Vref:{1} NoiseTh:{2} MaxIntvl:{3}sec".format(self.numOfChannels, vref, noiseThreshold, maxInterval/1000))
        self.chan = []
        for i, pin in enumerate(pinlist):
            self.chan.append(ADC(Pin(pin)))
            self.chan[i].atten(ADC.ATTN_11DB) # Full range: 0-3.3V
        if setupinfo: print("ADC setup:{0}".format(self.chan))   
        self.noiseThreshold = noiseThreshold
        self.sensorLastRead = [x for x in range(self.numOfChannels)]
        for x in range(self.numOfChannels): # initialize the first read for comparison later
            self.sensorLastRead[x] = self.chan[x].read()
        self.voltage = [x for x in range(self.numOfChannels)]
        self.sensor = [x for x in range(self.numOfChannels)]
        self.maxInterval = maxInterval # interval in ms to check for update
        self.time0 = utime.ticks_ms()   # time 0
    
    def _valmap(self, value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

    def getValue(self):
        sensorChanged = False
        timelimit = False
        if utime.ticks_diff(utime.ticks_ms(), self.time0) > self.maxInterval:
            timelimit = True
        for x in range(self.numOfChannels):
            self.sensor[x] = self.chan[x].read()
            if abs(self.sensor[x] - self.sensorLastRead[x]) > self.noiseThreshold:
                sensorChanged = True
        if sensorChanged or timelimit:
            for x in range(self.numOfChannels):
                self.sensorLastRead[x] = self.sensor[x]
                self.voltage[x] = self._valmap(self.sensor[x], 0, 4095, 0, self.vref) # 4mV change is approx 5
            self.time0 = utime.ticks_ms()
            return self.voltage
        
if __name__ == "__main__":
    import time
    # Run main loop
    pinlist = [34, 35]
    adc = espADC(pinlist, 3.3, 40, 10000, setupinfo=True)
    while True:
        print(adc.getValue())
        time.sleep(1)
