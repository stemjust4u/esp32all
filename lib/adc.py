''' esp32 ADC.  If any channel has a delta (current-previous) that is above the
noise threshold or if the max Time interval exceeded then the 
voltage from all initialized channels will be returned.
 When creating object, pass: Number of channels, Vref, noise threshold, and max time interval
 Will return a list with the voltage value for each channel.

Channels need to be manually linked in the init.

To find the noise threshold set noise threshold low and max time interval low.
Noise is in raw ADC

Max time interval is used to catch drift/creep that is below the noise threshold.

'''

from machine import Pin, ADC
import utime, ujson

class espADC:
    def __init__(self, pinlist, vref=3.3, noiseThreshold=35, maxInterval=1):
        self.vref = vref
        self.numOfChannels = len(pinlist)
        print("num of channels {0}".format(self.numOfChannels))
        self.chan = []
        for i, pin in enumerate(pinlist):
            self.chan.append(ADC(Pin(pin)))
            self.chan[i].atten(ADC.ATTN_11DB) # Full range: 0-3.3V
            print("setup pin:{0} for ADC".format(pin))   
        self.noiseThreshold = noiseThreshold
        self.numOfSamples = 10
        self.sensorAve = [x for x in range(self.numOfChannels)]
        self.sensorLastRead = [x for x in range(self.numOfChannels)]
        for x in range(self.numOfChannels): # initialize the first read for comparison later
            self.sensorLastRead[x] = self.chan[x].read()
        self.adcValue = [x for x in range(self.numOfChannels)]
        self.sensor = [[x for x in range(0, self.numOfSamples)] for x in range(0, self.numOfChannels)]
        self.maxInterval = maxInterval * 1000 # interval in ms to check for update
        self.time0 = utime.ticks_ms()   # time 0
    
    def valmap(self, value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

    def getValue(self):
        sensorChanged = False
        timelimit = False
        if utime.ticks_ms() - self.time0 > self.maxInterval:
            timelimit = True
        for x in range(self.numOfChannels):
            for i in range(self.numOfSamples):  # get samples points from analog pin and average
                self.sensor[x][i] = self.chan[x].read()
            self.sensorAve[x] = sum(self.sensor[x])/len(self.sensor[x])
            #print(abs(self.sensorAve[x] - self.sensorLastRead[x]))
            if abs(self.sensorAve[x] - self.sensorLastRead[x]) > self.noiseThreshold:
                sensorChanged = True
                #print(self.sensorAve[x] - self.sensorLastRead[x])
            self.sensorLastRead[x] = self.sensorAve[x]
            self.adcValue[x] = self.valmap(self.sensorAve[x], 0, 4095, 0, self.vref) # 4mV change is approx 500
            #print('chan: {0} value: {1:1.2f}'.format(x, self.adcValue[x]))
        if sensorChanged or timelimit:
            self.adcValue = ["%.3f"%pin for pin in self.adcValue] #format and send final adc results
            self.time0 = utime.ticks_ms()
            return self.adcValue
        else:
            pass
