import esp, machine
esp.osdebug(None)

if machine.reset_cause() == machine.DEEPSLEEP_RESET:
    print('woke from a deep sleep')

machine.freq(240000000)
cpufreqi = machine.freq()/(10**9)
print('{0}MHz'.format(cpufreqi))