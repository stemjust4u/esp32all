import utime, time

time0 = utime.ticks_ms()
time.sleep_ms(5001)
print(utime.ticks_diff(utime.ticks_ms(), time0))