from mq import *
import sys, time

try:
    print("Press CTRL+C to abort.")
    
    mq = MQ()
    while True:
        print("1")
        perc = mq.MQPercentage()
        sys.stdout.write("\r")
        sys.stdout.write("\033[K")
        sys.stdout.write("LPG: %g ppm, CO: %g ppm, Smoke: %g ppm" % (perc["GAS_LPG"], perc["CO"], perc["SMOKE"]))
        sys.stdout.flush()
        time.sleep(0.1)

except Exception as e: print(e)
    #print("\nAbort by user")
