#DriverCodeV15 Redux.py
#Driver Code - Andrea Anaya, Ann Ryan, Ethan de Leon, Caitlin Pasqualino
#Editor - Ethan de Leon

import sensor, time, pyb, math, utime
from pyb import Pin, Timer

COOLDOWNRATE = 2
MAXFINISHFRAMES = 3
CLOCK = time.clock()

WHITE = [(210,255)]
ROI1=(0,40,160,10) #far
ROI2=(0,80,160,10) #close

MAXPWM = 100
TANY = 30

#Screen Macros
FRAMESIZE = 160
CENTERFRAME = int(FRAMESIZE/2)
MIDDLEFRAME = int(FRAMESIZE/3)
LFRAMELIMIT = int(FRAMESIZE/3)
RFRAMELIMIT = int(FRAMESIZE - MIDDLEFRAME)

#Car Measurement Macros
WHEELRADIUS=3.5 # cm
WIDTH=14.5      # cm
FREQ=4000       # hz of the clock of the car that we sending the PWM
ANGFREQ=FREQ*2*math.pi # angular frequency


class LEDControl():
    def __init__(self) -> None:
        self.led_r=pyb.LED(1) #Red LED
        self.led_g=pyb.LED(2) #Green LED
        self.led_b=pyb.LED(3) #Blue LED
        self.off()

    def on_white(self):
        self.led_r.on()
        self.led_g.on()
        self.led_b.on()

    def on_red(self):
        self.led_r.on()
        self.led_g.off()
        self.led_b.off()

    def on_green(self):
        self.led_r.off()
        self.led_g.on()
        self.led_b.off()
    
    def on_blue(self):
        self.led_r.off()
        self.led_g.off()
        self.led_b.on()

    def on_yellow(self):
        self.led_r.on()
        self.led_g.on()
        self.led_b.off()

    def on_cyan(self):
        self.led_r.off()
        self.led_g.on()
        self.led_b.on()

    def on_purple(self):
        self.led_r.on()
        self.led_g.off()
        self.led_b.on()

    def off(self):
        self.led_r.off()
        self.led_g.off()
        self.led_b.off()

class MidiCamera(sensor):
    def __init__(self) -> None:
        self.reset()
        self.set_pixformat(self.GRAYSCALE)
        self.set_framesize(self.QQVGA)
        self.skip_frames(time = 2000)
        self.set_auto_gain(False)
        self.set_auto_whitebal(False)

    def take_snapshot(self):
        img = sensor.snapshot()

        self.blobList = img.find_blobs(WHITE, roi=ROI1, pixels_threshold=4,area_threshold=4,merge=True)
        self.blobList2= img.find_blobs(WHITE, roi=ROI2, pixels_threshold=4,area_threshold=4,merge=True)

        # region of interest coordinates
        roi1y = 0
        roi2y = 0
        roi1x = 0
        roi2x = 0

        # blob 1
        self.numBlobs = len(self.blobList)
        for blob in self.blobList:
            img.draw_rectangle(blob.rect(),color=0)
            img.draw_cross(blob.cx(), blob.cy(),color=0,size=4,thickness=1)
            if(blob.cy() < 10):
                self.numBlobs = self.numBlobs - 1
            roi1y = roi1y + blob.cy() # updating roi1 y coordinate
            roi1x = roi1x + blob.cx() # updating roi1 x coordinate

        if(len(self.blobList) > 0):
            # taking blob averages
            self.avg1x = roi1x / len(self.blobList)
            self.avg1y = roi1y / len(self.blobList)

        if(self.numBlobs >= 3):
            self.framesOfFinish += 1
        else:
            self.framesOfFinish = 0

        # blob 2
        for blob in self.blobList2:
            img.draw_rectangle(blob.rect(),color=0)
            img.draw_cross(blob.cx(), blob.cy(),color=0,size=4,thickness=1)
            roi2y = roi2y + blob.cy()
            roi2x = roi2x + blob.cx()

        if(len(self.blobList2) > 0):
            self.avg2x = roi2x / len(self.blobList2)
            self.avg2y = roi2y / len(self.blobList2)

class UltrasonicSensor():
    def __init__(self) -> None:
        self.outputPin = Pin('P8', Pin.OUT)
        self.echoPin = Pin('P9', Pin.IN)
        self.thresholdDistance = 65
    
    def echolocate(self) -> bool:
        prevEcho=0
        #checks in front 10 times
        for i in range(10):
            endflag=False

            # emits a 10 um pulse 
            self.outputPin.low()
            utime.sleep_us(2)
            self.outputPin.high()
            utime.sleep_us(10)
            self.outputPin.low()

            # listens for a bounced signal
            start = pyb.micros()
            beginTime=pyb.elapsed_micros(start)
            while(pyb.elapsed_micros(start)<100000)and(not(endflag)):
                if (prevEcho==1):
                    if(self.echoPin.value()==0):
                        elapsedTime = (pyb.elapsed_micros(start)-beginTime)/2
                        endflag=True
                prevEcho = self.echoPin.value()
            if not(endflag):
                elapsedTime = 0
            distance =.034*elapsedTime
            if (distance<=self.thresholdDistance)and(distance!=0):
                return True
            else:
                return False

class PWMMotorDriver():
    def __init__(self, channel, frequency, pin, pwm_perc) -> None:
        self.pwm_perc = pwm_perc
        self.pin = pin

        # initalizes motor timers and pwm signals
        self.timer = Timer(channel, frequency)
        self.timer_ch1 = self.timer.channel(1, Timer.PWM, pin=Pin(pin), pulse_width_percent=pwm_perc)

    def adjustPWM(self,newPWM):
        self.pwm_perc = newPWM
        self.timer_ch1 = self.timer.channel(1, Timer.PWM, pin=Pin(self.pin), pulse_width_percent=self.pwm_perc)
        
class MainDriver():
    def __init__(self, mode = 'regular', state = 'MOTION') -> None:
        self.mode = mode
        self.state = state
        self.camera = MidiCamera()
        self.lights = LEDControl()
        self.ultrasonic = UltrasonicSensor()
        self.left_motor = PWMMotorDriver(4,FREQ,"P7",self.lpwm_perc)
        self.right_moter = PWMMotorDriver(2, FREQ,"P6",self.rpwm_perc)

        self.reverse_left_motor = Pin('P5', Pin.OUT_PP)
        self.reverse_left_motor.low() #low = forward, high = reverse

    def reset_conditions(self):
        self.detection = ''
        self.left_motor.adjustPWM(0)
        self.right_moter.adjustPWM(0)
        self.delay = 0
        self.motion_count = 0
        self.maxPWM = MAXPWM
        if self.mode == "ULTRASONIC":
            self.maxPWM = int(MAXPWM/2)

    def addW(turnangle):
        result=(turnangle*WIDTH*FREQ)/WHEELRADIUS
        return result

    def mainloop(self):
        CLOCK.tick()
        LPWMperc = self.left_motor.pwm_perc
        RPWMperc = self.right_moter.pwm_perc

        #identifies if the car is stopped
        if ((LPWMperc<=0)and(RPWMperc<=0)and(self.state=="COOLDOWN")):
            self.state = "OFF"
        elif (self.camera.framesOfFinish>=MAXFINISHFRAMES):
            self.state = "FINISHLINE"

        self.camera.take_snapshot()
        if (self.delay>0):
            self.delay-=1
            if (self.delay>=0):
                self.reverse_left_motor.low()
        elif(self.state == "OFF"):
            self.lights.off()
        elif(self.state == "COOLDOWN"):
            LPWMperc-=COOLDOWNRATE
            RPWMperc-=COOLDOWNRATE
        elif(self.state == "FINISHLINE"):
            self.delay = 10
            self.state = "COOLDOWN"
            self.lights.on_white()
            straight=max([self.left_motor.pwm_perc,self.right_moter.pwm_perc])
            RPWMperc=straight
            LPWMperc=straight
        elif(self.state == "DETECT OBSTACLE"):
            if self.ultrasonic.echolocate():
                self.lights.on_purple()
                self.reverse_left_motor.high()
                RPWMperc=50
                LPWMperc=50
                self.delay=15
            else:
                self.motion_count = 25
            self.state="MOTION"
        elif(self.mode == "ULTRASONIC")and(self.motion_count<=0):
            RPWMperc=0
            LPWMperc=0
            self.lights.on_green()
            self.state = "DETECT OBSTACLE"
        elif(self.camera.numBlobs<=0):
            self.lights.on_cyan()

        #Turning
        elif(self.camera.avg1x<LFRAMELIMIT) or (self.camera.avg1x>RFRAMELIMIT) or \
            ((self.camera.avg2x-self.camera.avg1x)>MIDDLEFRAME):
            angle=math.atan(abs(self.camera.avg2x-self.camera.avg1x)/TANY)
            angleD=math.degrees(angle)
            angleS=math.atan(abs(CENTERFRAME-self.camera.avg1x)/TANY)
            subvel=MainDriver.addW(angleS)
            # turn to left turn
            if ((self.camera.avg2x<LFRAMELIMIT)and(angleD<25)) or (self.camera.avg2x>self.camera.avg1x):
                LPWMperc=self.maxPWM
                RPWMperc=(((ANGFREQ-subvel)/ANGFREQ))*self.maxPWM
                self.lights.on_blue()
            # turn right at curve
            elif (self.camera.avg2x>RFRAMELIMIT)and(angleD<25):
                RPWMperc=self.maxPWM
                LPWMperc=(((ANGFREQ-subvel)/ANGFREQ))*self.maxPWM
                self.lights.on_red()
            else:
                subvel=MainDriver.addW(angleS)
                RPWMperc=self.maxPWM
                LPWMperc=(((ANGFREQ-subvel)/ANGFREQ))*self.maxPWM
                self.lights.on_red()
        
        # Going Straight
        elif ((self.camera.avg1x>=LFRAMELIMIT)and(self.camera.avg1x<=RFRAMELIMIT)):
            LPWMperc=self.maxPWM
            RPWMperc=self.maxPWM
            self.lights.on_green()

        if(self.mode=="ULTRASONIC")and(self.motion_count>0):
            self.motion_count-=1
        self.left_motor.adjustPWM(LPWMperc)
        self.right_moter.adjustPWM(RPWMperc)

def main():
    program = MainDriver()
    while True:
        program.mainloop()

if __name__=='__main__':
    main()