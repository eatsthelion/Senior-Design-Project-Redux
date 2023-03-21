#DriverCodeV14 Comepleted Pass.py
#Driver Code- Andrea Anaya, Ann Ryan, Ethan de Leon, Caitlin Pasqualino
import sensor, image, time, pyb, math, utime
from pyb import Pin, Timer

white = [(210,255)]
roi1=(0,40,160,10) #far
roi2=(0,80,160,10) #close

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

multi=1
multi2=1

leftDetected = False
rightDetected = False
finishLineDetected = False

maxPWM = 80
xcenter = 80 #center of frame coord

#controllable initial variables
LPWMperc=0          #Right Motor Intial Speed: 0 rad/s
RPWMperc=0          #Right Motor Intial Speed: 0 rad/s
Framesize=160
cooldown=False      #cooldown: OFF
CooldownRate=2
delay=True          #starting delay: ON
delayCount=50       #delaytime(in ms) = delayCount/freq
tanY=30

#ultrasound variables
ultrasound=True
motionCount=0
dataGather=False
threshDis=65

#measurements
wheelRadius=3.5 # cm
width=14.5      # cm
freq=4000       # hz of the clock of the car that we sending the PWM
angFreq=freq*2*math.pi # turning angle

# splits frame into 3 sections
thirdFrame=int(Framesize/3)
LFrameLim=thirdFrame
RFrameLim=Framesize-thirdFrame

# LED Settings
led_red=pyb.LED(1)
led_green=pyb.LED(2)
led_blue=pyb.LED(3)
led_red.on()
led_blue.on()
led_green.on()

#Motor Timers and Pins
tim4 = Timer(4, freq=freq) #Left motor
tim4_ch1 = tim4.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=LPWMperc)

tim2 = Timer(2, freq=freq) #Right motor
tim2_ch1 = tim2.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width_percent=RPWMperc)

#Ultrasonic Pins
output = Pin('P8', Pin.OUT)
echoPin = Pin('P9', Pin.IN)


#Motor Direction Control
reverseL = Pin('P5', Pin.OUT_PP)
reverseL.low()

#average blob coordinates
avg1x = 0
avg1y = 0
avg2x = 0
avg2y = 0

framesOfFinish = 0
maxFramesOfFinish = 3

# calculates turning angle
def addW(turnangle):
    result=(turnangle*width*freq)/wheelRadius
    return result

while(True):
    clock.tick()
    img = sensor.snapshot()

    blobList = img.find_blobs(white, roi=roi1, pixels_threshold=4,area_threshold=4,merge=True)
    blobList2= img.find_blobs(white, roi=roi2, pixels_threshold=4,area_threshold=4,merge=True)

    # region of interest coordinates
    roi1y = 0
    roi2y = 0
    roi1x = 0
    roi2x = 0

    # blob 1
    numBlobs = len(blobList)
    for blob in blobList:
        img.draw_rectangle(blob.rect(),color=0)
        img.draw_cross(blob.cx(), blob.cy(),color=0,size=4,thickness=1)
        if(blob.cy() < 10):
            numBlobs = numBlobs - 1
        roi1y = roi1y + blob.cy() # updating roi1 y coordinate
        roi1x = roi1x + blob.cx() # updating roi1 x coordinate

    if(len(blobList) > 0):
        # taking blob averages
        avg1x = roi1x / len(blobList)
        avg1y = roi1y / len(blobList)

    if(numBlobs >= 3):
        framesOfFinish += 1
    else:
        framesOfFinish = 0

    # blob 2
    for blob in blobList2:
        img.draw_rectangle(blob.rect(),color=0)
        img.draw_cross(blob.cx(), blob.cy(),color=0,size=4,thickness=1)
        roi2y = roi2y + blob.cy()
        roi2x = roi2x + blob.cx()

    if(len(blobList2) > 0):
        avg2x = roi2x / len(blobList2)
        avg2y = roi2y / len(blobList2)

    # motor status
    LOff=(LPWMperc <= 0)
    ROff=(RPWMperc <= 0)
    Off=(LOff and ROff) and cooldown


    if(delay):
        delayCount=delayCount-1
        print("In delay:",delayCount)
        print("")
        print("")
        print("")
        print("")
        print("")
        if (delayCount<=0)and(ultrasound): #when in ultrasound state
            if (len(blobList)==0):
                delayCount=1
            else:
                reverseL.low() #sets the left motor back to forward
        delay=(delayCount>0)

    elif(Off):
        led_red.off()
        led_blue.off()      #no LED
        led_green.off()

        print("Off")

    elif(cooldown):
        LPWMperc=LPWMperc-CooldownRate
        RPWMperc=RPWMperc-CooldownRate

        print("Cooldown in progress...")

    #finish line
    elif ((numBlobs >=3)and(framesOfFinish >= maxFramesOfFinish)):
        cooldown= True
        delay=True
        delayCount=10
        straight=max([RPWMperc,LPWMperc])
        RPWMperc=straight
        LPWMperc=straight
        led_blue.on()
        led_red.on()        #white LED
        led_green.on()

        print("Finish Line Detected")

    elif(dataGather):
        prevEcho=0
        obstacle=False
        for i in range(10):
            endflag=False
            output.low()
            utime.sleep_us(2)
            output.high()
            utime.sleep_us(10)
            output.low()
            start = pyb.micros()
            beginTime=pyb.elapsed_micros(start)
            while(pyb.elapsed_micros(start)<100000)and(not(endflag)):
                if (prevEcho==1):
                    if(echoPin.value()==0):
                        elapsedTime=(pyb.elapsed_micros(start)-beginTime)/2
                        endflag=True
                prevEcho=echoPin.value()
            if not(endflag):
                elapsedTime=0
            distance=.034*elapsedTime
            if (distance<=threshDis)and(distance!=0):
                obstacle=True

        if(obstacle):
            reverseL.high() #reverses the left motor
            RPWMperc=50
            LPWMperc=50
            delay=True
            delayCount=15
            print("Obstacle Ahead")
            print("")
            led_red.on()
            led_blue.on()      #Purple LED
            led_green.off()
        else:
            RPWMperc=maxPWM
            LPWMperc=maxPWM
            motionCount=25
        dataGather=False

    #when in ultrasound state and car is not moving
    elif((ultrasound)and(motionCount<=0)):
        maxPWM=50
        RPWMperc=0
        LPWMperc=0
        led_red.on()
        led_blue.on()      #white LED
        led_green.on()
        dataGather=True
        print("Gathering Data...")
        print("")

    #no line detected
    elif (numBlobs <= 0):
        #HOLDING INSTEAD OF GOING SLOW STRAIGHT
        led_red.off()
        led_blue.on()      #aqua LED
        led_green.on()

        print("Track Not Found")

    #turning
    elif ((avg1x<LFrameLim) or (avg1x > RFrameLim)) or ((avg2x-avg1x) > thirdFrame):
        angle=math.atan(abs(avg2x-avg1x)/tanY) #changed from 30 to 40 because too sharp
        angleD=math.degrees(angle)
        angleS=math.atan(abs(xcenter-avg1x)/tanY)

        #left turn with straight track on left side
        if (avg2x<LFrameLim)and(angleD<25):
            subvel=addW(angleS)
            LPWMperc=maxPWM
            RPWMperc=(((angFreq-subvel)/angFreq))*maxPWM
            led_blue.on()
            led_red.off()       #blue LED
            led_green.off()

            print("Track on Left")
            print("Angle in Degrees:", angleD)
            print("x1:",avg1x,"x2:",avg2x,"y1:",avg1y,"y2:",avg2y)
            print("subvel:",subvel)
            print("")

        #right turn with straight track on right side
        elif (avg1x > RFrameLim)and(angleD<25):
            subvel=addW(angleS)
            RPWMperc=maxPWM
            LPWMperc=(((angFreq-subvel)/angFreq))*maxPWM
            led_red.on()
            led_blue.off()      #red LED
            led_green.off()

            print("Track on Right")
            print("Angle in Degrees:", angleD)
            print("x1:",avg1x,"x2:",avg2x,"y1:",avg1y,"y2:",avg2y)
            print("subvel:",subvel)
            print("")

        #left turn at curve
        elif(avg2x>avg1x):
            subvel=addW(angle)
            LPWMperc=maxPWM
            RPWMperc=(((angFreq-subvel)/angFreq))*maxPWM
            led_blue.on()
            led_red.off()       #blue LED
            led_green.off()

            print("Track on Left")
            print("Angle in Degrees:", angleD)
            print("x1:",avg1x,"x2:",avg2x,"y1:",avg1y,"y2:",avg2y)
            print("subvel:",subvel)
            print("")

        #turn right at curve
        else:
            subvel=addW(angle)
            RPWMperc=maxPWM
            LPWMperc=(((angFreq-subvel)/angFreq))*maxPWM
            led_red.on()
            led_blue.off()      #red LED
            led_green.off()

            print("Track on Right")
            print("Angle in Degrees:", angleD)
            print("x1:",avg1x,"x2:",avg2x,"y1:",avg1y,"y2:",avg2y)
            print("subvel:",subvel)
            print("")

    #straight
    elif ((avg1x>=LFrameLim)and(avg1x<=RFrameLim)):
        LPWMperc=maxPWM
        RPWMperc=maxPWM
        led_green.on()
        led_red.off()       #green LED
        led_blue.off()

        print("Track in Center")

    if(ultrasound)and(motionCount>0):
        motionCount=motionCount-1

    #updates speeds
    tim4_ch1 = tim4.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent = LPWMperc)
    tim2_ch1 = tim2.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width_percent = RPWMperc)

    #Prints every frame
    print("Right Motor duty cycle = ", tim4_ch1.pulse_width_percent())
    print("Left Motor duty cycle = ", tim2_ch1.pulse_width_percent())
    print("FPS:",clock.fps())
    print("motionCount:",motionCount)
    print("ultrasound:",ultrasound)
    print("")
