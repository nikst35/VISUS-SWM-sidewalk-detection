import numpy as np
import cv2
import time
import os
import keyboard


deviceIsRP=False
LLpin=29
LEpin=31
NApin=33
DEpin=35
DDpin=37

LLengage=False
LEengage=False
NAengage=False
DEengage=False
DDengage=False


LLstate=1
LEstate=1
NAstate=1
DEstate=1
DDstate=1

print ("\n\nVISUS software rev2 2021...........See with me V3\n\nSoftware in development by Nik Stanojevic and Val Vidmar\nPress 'x' to exit program.")

try:
    import RPi.GPIO as IO
    IO.setwarnings(False)
    IO.setmode(IO.BCM)

    IO.setup(LLpin,IO.OUT)
    IO.setup(LEpin,IO.OUT)
    IO.setup(NApin,IO.OUT)
    IO.setup(DEpin,IO.OUT)
    IO.setup(DDpin,IO.OUT)
    
    pwmLL=IO.PWM(LLpin,100)#freqency in this case 100Hz
    pwmLE=IO.PWM(LEpin,100)
    pwmNA=IO.PWM(NApin,100)
    pwmDE=IO.PWM(DEpin,100)
    pwmDD=IO.PWM(DDpin,100)

    pwmLL.start(0)
    pwmLE.start(0)
    pwmNA.start(0)
    pwmDE.start(0)
    pwmDD.start(0)
    os.system('cls' if os.name == 'nt' else 'clear')#not realy necessary ce bi bil kak performance drop se lahko na izi zakomira
    print("Raspberry pi mode initialized...\nImporting libraries....")
    deviceIsRP=True

except:
    print("\nError message: Tole pa ne laufa na raspberry pi al?")
cameraDetected=False
while cameraDetected == False:
    if keyboard.is_pressed('x'):
        break

    try:
        pipeline = rs.pipeline()
        config = rs.config()
        States =  [0,0,0,0,0,0]

        LL=1#navezuje se na counter "o", 
        LE=2
        NA=3
        DE=4
        DD=5

        thr1=1
        thr2=1.6

        #..................................
        preset=1        #1,2,3
        targetFPS=6     #6,15,30,60,90 
        #..................................

        if preset==1:
            width=848
            height=480

        elif preset==2:
            width=640
            height=360

        else:
            width=480
            height=270

        OneWidth=int(width/5)
        Zones = [1, int(OneWidth), int(OneWidth * 2), int(OneWidth * 3), int(OneWidth * 4), int(width)]


        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, targetFPS)
        pipeline.start(config)
        cameraDetected=True
    except:
        os.system('cls' if os.name == 'nt' else 'clear')
        print("\nCamera not connected yet... but will try my best to find it")

startTime=0
newTime=0
timec=0
totalFPS=0
frameNum=0

testcnt=0
while True:
   
    startTime=time.time()

    shortDistance=[0,0,0,0,0]
    mediumDistance=[0,0,0,0,0]
    longDistance=[0,0,0,0,0]

    MinDist = [100,100,100,100,100,100]
    CntARR1 = [0,0,0,0,0,0]
    CntARR2 = [0,0,0,0,0,0]
    States =  [0,0,0,0,0,0]
    minLenght=100
    x=31 #number of pixels skiped in x axis
    y=23 #number of pixels skiped in y axis
    o=1
    try:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_INFERNO)#DOBRE IZBIRE SO Å E:HSV, INFERNO, BONE, TURBO

        while y < height and x < width:
            if y>300:
                thr2=1.1
            else:
                thr2=1.6
        
            while True:
                if x < (OneWidth * o):
                    Zones[0] = o
                    if CntARR1[o] > 10:
                        if Zones[0] != 5:
                            x += OneWidth
                            Zones[0] += 1
                    break
                o += 1
            o = 1
            dist_to_center = depth_frame.get_distance(x,y)
            if dist_to_center == 0:
                dist_to_center=69

            if dist_to_center < minLenght:
                minLenght=dist_to_center
            if x<=(Zones[0]*OneWidth):
                if MinDist[Zones[0]]>dist_to_center:
                    MinDist[Zones[0]]=dist_to_center
                if dist_to_center <= thr1:
                    CntARR1[Zones[0]] += 1
                if dist_to_center > thr1 and dist_to_center <= thr2:
                    CntARR2[Zones[0]] += 1
            Zones[0] = 1
            if dist_to_center <= thr1:
                cv2.rectangle(depth_colormap1, (x - 2, y - 2), (x + 2, y + 2), (0, 0, 255), 2)
            elif dist_to_center > thr1 and dist_to_center <= thr2:
                cv2.rectangle(depth_colormap1, (x - 2, y - 2), (x + 2, y + 2), (255, 0, 0), 2)
            else:
                cv2.rectangle(depth_colormap1, (x - 2, y - 2), (x + 2, y + 2), (0, 0, 0), 2)

            x += 32
            if x >= width:
                x = 31
                y += 24
        

        while o <= 5:
            if CntARR1[o]>10:
                States[o] = 1
                shortDistance[o-1]=1
                cv2.rectangle(depth_colormap1, ((Zones[o-1]), 220), (Zones[o], 260), (0, 0, 255), 5)
                if o==1:
                    LLengage=True
                    LLstate=3   #3 pomeni zelo bliz, 2 sredne in 1 nobene ovire
                    if deviceIsRP:
                        pwmLL.start(100)
                if o==2:
                    LEengage=True
                    LEstate=3
                    if deviceIsRP:
                        pwmLE.start(100)
                if o==3:
                    NAengage=True
                    NAstate=3
                    if deviceIsRP:
                        pwmNA.start(100)
                    if deviceIsRP:
                        pwmNA.start(100)
                if o==4:
                    DEengage=True
                    DEstate=3
                    if deviceIsRP:
                        pwmDE.start(100)
                if o==5:
                    DDengage=True
                    DDstate=3
                    if deviceIsRP:
                        pwmDD.start(100)

            elif CntARR2[o]>10:
                States[o] = 1.1
                mediumDistance[o-1]=1
                cv2.rectangle(depth_colormap1, ((Zones[o-1]), 220), (Zones[o], 260), (255, 0, 0), 5)
                if o==1:
                    LLengage=True
                    LLstate=2   #3 pomeni zelo bliz, 2 sredne in 1 nobene ovire
                    if deviceIsRP:
                        pwmLL.start(50)     #MOGOC BI MOGLO BIT NAMEST '.start(50)' napisano '.ChangeDutyCycle(50)'
                if o==2:
                    LEengage=True
                    LEstate=2
                    if deviceIsRP:
                        pwmLE.start(50)
                if o==3:
                    NAengage=True
                    NAstate=2
                    if deviceIsRP:
                        pwmNA.start(50)
                if o==4:
                    DEengage=True
                    DEstate=2
                    if deviceIsRP:
                        pwmDE.start(50)
                if o==5:
                    DDengage=True
                    DDstate=2
                    if deviceIsRP:
                        pwmDD.start(50)
            else:
                longDistance[o-1]=1
                cv2.rectangle(depth_colormap1, ((Zones[o-1]), 220), (Zones[o], 260), (0, 0, 0), 5)
                States[o] = 0
                if o==1:
                    LLengage=False
                    LLstate=1   #3 pomeni zelo bliz, 2 sredne in 1 nobene ovire
                    if deviceIsRP: #RP output   to se da prestav izven tega if statmenta da pol prhrans par mikrosekund sm zdej sm tko pustu da se lep vid kaj je output
                        pwmLL.start(0)
                if o==2:
                    LEengage=False
                    LEstate=1
                    if deviceIsRP:
                        pwmLE.start(0)
                if o==3:
                    NAengage=False
                    NAstate=1
                    if deviceIsRP:
                        pwmNA.start(0)
                if o==4:
                    DEengage=False
                    DEstate=1
                    if deviceIsRP:
                        pwmDE.start(0)
                if o==5:
                    DDengage=False
                    DDstate=1
                    if deviceIsRP:
                        pwmDD.start(0)
            o+=1
        
        #imageBig = cv2.resize(depth_colormap1, (0, 0), fx=1.7, fy=1.7)
        #cv2.imshow('RealSense1', imageBig)
        if keyboard.is_pressed('x'):
            print("Averge FPS: ", round(totalFPS/frameNum,2),"\nSequence closed successfully!" )
            break
        cv2.imshow('RealSense1', depth_colormap1)
        cv2.waitKey(1)
        newTime=time.time()
        timec=newTime-startTime
        fps=1/timec
        frameNum+=1
        totalFPS+=fps
        if targetFPS>(fps+1):
            print("Low FPS warning!!!")

        os.system('cls' if os.name == 'nt' else 'clear')#not realy necessary ce bi bil kak performance drop se lahko na izi zakomira
        print("------------------------")
        print("Long:   ",longDistance)
        print("Medium: ",mediumDistance)
        print("Short:  ",shortDistance)
        print("RP out: ",LLstate,LEstate,NAstate,DEstate,DDstate)
        print("FPS: ",round(fps,2))

    except:
        shortDistance=[1,1,1,1,1]
        mediumDistance=[0,0,0,0,0]
        longDistance=[0,0,0,0,0]
        os.system('cls' if os.name == 'nt' else 'clear')#not realy necessary ce bi bil kak performance drop se lahko na izi zakomira
        print("Long:   ",longDistance)
        print("Medium: ",mediumDistance)
        print("Short:  ",shortDistance)
        print("Camera not connected!!!")
        if keyboard.is_pressed('x'):
            break
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, targetFPS)
            pipeline.start(config)
        except:
            print("Detecting camera...")
