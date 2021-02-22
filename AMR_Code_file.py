import RPi.GPIO as GPIO
import time
import cv2

import numpy as np
#from adafruit_servokit import ServoKit
#hardware work
mask_1=0
mask_2=0
GPIO.setmode(GPIO.BOARD)


GPIO_TRIGGER2 = 23   #Front ultrasonic sensor
GPIO_ECHO2 = 29
key=21               #Button
buz=16               #buzzer
MOTOR1B=33           #Left Motor
MOTOR1E=31

MOTOR2B=35  #Right Motor
MOTOR2E=37
#kit = ServoKit(channels=8)
 #If it finds the ball, then it will light up the led

# Set pins as output and input

GPIO.setup(GPIO_TRIGGER2,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO2,GPIO.IN)
GPIO.setup(key,GPIO.IN, pull_up_down=GPIO.PUD_UP)
aa=0
yy=0
kk=0
# Set trigger to False (Low)

GPIO.output(GPIO_TRIGGER2, False)


# Allow module to settle
GPIO.setup(buz,GPIO.OUT) 

GPIO.setup(MOTOR1B, GPIO.OUT)
GPIO.setup(MOTOR1E, GPIO.OUT)

GPIO.setup(MOTOR2B, GPIO.OUT)
GPIO.setup(MOTOR2E, GPIO.OUT)

def reverse():
     
      GPIO.output(MOTOR1B, GPIO.HIGH)
      GPIO.output(MOTOR1E, GPIO.LOW)
      GPIO.output(MOTOR2B, GPIO.HIGH)
      GPIO.output(MOTOR2E, GPIO.LOW)
      #print("forward")
     
def forward():
      GPIO.output(MOTOR1B, GPIO.LOW)
      GPIO.output(MOTOR1E, GPIO.HIGH)
      GPIO.output(MOTOR2B, GPIO.LOW)
      GPIO.output(MOTOR2E, GPIO.HIGH)
      #print("revrse")
def rightturn():
    

      GPIO.output(MOTOR1B,GPIO.HIGH)
      GPIO.output(MOTOR1E,GPIO.LOW)
      GPIO.output(MOTOR2B,GPIO.LOW)
      GPIO.output(MOTOR2E,GPIO.HIGH)
      #time.sleep(0.1)
      #print("right")
     
def leftturn():
     

      GPIO.output(MOTOR1B,GPIO.LOW)
      GPIO.output(MOTOR1E,GPIO.HIGH)
      GPIO.output(MOTOR2B,GPIO.HIGH)
      GPIO.output(MOTOR2E,GPIO.LOW)
      #print("left ")
      #time.sleep(0.1)
def stop():
      GPIO.output(MOTOR1E,GPIO.LOW)
      GPIO.output(MOTOR1B,GPIO.LOW)
      GPIO.output(MOTOR2E,GPIO.LOW)
      GPIO.output(MOTOR2B,GPIO.LOW)
      #print("stop")

def sonar(GPIO_TRIGGER,GPIO_ECHO):
      start=0
      stop=0
      # Set pins as output and input
      GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
      GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo
     
      # Set trigger to False (Low)
      GPIO.output(GPIO_TRIGGER, False)
     
      # Allow module to settle
      time.sleep(0.01)
           
      #while distance > 5:
      #Send 10us pulse to trigger
      GPIO.output(GPIO_TRIGGER, True)
      time.sleep(0.00001)
      GPIO.output(GPIO_TRIGGER, False)
      begin = time.time()
      while GPIO.input(GPIO_ECHO)==0 and time.time()<begin+0.05:
            start = time.time()
     
      while GPIO.input(GPIO_ECHO)==1 and time.time()<begin+0.1:
            stop = time.time()
     
      # Calculate pulse length
      elapsed = stop-start
      # Distance pulse travelled in that time is time
      # multiplied by the speed of sound (cm/s)
      distance = elapsed * 34000
     
      # That was the distance there and back so halve the value
      distance = distance / 2
     
      #print ("Distance : %.1f" % distance)
      # Reset GPIO settings
      return distance
     
#Image analysis work
def segment_colour(frame):    #returns only the red colors in the frame
    hsv_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_1=0
    mask_2=0
    if(aa==0):
          mask_1 = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255])) # red colour
          
    if(aa==1):
          mask_1 = cv2.inRange(hsv_roi, np.array([20, 50,100]), np.array([42,255,255]))  # yellow colour
          
    if(aa==2):
          mask_1 = cv2.inRange(hsv_roi, np.array([44, 54,63]), np.array([90,255,255])) # green colour
          
    if(aa==3):      
          mask_1 = cv2.inRange(hsv_roi, np.array([110, 130,50]), np.array([130,255,255])) #blue colour
          
    if(aa==4):
          
          mask_1 = cv2.inRange(hsv_roi, np.array([10, 100,20]), np.array([25,255,255]))   # orange clour
          
    if(aa==5):
          
          mask_1 = cv2.inRange(hsv_roi, np.array([0, 100,0]), np.array([20,255,255]))  # brown
         
    


    ycr_roi=cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb)
    if(yy==0):
          mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,0.)), np.array((255.,255.,255.))) #red color

    if(yy==1):
          mask_2=cv2.inRange(ycr_roi, np.array((0.,0.,165.)), np.array((255.,255.,255.))) #yellow and blue color
    if(yy==2):
          mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,165.)), np.array((255.,255.,255.))) # green ,brown and orange colour
    
    mask = mask_1 | mask_2
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode)      #Eroding
    mask=cv2.dilate(mask,kern_dilate)     #Dilating
    cv2.imshow('mask',mask)
    return mask


def find_blob(blob): #returns the red colored circle
    largest_contour=0
    cont_index=0
    _,contours, hierarchy = cv2.findContours(blob,cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area >largest_contour) :
            largest_contour=area
           
            cont_index=idx
            #if res>15 and res<18:
            #    cont_index=idx
                              
    r=(0,0,2,2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])
       
    return r,largest_contour

def target_hist(frame):
    hsv_img=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
    hist=cv2.calcHist([hsv_img],[0],None,[50],[0,255])
    return hist

#CAMERA CAPTURE
#initialize the camera and grab a reference to the raw camera capture

 
# allow the camera to warmup
time.sleep(0.001)
cap = cv2.VideoCapture(0)
cap.set(3,128)
cap.set(4,128) 
# capture frames from the camera
se=0
while 1:
      #grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
      k1 = GPIO.input(21)
      if(k1==0  and kk==0):
            aa=1
            kk=1
            k1=1
            yy=1
            se=0
            print("yellow color")
            
            reverse()
            time.sleep(2)
            stop()
      if(k1==0  and kk==1):
            aa=2
            kk=2
            k1=1
            yy=2
            se=0
            print("green color")
            reverse()
            time.sleep(2)
            stop()
      if(k1==0  and kk==2):
            aa=3
            kk=3
            k1=1
            yy=1
            se=0
            print("blue color")
            reverse()
            time.sleep(2)
            stop()
      if(k1==0  and kk==3):
            aa=0
            kk=0
            k1=1
            yy=0
            se=0
            print("red color")
            reverse()
            time.sleep(2)
            stop()
            
      _,frame = cap.read()
      global centre_x
      global centre_y
      centre_x=0.
      centre_y=0.
      hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      if (se==0):
            mask_red=segment_colour(frame)      #masking red the frame
            loct,area=find_blob(mask_red)
            x,y,w,h=loct
      distanceC = sonar(GPIO_TRIGGER2,GPIO_ECHO2)
      print( distanceC) 
      
      if (w*h) < 10:
            found=0
      else:
            found=1
            simg2 = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
            centre_x=x+((w)/2)
            centre_y=y+((h)/2)
            cv2.circle(frame,(int(centre_x),int(centre_y)),3,(0,110,255),-1)
            centre_x-=80
            centre_y=6--centre_y
            #print (area)
            #print (centre_x,centre_y)
      initial=400
      flag=0
               
      if(found==0):
            #if the ball is not found and the last time it sees ball in which direction, it will start to rotate in that direction
            if flag==0:
                  rightturn()
                  time.sleep(0.1)
                  print("right")
            else:
                  leftturn()
                  time.sleep(0.1)
                  print(" left")
            stop()
            time.sleep(0.125)
            print("stop1")
     
      elif(found==1):
            if( area<initial):
                  if (distanceC>25):
                 
                        forward()
                        time.sleep(0.0625)
                        print("forward")
                        GPIO.output(buz,GPIO.LOW)
                  else:
                              stop()
                              time.sleep(0.0625)
                              print("object detected")
                              GPIO.output(buz,GPIO.HIGH)
            elif(area>=initial):
                  initial2=6700
                  if(area<initial2):
                        if (distanceC>25):
                        
                              #it brings coordinates of ball to center of camera's imaginary axis.
                              if(centre_x<=-20 or centre_x>=20):
                                    if(centre_x<0):
                                          flag=0
                                          leftturn()
                                          time.sleep(0.025)
                                          print("object left")
                                    elif(centre_x>0):
                                          flag=1
                                          rightturn()
                                          time.sleep(0.025)
                                          print("object right")
                              forward()
                              time.sleep(0.025)
                              stop()
                              time.sleep(0.0625)
                              print("object near")
                              GPIO.output(buz,GPIO.LOW)
                        else:
                              stop()
                              time.sleep(0.0625)
                              print("object detected")
                              GPIO.output(buz,GPIO.HIGH)

                  else:
                        #if it founds the ball and it is too close it lights up the led.
                        print("object close stop")
                        aa=7
                        yy=7
                        se=1
                        time.sleep(0.1)
                        stop()
                        time.sleep(0.1)
      cv2.imshow("draw",frame)    
      #rawCapture.truncate(0)  # clear the stream in preparation for the next frame
         
      if(cv2.waitKey(1) & 0xff == ord('q')):
            break

GPIO.cleanup() #free all the GPIO pins
