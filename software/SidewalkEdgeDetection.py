import matplotlib.pylab as plt
import cv2
import numpy as np

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices,255)
    masked_image = cv2.bitwise_and(img,mask)
    return masked_image

frameNumber=0
vsiRobovi=0
prviFilter=0
drugiFilter=0
tretjiFilter=0

leviRoboviCounter=0
desniRoboviCounter=0

vrednostiKL=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#ROI=[(0,550),(380,250),(640,400),(900,250),(1280,550),(1280,720),(0,720)]
ROI=[(0,720),(0,450),(200,250),(540,400),(540,700),(740,700),(740,400),(1080,250),(1280,450),(1280,720)]

#ROI=[(0,0),(1280,0),(1280,720),(0,720)]

#cap = cv2.VideoCapture('C:\\Users\\nikst\\PycharmProjects\\OpencvPython\\Resourcces/doberTest.mp4')
#cap = cv2.VideoCapture('http://192.168.178.21:8080/video')
cap = cv2.VideoCapture('C:\\Users\\nikst\\PycharmProjects\\OpencvPython\\Resourcces/valovi2.mp4')
oldlx1=0
oldlx2=0
oldly1=0
oldly2=0

olddx1=0
olddx2=0
olddy1=0
olddy2=0

cnt=0
cuntl=0
cuntd=0
smallpp=0
smaddpp=1280

povl=0
cl=0 #counter za array levi
ui =0
j=0

spodnlX=0
zgornlX=0
spodndX=0
zgorndX=0

while (cap.isOpened()):

    ret, img = cap.read()
    negro=img.copy()
    frameNumber+=1
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    blurFrame = cv2.bilateralFilter(gray, 9, 200, 200)
    #blurGausian = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurFrame,65,65,apertureSize=3)

    roiIMG=region_of_interest(edges,np.array([ROI],np.int32))

    lines = cv2.HoughLinesP(roiIMG,1,np.pi/180,45,lines=np.array([]),minLineLength=40,maxLineGap=100)

    leftState = False
    rightState = False

    if lines is not None:
        vsiRobovi+=1
        for line in lines:
            x1, y1, x2, y2 = line[0]


            if abs(y1-y2) > 200 and abs(x1-x2) > 120:
                prviFilter+=1

                if x1<640 and x2<640 and x1<x2 and y2<y1 and leftState==False:
                    #cv2.putText(img,"LEV ROB",(pos1,pos2),0,1.3,(255,255,255),3)
                    kl = round(((x1 + x2) / 2) / ((y2 + 720) / 2), 4)
                    if cl ==20:
                        cl=0
                        j=0
                        #print(vrednostiKL)
                    j=j+kl
                    kl = vrednostiKL[cl]
                    povl = round(j/20,4)
                    #print("kl: ",kl,"\npovl: ",povl,"\nkl-povl: ",abs(kl-povl))939 1176
                    if abs(kl-povl) <1:

                        spodnlX = int(x1 - (720 - y1) / ((y2 - y1) / (x1 - x2)))
                        zgornlX = int(x2 - (300 - y2) / ((y2 - y1) / (x1 - x2)))
                        cv2.line(img, (spodnlX, 720), (zgornlX, 300), (0, 0, 0), 10)
                        leftState = True
                        oldlx1=x1
                        oldlx2=x2
                        oldly1=y1
                        oldly2=y2
                        smallpp = int((x1+x2)/2)
                        cuntl+=1
                        vrednostiKL[cl]=kl
                        leviRoboviCounter+=1
                    cl+=1
                    if cuntl >= 20:
                        cuntl=0
                        smallpp=0


#fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
                elif x2>640 and x1>640 and x1<x2 and y1<y2 and rightState==False:
                    zgorndX = int(x1 - (300 - y1) / ((y2 - y1) / (x1 - x2)))
                    spodndX = int(x2 - (720 - y2) / ((y2 - y1) / (x1 - x2)))

                    cv2.line(img, (zgorndX, 300), (spodndX, 720), (0, 0, 0), 10)
                    rightState = True
                    olddx1=x1
                    olddx2=x2
                    olddy1=y1
                    olddy2=y2
                    smaddpp = int((x1+x2)/2)
                    cuntd += 1
                    desniRoboviCounter+=1

                    if cuntd >= 20:
                        cuntd=0
                        smaddpp=1280

        if leftState == False and rightState == False:
            cv2.putText(img, "NI ZAZNANIH ROBOV", (600, 150), 0, 1.5, (0, 0, 0), 5)
            #cv2.line(img, (spodnlX, 720), (zgornlX, 300), (255, 0, 0), 20)
            #cv2.line(img, (zgorndX, 300), (spodndX, 720), (0, 255, 0), 20)

            cnt+=1
        elif leftState == True and rightState == False:
            cv2.putText(img, "LEV ROB JE ZAZNAN", (600, 150), 0, 1.5, (0, 0, 0), 3)
            #cv2.line(img, (zgorndX, 300), (spodndX, 720), (0, 255, 0), 20)
            # cv2.putText(img, "PREDVIDEN DESEN ROB", (pos1, pos2), 0, 1.3, (0, 0, 255), 3)
            cnt += 1
        elif leftState == False and rightState == True:
            cv2.putText(img, "DESEN ROB JE ZAZNAN", (600, 150), 0, 1.5, (0, 0, 0), 3)
            #cv2.line(img, (spodnlX, 720), (zgornlX, 300), (255, 0, 0), 20)
            #cv2.putText(img, "PREDVIDEN LEV ROB", (pos1, pos2), 0, 1.3, (0, 0, 255), 3)
            cnt+=1

        else:
            cv2.putText(img, "OBA ROBOVA ZAZNANA", (600, 150), 0, 1.5, (0, 0, 0), 3)
            cnt=0

    if cnt >= 50:
        oldlx1 = 0
        oldlx2 = 0
        oldly1 = 0
        oldly2 = 0
        olddx1 = 0
        olddx2 = 0
        olddy1 = 0
        olddy2 = 0
        cnt=0

    kordinate = [(zgornlX, 300), (zgorndX, 300), (spodndX, 720), (spodnlX, 720)]
    cv2.fillPoly(negro, (np.array([kordinate],np.int32)),(26,26,66),)
    cv2.line(img, (zgornlX, 300), (zgorndX, 300), (0, 0, 0), 10)

    image_new = cv2.addWeighted(negro, 0.8, img, 1 - 0.8, 0)
    cv2.imshow('image',image_new)
    cv2.imshow('image2',edges)



    if cv2.waitKey(1) & 0xFF == ord('x') or frameNumber>=1620:
        break


print("stevilo vseh levih robov : ",vsiRobovi)
print("prvi filter : ",drugiFilter)
print("stevilo pravilno zaznanih levih robov ",leviRoboviCounter)

cap.release()
cv2.destroyAllWindows()

'''rezultati testov: num1(threshoold == 40)
stevilo vseh frejmov:  1625
stevilo najdenih levih robov :  1140
stevilo najdenih desih robov :  1206

num2(threshoold==20)  no improvement!!!!!!
stevilo vseh frejmov:  1626
stevilo najdenih levih robov :  1109
stevilo najdenih desih robov :  1265

num3: (threshoold == 60) no improvement!!!!!!
stevilo vseh frejmov:  1623
stevilo najdenih levih robov :  1043
stevilo najdenih desih robov :  1189

num3:(threshoold == 45)


'''
