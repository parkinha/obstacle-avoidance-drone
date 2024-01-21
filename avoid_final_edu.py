from djitellopy import tello
import cv2
import numpy as np
import math
import time
from matplotlib import pyplot as plt

Tello = tello.Tello()
Tello.connect()
print("start battery", Tello.get_battery())
Tello.streamon()
Tello.takeoff()
Tello.move_up(42)

xx = 0 # 드론 실시간 x좌표 - 출발지점
yy = 0 # 드론 실시간 y좌표 - 출발지점
points_x = [] # 드론의 이동 경로 저장 배열
points_y = []
points_xx = []
points_yy = []
wt, ht = 720, 480 # 화면 픽셀수
disrange=[250,300] # 물체와 유지되는 거리
tt = 0 # 이전 루프 시간
target_point = [[0,100],[100,100],[100,0],[0,0]] # 목표지점 단위 : cm, 여러개 지정 가능
breakk = 0 # 루프 탈출 변수
loop_count = 0 #루프 돈 횟수
t_loop = 0 # 몇바퀴 돌았나?
#그래프 최소, 최대값 정해주기
x_target = [i[0] for i in target_point]
y_target = [i[1] for i in target_point]
x_min = min(x_target) - 100
x_max = max(x_target) + 100
y_min = min(y_target) - 100
y_max = max(y_target) + 100

tim2 = 0 # 오류 방지(맨 처음 실행 시 tim2값 지정 안된 상태)
loop_max = len(target_point)

#객체 검출 함수
def findFace():
    global loc, aa, dia, count, fail
    img2 = Tello.get_frame_read().frame
    img2 = cv2.resize(img2,(720,480))
    faceCascade = cv2.CascadeClassifier("Resources/haarcascade_fullbody.xml")
    imgGray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.03, 11, minSize=(30,30))

    myfaceListC = []
    myfaceListArea = []
    bodyy = []
    for (x, y, w, h) in faces:
        cv2.rectangle(img2, (x,y),(x+w,y+h),(0,0,255),2)
        cx = x+w//2
        cy = y+h//2
        area = w*h
        cv2.circle(img2,(cx,cy),5,(0,255,0),cv2.FILLED)
        myfaceListArea.append(area)
        myfaceListC.append([cx,cy])
        bodyy.append([y,h])

    if len(myfaceListArea) != 0:
        i = myfaceListArea.index(max(myfaceListArea))
        loc = myfaceListC[i] # 인식된 물체 중심 좌표(가장 가까운 것)
        a = bodyy[i]
        aa = 480 - (a[1]+a[0])+58 #발위치 픽셀값 58은 보정값
    else:
        aa = 0
    return img2

#드론의 현재 좌표값 구하고 그래프 그리는 함수
def locate():
    global xx, yy, current_yaw, sho, tim2, t_loop
    x_speed = Tello.get_speed_x()
    y_speed = Tello.get_speed_y()
    current_yaw = Tello.get_yaw()
    tim1 = time.time()
    if tim2 == 0: # 오류 방지(맨 처음 실행 시 tim2값 지정 안된 상태)
        timm = 0
    else:
        timm = tim1 - tim2
    tim2 = tim1
    yy -= 6*timm*x_speed
    xx -= 6*timm*y_speed
    
    if t_loop == 0:
        points_x.append(xx)
        points_y.append(yy)
        sho = plt.scatter(points_x, points_y,c='red')
    else:
        points_xx.append(xx)
        points_yy.append(yy)
        sho = plt.scatter(points_x, points_y,c='red') # 실시간 좌표 점 찍기
        sho = plt.scatter(points_xx, points_yy,c='green')
        
    #그래프 최대, 최소값 지정
    plt.xlim([x_min,x_max])
    plt.ylim([y_min,y_max])
    plt.pause(0.00001)

# 비행 함수
def flyy():
    global dia, aa, breakk, current_yaw, loc, xx, yy, loop_count, t_loop

    #루프 돌아야할 횟수
    #항상 목표지점을 바라본다
    target_yaw = math.degrees(math.atan2(target_point[loop_count][0]-xx,target_point[loop_count][1]-yy)) # 목표지점을 바라보기 위한 각도

    speed_yaw = (target_yaw - current_yaw)*2 # 회전각도 조절
    # if speed_yaw > 180:
    #     speed_yaw = (current_yaw-360-target_yaw)*2
    # if speed_yaw < -180:
    #     speed_yaw = (current_yaw+360-target_yaw)*2

    speed_yaw = int(np.clip(speed_yaw,-90,90))
    if t_loop == 1: # 첫 바퀴 돌고 두번째 바퀴 - 장애물 피함

        if aa != 0: # 인식된 물체가 있는 경우 거리를 계산하고 계속해서 함수 진행
            dia = (aa * (2.3902) - 27) # 발 위치 - 직선거리 계산식
            d = 0
        else: # 인식된 물체가 없는경우 목표지점만 바라보고 직진(case 3)
            speed_forward = 20
            speed_right = 0
            d = 1
        if d == 0:
            if dia < disrange[0] and (loc[0]>wt*1/3 and loc[0] < wt*2/3): # case 1 : 물체와의 거리가 매우 가깝고 이동경로 내에 있는 경우
                speed_forward = -20
                speed_right = int((wt/2 - loc[0]))
            if dia < disrange[1] and (loc[0]>wt*1/3 and loc[0] < wt*2/3): # case 2 :물체와의 거리가 가깝고 이동경로 내에 있는 경우
                speed_forward = 10
                speed_right = int((wt/2 - loc[0]))
            else: # case 3 : 이동경로 내에 장애물이 없는 경우
                speed_forward = 20
                speed_right = 0
    else: # 첫 바퀴 돌때 - 장애물 피하지 않음
        speed_right = 0
        speed_forward = 20

    Tello.send_rc_control(speed_right, speed_forward, 0, speed_yaw)

    #지정된 경로대로 한바퀴 돌면 착륙
    if (xx > (target_point[loop_count][0] - 40) and xx < (target_point[loop_count][0] + 40)) and (yy > (target_point[loop_count][1] - 40) and yy < (target_point[loop_count][1] + 40)):
        if (loop_count >= loop_max-1) and t_loop == 1: 
            breakk = 1
        elif loop_count >= loop_max-1:
            t_loop = 1
            loop_count = 0
        else:
            loop_count += 1
    print(target_point[loop_count][0], target_point[loop_count][1])
    # print(xx,yy)

while True:
    img_1 = findFace()
    
    locate() #plt.show(sho)랑 세트
    
    flyy()

    cv2.imshow("Output",img_1)
    cv2.waitKey(1)

    if (cv2.waitKey(1) & 0xFF == ord('q')) or breakk == 1:
        Tello.land()
        break
plt.show()