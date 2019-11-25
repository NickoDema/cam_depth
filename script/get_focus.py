# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time

def main():

    try:
        param_file = open('camera_param.txt', 'r')
    except:
        raise BlockingIOError('Can\'t find config file camera_param.txt ')
    params = []
    for line in param_file:
        for t in line.split():
            try:
                params.append(float(t))
            except ValueError:
                pass
    distance_to_object, object_width, matrix_w, matrix_h = params
    param_file.close()

    print('Расстояние до объекта: {}'.format(str(distance_to_object)))
    print('Ширина референсного объекта: {}'.format(str(object_width)))
    time.sleep(1)

    try:
        cap = cv2.VideoCapture(0)
        cap.set(3, 1920)
        cap.set(4, 1080)
    except:
        raise Exception('Невозможно получить данные от камеры')

    while True:
        rtv, frame = cap.read()
        frame_half_width = frame.shape[1]//2
        frame_half_height = frame.shape[0]//2
        print(frame_half_width, frame_half_height)

        canny = cv2.Canny(frame, 100, 100)

        #Нахождение крайних точек
        right_point = frame_half_width        
        while canny[frame_half_height,frame_half_width]==0 and canny[frame_half_height-1,frame_half_width]==0 and canny[frame_half_height+1,frame_half_width]==0:
            right_point+=1
            if right_point == frame.shape[1]-1:
                break

        cv2.circle(canny, (frame_half_width, frame_half_height), 7, 255, -1)

        p=frame_half_width
        frame_half_width = canny.shape[1]//2
        while canny[frame_half_height,frame_half_width]==0 and canny[frame_half_height-1,frame_half_width]==0 and canny[frame_half_height+1,frame_half_width]==0:
            frame_half_width-=1
            if frame_half_width == 0:
                break

        cv2.circle(canny, (frame_half_width, frame_half_height), 7, 255, -1)
        p-=frame_half_width
        canny = cv2.resize(canny, (960, 540), interpolation=cv2.INTER_NEAREST)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        #Расчет фокусного расстояния
        if p<frame.shape[1]-10 and p>10:
            focus = matrix_w*distance_to_object/(frame.shape[1]*object_width/p)
            cv2.putText(canny, '%4.2f' % focus, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255, thickness = 3)

        cv2.imshow('canny', canny)
        frame = cv2.resize(frame, (960, 540), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('frame', frame)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
