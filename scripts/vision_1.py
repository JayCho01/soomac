import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import Float32MultiArray  # ROS 메시지 타입

def detect_features():
    # ROS 노드 초기화
    rospy.init_node('feature_detection', anonymous=True)
    pub = rospy.Publisher('feature_detection_topic', Float32MultiArray, queue_size=10)

    cap = cv2.VideoCapture(0)  # 웹캠 열기, 인덱스 2는 세 번째 카메라를 의미함

    start_time = time.time()

    def is_rectangle(contour):
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) != 4:
            return False, approx

        (x, y, w, h) = cv2.boundingRect(approx)
        aspectRatio = w / float(h)

        # 확인하는 기준
        if 0.8 <= aspectRatio <= 1.2:  # 사각형 모양인지 확인
            return True, approx
        return False, approx

    while not rospy.is_shutdown():
        ret, frame = cap.read()  # 프레임 읽기

        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 회색조 변환
        edges = cv2.Canny(gray, 100, 200)  # Canny edge detection

        contours, hierarchy = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)  # 윤곽선 검출

        frame_centers_angles = []
        for contour in contours:
            is_rect, approx = is_rectangle(contour)
            if is_rect:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])
                    
                    rect = cv2.minAreaRect(contour)  # 최소 외접 직사각형 계산
                    angle = rect[2]  # 각도 추출
                    
                    # 각도를 조정하여 x축에 가장 가까운 변의 각도로 변환
                    if angle < -45:
                        angle = 90 + angle
                    
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    frame_centers_angles.append((cX, cY, angle))
                    cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                    
                    # 각도를 시각적으로 표시하는 선 추가
                    box_center = np.mean(box, axis=0)
                    length = 50  # 선의 길이
                    radian_angle = np.deg2rad(angle)
                    x2 = int(box_center[0] + length * np.cos(radian_angle))
                    y2 = int(box_center[1] - length * np.sin(radian_angle))  # Y축 방향 반전
                    cv2.line(frame, (int(box_center[0]), int(box_center[1])), (x2, y2), (255, 0, 0), 2)
                    
                    cv2.circle(frame, (cX, cY), 3, (255, 0, 0), -1)

        # 1초마다 중심점 좌표와 각도 출력 및 publish
        current_time = time.time()
        if current_time - start_time >= 1:
            if frame_centers_angles:
                feature_data = Float32MultiArray()
                feature_data.data = [val for center_angle in frame_centers_angles for val in center_angle]
                pub.publish(feature_data)
                print("Published data:", feature_data.data)
            start_time = current_time

        cv2.imshow('Feature and Center Detection', frame)  # 결과 프레임 표시
        cv2.imshow('Canny Detection', edges)  # Canny edge detection 결과 프레임 표시

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()  # 필요 없어진 부분

if __name__ == '__main__':
    try:
        detect_features()
    except rospy.ROSInterruptException:
        pass
