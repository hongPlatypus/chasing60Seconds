# import cv2

# def find_webcams():
#     webcam_list = []
#     for i in range(10):  # 0부터 9까지 웹캠 번호 시도
#         cap = cv2.VideoCapture(i)
#         if cap.isOpened():  # 웹캠이 정상적으로 열리면
#             webcam_list.append(i)
#             cap.release()
#     return webcam_list

# def main():
#     webcams = find_webcams()
#     if webcams:
#         print(f"웹캠 번호들: {webcams}")
#     else:
#         print("웹캠이 연결되지 않았습니다.")

# if __name__ == "__main__":
#     main()

import cv2
print(cv2.__version__)
