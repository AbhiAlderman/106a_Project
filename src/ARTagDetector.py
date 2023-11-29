import cv2
import numpy as np

class ARTagDetector:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def detect_tags(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        tag_info = []

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, tag_id in enumerate(ids.flatten()):
                tag_corners = corners[i].flatten().tolist()
                tag_info.append([tag_id, tag_corners])
        
        return frame, tag_info

    def release(self):
        self.cap.release()

# Example usage
if __name__ == "__main__":
    detector = ARTagDetector()
    try:
        while True:
            frame, tag_info = detector.detect_tags()
            if frame is not None:
                print(f'Detected Tags: {tag_info}')
                cv2.imshow('Frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        detector.release()
        cv2.destroyAllWindows()
