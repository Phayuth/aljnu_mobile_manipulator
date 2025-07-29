import cv2
import numpy as np


class Camera:

    def __init__(self):
        self.w = 640
        self.h = 480
        self.k = None
        self.d = None
        self.capture = cv2.VideoCapture(4)

    @classmethod
    def fake(cls):
        camera = cls()
        camera.k = np.array(
            [
                [599.639625, 0.0, 328.841620],
                [0.0, 602.139246, 232.169169],
                [0.0, 0.0, 1.0],
            ]
        )
        camera.d = np.array([0.143990, -0.280626, 0.002779, -0.000829, 0.000000])
        return camera

    @classmethod
    def from_parameters(cls, k, d):
        camera = cls()
        camera.k = np.array(k)
        camera.d = np.array(d)
        return camera

    def reading(self):
        if not self.capture.isOpened():
            raise RuntimeError("Failed to open camera")
        while True:
            ret, frame = self.capture.read()
            if not ret:
                raise RuntimeError("Failed to read frame from camera")
            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        self.capture.release()
        cv2.destroyAllWindows()


class ARUCOBoardPose:
    # https://docs.opencv.org/4.9.0/db/da9/tutorial_aruco_board_detection.html
    def __init__(self) -> None:
        # detection
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.board = cv2.aruco.GridBoard(
            (5, 7), 0.0275, 0.006875, self.dictionary, None
        )
        self.detectorParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary, self.detectorParams
        )

    def run(self, camera_k, camera_d, imgraw):
        corners, ids, rej = self.detector.detectMarkers(imgraw)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(imgraw, corners, ids)  # aruco corner

            objPoints, imgPoints = self.board.matchImagePoints(
                corners, ids, None, None
            )

            retval, rvc, tvc = cv2.solvePnP(
                objPoints,
                imgPoints,
                camera_k,
                camera_d,
                None,
                None,
                False,
            )
            R, _ = cv2.Rodrigues(rvc)

            if objPoints is not None:
                cv2.drawFrameAxes(
                    imgraw,
                    camera_k,
                    camera_d,
                    rvc,
                    tvc,
                    0.1,
                    3,
                )

            return tvc, R
        return None


class ARUCOGenerate:

    def __init__(self) -> None:
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        size = (5, 7)
        markerLength = 0.04
        markerSeparation = 0.01
        self.board = cv2.aruco.GridBoard(
            size, markerLength, markerSeparation, self.dictionary, None
        )

        image = self.board.generateImage(
            outSize=(500, 700), marginSize=10, borderBits=1
        )

        cv2.imwrite("aruco_board.png", image)
        image = cv2.imread("aruco_board.png")
        cv2.imshow("ARUCO Board", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


class CameraAruco:

    def __init__(self):
        self.camera = Camera.fake()
        self.aruco_board = ARUCOBoardPose()

    def run(self):
        while True:
            ret, frame = self.camera.capture.read()
            if not ret:
                break
            pose = self.aruco_board.run(self.camera.k, self.camera.d, frame)
            cv2.imshow("ARUCO Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        self.camera.capture.release()
        cv2.destroyAllWindows()


def generate_aruco_board():
    aruco_gen = ARUCOGenerate()


def detect_aruco_board():
    camera_aruco = CameraAruco()
    camera_aruco.run()


if __name__ == "__main__":

    func = [
        generate_aruco_board,
        detect_aruco_board,
    ]
    for i, f in enumerate(func):
        print(f"{i+1}: {f.__name__}")

    arg = input("Select function to run: ")
    if arg.isdigit() and 1 <= int(arg) < len(func):
        func[int(arg - 1)]()
    else:
        print("Invalid selection. Exiting.")
        exit(1)
