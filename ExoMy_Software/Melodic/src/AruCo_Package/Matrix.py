import cv2
import numpy as np

# Function to calibrate the camera
def calibrate_camera(images_folder, pattern_size):
    # Termination criteria for the cornerSubPix function
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points, like (0, 0, 0), (1, 0, 0), ..., (pattern_size[0]-1, pattern_size[1]-1, 0)
    objp = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    # Load images
    images = [cv2.imread(f"{images_folder}/{i}.jpg") for i in range(1, 16)]  # Change range accordingly

    for i, img in enumerate(images):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)  # Adjust as needed

    cv2.destroyAllWindows()

    # Calibrate camera
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return ret, camera_matrix, dist_coeffs

if __name__ == "__main__":
    images_folder = "calimages"  # Replace with the path to your calibration images
    pattern_size = (9, 6)  # Change to match your calibration pattern

    ret, camera_matrix, dist_coeffs = calibrate_camera(images_folder, pattern_size)

    if ret:
        print("Calibration successful.")
        print("Camera Matrix:")
        print(camera_matrix)
        print("\nDistortion Coefficients:")
        print(dist_coeffs)
    else:
        print("Calibration failed.")
