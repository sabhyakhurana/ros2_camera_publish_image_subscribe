import cv2
import time

def test_camera(device_index=0):
    cap = cv2.VideoCapture(device_index)
    time.sleep(3)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    while True:

        for _ in range(20):  # Read 20 frames to allow the camera to stabilize
            _, img = cap.read()

        ret, frame = cap.read()
        print(ret)
        print(frame)
        if not ret:
            print("Error: Could not read frame.")
            break
        cv2.imshow('Camera Test', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera()

############################

# import subprocess
# import time

# def test_camera():
#     # Wait for the camera to stabilize
#     time.sleep(3)

#     # Capture an image
#     result = subprocess.run(['libcamera-still', '-o', 'image.jpg'], capture_output=True, text=True)

#     # Check if the command was successful
#     if result.returncode != 0:
#         print("Error: Could not capture image.")
#         print("Output:", result.stdout)
#         print("Error:", result.stderr)

# if __name__ == "__main__":
#     test_camera()

##############################

# import gi
# import time
# import numpy as np
# gi.require_version('Gst', '1.0')
# from gi.repository import Gst

# def test_camera(device_index=0):
#     Gst.init(None)

#     pipeline = Gst.parse_launch(f'v4l2src device=/dev/video{device_index} ! videoconvert ! appsink')
#     appsink = pipeline.get_by_name('appsink')

#     pipeline.set_state(Gst.State.PLAYING)

#     # Wait for the camera to stabilize
#     time.sleep(3)

#     while True:
#         sample = appsink.emit('pull-sample')
#         if sample is None:
#             print("Error: Could not read frame.")
#             break

#         buf = sample.get_buffer()
#         caps = sample.get_caps()
#         array = np.ndarray((caps.get_structure(0).get_value('height'),
#                             caps.get_structure(0).get_value('width'), 3),
#                            buffer=buf.extract_dup(0, buf.get_size()),
#                            dtype=np.uint8)

#         print(array)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     pipeline.set_state(Gst.State.NULL)

# if __name__ == "__main__":
#     test_camera()