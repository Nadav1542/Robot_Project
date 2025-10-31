import cv2
import mediapipe as mp
import numpy as np

# Load MediaPipe's gesture recognizer model
BaseOptions = mp.tasks.BaseOptions
GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult

# Create a variable to store the latest gesture results
latest_result = None

# Create a callback function to handle the results
def result_callback(result: GestureRecognizerResult, output_image: mp.Image, timestamp_ms: int):
    global latest_result
    latest_result = result

# Configure the gesture recognizer options
options = GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path='gesture_recognizer.task'),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=result_callback
)

# Start the gesture recognizer and camera feed
with GestureRecognizer.create_from_options(options) as recognizer:
    # Initialize the camera (adjust index if needed, e.g., 1, 2)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    print("Camera connected. Look at the camera and show gestures.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to RGB format
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        # Process the frame for gesture recognition
        timestamp_ms = int(cap.get(cv2.CAP_PROP_POS_MSEC))
        recognizer.recognize_async(mp_image, timestamp_ms)

        # Display the latest recognized gesture on the frame
        if latest_result and latest_result.gestures:
            # Get the top gesture category for the first hand detected
            gesture_category = latest_result.gestures[0][0].category_name
            cv2.putText(
                frame,
                f'Gesture: {gesture_category}',
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

        # Display the camera feed in a window
        cv2.imshow('Gesture Recognition', frame)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and destroy all windows
    cap.release()
    cv2.destroyAllWindows()
