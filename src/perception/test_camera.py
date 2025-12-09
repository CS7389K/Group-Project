
import cv2

gst_pipeline= (
    'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), ',
    'width={image_width},height={image_height},framerate=30/1,format=NV12 ! ',
    'nvvidconv ! video/x-raw,format=BGRx,width={image_width},height={image_height} ! ',
    'videoconvert ! video/x-raw,format=BGR ! appsink drop=1'
)
gst_pipeline = ''.join(gst_pipeline).format(image_width=640, image_height=480)

# Test 3: Check camera settings
print("Test 3: Camera Properties")
print("Opening camera...")
pipeline = "".join(gst_pipeline).format(
    image_width=640,
    image_height=480
)
print(f"GStreamer pipeline: {pipeline}")
capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
print(f"Camera opened: {capture.isOpened()}")
if not capture.isOpened():
    raise RuntimeError("Error: Unable to open camera")

if not capture.isOpened():
    print("Camera is not open!")

ok, frame = capture.read()
if not ok:
    print("Failed to read frame from camera")
else:
    cv2.imshow('YOLO Detection', frame)
    cv2.waitKey(1)  # Process events to keep window responsive
