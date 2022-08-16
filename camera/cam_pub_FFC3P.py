import depthai as dai
import cv2

# Create pipeline
pipeline = dai.Pipeline()
camRgb = pipeline.create(dai.node.ColorCamera)
# Set the camera used?
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
camRgb.setFps(30)
# 4056*3040-> 1248*936
# This scale has most things supported 
#   see the spreed sheet in 
#   https://docs.luxonis.com/projects/api/en/latest/components/nodes/color_camera/#limitations
camRgb.setIspScale(4, 13)
stillEncoder = pipeline.create(dai.node.VideoEncoder)

controlIn = pipeline.create(dai.node.XLinkIn)
ispOut = pipeline.create(dai.node.XLinkOut)
stillMjpegOut = pipeline.create(dai.node.XLinkOut)

controlIn.setStreamName('control')
ispOut.setStreamName('isp')
stillMjpegOut.setStreamName('still')

# Properties
stillEncoder.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)

# Linking
camRgb.isp.link(ispOut.input)
camRgb.still.link(stillEncoder.input)
controlIn.out.link(camRgb.inputControl)
stillEncoder.bitstream.link(stillMjpegOut.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # Get data queues
    controlQueue = device.getInputQueue('control')
    ispQueue = device.getOutputQueue('isp')
    stillQueue = device.getOutputQueue('still')

    while True:
        ispFrames = ispQueue.tryGetAll()
        for ispFrame in ispFrames:
            cv2.imshow('isp', ispFrame.getCvFrame())

        stillFrames = stillQueue.tryGetAll()
        for stillFrame in stillFrames:
            # Decode JPEG
            frame = cv2.imdecode(stillFrame.getData(), cv2.IMREAD_UNCHANGED)
            # Display
            cv2.imshow('still', frame)
            cv2.imwrite("test.jpg", frame)

        # Update screen (1ms pooling rate)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('c'):
            ctrl = dai.CameraControl()
            ctrl.setCaptureStill(True)
            controlQueue.send(ctrl)