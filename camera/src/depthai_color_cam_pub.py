#!/usr/bin/evn python

# TODO: set the isp scale to the size the same as VSLAM system's input size

"""
Modify from: https://github.com/luxonis/depthai-python/blob/b763ca44cd94ea43144b5d7504e6c018171f7ceb/examples/ColorCamera/rgb_camera_control.py
This example shows usage of Camera Control message as well as ColorCamera configInput to change crop x and y
Uses 'WASD' controls to move the crop window, 'C' to capture a still image, 'T' to trigger autofocus, 'IOKL,.NM'
for manual exposure/focus/white-balance:
  Control:      key[dec/inc]  min..max
  exposure time:     I   O      1..33000 [us]
  sensitivity iso:   K   L    100..1600
  focus:             ,   .      0..255 [far..near]
  white balance:     N   M   1000..12000 (light color temperature K)
To go back to auto controls:
  'E' - autoexposure
  'F' - autofocus (continuous)
  'B' - auto white-balance
Other controls:
    '1' - AWB lock (true / false)
    '2' - AE lock (true / false)
    '3' - Select control: AWB mode
    '4' - Select control: AE compensation
    '5' - Select control: anti-banding/flicker mode
    '6' - Select control: effect mode
    '7' - Select control: brightness
    '8' - Select control: contrast
    '9' - Select control: saturation
    '0' - Select control: sharpness
    '[' - Select control: luma denoise
    ']' - Select control: chroma denoise
For the 'Select control: ...' options, use these keys to modify the value:
  '-' or '_' to decrease
  '+' or '=' to increase
"""

from itertools import cycle
from typing import Tuple

import cv2
import depthai as dai
import rospy
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from numpy import ndarray
from sensor_msgs.msg import Image

# Step size ('W','A','S','D' controls)
STEP_SIZE = 8
# Manual exposure/focus/white-balance set step
EXP_STEP = 500  # us
ISO_STEP = 50
LENS_STEP = 3
WB_STEP = 200


def clamp(num, v0, v1):
    return max(v0, min(num, v1))


def get_pipeline(videoSize: Tuple[int, int]):
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    colorCam = pipeline.create(dai.node.ColorCamera)
    # stillEncoder = pipeline.create(dai.node.VideoEncoder)

    controlIn = pipeline.create(dai.node.XLinkIn)
    configIn = pipeline.create(dai.node.XLinkIn)
    ispOut = pipeline.create(dai.node.XLinkOut)
    videoOut = pipeline.create(dai.node.XLinkOut)
    # stillMjpegOut = pipeline.create(dai.node.XLinkOut)

    controlIn.setStreamName('control')
    configIn.setStreamName('config')
    ispOut.setStreamName('isp')
    videoOut.setStreamName('video')
    # stillMjpegOut.setStreamName('still')

    # Properties
    # Set the camera used?
    colorCam.setResolution(
        dai.ColorCameraProperties.SensorResolution.THE_12_MP)
    colorCam.setFps(30)
    # 4056*3040 -> 1248*936
    # This scale has most things(ISP) supported
    #   see the spreed sheet in
    #   https://docs.luxonis.com/projects/api/en/latest/components/nodes/color_camera/#limitations
    colorCam.setIspScale(4, 13)
    colorCam.setVideoSize(videoSize[0], videoSize[1])
    # stillEncoder.setDefaultProfilePreset(
    #     1, dai.VideoEncoderProperties.Profile.MJPEG)

    # Linking
    colorCam.isp.link(ispOut.input)
    # camRgb.still.link(stillEncoder.input)
    colorCam.video.link(videoOut.input)
    controlIn.out.link(colorCam.inputControl)
    configIn.out.link(colorCam.inputConfig)
    # stillEncoder.bitstream.link(stillMjpegOut.input)
    return pipeline, colorCam


class ImagePublisher(object):
    def __init__(self, cvbridge: CvBridge,
                 topic: str,
                 frameId: str = "",
                 encoding: str = "passthrough") -> None:
        self.cvbridge = cvbridge
        self.topic = topic
        self.frameId = frameId
        self.encoding = encoding

        self.pubImg = rospy.Publisher(topic, Image)

    def publish(self, image: ndarray) -> None:
        try:
            img_msg = self.cvbridge.cv2_to_imgmsg(image, self.encoding)
        except CvBridgeError as e:
            print(e)
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = self.frameId
        img_msg.header = h
        self.pubImg.publish(img_msg)
        # rospy.loginfo(f"{self.topic} published")


if __name__ == "__main__":
    rospy.init_node("image_publisher", anonymous=True)
    rate = rospy.Rate(10)  # ? Hz

    pipeline, colorCam = get_pipeline((640, 480))
    cvbridge = CvBridge()
    colorIspPub = ImagePublisher(cvbridge, "color/isp", "camera0")
    colorVidPub = ImagePublisher(cvbridge, "color/video", "camera0")

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Get data queues
        controlQueue = device.getInputQueue('control', 10, blocking=False)
        configQueue = device.getInputQueue('config', 10, blocking=False)
        ispQueue = device.getOutputQueue('isp', 10, blocking=False)
        videoQueue = device.getOutputQueue('video', 1, blocking=False)
        # stillQueue = device.getOutputQueue('still')

        # Max cropX & cropY
        maxCropX = (colorCam.getIspWidth() - colorCam.getVideoWidth()
                    ) / colorCam.getIspWidth()
        maxCropY = (colorCam.getIspHeight() - colorCam.getVideoHeight()
                    ) / colorCam.getIspHeight()
        print(maxCropX, maxCropY, colorCam.getIspWidth(),
              colorCam.getIspHeight())

        # Default crop
        cropX = 0
        cropY = 0
        sendCamConfig = True

        # Defaults and limits for manual focus/exposure controls
        lensPos = 150
        expTime = 20000
        sensIso = 800
        wbManual = 4000
        ae_comp = 0
        ae_lock = False
        awb_lock = False
        saturation = 0
        contrast = 0
        brightness = 0
        sharpness = 0
        luma_denoise = 0
        chroma_denoise = 0
        control = 'none'

        awb_mode = cycle([item for name, item in vars(
            dai.CameraControl.AutoWhiteBalanceMode).items() if name.isupper()])
        anti_banding_mode = cycle([item for name, item in vars(
            dai.CameraControl.AntiBandingMode).items() if name.isupper()])
        effect_mode = cycle([item for name, item in vars(
            dai.CameraControl.EffectMode).items() if name.isupper()])

        while not rospy.is_shutdown():
            vidFrames = videoQueue.tryGetAll()
            # for vidFrame in vidFrames:
            #     colorVidPub.publish(vidFrame.getCvFrame())
            # cv2.imshow(f"Publisher: {colorVidPub.topic}", vidFrame.getCvFrame())

            ispFrames = ispQueue.tryGetAll()
            for ispFrame in ispFrames:
                frame = ispFrame.getCvFrame()
                # # Show video cropped region
                # x = round(cropX)
                # y = round(cropY)
                # cv2.rectangle(frame, (x, y),
                #               (x+10, y+10), (255, 0, 0))
                # cv2.imshow(
                #     f"Publisher: {colorIspPub.topic}", frame)
                colorIspPub.publish(frame)
                print(
                    f"[{rospy.Time.now().secs}]published: {colorIspPub.topic}", end="\r")

                # Send new cfg to camera
                if sendCamConfig:
                    cfg = dai.ImageManipConfig()
                    cfg.setCropRect(cropX, cropY, 0, 0)
                    configQueue.send(cfg)
                    print('Sending new crop - x: ', cropX, ' y: ', cropY)
                    sendCamConfig = False

            # Update screen (1ms pooling rate)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('c'):
                ctrl = dai.CameraControl()
                ctrl.setCaptureStill(True)
                controlQueue.send(ctrl)
            elif key == ord('t'):
                print("Autofocus trigger (and disable continuous)")
                ctrl = dai.CameraControl()
                ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
                ctrl.setAutoFocusTrigger()
                controlQueue.send(ctrl)
            elif key == ord('f'):
                print("Autofocus enable, continuous")
                ctrl = dai.CameraControl()
                ctrl.setAutoFocusMode(
                    dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)
                controlQueue.send(ctrl)
            elif key == ord('e'):
                print("Autoexposure enable")
                ctrl = dai.CameraControl()
                ctrl.setAutoExposureEnable()
                controlQueue.send(ctrl)
            elif key == ord('b'):
                print("Auto white-balance enable")
                ctrl = dai.CameraControl()
                ctrl.setAutoWhiteBalanceMode(
                    dai.CameraControl.AutoWhiteBalanceMode.AUTO)
                controlQueue.send(ctrl)
            elif key in [ord(','), ord('.')]:
                if key == ord(','):
                    lensPos -= LENS_STEP
                if key == ord('.'):
                    lensPos += LENS_STEP
                lensPos = clamp(lensPos, 0, 255)
                print("Setting manual focus, lens position: ", lensPos)
                ctrl = dai.CameraControl()
                ctrl.setManualFocus(lensPos)
                controlQueue.send(ctrl)
            elif key in [ord('i'), ord('o'), ord('k'), ord('l')]:
                if key == ord('i'):
                    expTime -= EXP_STEP
                if key == ord('o'):
                    expTime += EXP_STEP
                if key == ord('k'):
                    sensIso -= ISO_STEP
                if key == ord('l'):
                    sensIso += ISO_STEP
                expTime = clamp(expTime, 1, 33000)
                sensIso = clamp(sensIso, 100, 1600)
                print("Setting manual exposure, time: ",
                      expTime, "iso: ", sensIso)
                ctrl = dai.CameraControl()
                ctrl.setManualExposure(expTime, sensIso)
                controlQueue.send(ctrl)
            elif key in [ord('n'), ord('m')]:
                if key == ord('n'):
                    wbManual -= WB_STEP
                if key == ord('m'):
                    wbManual += WB_STEP
                wbManual = clamp(wbManual, 1000, 12000)
                print("Setting manual white balance, temperature: ", wbManual, "K")
                ctrl = dai.CameraControl()
                ctrl.setManualWhiteBalance(wbManual)
                controlQueue.send(ctrl)
            elif key in [ord('w'), ord('a'), ord('s'), ord('d')]:
                if key == ord('a'):
                    cropX = cropX - \
                        (maxCropX / colorCam.getResolutionWidth()) * STEP_SIZE
                    if cropX < 0:
                        cropX = 0
                elif key == ord('d'):
                    cropX = cropX + \
                        (maxCropX / colorCam.getResolutionWidth()) * STEP_SIZE
                    if cropX > maxCropX:
                        cropX = maxCropX
                elif key == ord('w'):
                    cropY = cropY - \
                        (maxCropY / colorCam.getResolutionHeight()) * STEP_SIZE
                    if cropY < 0:
                        cropY = 0
                elif key == ord('s'):
                    cropY = cropY + \
                        (maxCropY / colorCam.getResolutionHeight()) * STEP_SIZE
                    if cropY > maxCropY:
                        cropY = maxCropY
                sendCamConfig = True
            elif key == ord('1'):
                awb_lock = not awb_lock
                print("Auto white balance lock:", awb_lock)
                ctrl = dai.CameraControl()
                ctrl.setAutoWhiteBalanceLock(awb_lock)
                controlQueue.send(ctrl)
            elif key == ord('2'):
                ae_lock = not ae_lock
                print("Auto exposure lock:", ae_lock)
                ctrl = dai.CameraControl()
                ctrl.setAutoExposureLock(ae_lock)
                controlQueue.send(ctrl)
            elif key >= 0 and chr(key) in '34567890[]':
                if key == ord('3'):
                    control = 'awb_mode'
                elif key == ord('4'):
                    control = 'ae_comp'
                elif key == ord('5'):
                    control = 'anti_banding_mode'
                elif key == ord('6'):
                    control = 'effect_mode'
                elif key == ord('7'):
                    control = 'brightness'
                elif key == ord('8'):
                    control = 'contrast'
                elif key == ord('9'):
                    control = 'saturation'
                elif key == ord('0'):
                    control = 'sharpness'
                elif key == ord('['):
                    control = 'luma_denoise'
                elif key == ord(']'):
                    control = 'chroma_denoise'
                print("Selected control:", control)
            elif key in [ord('-'), ord('_'), ord('+'), ord('=')]:
                change = 0
                if key in [ord('-'), ord('_')]:
                    change = -1
                if key in [ord('+'), ord('=')]:
                    change = 1
                ctrl = dai.CameraControl()
                if control == 'none':
                    print(
                        "Please select a control first using keys 3..9 0 [ ]")
                elif control == 'ae_comp':
                    ae_comp = clamp(ae_comp + change, -9, 9)
                    print("Auto exposure compensation:", ae_comp)
                    ctrl.setAutoExposureCompensation(ae_comp)
                elif control == 'anti_banding_mode':
                    abm = next(anti_banding_mode)
                    print("Anti-banding mode:", abm)
                    ctrl.setAntiBandingMode(abm)
                elif control == 'awb_mode':
                    awb = next(awb_mode)
                    print("Auto white balance mode:", awb)
                    ctrl.setAutoWhiteBalanceMode(awb)
                elif control == 'effect_mode':
                    eff = next(effect_mode)
                    print("Effect mode:", eff)
                    ctrl.setEffectMode(eff)
                elif control == 'brightness':
                    brightness = clamp(brightness + change, -10, 10)
                    print("Brightness:", brightness)
                    ctrl.setBrightness(brightness)
                elif control == 'contrast':
                    contrast = clamp(contrast + change, -10, 10)
                    print("Contrast:", contrast)
                    ctrl.setContrast(contrast)
                elif control == 'saturation':
                    saturation = clamp(saturation + change, -10, 10)
                    print("Saturation:", saturation)
                    ctrl.setSaturation(saturation)
                elif control == 'sharpness':
                    sharpness = clamp(sharpness + change, 0, 4)
                    print("Sharpness:", sharpness)
                    ctrl.setSharpness(sharpness)
                elif control == 'luma_denoise':
                    luma_denoise = clamp(luma_denoise + change, 0, 4)
                    print("Luma denoise:", luma_denoise)
                    ctrl.setLumaDenoise(luma_denoise)
                elif control == 'chroma_denoise':
                    chroma_denoise = clamp(chroma_denoise + change, 0, 4)
                    print("Chroma denoise:", chroma_denoise)
                    ctrl.setChromaDenoise(chroma_denoise)
                controlQueue.send(ctrl)
            rate.sleep()
