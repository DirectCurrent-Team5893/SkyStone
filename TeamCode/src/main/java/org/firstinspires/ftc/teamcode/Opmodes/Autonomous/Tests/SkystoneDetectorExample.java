package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FTC_5893_2019.CustomTenserFlow5893;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class SkystoneDetectorExample {
    OpMode opMode;
    OpenCvCamera camera;

    private  Point BLUE_LEFT_TL = new Point(20, 150);
    private  Point BLUE_LEFT_BR = new Point(900, 200);
    private  Point BLUE_MIDDLE_TL = new Point(120, 150);
    private  Point BLUE_MIDDLE_BR = new Point(200, 200);
    private  Point BLUE_RIGHT_TL = new Point(230, 150);
    private  Point BLUE_RIGHT_BR = new Point(310, 200);

    private  Point RED_LEFT_TL = new Point(30, 130);
    private  Point RED_LEFT_BR = new Point(80, 170);
    private  Point RED_MIDDLE_TL = new Point(100, 130);
    private  Point RED_MIDDLE_BR = new Point(150, 170);
    private  Point RED_RIGHT_TL = new Point(170, 130);
    private  Point RED_RIGHT_BR = new Point(230, 170);

    private Point leftTL;
    private Point leftBR;
    private Point middleTL;
    private Point middleBR;
    private Point rightTL;
    private Point rightBR;

    private RGBColor left;
    private RGBColor middle;
    private RGBColor right;

    public SkystoneDetectorExample(OpMode opmode, boolean useWebcam,boolean isBlue) {
        opMode = opmode;
//        leftBR =LBR;
//        leftTL = LTL;
//        middleBR = MBR;
//        middleTL = MTL;
//        rightBR = RBR;
//        rightTL = RTL;
leftTL =isBlue ? BLUE_LEFT_TL :RED_LEFT_TL;
leftBR = isBlue ? BLUE_LEFT_BR : RED_LEFT_BR;
middleTL = isBlue ? BLUE_MIDDLE_TL : RED_MIDDLE_TL;
middleBR = isBlue ? BLUE_MIDDLE_BR : RED_MIDDLE_BR;
rightTL = isBlue ? BLUE_RIGHT_TL : RED_RIGHT_TL;
rightBR = isBlue ? BLUE_RIGHT_BR : RED_RIGHT_BR;


        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        if (useWebcam) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }
        CustomPipeline pipeline = new CustomPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

    }
    public void stopStreaming(){
        camera.stopStreaming();
    }
    public void startStreaming(){
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }
    public SkystoneDetectorExample.SkyStonePosition getDecision(){

        int leftValue = left.getValue();
        int middleValue = middle.getValue();
        int rightValue = right.getValue();

        if (leftValue < middleValue && leftValue < rightValue){
            return SkystoneDetectorExample.SkyStonePosition.LEFT;
        }
        if (middleValue < leftValue && middleValue < rightValue){
            return SkystoneDetectorExample.SkyStonePosition.MIDDLE;
        }
        if (rightValue < leftValue && rightValue < middleValue){
            return SkystoneDetectorExample.SkyStonePosition.RIGHT;
        }
        return SkystoneDetectorExample.SkyStonePosition.UNKNOWN;
    }
    class CustomPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {

            int thickness = 2;

            Imgproc.rectangle(input, leftTL, leftBR, new Scalar(0, 255, 0), thickness);
            Imgproc.rectangle(input, middleTL, middleBR, new Scalar(0, 255, 0), thickness);
            Imgproc.rectangle(input, rightTL, rightBR, new Scalar(0, 255, 0), thickness);


            left=getAverageColor(input,leftTL,leftBR);
            middle=getAverageColor(input,middleTL,middleBR);
            right =getAverageColor(input,rightTL,rightBR);

            sendTelemetry();
            return input;

        }


        private RGBColor getAverageColor(Mat mat, Point topLeft, Point bottomRight) {
            int red = 0;
            int green = 0;
            int blue = 0;

            int total = 0;

            for (int x = (int) topLeft.x; x < bottomRight.x; x++) {
                for (int y = (int) topLeft.y; y < bottomRight.y; y++) {
                    red += mat.get(y, x)[0];
                    green += mat.get(y, x)[1];
                    blue += mat.get(y, x)[2];
                    total++;
                }
            }

            red /= total;
            green /= total;
            blue /= total;

            return new RGBColor(red, green, blue);
        }
        public void sendTelemetry(){
            opMode.telemetry.addLine("Left :" + " R " + left.getRed() + " G " + left.getGreen() + " B " + left.getBlue());
            opMode.telemetry.addLine("Middle :" + " R " + middle.getRed() + " G " + middle.getGreen() + " B " + middle.getBlue());
            opMode.telemetry.addLine("Right :" + " R " + right.getRed() + " G " + right.getGreen() + " B " + right.getBlue());
            opMode.telemetry.update();
        }
    }
    public enum SkyStonePosition{
        LEFT,MIDDLE,RIGHT,UNKNOWN
    }
}