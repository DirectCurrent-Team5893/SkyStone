package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


@Autonomous(group = "Autonomous", name = "OpenCvPhoneExample")
public class OpenCvPhoneExample extends LinearOpMode {

    SkystoneDetectorExample detector;

    @Override
    public void runOpMode() {

        detector = new SkystoneDetectorExample(this, false);

        waitForStart();


        while (opModeIsActive()) {

           sleep(100);
//            telemetry.addData("Decision is ", detector.getDecision());
        }
    }
}