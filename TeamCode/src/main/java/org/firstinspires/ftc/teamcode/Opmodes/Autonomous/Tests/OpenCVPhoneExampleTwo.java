package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Tests;

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
@Autonomous(group = "Autonomous", name = "OpenCvPhoneExampleTwo")

public class OpenCVPhoneExampleTwo extends LinearOpMode {

    SkystoneDetectorExample detector;

    @Override
    public void runOpMode() {

        detector = new SkystoneDetectorExample(this, true,true);


        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("decision:", detector.getDecision());
            sleep(1000);
        }
        while (opModeIsActive()) {
            sleep(100);
        }
    }
}
