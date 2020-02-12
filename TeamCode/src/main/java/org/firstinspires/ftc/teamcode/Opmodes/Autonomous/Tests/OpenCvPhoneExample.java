package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


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