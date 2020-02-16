package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Point;


@Autonomous(group = "Autonomous", name = "OpenCvPhoneExample")
public class OpenCvPhoneExample extends LinearOpMode {

    boolean gamepad2bHeld = false;
    boolean gamepad1aHeld = false;
    boolean gamepad2aHeld = false;
    boolean gamepad2xHeld = false;
    boolean gamepad1xHeld = false;
    boolean gamepad1bHeld = false;
    boolean gamepad1yHeld = false;

    boolean gamepad2dpadUpHeld = false;
    boolean gamepad2dpadDownHeld = false;
    boolean gamepad2dpadLeftHeld = false;
    boolean gamepad2dpadRightHeld = false;

    boolean gamepad1dpadUpHeld = false;
    boolean gamepad1dpadDownHeld = false;
    boolean gamepad1dpadLeftHeld = false;
    boolean gamepad1dpadRightHeld = false;

    boolean leftBlockSelected = false;
    boolean rightBlockSelected = false;
    boolean middleBlockSelected = false;
    int LMU = -100;
    int LML = -100;
    int MMU = -100;
    int MML = -100;
    int RMU = -100;
    int RML = -100;

    int SU = -100;
    int SL = -100;
    int PixelIncrease = 10;
    SkystoneDetectorExample detector;
    Point leftTL = new Point(0, 0);
    Point leftBR = new Point(0, 0);
    Point middleTL = new Point(0, 0);
    Point middleBR = new Point(0, 0);
    Point rightTL = new Point(0, 0);
    Point rightBR = new Point(0, 0);

    public void setLeftBR(Point leftBR) {
        this.leftBR = leftBR;
    }

    public void setMiddleBR(Point middleBR) {
        this.middleBR = middleBR;
    }

    public void setRightBR(Point rightBR) {
        this.rightBR = rightBR;
    }

    public void setLeftTL(Point leftTL) {
        this.leftTL = leftTL;
    }

    public void setMiddleTL(Point middleTL) {
        this.middleTL = middleTL;
    }

    public void setRightTL(Point rightTL) {
        this.rightTL = rightTL;
    }

    @Override
    public void runOpMode() {



        detector = new SkystoneDetectorExample(this, false,false);

        waitForStart();


        while (opModeIsActive()) {
            // This is going to be used mainly for the use of being able to size or move them down to the general area for detection
            //After we get close enough we should use separate cases
            telemetry.addData("leftTL", leftTL);
            telemetry.addData("leftBR", leftBR);
            telemetry.addData("middleTL",middleTL);
            telemetry.addData("middleBR",middleBR);
            telemetry.addData("rightTL",rightTL);
            telemetry.addData("rightBR",rightBR);
            telemetry.addData("rightBlockSelected",rightBlockSelected);
            telemetry.addData("middleBlockSelected",middleBlockSelected);
            telemetry.addData("leftBlockSelected",leftBlockSelected);
            telemetry.update();
            if (!(leftBlockSelected && middleBlockSelected && rightBlockSelected)) {
//                setLeftBR(new Point(-SL - LML, -SU - LMU));
//                setLeftTL(new Point(-LML, -LMU));
//                setRightTL(new Point(-RML, -RMU));
//                setRightBR(new Point(-SL - RML, -SU - RMU));
//                setMiddleBR(new Point(-SL - MML, -SU - MMU));
//                setMiddleTL(new Point(-MML, -MMU));
                leftBR = new Point(-SL-LML,-SU-LMU);
                leftTL = new Point(-LML, -LMU);
                rightTL =new Point(-RML, -RMU);
                rightBR = new Point(-SL - RML, -SU - RMU);
                middleBR = new Point(-SL - MML, -SU - MMU);
                middleTL = new Point(-MML, -MMU);
            }

            // Here is the portion for left case
            if (gamepad2.x && !gamepad2xHeld) {
                gamepad2xHeld = true;
                leftBlockSelected = !leftBlockSelected;
            }
            if(!gamepad2.x){
                gamepad2xHeld = false;
            }
            if(leftBlockSelected){
                rightBlockSelected = false;
                middleBlockSelected = false;
            }

            if (gamepad2.a && !gamepad2aHeld) {
                gamepad2aHeld = true;
                middleBlockSelected = !middleBlockSelected;
            }
            if(!gamepad2.a){
                gamepad2aHeld = false;
            }
            if(middleBlockSelected){
                rightBlockSelected = false;
                leftBlockSelected = false;
            }

            if (gamepad2.b && !gamepad2bHeld) {
                gamepad2bHeld = true;
                rightBlockSelected = !rightBlockSelected;
            }
            if(!gamepad2.b){
                gamepad2bHeld = false;
            }
            if(rightBlockSelected){
                leftBlockSelected = false;
                middleBlockSelected = false;
            }


            if (gamepad2.dpad_up && !gamepad2dpadUpHeld) {
                gamepad2dpadUpHeld = true;
                if(leftBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    LMU += PixelIncrease;
                }
                if(middleBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    MMU += PixelIncrease;
                }
                if(rightBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    RMU += PixelIncrease;
                }

            }
            if (!gamepad2.dpad_up) {
                gamepad2dpadUpHeld = false;
            }


            if (gamepad2.dpad_down && gamepad2dpadDownHeld) {
                gamepad2dpadDownHeld = true;
                if(leftBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    LMU -= PixelIncrease;
                }
                if(middleBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    MMU -= PixelIncrease;
                }
                if(rightBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    RMU -= PixelIncrease;
                }
            }
            if (!gamepad2.dpad_down) {
                gamepad2dpadDownHeld = false;
            }


            if (gamepad2.dpad_left && !gamepad2dpadLeftHeld) {
                gamepad2dpadLeftHeld = true;
                if(leftBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    LML += PixelIncrease;
                }
                if(middleBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    MML += PixelIncrease;
                }
                if(rightBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    RML += PixelIncrease;
                }

            }
            if (!gamepad2.dpad_left) {
                gamepad2dpadLeftHeld = false;
            }


            if (gamepad2.dpad_right && !gamepad2dpadRightHeld) {
                gamepad2.dpad_right = true;
                if(leftBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    LML -= PixelIncrease;
                }
                if(middleBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    MML -= PixelIncrease;
                }
                if(rightBlockSelected ||(!leftBlockSelected&&!rightBlockSelected&&!middleBlockSelected)){
                    RML -= PixelIncrease;
                }

            }
            if (!gamepad2.dpad_right) {
                gamepad2dpadRightHeld = false;
            }


            if (gamepad1.dpad_up && !gamepad1dpadUpHeld) {
                gamepad1dpadUpHeld = true;
                SU += PixelIncrease;
            }
            if (!gamepad1.dpad_up) {
                gamepad1dpadUpHeld = false;
            }


            if (gamepad2.dpad_down && gamepad2dpadDownHeld) {
                gamepad2dpadDownHeld = true;
                SU -= PixelIncrease;
            }
            if (!gamepad1.dpad_down) {
                gamepad1dpadDownHeld = false;
            }

            if (gamepad1.dpad_left && !gamepad1dpadLeftHeld) {
                gamepad2dpadLeftHeld = true;
                SL += PixelIncrease;

            }
            if (!gamepad1.dpad_left) {
                gamepad2dpadLeftHeld = false;
            }

            if (gamepad1.dpad_right && !gamepad1dpadRightHeld) {
                gamepad1.dpad_right = true;
                SL -= PixelIncrease;

            }
            if (!gamepad1.dpad_right) {
                gamepad1dpadRightHeld = false;
            }
            telemetry.addData("leftTL", leftTL);
            telemetry.addData("leftBR", leftBR);
            telemetry.addData("middleTL",middleTL);
            telemetry.addData("middleBR",middleBR);
            telemetry.addData("rightTL",rightTL);
            telemetry.addData("rightBR",rightBR);
            telemetry.addData("rightBlockSelected",rightBlockSelected);
            telemetry.addData("middleBlockSelected",middleBlockSelected);
            telemetry.addData("leftBlockSelected",leftBlockSelected);
            telemetry.update();

                detector = new SkystoneDetectorExample(this, false,false);

            telemetry.addData("leftTL", leftTL);
            telemetry.addData("leftBR", leftBR);
            telemetry.addData("middleTL",middleTL);
            telemetry.addData("middleBR",middleBR);
            telemetry.addData("rightTL",rightTL);
            telemetry.addData("rightBR",rightBR);
            telemetry.addData("rightBlockSelected",rightBlockSelected);
            telemetry.addData("middleBlockSelected",middleBlockSelected);
            telemetry.addData("leftBlockSelected",leftBlockSelected);
            telemetry.update();

//            telemetry.addData("Decision is ", detector.getDecision());
        }
    }

}