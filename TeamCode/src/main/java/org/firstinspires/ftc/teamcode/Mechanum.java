package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "MainTeleOp", group = "Linear Opmode")

public class Mechanum extends LinearOpMode {

    //define the motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;
    private DcMotor OuttakeLift = null;
    private DcMotor HorizontalLift = null;

    Servo Grabber;
    Servo LeftBlockGrabber;
    Servo RightBlockGrabber;
    Servo LeftBaseplateShover;
    Servo RightBaseplateShover;
    Servo ShoveBlock;


    @Override
    public void runOpMode() {

//hardware mapping
        leftIntake = hardwareMap.get(DcMotor.class, "Left Intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right intake");
        backLeft = hardwareMap.get(DcMotor.class, "back left wheel");
        backRight = hardwareMap.get(DcMotor.class, "back right wheel");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right wheel");
        frontLeft = hardwareMap.get(DcMotor.class, "Front Left wheel");
        HorizontalLift = hardwareMap.get(DcMotor.class, "HL");
        OuttakeLift = hardwareMap.get(DcMotor.class, "OL");
        Grabber = hardwareMap.get(Servo.class, "Grabber");
        LeftBlockGrabber = hardwareMap.get(Servo.class, "LBG");
        RightBlockGrabber = hardwareMap.get(Servo.class, "RBG");
        LeftBaseplateShover = hardwareMap.get(Servo.class, "LBS");
        RightBaseplateShover = hardwareMap.get(Servo.class, "RBS");
        ShoveBlock = hardwareMap.get(Servo.class, "SB");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        HorizontalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OuttakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int STOP = 0;
        int FORWARD = 1;
        int BACKWARD = -1;
        int grabberChanger = 1;
        int baseplateChanger = 1;
        int rightBlockMover = 1;
        int leftBlockMover = 1;

        boolean gamepad2bHeld = false;
        boolean gamepad1aHeld = false;
        boolean gamepad1xHeld = false;
        boolean gamepad1bHeld = false;
        boolean gamepad1dpadDownHeld = false;
        boolean gamepad2dpadUpHeld = false;
        boolean gamepad2dpadDownHeld = false;
        int ranMethod = 0;

        double IntakePower = 1;
        waitForStart();

        while (opModeIsActive()) {

            drivetrain(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if (gamepad1.right_bumper) {
                leftIntake.setPower(IntakePower);
                rightIntake.setPower(-IntakePower);
            } else if (gamepad1.left_bumper) {
                leftIntake.setPower(-.5);
                rightIntake.setPower(.5);
            } else {
                leftIntake.setPower(STOP);
                rightIntake.setPower(STOP);
            }
            if (gamepad1.dpad_down && !gamepad1dpadDownHeld) {
                gamepad1dpadDownHeld = true;
                if (IntakePower == 1) {
                    IntakePower = .3;
                } else {
                    IntakePower = 1;
                }
            }
            if (!gamepad1.dpad_down) {
                gamepad1dpadDownHeld = false;
            }



            GrabberPositions[] GRABBERPOSITIONS = {GrabberPositions.DOWN_POSITION, GrabberPositions.UP_POSITION};
            telemetry.addData("gamepad2.b is", gamepad2.b);
            telemetry.addData("gamepad2b held is", gamepad2bHeld);
            telemetry.addData("grabber changer is", grabberChanger);
            telemetry.addData("Ran method is", ranMethod);
            telemetry.addData("changing position RightBlockMover is ", rightBlockMover);
            telemetry.addData("changing position LeftBlockMover is ", leftBlockMover);
            telemetry.addData("changing position GrabberChanger is ", baseplateChanger);
            telemetry.addData("changing position GrabberChanger is ", baseplateChanger);

            if (gamepad2.b && gamepad2bHeld == false) {
                ranMethod++;
                gamepad2bHeld = true;
                telemetry.addData("changing position GrabberChanger is ", grabberChanger);

                SetPosition(GRABBERPOSITIONS[grabberChanger]);
                grabberChanger++;
                grabberChanger = grabberChanger % 2;
            }
            if (!gamepad2.b) {
                gamepad2bHeld = false;
            }
            BaseplateMoverPositions[] BASEPLATEMOVERPOSITIONS = {BaseplateMoverPositions.DOWN_POSITION, BaseplateMoverPositions.UP_POSITION};

            if (gamepad1.a && gamepad1aHeld == false) {
                ranMethod++;
                gamepad1aHeld = true;
                telemetry.addData("changing position GrabberChanger is ", grabberChanger);

                SetBaseplateMoverPosition(BASEPLATEMOVERPOSITIONS[baseplateChanger]);
                baseplateChanger++;
                baseplateChanger = baseplateChanger % 2;
            }
            if (!gamepad1.a) {
                gamepad1aHeld = false;
            }


            leftBlockMoverPositions[] LEFTBLOCKMOVERPOSITIONS = {leftBlockMoverPositions.DOWN_POSITION, leftBlockMoverPositions.UP_POSITION};

            if (gamepad1.x && gamepad1xHeld == false) {
                ranMethod++;
                gamepad1xHeld = true;
                telemetry.addData("changing position GrabberChanger is ", grabberChanger);

                SetLeftBlockMoverPosition(LEFTBLOCKMOVERPOSITIONS[leftBlockMover]);
                leftBlockMover++;
                leftBlockMover = leftBlockMover % 2;
            }
            if (!gamepad1.x) {
                gamepad1xHeld = false;
            }

            if (gamepad2.dpad_up && gamepad2dpadUpHeld == false) {
                ranMethod++;
                gamepad2dpadUpHeld = true;
                if(OuttakeLift.getCurrentPosition()<=50)
                {
                    VerticalLiftPostions(.6,1,0);
                }
                VerticalLiftPostions(.6,2,0);
            }
            if (!gamepad2.dpad_down) {
                gamepad2dpadDownHeld = false;
            }
            if (gamepad2.dpad_down && gamepad2dpadDownHeld == false) {
                ranMethod++;
                gamepad2dpadDownHeld = true;
                OuttakeLift.setTargetPosition(0);
                OuttakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (!gamepad2.dpad_down) {
                gamepad2dpadDownHeld = false;
            }

            rightBlockMoverPositions[] RIGHTBLOCKMOVERPOSITIONS = {rightBlockMoverPositions.UP_POSITION, rightBlockMoverPositions.DOWN_POSITION};
            if (gamepad1.b && gamepad1bHeld == false) {

                gamepad1bHeld = true;

                SetRightBlockMoverPosition(RIGHTBLOCKMOVERPOSITIONS[rightBlockMover]);
                rightBlockMover++;
                rightBlockMover = rightBlockMover % 2;
            }
            if (!gamepad1.b) {
                gamepad1bHeld = false;
            }

//             if(gamepad2.left_bumper)
//             {
//                 HorizontalLift.setPower(.9);
//             }
//             else if(gamepad2.left_trigger> .1)
//             {
//                 HorizontalLift.setPower(-gamepad2.left_trigger);
//             }
//             else if(!gamepad2.left_bumper && gamepad2.left_trigger<.1)
//             {
//                 HorizontalLift.setPower(STOP);
//             }
            HorizontalLift.setPower(gamepad2.left_stick_y);
            //OuttakeLift.setPower(gamepad2.right_stick_y);
            telemetry.addData("OuttakeLift",OuttakeLift.getCurrentPosition());
            telemetry.update();
//             if(gamepad2.left_stick_y>
//            if(gamepad2.right_bumper)
//            {
//                OuttakeLift.setPower(.9);
//            }
//            else if(gamepad2.right_trigger> .1)
//            {
//                OuttakeLift.setPower(-gamepad2.right_trigger);
//            }
//            else if(!gamepad2.right_bumper && gamepad2.right_trigger<.1)
//            {
//                OuttakeLift.setPower(STOP);
//            }
            telemetry.update();
        }
    }

    public enum GrabberPositions {
        UP_POSITION, DOWN_POSITION
    }

    public void SetPosition(final GrabberPositions POSITION) {
        double targetPosition;
        switch (POSITION) {
            case UP_POSITION:
                Grabber.setPosition(.9);
                targetPosition = .9;
                break;
            case DOWN_POSITION:
                Grabber.setPosition(.5);
                targetPosition = .5;
                break;
        }
    }

    public enum BaseplateMoverPositions {
        UP_POSITION, DOWN_POSITION
    }

    public void SetBaseplateMoverPosition(final BaseplateMoverPositions POSITION) {
        switch (POSITION) {
            case UP_POSITION:
                RightBaseplateShover.setPosition(1);
                LeftBaseplateShover.setPosition(0);

                break;
            case DOWN_POSITION:
                RightBaseplateShover.setPosition(.2);
                LeftBaseplateShover.setPosition(.7);
                break;
        }
    }

    public enum rightBlockMoverPositions {
        UP_POSITION, DOWN_POSITION
    }

    public void SetRightBlockMoverPosition(final rightBlockMoverPositions POSITION) {
        switch (POSITION) {
            case UP_POSITION:
                RightBlockGrabber.setPosition(.7);

                break;
            case DOWN_POSITION:
                RightBlockGrabber.setPosition(0);
                break;
        }
    }

    public enum leftBlockMoverPositions {
        UP_POSITION, DOWN_POSITION
    }

    public void SetLeftBlockMoverPosition(final leftBlockMoverPositions POSITION) {
        switch (POSITION) {
            case UP_POSITION:

                LeftBlockGrabber.setPosition(.7);
                break;
            case DOWN_POSITION:
                LeftBlockGrabber.setPosition(.2);
                break;
        }
    }


    private void drivetrain(double forward, double right, double turn) {

        forward = checkValue(forward);
        right = checkValue(right);
        turn = checkValue(turn);
        double leftFrontPower = forward - right - turn;
        double leftBackPower = forward + right - turn;
        double rightFrontPower = forward + right + turn;
        double rightBackPower = forward - right + turn;
        double[] powers = {leftBackPower, leftFrontPower, rightBackPower, rightFrontPower};
        boolean isItTooBig = false;
        for (double power : powers) {
            if (Math.abs(power) > 1) {
                isItTooBig = true;
            }
        }
        //scale all the powers so that the motor isn't set to a value above one
        if (isItTooBig) {
            double greatest = 0;
            for (double power : powers) {

                if (Math.abs(power) > greatest) {
                    greatest = Math.abs(power);
                }

            }
            leftBackPower = leftBackPower / greatest;
            leftFrontPower = leftFrontPower / greatest;
            rightBackPower = rightBackPower / greatest;
            rightFrontPower = rightFrontPower / greatest;
        }
        boolean stop = true;
        for (double power : powers) {

            if (Math.abs(power) > .1) {
                stop = false;
            }
        }
        if (stop) {
            frontRight.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
            frontLeft.setPower(0);
        } else {
            frontRight.setPower(rightFrontPower);
            backRight.setPower(rightBackPower);
            backLeft.setPower(leftBackPower);
            frontLeft.setPower(leftFrontPower);
        }
        telemetry.addData("left y-stick", gamepad1.left_stick_y);
        telemetry.addData("left x-stick", gamepad1.left_stick_x);
        telemetry.addData("right x-stick", gamepad1.right_stick_x);
        telemetry.addData("Front right power ", rightFrontPower);
        telemetry.addData("Front left power ", leftFrontPower);
        telemetry.addData("Back right power ", rightBackPower);
        telemetry.addData("Back left power ", leftBackPower);
        //telemetry.update();
    }

    private double checkValue(double hardInput) {
        hardInput = Range.clip(hardInput, -1, 1);
        hardInput = Math.pow(hardInput, 3);
        return hardInput;
    }
    public void VerticalLiftPostions(double speed, int VerticalLiftPostions,double Timeout) {
        int newTargetVerticalLiftPositions;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTargetVerticalLiftPositions =(OuttakeLift.getCurrentPosition()+(-457*(VerticalLiftPostions-1)));

            OuttakeLift.setTargetPosition(newTargetVerticalLiftPositions);
            // Turn On RUN_TO_POSITION
            OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            OuttakeLift.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < Timeout) &&
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newTargetVerticalLiftPositions);
                telemetry.addData("Path2", "Running at %7d :%7d",

                        OuttakeLift.getCurrentPosition());
                telemetry.update();

            }

            OuttakeLift.setPower(0);
            if(VerticalLiftPostions ==1)
            {
                OuttakeLift.setTargetPosition(OuttakeLift.getCurrentPosition()-477);
            }
           OuttakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}

