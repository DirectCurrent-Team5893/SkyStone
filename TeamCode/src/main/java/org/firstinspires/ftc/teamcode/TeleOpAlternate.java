package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ServoTest", group = "Linear Opmode")

public class TeleOpAlternate extends LinearOpMode {

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
    Servo CapstoneDeployment;


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
        CapstoneDeployment = hardwareMap.get(Servo.class,"CD");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        HorizontalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OuttakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double MAX_SPEED = 1;
        double FAST_MODE =1;
        double SLOW_MODE =0.3;
        int STOP = 0;
        int FORWARD = 1;
        int BACKWARD = -1;
        int grabberChanger = 1;
        int baseplateChanger = 1;
        int rightBlockMover = 1;
        int leftBlockMover = 1;
        int CapstoneDeploymentChanger = 1;

        boolean gamepad2bHeld = false;
        boolean gamepad1aHeld = false;
        boolean gamepad2aHeld = false;
        boolean gamepad2xHeld = false;
        boolean gamepad1xHeld = false;
        boolean gamepad1bHeld = false;
        boolean gamepad1yHeld = false;
        boolean gamepad1dpadDownHeld = false;
        boolean gamepad2dpadUpHeld = false;
        boolean gamepad2dpadDownHeld = false;
        boolean manualMode = true;
        boolean gamepad2rightStickButtonHeld = false;
        boolean gamepad2rightStickYHeld = false;
        int ranMethod = 0;
        int ranMethodV2 = 0;

        double IntakePower = .5;
        waitForStart();

        while (opModeIsActive()) {

           if(gamepad1.a)
           {
               Grabber.setPosition(1);
           }
           if(gamepad1.b)
           {
               Grabber.setPosition(0);
           }
           if(gamepad1.x)
           {
               Grabber.setPosition(1);
               sleep(1500);
               Grabber.setPosition(0);
           }
        }
    }

    public enum GrabberPositions {
        UP_POSITION, DOWN_POSITION
    }

    public void SetPosition(final GrabberPositions POSITION) {
        double targetPosition;
        switch (POSITION) {
            case UP_POSITION:
                Grabber.setPosition(.8

                );
                targetPosition = .9;
                break;
            case DOWN_POSITION:
                Grabber.setPosition(.4);
                targetPosition = .5;
                break;
        }
    }
    public enum CapstoneDeploymentPositions {
        UP_POSITION, DOWN_POSITION
    }

    public void SetCapstoneDeploymentPosition(final CapstoneDeploymentPositions POSITION) {
        double targetPosition;
        switch (POSITION) {
            case UP_POSITION:
                CapstoneDeployment.setPosition(0);
                targetPosition = .9;
                break;
            case DOWN_POSITION:
                CapstoneDeployment.setPosition(1);
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


    private void drivetrain(double forward, double right, double turn,double MAX_SPEED) {

        forward = checkValue(forward,MAX_SPEED);
        right = checkValue(right,MAX_SPEED);
        turn = checkValue(turn,MAX_SPEED);
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

    private double checkValue(double hardInput,double MAX_SPEED) {
        hardInput = Range.clip(hardInput, -MAX_SPEED, MAX_SPEED);
        if(MAX_SPEED == 1) {
            hardInput = Math.pow(hardInput, 3);
        }
        return hardInput;
    }
    public void VerticalLiftPostions(double speed,double Timeout,boolean manualMode) {
        int newTargetVerticalLiftPosition;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTargetVerticalLiftPosition =(OuttakeLift.getCurrentPosition()-457);

            OuttakeLift.setTargetPosition(newTargetVerticalLiftPosition);
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
                    (OuttakeLift.isBusy()) && !manualMode) {
                // Display it for the driver.
                telemetry.addData("Path1", newTargetVerticalLiftPosition);
                telemetry.addData("Path2",  OuttakeLift.getCurrentPosition());
                telemetry.update();

            }

            OuttakeLift.setPower(0);
           OuttakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}

