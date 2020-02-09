package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MainTeleOp", group = "Linear Opmode")

public class TeleOp extends LinearOpMode {

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

    //define the servos
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
        CapstoneDeployment = hardwareMap.get(Servo.class, "CD");

        //make motors all run forward
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        //set the way the lift stops
        HorizontalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OuttakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //reset encoders
        OuttakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //call variables

        double MAX_SPEED = 1;
        double FAST_MODE = 1;
        double SLOW_MODE = 0.3;
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

        double IntakePower = 1;
        waitForStart();

        while (opModeIsActive()) {

            //assigns power and buttons to intake
            drivetrain(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, MAX_SPEED);
            if (gamepad1.right_bumper) {
                leftIntake.setPower(IntakePower);
                rightIntake.setPower(-IntakePower);
            } else if (gamepad1.left_bumper) {
                leftIntake.setPower(-.3);
                rightIntake.setPower(.3);
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
            //button hed function allows for switch between a slow and fast mode on the intake
            if (!gamepad1.dpad_down) {
                gamepad1dpadDownHeld = false;
            }
            if (gamepad1.y && !gamepad1yHeld) {
                gamepad1yHeld = true;
                if (MAX_SPEED == FAST_MODE) {
                    MAX_SPEED = SLOW_MODE;
                } else if (MAX_SPEED == SLOW_MODE) {
                    MAX_SPEED = FAST_MODE;
                }
            }
            if (!gamepad1.y) {
                gamepad1yHeld = false;
            }
            //allows switch positions to be enabled through one button rather than two
            GrabberPositions[] GRABBERPOSITIONS = {GrabberPositions.DOWN_POSITION, GrabberPositions.UP_POSITION};
            telemetry.addData("gamepad2.b is", gamepad2.b);
            telemetry.addData("gamepad2b held is", gamepad2bHeld);
            telemetry.addData("grabber changer is", grabberChanger);
            telemetry.addData("Ran method is", ranMethod);
            telemetry.addData("Ran MethodV2 is ", ranMethodV2);
            telemetry.addData("changing position RightBlockMover is ", rightBlockMover);
            telemetry.addData("changing position LeftBlockMover is ", leftBlockMover);
            telemetry.addData("changing position GrabberChanger is ", baseplateChanger);
            telemetry.addData("changing position GrabberChanger is ", baseplateChanger);
            telemetry.addData("Manual Mode", manualMode);
            CapstoneDeploymentPositions[] CAPSTONEDEPLOYMENTPOSITIONS = {CapstoneDeploymentPositions.DOWN_POSITION, CapstoneDeploymentPositions.UP_POSITION};
            if (gamepad2.x && gamepad2xHeld == false) {
                ranMethod++;
                gamepad2xHeld = true;
                telemetry.addData("changing position GrabberChanger is ", grabberChanger);

                SetCapstoneDeploymentPosition(CAPSTONEDEPLOYMENTPOSITIONS[CapstoneDeploymentChanger]);
                CapstoneDeploymentChanger++;
                CapstoneDeploymentChanger = CapstoneDeploymentChanger % 2;
            }
            if (!gamepad2.x) {
                gamepad2xHeld = false;
            }
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
            //baseplate mover one button switch position code
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
            if (gamepad2.a && !gamepad2aHeld) {
                ranMethod++;
                gamepad2aHeld = true;
                telemetry.addData("changing position GrabberChanger is ", grabberChanger);

                SetBaseplateMoverPosition(BASEPLATEMOVERPOSITIONS[baseplateChanger]);
                baseplateChanger++;
                baseplateChanger = baseplateChanger % 2;
            }
            if (!gamepad2.a) {
                gamepad2aHeld = false;
            }
            if (gamepad2.right_stick_button && !gamepad2rightStickButtonHeld) {
                gamepad2rightStickButtonHeld = true;
                manualMode = !manualMode;
            }
            if (!gamepad2.right_stick_button) {
                gamepad2rightStickButtonHeld = false;
            }

            //one button change position code for left block mover
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
            if (gamepad2.y) {
                OuttakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //code to switch between levels on vertical lift and manual mode
            if (!manualMode) {


                if (gamepad2.dpad_up && !gamepad2dpadUpHeld && !manualMode) {
                    gamepad2dpadUpHeld = true;
                    if (OuttakeLift.getCurrentPosition() >= -50) {
                        ranMethodV2++;
                        OuttakeLift.setTargetPosition(-477);
                        OuttakeLift.setPower(.6);
                        OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        while (opModeIsActive() &&
                                (OuttakeLift.isBusy())) {
                            // Display it for the driver.
                            telemetry.addData("Go Up",
                                    OuttakeLift.getCurrentPosition());
                            telemetry.update();

                            drivetrain(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, MAX_SPEED);

                            if (gamepad1.right_bumper) {
                                leftIntake.setPower(IntakePower);
                                rightIntake.setPower(-IntakePower);
                            } else if (gamepad1.left_bumper) {
                                leftIntake.setPower(-.3);
                                rightIntake.setPower(.3);
                            } else {
                                leftIntake.setPower(STOP);
                                rightIntake.setPower(STOP);
                            }

                            HorizontalLift.setPower(gamepad2.left_stick_y);


                        }

                        OuttakeLift.setPower(0);
                    } else {
                        VerticalLiftPostions(.6, 10, manualMode, MAX_SPEED, IntakePower);
                    }
                }
                if (!gamepad2.dpad_up) {
                    gamepad2dpadUpHeld = false;
                }
                if (gamepad2.dpad_down && !gamepad2dpadDownHeld && gamepad2.right_stick_y < .1) {
                    ranMethod++;
                    gamepad2dpadDownHeld = true;
                    OuttakeLift.setTargetPosition(0);
                    OuttakeLift.setPower(.6);
                    OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (opModeIsActive() &&
                            (OuttakeLift.isBusy())) {
                        // Display it for the driver.
                        telemetry.addData("Go Up",
                                OuttakeLift.getCurrentPosition());
                        telemetry.update();
                        drivetrain(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, MAX_SPEED);

                        if (gamepad1.right_bumper) {
                            leftIntake.setPower(IntakePower);
                            rightIntake.setPower(-IntakePower);
                        } else if (gamepad1.left_bumper) {
                            leftIntake.setPower(-.3);
                            rightIntake.setPower(.3);
                        } else {
                            leftIntake.setPower(STOP);
                            rightIntake.setPower(STOP);
                        }

                        HorizontalLift.setPower(gamepad2.left_stick_y);


                    }
                    OuttakeLift.setPower(0);
                    OuttakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    OuttakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (!gamepad2.dpad_down) {

                    gamepad2dpadDownHeld = false;
                }


            }
            //One button switch for right block mover positions
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
            //lift levels code
            if (gamepad2.dpad_down && !gamepad2dpadDownHeld && gamepad2.right_stick_y < .1) {
                ranMethod++;
                gamepad2dpadDownHeld = true;
                OuttakeLift.setTargetPosition(0);
                OuttakeLift.setPower(.6);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() &&
                        (OuttakeLift.isBusy())) {
                    // Display it for the driver.
                    telemetry.addData("Go Up",
                            OuttakeLift.getCurrentPosition());
                    telemetry.update();

                }
                OuttakeLift.setPower(0);
                OuttakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (!gamepad2.dpad_down) {
                gamepad2dpadDownHeld = false;
            }


            OuttakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            OuttakeLift.setPower(gamepad2.right_stick_y);
            telemetry.addData("Lift Power", gamepad2.right_stick_y);
            HorizontalLift.setPower(gamepad2.left_stick_y);
            telemetry.addData("OuttakeLift", OuttakeLift.getCurrentPosition());
            telemetry.update();
        }
    }

    //enum for Grabber position switch
    public enum GrabberPositions {
        UP_POSITION, DOWN_POSITION
    }

    //function for setting position
    public void SetPosition(final GrabberPositions POSITION) {
        double targetPosition;
        switch (POSITION) {
            case UP_POSITION:
                Grabber.setPosition(.7);
                break;
            case DOWN_POSITION:
                Grabber.setPosition(.3);
                break;
        }
    }

    //enum for Capstone servo position switch
    public enum CapstoneDeploymentPositions {
        UP_POSITION, DOWN_POSITION
    }

    //function for setting position
    public void SetCapstoneDeploymentPosition(final CapstoneDeploymentPositions POSITION) {
        double targetPosition;
        switch (POSITION) {
            case UP_POSITION:
                CapstoneDeployment.setPosition(.15);

                break;
            case DOWN_POSITION:
                CapstoneDeployment.setPosition(1);
                break;
        }
    }

    //enum for Baseplate mover servo position switch
    public enum BaseplateMoverPositions {
        UP_POSITION, DOWN_POSITION
    }

    //function for setting position of the servos
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

    //enum for right side block mover servo position switch
    public enum rightBlockMoverPositions {
        UP_POSITION, DOWN_POSITION
    }

    //function for setting position of the servo
    public void SetRightBlockMoverPosition(final rightBlockMoverPositions POSITION) {
        switch (POSITION) {
            case UP_POSITION:
                RightBlockGrabber.setPosition(0);

                break;
            case DOWN_POSITION:
                RightBlockGrabber.setPosition(.8);
                break;
        }
    }

    //enum for left side block mover servo position switch
    public enum leftBlockMoverPositions {
        UP_POSITION, DOWN_POSITION
    }

    //function for setting position of the servos
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

    //mechanum drive train motion calculation function
    private void drivetrain(double forward, double right, double turn, double MAX_SPEED) {

        forward = checkValue(forward, MAX_SPEED);
        right = checkValue(right, MAX_SPEED);
        turn = checkValue(turn, MAX_SPEED);
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

    private double checkValue(double hardInput, double MAX_SPEED) {
        hardInput = Range.clip(hardInput, -MAX_SPEED, MAX_SPEED);
        if (MAX_SPEED == 1) {
            hardInput = Math.pow(hardInput, 3);
        }
        return hardInput;
    }

    public void VerticalLiftPostions(double speed, double Timeout, boolean manualMode,double MAX_SPEED,double IntakePower) {
        int newTargetVerticalLiftPosition;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTargetVerticalLiftPosition = (OuttakeLift.getCurrentPosition() - 457);

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
                telemetry.addData("Path2", OuttakeLift.getCurrentPosition());
                telemetry.update();
                drivetrain(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, MAX_SPEED);

                if (gamepad1.right_bumper) {
                    leftIntake.setPower(IntakePower);
                    rightIntake.setPower(-IntakePower);
                } else if (gamepad1.left_bumper) {
                    leftIntake.setPower(-.2);
                    rightIntake.setPower(.2);
                } else {
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                }

                HorizontalLift.setPower(gamepad2.left_stick_y);


            }

            OuttakeLift.setPower(0);
            OuttakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

