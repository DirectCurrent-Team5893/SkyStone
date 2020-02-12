package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Enabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FTC_5893_2019.CustomTenserFlow5893;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Blue(Left)BuildingSite(Platform)", group = "Linear Opmode")

public class LeftBuildingSiteAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;

    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;
    private DcMotor OuttakeLift = null;
    private DcMotor HorizontalLift = null;

    private
    Servo Grabber;
    Servo LeftBlockGrabber;
    Servo RightBlockGrabber;
    Servo LeftBaseplateShover;
    Servo RightBaseplateShover;
    Servo ShoveBlock;


    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

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

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


/*
        Guides for strafeing
        encoderDrive(0.6,  -10,  10,-10,-10, 5.0);
         right 10 Inches with 5 Sec timeout
       encoderDrive(.6, 10,-10, 10,10, 5.0);
        left 10 inches with 5 sec timeout?
 */
        telemetry.addData("Reset foundation mover", "Start");
        LeftBaseplateShover.setPosition(0);
        RightBaseplateShover.setPosition(1);
        telemetry.addData("Reset Foundation mover", "Completed");
        telemetry.update();

        telemetry.addData("move Forward 6 inches", "Begun");
        telemetry.update();
        encoderDrive(1, -6, -6, 6, -6, 0);
        telemetry.addData("Move Forward 6 inches", "Complete");
        TurnOffAllMotors();

        telemetry.addData("right Strafe", "Begun");
        telemetry.update();
        encoderDrive(1, -10, 10, -10, -10, 0);
        telemetry.addData("right Strafe", "Complete");
        TurnOffAllMotors();

        telemetry.addData("left 90 degree turn", "Begun");
        telemetry.update();
        encoderDrive(1, 23, -23, -23, -23, 0);
        telemetry.addData("left 90 degree turn", "Complete");
        TurnOffAllMotors();

        telemetry.addData("move Forward 25 inches", "Begun");
        telemetry.update();
        encoderDrive(1, 21, 21, -21, 21, 0);
        encoderDrive(.6, 4, 4, -4, 4, 0);
        telemetry.addData("Move Forward 25 inches", "Complete");
        TurnOffAllMotors();

        telemetry.addData("Lower foundation mover", "Start");
        LeftBaseplateShover.setPosition(.7);
        RightBaseplateShover.setPosition(.2);
        telemetry.addData("Lower Foundation mover", "Completed");
        telemetry.update();

        Thread.sleep(1500);

        telemetry.addData("right Strafe", "Begun");
        telemetry.update();
        encoderDrive(1, 7, -7, 7, 7, 0);
        encoderDrive(.4, 10, -10, 10, 10, 0);
        telemetry.addData("right Strafe", "Complete");
        TurnOffAllMotors();

        telemetry.addData("left Strafe", "Begun");
        telemetry.update();
        encoderDrive(1, -8, 8, -8, -8, 0);
        telemetry.addData("left Strafe", "Complete");
        TurnOffAllMotors();

        telemetry.addData("Arc", "Begun");
        telemetry.update();
        arcTurn(.8, 44, -32, -33, 0);
        telemetry.addData("Arc", "Complete");
        TurnOffAllMotors();

        telemetry.addData("left Strafe", "Begun");
        telemetry.update();
        encoderDrive(1, -45, 45, -45, -45, 0);
        telemetry.addData("left Strafe", "Complete");
        TurnOffAllMotors();

        telemetry.addData("left 45 degree turn", "Begun");
        telemetry.update();
        encoderDrive(1, 14, -14, -14, -14, 0);
        telemetry.addData("left 45 degree turn", "Complete");
        TurnOffAllMotors();

        telemetry.addData("Raise foundation mover", "Start");
        LeftBaseplateShover.setPosition(0);
        RightBaseplateShover.setPosition(1);
        telemetry.addData("Raise Foundation mover", "Completed");
        telemetry.update();
        TurnOffAllMotors();

        Thread.sleep(1500);

        telemetry.addData("left 15 degree turn", "Begun");
        telemetry.update();
        encoderDrive(1, -6, 6, 6, 6, 0);
        telemetry.addData("left 15 degree turn", "Complete");
        TurnOffAllMotors();

        telemetry.addData("move Forward 5 inches", "Begun");
        telemetry.update();
        encoderDrive(1, 5, 5, -5, 5, 0);
        telemetry.addData("Move Forward 5 inches", "Complete");
        TurnOffAllMotors();

        telemetry.addData("move Backward 44 inches to park", "Begun");
        telemetry.update();
        encoderDrive(1, -38, -38, 38, -38, 0);
        telemetry.addData("Move Backward 44 inches to park", "Complete");
        TurnOffAllMotors();
    }

    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches, double backLeftInches,
                             double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        double ErrorAmount;
        boolean goodEnough = false;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() || frontRight.isBusy()) || (backLeft.isBusy() || backRight.isBusy())&& !goodEnough) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",

                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
                telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
                telemetry.addData("backLeft", backLeft.getCurrentPosition());
                telemetry.addData("frontRight", frontRight.getCurrentPosition());
                telemetry.addData("backright", backRight.getCurrentPosition());

                telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget)-(backLeft.getCurrentPosition())))
                        +(Math.abs(((newFrontLeftTarget)-(frontLeft.getCurrentPosition()))))
                        +(Math.abs((newBackRightTarget)-(backRight.getCurrentPosition())))
                        +(Math.abs(((newFrontRightTarget)-(frontRight.getCurrentPosition())))))/COUNTS_PER_INCH);
                if(ErrorAmount<.4)
                {
                    goodEnough = true;
                }
            }

            // Stop all motion;

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after eah move
        }
    }

    public void arcTurn(double speed,
                        double frontLeftInches, double frontRightInches, double backRightInches,
                        double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() || frontRight.isBusy()) || backRight.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",

                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after eah move
        }
    }
    public void TurnOffAllMotors()
    {
        // Stop all motion;

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}

