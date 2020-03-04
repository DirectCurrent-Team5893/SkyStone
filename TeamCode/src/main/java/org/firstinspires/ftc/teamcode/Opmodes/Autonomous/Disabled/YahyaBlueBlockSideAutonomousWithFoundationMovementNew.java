package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Disabled;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Tests.SkystoneDetectorExample;

@Autonomous(name = "NewBlueBlockSideWithFoundation-Direct", group = "Concept")
public class YahyaBlueBlockSideAutonomousWithFoundationMovementNew extends LinearOpMode {


    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION*.95) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.8;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    public double amountError = 0.64;
    public SkystoneDetectorExample.SkyStonePosition skystonePostion = SkystoneDetectorExample.SkyStonePosition.UNKNOWN;
    SkystoneDetectorExample detector;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    Servo Grabber;
    Servo LeftBlockGrabber;
    Servo RightBlockGrabber;
    Servo LeftBaseplateShover;
    Servo RightBaseplateShover;
    Servo ShoveBlock;
    ModernRoboticsI2cGyro gyro = null; // Additional Gyro device
    DistanceSensor sensorRange;
    int numOfTimesMoved = 0;
    double DOWN_POSITION = .8;
    double STRAFE_TO_BLOCK = 14;
    double initDistanceFromBlocks;
    double distanceToDifferentBlock = 14;
    int TIME_FOR_ARM_TO_DROP = 1500;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;
    private DcMotor OuttakeLift = null;
    private DcMotor HorizontalLift = null;


    @Override
    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();
        telemetry.addLine("Timer is made");
        telemetry.update();
        /*
         * Retrieve the camera we are to use.
         */
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

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        telemetry.addLine("Config Finish");


        telemetry.addLine("motor direction");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        HorizontalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        telemetry.addLine("reseting Encoders");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        timer.reset();
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        detector = new SkystoneDetectorExample(this, true, true);
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();

//full movement=33 inches
        initDistanceFromBlocks = 28;
        waitForStart();
        Grabber.setPosition(.53);
        skystonePostion = detector.getDecision();
        telemetry.addData("Decision:", skystonePostion);
        switch (skystonePostion) {
            case LEFT:
                encoderDrive(.7, initDistanceFromBlocks, -initDistanceFromBlocks, -initDistanceFromBlocks, initDistanceFromBlocks, 5);

                gyroTurn(.6, 0);

                gyroDrive(.7, distanceToDifferentBlock + 2, distanceToDifferentBlock + 2, distanceToDifferentBlock + 2, distanceToDifferentBlock + 2, 0, 5);

               gyroTurn(1, 270);

//                encoderDrive(.7, 4, -4, -4, 4, 0);

                encoderCollectionDrive(.7, 1, -10, -10, -10, -10, 0);

                gyroTurn(.6, 0);

                encoderDrive(.2, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, 0);

                Grabber.setPosition(.2);

                gyroDrive(.7, 75, 75, 75, 75, 0, 0);

                gyroTurn(.6, 90);

                telemetry.addData("move Backward 25 inches", "Begun");
                telemetry.update();
                encoderDrive(.7, 7, 7, 7, 7, 0);
                encoderDrive(.4, 4, 4, 4, 4, 0);
                telemetry.addData("Move Backward 25 inches", "Complete");
                //TurnOffAllMotors();

                telemetry.addData("Lower foundation mover", "Start");
                LeftBaseplateShover.setPosition(1);
                RightBaseplateShover.setPosition(0);
                telemetry.addData("Lower Foundation mover", "Completed");
                telemetry.update();
                //TurnOffAllMotors();

                sleep(1000);

                telemetry.addData("Arc", "Begun");
                telemetry.update();
                gyroTurn(DRIVE_SPEED, 180);
                telemetry.addData("Arc", "Complete");
                //TurnOffAllMotors();

                HorizontalLift.setTargetPosition(50);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                OuttakeLift.setTargetPosition(-457);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);

                HorizontalLift.setTargetPosition(150);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                telemetry.addData("Raise foundation mover", "Start");
                LeftBaseplateShover.setPosition(0);
                RightBaseplateShover.setPosition(1);
                telemetry.addData("Raise Foundation mover", "Completed");
                telemetry.update();
                //TurnOffAllMotors();

                sleep(1000);

                HorizontalLift.setPower(0);

                OuttakeLift.setPower(0);
                Grabber.setPosition(.53);

                HorizontalLift.setTargetPosition(0);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                gyroDrive(.7, -2, -2, -2, -2, 0, 0);

                OuttakeLift.setTargetPosition(0);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);

                gyroTurn(1, 0);

                HorizontalLift.setPower(0);
                OuttakeLift.setPower(1);

                gyroDrive(.7, -91, -91, -91, -91, 0, 0);

                gyroTurn(.6, 0);

                encoderDrive(.7, -STRAFE_TO_BLOCK - 25, STRAFE_TO_BLOCK + 25, STRAFE_TO_BLOCK + 25, -STRAFE_TO_BLOCK - 25, 5);

                encoderCollectionDrive(.7, 1, -6, -6, -6, -6, 0);

                Grabber.setPosition(.2);

                encoderDrive(.7, STRAFE_TO_BLOCK + 29, -STRAFE_TO_BLOCK - 29, -STRAFE_TO_BLOCK - 29, STRAFE_TO_BLOCK + 29, 5);

                gyroDrive(.7, -90, -90, -90, -90, 0, 0);

                HorizontalLift.setTargetPosition(50);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                OuttakeLift.setTargetPosition(-914);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);

                HorizontalLift.setTargetPosition(150);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                Grabber.setPosition(.53);

                HorizontalLift.setTargetPosition(0);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                gyroDrive(.7, -2, -2, -2, -2, 0, 0);

                OuttakeLift.setTargetPosition(0);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);
                gyroTurn(1, 0);

                HorizontalLift.setPower(0);
                OuttakeLift.setPower(0);

                gyroDrive(.7, -38, -38, -38, -38, 0, 0);

                OuttakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;


            case MIDDLE:


                encoderDrive(.7, initDistanceFromBlocks, -initDistanceFromBlocks, -initDistanceFromBlocks, initDistanceFromBlocks, 0);

                gyroTurn(.6, 0);

                gyroDrive(.7, distanceToDifferentBlock - 4, distanceToDifferentBlock - 4, distanceToDifferentBlock - 4, distanceToDifferentBlock - 4, 0, 0);

                gyroTurn(1, 270);

                encoderDrive(.7, 4, -4, -4, 4, 0);

                encoderCollectionDrive(.7, 1, -6, -6, -6, -6, 0);

                gyroTurn(.6, 0);

                encoderDrive(.4, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, 0);

                Grabber.setPosition(.2);

                gyroDrive(.7, 83, 83, 83, 83, 0, 0);

                gyroTurn(.6, 90);

                telemetry.addData("move Backward 25 inches", "Begun");
                telemetry.update();
                encoderDrive(.7, 7, 7, 7, 7, 0);
                encoderDrive(.4, 4, 4, 4, 4, 0);
                telemetry.addData("Move Backward 25 inches", "Complete");
                //TurnOffAllMotors();

                telemetry.addData("Lower foundation mover", "Start");
                LeftBaseplateShover.setPosition(1);
                RightBaseplateShover.setPosition(0);
                telemetry.addData("Lower Foundation mover", "Completed");
                telemetry.update();
                //TurnOffAllMotors();

                sleep(1000);

                telemetry.addData("Arc", "Begun");
                telemetry.update();
                gyroTurn(DRIVE_SPEED, 180);
                telemetry.addData("Arc", "Complete");
                //TurnOffAllMotors();

                HorizontalLift.setTargetPosition(50);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                OuttakeLift.setTargetPosition(-457);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);

                HorizontalLift.setTargetPosition(150);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                telemetry.addData("Raise foundation mover", "Start");
                LeftBaseplateShover.setPosition(0);
                RightBaseplateShover.setPosition(1);
                telemetry.addData("Raise Foundation mover", "Completed");
                telemetry.update();
                //TurnOffAllMotors();

                sleep(1000);

                HorizontalLift.setPower(0);

                OuttakeLift.setPower(0);
                Grabber.setPosition(.53);

                HorizontalLift.setTargetPosition(0);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                gyroDrive(.7, -2, -2, -2, -2, 0, 0);

                OuttakeLift.setTargetPosition(0);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);

                gyroTurn(1, 0);

                HorizontalLift.setPower(0);
                OuttakeLift.setPower(0);

                gyroDrive(.7, -99, -99, -99, -99, 0, 0);

                gyroTurn(.6, 0);

                encoderDrive(.7, -STRAFE_TO_BLOCK - 25, STRAFE_TO_BLOCK + 25, STRAFE_TO_BLOCK + 25, -STRAFE_TO_BLOCK - 25, 5);

                encoderCollectionDrive(.7, 1, -6, -6, -6, -6, 0);

                Grabber.setPosition(.2);

                encoderDrive(.7, STRAFE_TO_BLOCK + 29, -STRAFE_TO_BLOCK - 29, -STRAFE_TO_BLOCK - 29, STRAFE_TO_BLOCK + 29, 5);

                gyroDrive(.7, -98, -98, -98, -98, 0, 0);

                HorizontalLift.setTargetPosition(50);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                OuttakeLift.setTargetPosition(-914);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);

                HorizontalLift.setTargetPosition(150);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                Grabber.setPosition(.53);

                HorizontalLift.setTargetPosition(0);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                gyroDrive(.7, -2, -2, -2, -2, 0, 0);

                OuttakeLift.setTargetPosition(0);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);
                gyroTurn(1, 0);

                HorizontalLift.setPower(0);
                OuttakeLift.setPower(0);

                gyroDrive(.7, -38, -38, -38, -38, 0, 0);

                OuttakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;


            case RIGHT:


                encoderDrive(.7, initDistanceFromBlocks, -initDistanceFromBlocks, -initDistanceFromBlocks, initDistanceFromBlocks, 0);

                gyroTurn(.6, 0);

//                gyroDrive(.7,distanceToDifferentBlock-4,distanceToDifferentBlock-4,distanceToDifferentBlock-4,distanceToDifferentBlock-4,0,0);

                gyroTurn(1, 270);

                encoderDrive(.7, 4, -4, -4, 4, 0);

                encoderCollectionDrive(.7, 1, -6, -6, -6, -6, 0);

                gyroTurn(.6, 0);

                encoderDrive(.4, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, 0);

                Grabber.setPosition(.2);

                gyroDrive(.7, 91, 91, 91, 91, 0, 0);

                gyroTurn(.6, 90);

                telemetry.addData("move Backward 25 inches", "Begun");
                telemetry.update();
                encoderDrive(.7, 7, 7, 7, 7, 0);
                encoderDrive(.4, 4, 4, 4, 4, 0);
                telemetry.addData("Move Backward 25 inches", "Complete");
                //TurnOffAllMotors();

                telemetry.addData("Lower foundation mover", "Start");
                LeftBaseplateShover.setPosition(1);
                RightBaseplateShover.setPosition(0);
                telemetry.addData("Lower Foundation mover", "Completed");
                telemetry.update();
                //TurnOffAllMotors();

                sleep(1000);

                telemetry.addData("Arc", "Begun");
                telemetry.update();
                gyroTurn(DRIVE_SPEED, 180);
                telemetry.addData("Arc", "Complete");
                //TurnOffAllMotors();

                HorizontalLift.setTargetPosition(50);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                OuttakeLift.setTargetPosition(-457);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);

                HorizontalLift.setTargetPosition(150);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                telemetry.addData("Raise foundation mover", "Start");
                LeftBaseplateShover.setPosition(0);
                RightBaseplateShover.setPosition(1);
                telemetry.addData("Raise Foundation mover", "Completed");
                telemetry.update();
                //TurnOffAllMotors();

                sleep(1000);

                HorizontalLift.setPower(0);

                OuttakeLift.setPower(0);
                Grabber.setPosition(.53);

                HorizontalLift.setTargetPosition(0);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                gyroDrive(.7, -2, -2, -2, -2, 0, 0);

                OuttakeLift.setTargetPosition(0);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);

                gyroTurn(1, 0);

                HorizontalLift.setPower(0);
                OuttakeLift.setPower(0);

                gyroDrive(.7, -107, -107, -107, -107, 0, 0);

                gyroTurn(.6, 0);

                encoderDrive(.7, -STRAFE_TO_BLOCK - 25, STRAFE_TO_BLOCK + 25, STRAFE_TO_BLOCK + 25, -STRAFE_TO_BLOCK - 25, 5);

                encoderCollectionDrive(.7, 1, -6, -6, -6, -6, 0);

                Grabber.setPosition(.2);

                encoderDrive(.7, STRAFE_TO_BLOCK + 29, -STRAFE_TO_BLOCK - 29, -STRAFE_TO_BLOCK - 29, STRAFE_TO_BLOCK + 29, 5);

                gyroDrive(.7, -105, -105, -105, -105, 0, 0);

                HorizontalLift.setTargetPosition(50);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                OuttakeLift.setTargetPosition(-914);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);

                HorizontalLift.setTargetPosition(150);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                Grabber.setPosition(.53);

                HorizontalLift.setTargetPosition(0);
                HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HorizontalLift.setPower(1);

                gyroDrive(.7, -2, -2, -2, -2, 0, 0);

                OuttakeLift.setTargetPosition(0);
                OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                OuttakeLift.setPower(1);
                gyroTurn(1, 0);

                HorizontalLift.setPower(0);
                OuttakeLift.setPower(0);

                gyroDrive(.7, -38, -38, -38, -38, 0, 0);

                OuttakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
        }


    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed           Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param frontLeftInches Distance (in inches) to move from current position for front Left.  Negative distance means move backwards.
     * @param angle           Absolute Angle (in Degrees) relative to last gyro reset.
     *                        0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                        If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double frontLeftInches, double frontRightInches, double backLeftInches,
                          double backRightInches,
                          double angle, double timeoutS) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int moveCounts;

        double HalfMaxOne;
        double HalfMaxTwo;

        double max;

        double error;
        double steer;
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;

        double ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);


            // Set Target and Turn On RUN_TO_POSITION
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy()) && !goodEnough) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (frontLeftInches < 0 && frontRightInches < 0 && backLeftInches < 0 && backRightInches < 0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                frontLeft.setPower(frontLeftSpeed);
                frontRight.setPower(frontRightSpeed);
                backLeft.setPower(backLeftSpeed);
                backRight.setPower(backRightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Actual", "%7d:%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
                telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (backLeft.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (frontLeft.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (backRight.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (frontRight.getCurrentPosition()))))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
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
        }
    }

    public void gyroDriveWithUpAndOut(double speed,
                                      double frontLeftInches, double frontRightInches, double backLeftInches,
                                      double backRightInches,
                                      double angle, double timeoutS) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int moveCounts;

        double HalfMaxOne;
        double HalfMaxTwo;

        double max;

        double error;
        double steer;
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;

        double ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);


            // Set Target and Turn On RUN_TO_POSITION
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);
            OuttakeLift.setTargetPosition(-60);
            HorizontalLift.setTargetPosition(-200);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            OuttakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            HorizontalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy()) && !goodEnough) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
                if (runtime.seconds() > 1) {
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    Grabber.setPosition(.2);
                }
                if (runtime.seconds() > 2) {
                    HorizontalLift.setPower(.8);
                }
                if (runtime.seconds() > 2.5) {
                    OuttakeLift.setPower(.8);
                }
                // adjust relative speed
                // based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (frontLeftInches < 0 && frontRightInches < 0 && backLeftInches < 0 && backRightInches < 0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                frontLeft.setPower(frontLeftSpeed);
                frontRight.setPower(frontRightSpeed);
                backLeft.setPower(backLeftSpeed);
                backRight.setPower(backRightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Actual", "%7d:%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
                telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (backLeft.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (frontLeft.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (backRight.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (frontRight.getCurrentPosition()))))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            OuttakeLift.setPower(0);
            HorizontalLift.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroCollectionDrive(double speed,
                                    double frontLeftInches, double frontRightInches, double backLeftInches,
                                    double backRightInches, double intakeSpeed,
                                    double angle, double timeoutS) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int moveCounts;

        double HalfMaxOne;
        double HalfMaxTwo;

        double max;

        double error;
        double steer;
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;

        double ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);


            // Set Target and Turn On RUN_TO_POSITION
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy()) && !goodEnough) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (frontLeftInches < 0 && frontRightInches < 0 && backLeftInches < 0 && backRightInches < 0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                frontLeft.setPower(frontLeftSpeed);
                frontRight.setPower(frontRightSpeed);
                backLeft.setPower(backLeftSpeed);
                backRight.setPower(backRightSpeed);
                rightIntake.setPower(intakeSpeed);
                leftIntake.setPower(intakeSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Actual", "%7d:%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
                telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (backLeft.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (frontLeft.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (backRight.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (frontRight.getCurrentPosition()))))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            rightIntake.setPower(0);
            leftIntake.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */


    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();


        }

        // Stop all motion;
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontLeft.setPower(leftSpeed);
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);
        frontRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -DRIVE_SPEED, 1);
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
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy()) && !goodEnough) {

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

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (backLeft.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (frontLeft.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (backRight.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (frontRight.getCurrentPosition()))))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
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

    public void encoderCollectionDrive(double speed, double intakeSpeed,
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
            rightIntake.setPower(intakeSpeed);
            leftIntake.setPower(intakeSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy()) && !goodEnough) {

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

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (backLeft.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (frontLeft.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (backRight.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (frontRight.getCurrentPosition()))))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            // Stop all motion;

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            rightIntake.setPower(0);
            leftIntake.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after eah move
        }
    }


}
