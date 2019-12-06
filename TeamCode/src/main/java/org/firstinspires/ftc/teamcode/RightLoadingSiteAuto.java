package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="RightLoadingSite", group="Linear Opmode")

public class LeftLoadingSiteAuto extends LinearOpMode {

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


    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        //hardware mapping
        leftIntake = hardwareMap.get(DcMotor.class, "Left Intake");
        rightIntake = hardwareMap.get(DcMotor.class,"right intake");
        backLeft = hardwareMap.get(DcMotor.class,"back left wheel") ;
        backRight = hardwareMap.get(DcMotor.class,"back right wheel");
        frontRight = hardwareMap.get(DcMotor.class,"Front Right wheel");
        frontLeft = hardwareMap.get(DcMotor.class,"Front Left wheel");
        HorizontalLift = hardwareMap.get(DcMotor.class,"HL");
        OuttakeLift = hardwareMap.get(DcMotor.class, "OL");
        Grabber = hardwareMap.get(Servo.class, "Grabber");
        LeftBlockGrabber = hardwareMap.get(Servo.class,"LBG");
        RightBlockGrabber = hardwareMap.get(Servo.class, "RBG");
        LeftBaseplateShover = hardwareMap.get(Servo.class,"LBS");
        RightBaseplateShover = hardwareMap.get(Servo.class,"RBS");
        ShoveBlock = hardwareMap.get(Servo.class, "SB");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

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

//Guides for strafeing
        //encoderDrive(0.6,  -10,  10,-10,-10, 5.0);
        // right 10 Inches with 5 Sec timeout
        //encoderDrive(.6, 10,-10, 10,10, 5.0);
        //left 10 inches with 5 sec timeout?

        telemetry.addData("Initial Left Strafe", "Begun");
        telemetry.update();
        encoderDrive(.6, 26,-26,26,26,10);
        telemetry.addData("Initial Left Strafe", "Complete");

        telemetry.addData("move backward 16.5 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, 16.5, 16.5, -16.5, 16.5, 10);
        telemetry.addData("Move backward 16.5 inches", "Complete");

        telemetry.addData("strafe left 3.7 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, 3.7, -3.7, 3.7, 3.7, 10);
        telemetry.addData("strafe right 3.7 inches", "Complete");

        telemetry.addData("Lower Right Block Grabber", "Begun");
        telemetry.update();
        RightBlockGrabber.setPosition(.7);
        telemetry.addData("Lower Right Block Grabber", "Complete");

        while (RightBlockGrabber.isBusy)
        {

        }

        telemetry.addData("Strafe Right 4 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, -4, 4, -4, -4, 10);
        telemetry.addData("Strafe Right 4 inches", "Complete");

        telemetry.addData("move Forward 60 inches to the foundation", "Begun");
        telemetry.update();
        encoderDrive(.6, -58,-58,58, -58,10);
        telemetry.addData("Move Forward 60 inches to the foundation", "Complete");

        telemetry.addData("Raise Right Block Grabber", "Begun");
        telemetry.update();
        RightBlockGrabber.setPosition(0);
        telemetry.addData("Raise Right Block Grabber", "Complete");

        while (RightBlockGrabber.isBusy)
        {

        }

        telemetry.addData("move backward 72 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, 72, 72, -72, 72, 10);
        telemetry.addData("Move backward 72 inches", "Complete");

        telemetry.addData("strafe left 4.7 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, 4.7, -4.7, 4.7, 4.7, 10);
        telemetry.addData("strafe Left 3.7 inches", "Complete");

        telemetry.addData("Lower Right Block Grabber", "Begun");
        telemetry.update();
        RightBlockGrabber.setPosition(.7);
        telemetry.addData("Lower Right Block Grabber", "Complete");

        while (RightBlockGrabber.isBusy)
        {

        }

        telemetry.addData("Strafe Right 4 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, -4, 4, -4, -4, 10);
        telemetry.addData("Strafe Right 3 inches", "Complete");

        telemetry.addData("move Forward 68 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, -68,-68,68, -68,10);
        telemetry.addData("Move Forward 68 inches", "Complete");

        telemetry.addData("Raise Right Block Grabber", "Begun");
        telemetry.update();
        RightBlockGrabber.setPosition(0);
        telemetry.addData("Raise Right Block Grabber", "Complete");

        while (RightBlockGrabber.isBusy)
        {

        }

        telemetry.addData("move backward 10 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, 10, 10, -10, 10, 10);
        telemetry.addData("Move backward 10 inches", "Complete");
        telemetry.update();
    }
    //    public void MovingStraight(double inches, String Status) {
//        //name variables
//        double ENCODERS_PER_INCH=1440;
//        double FRONTLEFT_CURRENT_POSITION = Math.abs(frontLeft.getCurrentPosition());
//        double FRONTRIGHT_CURRENT_POSITION =Math.abs(frontRight.getCurrentPosition());
//        double BACKLEFT_CURRENT_POSITION = Math.abs(backLeft.getCurrentPosition());
//        double BACKRIGHT_CURRENT_POSITION = Math.abs(backRight.getCurrentPosition());
//        double FRONTLEFT_TARGET_POSITION = Math.abs(frontLeft.getCurrentPosition())+ inches*ENCODERS_PER_INCH;
//        double FRONTRIGHT_TARGET_POSITION = Math.abs(frontRight.getCurrentPosition())+ inches*ENCODERS_PER_INCH;
//        double BACKLEFT_TARGET_POSITION = Math.abs(backLeft.getCurrentPosition())+ inches*ENCODERS_PER_INCH;
//        double BACKRIGHT_TARGET_POSITION = Math.abs(backRight.getCurrentPosition()) + inches*ENCODERS_PER_INCH;
//
//        //create an array for variables
//        double [] currentPositions= {FRONTLEFT_CURRENT_POSITION, FRONTRIGHT_CURRENT_POSITION, BACKLEFT_CURRENT_POSITION, BACKRIGHT_CURRENT_POSITION};
//        double [] targetPositions = {FRONTLEFT_TARGET_POSITION, FRONTRIGHT_TARGET_POSITION, BACKLEFT_TARGET_POSITION, BACKRIGHT_TARGET_POSITION};
//
//        //Boolean
//        boolean currentEqualsTarget = false;
//            while (currentEqualsTarget == false) {
//                 FRONTLEFT_CURRENT_POSITION = Math.abs(frontLeft.getCurrentPosition());
//                 FRONTRIGHT_CURRENT_POSITION =Math.abs(frontRight.getCurrentPosition());
//                 BACKLEFT_CURRENT_POSITION = Math.abs(backLeft.getCurrentPosition());
//                 BACKRIGHT_CURRENT_POSITION = Math.abs(backRight.getCurrentPosition());;
//
//                    for (int i=0;i<=3;i++ ) {
//                        if (currentPositions[i] >= targetPositions[i]) {
//                            currentEqualsTarget = true;
//                        } else if (currentPositions[i] < targetPositions[i]) {
//                            currentEqualsTarget = false;
//                            break;
//                        }
//                    }
//
//                    if(currentEqualsTarget ==  false)
//                    {
//                        frontRight.setPower(-1);
//                        backRight.setPower(-1);
//                        backLeft.setPower(-1);
//                        frontLeft.setPower(-1);
//                        telemetry.addData("Back Left ",BACKLEFT_CURRENT_POSITION);
//                        telemetry.addData("Front Left ",FRONTLEFT_CURRENT_POSITION);
//                        telemetry.addData("Back Right ",BACKRIGHT_CURRENT_POSITION);
//                        telemetry.addData("Front Right ",FRONTRIGHT_CURRENT_POSITION);
//                        telemetry.update();
//
//                    }
//            }
//
//        if (currentEqualsTarget)
//        {
//            frontRight.setPower(0);
//            backRight.setPower(0);
//            backLeft.setPower(0);
//            frontLeft.setPower(0);
//        }
//    }
    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
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
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",

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
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after eah move
        }
    }


}

