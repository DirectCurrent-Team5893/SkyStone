/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="BlueBlockSide", group ="Concept")
public class BlueBlockSiteWebcamTest extends LinearOpMode {

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            " AUZDG8f/////AAABmaazVMUrgE3DsW62VUJjlMc1wra8JWNvNazE8edoKBKAtbY04213dls4wAW5jKyxCVXhGR0uR2AD90/bEtm+e7U5z63qpgQDnlDtEplsZwLZsNCsjguBCCGZuAcjvnbpfLuBQDVPJ9v0IepRczqFVg2LsMaZgjIJhYwrJOAS0xrNgDXy571FjcP9JTTsnofDkjL3vyi1tJgBWsIfCKNpkJBeMjtrM1GenDtHzwgEULtIv3XkRb0rIu1Xh/OF4N37wWOyEIm1NaT0hDJq5mHBWj/uxDnXIthdO7zxgLymdRxoWsHQg7IBfeWzp3apJnZog3OIVh7RSbn9X8b+zCgKdPGUli6K/NjBHCKIY4j/gnFm ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
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

    Servo Grabber;
    Servo LeftBlockGrabber;
    Servo RightBlockGrabber;
    Servo LeftBaseplateShover;
    Servo RightBaseplateShover;
    Servo ShoveBlock;
    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.8;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    int numOfTimesMoved = 0;
    double DOWN_POSITION = 1;
    double STRAFE_TO_BLOCK = 17;
    public double amountError = 0.64;


    @Override public void runOpMode() {
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

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        telemetry.addLine("Config Finish");
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
telemetry.addLine("Vuforia SetUp");
        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:
        telemetry.addLine("motor direction");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        HorizontalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();
        waitForStart();
        targetsSkyStone.activate();
        boolean skystoneVisible = false;
        targetVisible = false;




        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.




            gyroDrive(DRIVE_SPEED, 6, 6,6, 6, 0,100);
            telemetry.update();
            telemetry.addData("Initial Right Strafe", "Begun");
            telemetry.update();
            encoderDrive(DRIVE_SPEED,23, -23, -23, 23, 60);
            telemetry.addData("Initial Right Strafe", "Complete");

            telemetry.addLine("Vision Starts");

            int skystonePosition = 1;
            double frontRightInitialEncoders = 0;
            double frontLeftInitialEncoders = 0;
            double backRightInitialEncoders = 0;
            double backLeftInitialEncoders =0;

            double frontRightFinalEncoders = 0;
            double frontLeftFinalEncoders = 0;
            double backRightFinalEncoders = 0;
            double backLeftFinalEncoders = 0;

            frontLeftInitialEncoders = frontLeft.getCurrentPosition();
            frontRightInitialEncoders = frontRight.getCurrentPosition();
            backLeftInitialEncoders = backLeft.getCurrentPosition();
            backRightInitialEncoders = backRight.getCurrentPosition();
            gyroTurn(TURN_SPEED,0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            while (!skystoneVisible && !isStopRequested() && numOfTimesMoved <= 6) {
=======
            //checks for visibility of a skystone
=======
>>>>>>> parent of 913a779... added comments for code printing
=======
>>>>>>> parent of 913a779... added comments for code printing
=======
>>>>>>> parent of 913a779... added comments for code printing
            while (!skystoneVisible && !isStopRequested()) {
>>>>>>> 913a7793b8344bd0293f84cadb4131b3a884c847
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        if (trackable.getName().equals("Stone Target")) {
                            telemetry.addLine("Stone Target is Life");
                            skystoneVisible = true;
                            telemetry.addData("skystoneVisible", skystoneVisible);

                        } else {
                            targetVisible = true;
                        }
                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                }
                else {
                    telemetry.addData("Visible Target", "none");
                }

                if ((targetVisible && skystoneVisible)|| numOfTimesMoved>=6) {
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);

//                frontLeftFinalEncoders = Math.abs(frontLeft.getCurrentPosition());
//                frontRightFinalEncoders = Math.abs(frontRight.getCurrentPosition());
//                backLeftFinalEncoders = Math.abs(backLeft.getCurrentPosition());
//                backRightFinalEncoders = Math.abs(backRight.getCurrentPosition());
                    String[] Positions = {"UNKNOWN POSITION", "LEFT POSITION", "MIDDLE POSITION", "RIGHT POSITION"};
                    telemetry.addLine("Skystone detected");
                    gyroTurn(TURN_SPEED,0);
                    telemetry.addData("move Forward 60 inches to the foundation", "Begun");
                    telemetry.update();
                  gyroDrive(DRIVE_SPEED, -1, -1, -1, -1, 0,10);
                    telemetry.addData("Move Forward 60 inches to the foundation", "Complete");

                    telemetry.addData("strafe right 3.7 inches", "Begun");
                    telemetry.update();
                    encoderDrive(DRIVE_SPEED, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, 10);
                    telemetry.addData("strafe right 3.7 inches", "Complete");
                    telemetry.addData("Lower Right Block Grabber", "Begun");
                    telemetry.update();
                    RightBlockGrabber.setPosition(DOWN_POSITION);
                    sleep(1500);
                    telemetry.addData("Lower Right Block Grabber", "Complete");
                    telemetry.addData("Position is", Positions[skystonePosition]);
                    break;

                } else if (!targetVisible && !skystoneVisible) {


                    telemetry.addData("SKYSTONE NOT FOUND", "We be moving backward still");
//                frontLeftFinalEncoders = Math.abs(frontLeft.getCurrentPosition());
//                frontRightFinalEncoders = Math.abs(frontRight.getCurrentPosition());
//                backLeftFinalEncoders = Math.abs(backLeft.getCurrentPosition());
//                backRightFinalEncoders = Math.abs(backRight.getCurrentPosition());
                    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    encoderDrive(DRIVE_SPEED,-4,-4,-4,-4,5);
                    numOfTimesMoved ++;
                    backLeftFinalEncoders = backLeft.getCurrentPosition();
                    backRightFinalEncoders = backRight.getCurrentPosition();
                    frontLeftFinalEncoders = frontLeft.getCurrentPosition();
                    frontRightFinalEncoders = frontRight.getCurrentPosition();


                }
            }
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            double frontRightInchesMoved = Math.abs((frontRightFinalEncoders - frontRightInitialEncoders) / COUNTS_PER_INCH);
            double backRightInchesMoved = Math.abs((backRightFinalEncoders - backRightInitialEncoders) / COUNTS_PER_INCH);
            double frontLeftInchesMoved = Math.abs((frontLeftFinalEncoders - frontLeftInitialEncoders) / COUNTS_PER_INCH);
            double backLeftInchesMoved = Math.abs(backLeftFinalEncoders - backLeftInitialEncoders) / COUNTS_PER_INCH;
            double averageInchesMoved = Math.abs((frontLeftInchesMoved + backLeftInchesMoved + frontRightInchesMoved + backRightInchesMoved)/4);
            telemetry.addData("numOfTimesMoved",numOfTimesMoved);

            telemetry.addData("Strafe Left 4 inches", "Begun");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, 10);
            telemetry.addData("Strafe Left 4 inches", "Complete");


            telemetry.addData("move Forward 60 inches to the foundation", "Begun");
            telemetry.update();
            gyroDrive(DRIVE_SPEED, 32+averageInchesMoved, 32+averageInchesMoved, 32+averageInchesMoved, 32+averageInchesMoved, 0,10);
            telemetry.addData("Move Forward 60 inches to the foundation", "Complete");

            telemetry.addData("Raise Right Block Grabber", "Begun");
            telemetry.update();
            RightBlockGrabber.setPosition(0);
            telemetry.addData("Raise Right Block Grabber", "Complete");
            gyroTurn(TURN_SPEED,0);
            telemetry.addData("move backward 72 inches", "Begun");
            telemetry.update();
            gyroDrive(DRIVE_SPEED, -56-averageInchesMoved, -56-averageInchesMoved, -56-averageInchesMoved, -56-averageInchesMoved, 0,10);
            if(numOfTimesMoved>5)
            {
                gyroDrive(DRIVE_SPEED,-3,-3,-3,-3,0,100);
            }
            telemetry.addData("Move backward 72 inches", "Complete");
            gyroTurn(TURN_SPEED,0);
            telemetry.addData("move Forward 60 inches to the foundation", "Begun");
            telemetry.update();
            //gyroDrive(3, -3, -3, -3, -3, 0,10);
            telemetry.addData("Move Forward 60 inches to the foundation", "Complete");
            telemetry.addData("strafe right 3.7 inches", "Begun");
            telemetry.update();
            if(numOfTimesMoved< 6)
            {
                gyroDrive(DRIVE_SPEED, 2, 2, 2, 2, 0, 10);
            }
            encoderDrive(DRIVE_SPEED, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, 10);
            telemetry.addData("strafe right 3.7 inches", "Complete");
            telemetry.update();
            RightBlockGrabber.setPosition(DOWN_POSITION);
            sleep(1500);
            telemetry.addData("strafe right 3.7 inches", "Begun");
            telemetry.update();

            gyroTurn(TURN_SPEED,0);
            encoderDrive(DRIVE_SPEED, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, 10);
            telemetry.addData("strafe right 3.7 inches", "Complete");
            telemetry.addData("Lower Right Block Grabber", "Complete");
            telemetry.addData("move Forward 68 inches", "Begun");
            telemetry.update();
            gyroDrive(DRIVE_SPEED, 58+averageInchesMoved, 58+averageInchesMoved, 58+averageInchesMoved,58+averageInchesMoved, 0,10);
            telemetry.addData("Move Forward 68 inches", "Complete");

            telemetry.addData("Raise Right Block Grabber", "Begun");
            telemetry.update();
            RightBlockGrabber.setPosition(.1);
            telemetry.addData("Raise Right Block Grabber", "Complete");

            gyroTurn(TURN_SPEED,0);
            if (numOfTimesMoved <=3){
             gyroDrive(DRIVE_SPEED,-76-averageInchesMoved,-76-averageInchesMoved,-76-averageInchesMoved,-76-averageInchesMoved,0,10);
                gyroTurn(TURN_SPEED,0);
                telemetry.addData("move Forward 60 inches to the foundation", "Begun");
                telemetry.update();
                //gyroDrive(3, -3, -3, -3, -3, 0,DRIVE_SPEED0);
                telemetry.addData("Move Forward 60 inches to the foundation", "Complete");
                telemetry.addData("strafe right 3.7 inches", "Begun");
                telemetry.update();
                gyroDrive(DRIVE_SPEED, 2, 2, 2, 2, 0,10);
                encoderDrive(DRIVE_SPEED, 14, -14, -14, 14, 10);
                telemetry.addData("strafe right 3.7 inches", "Complete");
                telemetry.update();
                RightBlockGrabber.setPosition(DOWN_POSITION);
                sleep(1500);
                encoderDrive(DRIVE_SPEED, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, 10);
                gyroDrive(DRIVE_SPEED,72+averageInchesMoved,72+averageInchesMoved,72+averageInchesMoved,72+averageInchesMoved,0,10);
                RightBlockGrabber.setPosition(.1);
            }
            else if(numOfTimesMoved<6 && numOfTimesMoved>3)
            {
                gyroDrive(DRIVE_SPEED,-20-averageInchesMoved,-20-averageInchesMoved,-20-averageInchesMoved,-20-averageInchesMoved,0,10);
                gyroTurn(TURN_SPEED,0);
                telemetry.addData("move Forward 60 inches to the foundation", "Begun");
                telemetry.update();
                //gyroDrive(3, -3, -3, -3, -3, 0,10);
                telemetry.addData("Move Forward 60 inches to the foundation", "Complete");
                telemetry.addData("strafe right 3.7 inches", "Begun");
                telemetry.update();
                gyroDrive(DRIVE_SPEED, 2, 2, 2, 2, 0,10);
                encoderDrive(DRIVE_SPEED, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, 10);
                telemetry.addData("strafe right 3.7 inches", "Complete");
                telemetry.update();
                RightBlockGrabber.setPosition(DOWN_POSITION);
                sleep(1250);
                encoderDrive(DRIVE_SPEED, -STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, STRAFE_TO_BLOCK, -STRAFE_TO_BLOCK, 10);
                gyroDrive(DRIVE_SPEED,20+averageInchesMoved,20+averageInchesMoved,20+averageInchesMoved,20+averageInchesMoved,0,10);
                RightBlockGrabber.setPosition(.1);
            }
            telemetry.addData("move backward 10 inches", "Begun");
            telemetry.update();
            gyroDrive(DRIVE_SPEED, -12, -18, -18, -12, 0,10);
            telemetry.addData("Move backward 10 inches", "Complete");
            telemetry.update();


            // Disable Tracking when we are done;
            targetsSkyStone.deactivate();
            telemetry.update();


        // Disable Tracking when we are done
        targetsSkyStone.deactivate();
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param frontLeftInches  Distance (in inches) to move from current position for front Left.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double frontLeftInches, double frontRightInches, double backLeftInches,
                            double backRightInches,
                            double angle,double timeoutS) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int     moveCounts;

        double HalfMaxOne;
        double HalfMaxTwo;

        double  max;

        double  error;
        double  steer;
        double  frontLeftSpeed;
        double  frontRightSpeed;
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
                if (frontLeftInches< 0 && frontRightInches <0 && backLeftInches<0 && backRightInches <0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo =Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max= Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0)
                {
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
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newBackLeftTarget,newBackRightTarget,newFrontLeftTarget,newFrontRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",    backLeft.getCurrentPosition(),backRight.getCurrentPosition(),frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
                telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget)-(backLeft.getCurrentPosition())))
                        +(Math.abs(((newFrontLeftTarget)-(frontLeft.getCurrentPosition()))))
                        +(Math.abs((newBackRightTarget)-(backRight.getCurrentPosition())))
                        +(Math.abs(((newFrontRightTarget)-(frontRight.getCurrentPosition())))))/COUNTS_PER_INCH);
                if(ErrorAmount<amountError)
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
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */



    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */




    public void gyroHold( double speed, double angle, double holdTime) {

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
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontLeft.setPower(leftSpeed);
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);
        frontRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed,rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
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
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy())&& !goodEnough) {

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
                if(ErrorAmount<amountError)
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
}
