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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 * <p>
 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@Autonomous(name = "BlueBlockSide", group = "Concept")
@Disabled
public class BlueSiteAutoSarahV extends LinearOpMode {

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

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
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

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
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;


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
    public void runOpMode() {
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:
        telemetry.addLine("Robot Initialized");
        waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        boolean skystoneVisible = false;
        targetVisible = false;

//            for (VuforiaTrackable trackable : allTrackables) {
//                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                    telemetry.addData("Visible Target", trackable.getName());
//                    targetVisible = true;
//
//                    if (trackable.getName().equals("Stone Target")) {
//                        telemetry.addLine("Stone Target is Life");
//                        skystoneVisible = true;
//                        telemetry.addData("skystoneVisible", skystoneVisible);
//
//                    } else {
//                        targetVisible = true;
//                    }
//                    // getUpdatedRobotLocation() will return null if no new information is available since
//                    // the last time that call was made, or if the trackable is not currently visible.
//                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                    if (robotLocationTransform != null) {
//                        lastLocation = robotLocationTransform;
//                    }
//                    break;
//                }
//            }


        // Provide feedback as to where the robot is located (if we know).

        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }
        encoderDrive(.6, -8, -8, 8, -8, 10);
        telemetry.update();
        telemetry.addData("Initial Right Strafe", "Begun");
        telemetry.update();
        encoderDrive(.6, -22, 22, -22, -22, 60);
        telemetry.addData("Initial Right Strafe", "Complete");

//            telemetry.addData("A little shove to help", "Start");
//            encoderDrive(.6, -3.3, -3.3, 3.3, -3.3, 10);
//            telemetry.addData("A little shove to help", "Completed because Sarah is always right");
//            telemetry.update();
//            telemetry.addLine();
        telemetry.addLine("Vision Starts, and Anish career ends.");

        int skystonePosition = 1;
        double frontRightInitialEncoders = 0;
        double frontLeftInitialEncoders = 0;
        double backRightInitialEncoders = 0;
        double backLeftInitialEncoders =0;

        double frontRightFinalEncoders = 0;
        double frontLeftFinalEncoders = 0;
        double backRightFinalEncoders = 0;
        double backLeftFinalEncoders = 0;

        while (!skystoneVisible) {
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
            if (targetVisible && skystoneVisible) {
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

                telemetry.addData("strafe right 3.7 inches", "Begun");
                telemetry.update();
                encoderDrive(.6, -14, 14, -14, -14, 10);
                telemetry.addData("strafe right 3.7 inches", "Complete");
                telemetry.addData("Lower Right Block Grabber", "Begun");
                telemetry.update();
                RightBlockGrabber.setPosition(.8);
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
                frontLeft.setPower(.12);
                frontRight.setPower(.12);
                backLeft.setPower(-.12);
                backRight.setPower(.12);
            }
        }

        double frontRightInchesMoved = (frontRightFinalEncoders - frontRightInitialEncoders) / COUNTS_PER_INCH;
        double backRightInchesMoved = (backRightFinalEncoders - backRightInitialEncoders) / COUNTS_PER_INCH;
        double frontLeftInchesMoved = (frontLeftFinalEncoders - frontLeftInitialEncoders) / COUNTS_PER_INCH;
        double backLeftInchesMoved = (backLeftFinalEncoders - backLeftInitialEncoders) / COUNTS_PER_INCH;

//        telemetry.addData("move backward 16.5 inches", "Begun");
//        telemetry.update();
//        encoderDrive(.6, 16.5, 16.5, -16.5, 16.5, 10);
//        telemetry.addData("Move backward 16.5 inches", "Complete");


        telemetry.addData("Strafe Left 4 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, 14, -14, 14, 14, 10);
        telemetry.addData("Strafe Left 4 inches", "Complete");

        telemetry.addData("move Forward 60 inches to the foundation", "Begun");
        telemetry.update();
        encoderDrive(1, -58, -58, 58, -58, 10);
        telemetry.addData("Move Forward 60 inches to the foundation", "Complete");


        telemetry.addData("Raise Right Block Grabber", "Begun");
        telemetry.update();
        RightBlockGrabber.setPosition(0);
        telemetry.addData("Raise Right Block Grabber", "Complete");

        telemetry.addData("move backward 72 inches", "Begun");
        telemetry.update();
        encoderDrive(1, 82, 82, -82, 82, 10);
        telemetry.addData("Move backward 72 inches", "Complete");

        telemetry.addData("Strafe Left 4 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, 4.5, -4.5, 4.5, 4.5, 12);
        telemetry.addData("Strafe Left 3 inches", "Complete");




                    String[] Positions = {"UNKNOWN POSITION", "LEFT POSITION", "MIDDLE POSITION", "RIGHT POSITION"};
                    telemetry.addLine("Skystone detected");
                    encoderDrive(.5, -1.5, -1.5, 1.5, -1.5, 10);
                    telemetry.addData("strafe right 3.7 inches", "Begun");
                    telemetry.update();
                    encoderDrive(.6, -14, 14, -14, -14, 10);
                    telemetry.addData("strafe right 3.7 inches", "Complete");
                    telemetry.addData("Lower Right Block Grabber", "Begun");
                    telemetry.update();
                    RightBlockGrabber.setPosition(.8);
                    sleep(1500);
                    telemetry.addData("Lower Right Block Grabber", "Complete");
                    telemetry.addData("Position is", Positions[skystonePosition]);


        telemetry.addData("Strafe Left 4 inches", "Begun");
        telemetry.update();
        encoderDrive(.6, 16, -16, 16, 16, 10);
        telemetry.addData("Strafe Left 4 inches", "Complete");

        telemetry.addData("move Forward 68 inches", "Begun");
        telemetry.update();
        encoderDrive(1, -80, -80, 80, -80, 10);
        telemetry.addData("Move Forward 68 inches", "Complete");

        telemetry.addData("Raise Right Block Grabber", "Begun");
        telemetry.update();
        RightBlockGrabber.setPosition(0);
        telemetry.addData("Raise Right Block Grabber", "Complete");


        telemetry.addData("move backward 10 inches", "Begun");
        telemetry.update();
        encoderDrive(1, 16, 16, -16, 16, 10);
        telemetry.addData("Move backward 10 inches", "Complete");
        telemetry.update();


        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }

    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches, double backLeftInches,
                             double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

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
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy())) {

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




