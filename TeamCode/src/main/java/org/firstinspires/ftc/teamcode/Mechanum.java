package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="Mechanum", group="Linear Opmode")

public class Mechanum extends LinearOpMode {

    //define the motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;

    @Override
    public void runOpMode() {

//hardware mapping
        leftIntake = hardwareMap.get(DcMotor.class, "Left Intake");
        rightIntake = hardwareMap.get(DcMotor.class,"right intake");
        backLeft = hardwareMap.get(DcMotor.class,"back left wheel") ;
        backRight = hardwareMap.get(DcMotor.class,"back right wheel");
        frontRight = hardwareMap.get(DcMotor.class,"Front Right wheel");
        frontLeft = hardwareMap.get(DcMotor.class,"Front Left wheel");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        int STOP = 0;
        int FORWARD = 1;
        int BACKWARD = -1;
        waitForStart();
        while (opModeIsActive()) {
            drivetrain(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if (gamepad1.right_bumper)
            {
                leftIntake.setPower(FORWARD);
                rightIntake.setPower(BACKWARD);
            }
            else if (gamepad1.left_bumper)
            {
                leftIntake.setPower(BACKWARD);
                rightIntake.setPower(FORWARD);
            }
            else
            {
                leftIntake.setPower(STOP);
                rightIntake.setPower(STOP);
            }
            if (gamepad2.a)
            {
                backLeft.setPower(1);
            }
            else if (gamepad2.b)
            {
                frontLeft.setPower(BACKWARD);
            }
            else if(gamepad2.x)
            {
                frontRight.setPower(1);
            }
            else if(gamepad2.y)
            {
                backRight.setPower(1);
            }
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
        if (isItTooBig) {
            double greatest = 0;
            for(double power :powers){

                if(Math.abs(power)>greatest){
                    greatest=Math.abs(power);
                }

            }
            leftBackPower =leftBackPower/greatest;
            leftFrontPower =leftFrontPower/greatest;
            rightBackPower =rightBackPower/greatest;
            rightFrontPower =rightFrontPower/greatest;
        }
        boolean stop = true;
        for (double power : powers)
        {

            if(Math.abs(power)>.1)
            {
                stop = false;
            }
        }
        if (stop)
        {
            frontRight.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
            frontLeft.setPower(0);
        }
        else {
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
        telemetry.update();
    }
    private double checkValue(double hardInput) {
        hardInput = Range.clip(hardInput, -1,1);
        hardInput = Math.pow(hardInput,3);
        return hardInput;
    }
}

