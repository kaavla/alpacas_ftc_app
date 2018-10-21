
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="M_A_Gamepad Linear OpMode", group="Linear Opmode")
//@Disabled
public class AlphaGamepad extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //  private DcMotor leftDrive = null;
    // private DcMotor rightDrive = null;

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor backrightMotor = null;
    private DcMotor backleftMotor = null;
    private DcMotor tempMotor = null;


    @Override
    public void runOpMode() {

        double motor_power = 0.5;
        int count = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initialize motor variables
        leftMotor       = hardwareMap.get(DcMotor.class, "MFrontLeft");
        rightMotor      = hardwareMap.get(DcMotor.class, "MFrontRight");
        backleftMotor   = hardwareMap.get(DcMotor.class, "MBackLeft");
        backrightMotor  = hardwareMap.get(DcMotor.class, "MBackRight");
        tempMotor       = hardwareMap.get(DcMotor.class, "tempMotor");

        //Set default reverse for right motors to account for inverse motors
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "FL(%.1f),FR(%.1f),BL(%.1f),BR(%.1f), count(%d)", motor_power, motor_power, motor_power, motor_power, count);
            telemetry.update();

            if (gamepad1.dpad_up)
            {
                //Go Forward
                leftMotor.setPower(motor_power);
                rightMotor.setPower(motor_power);
                backleftMotor.setPower(motor_power);
                backrightMotor.setPower(motor_power);
            }
            else if (gamepad1.dpad_down)
            {
                //Go Backward
                leftMotor.setPower(-1*motor_power);
                rightMotor.setPower(-1*motor_power);
                backleftMotor.setPower(-1*motor_power);
                backrightMotor.setPower(-1*motor_power);
            }
            else if (gamepad1.dpad_right)
            {
                //Rotate Right on Axis
                leftMotor.setPower(motor_power);
                rightMotor.setPower(-1*motor_power);
                backleftMotor.setPower(motor_power);
                backrightMotor.setPower(-1*motor_power);
            }
            else if (gamepad1.dpad_left)
            {
                //Rotate Left on axis
                leftMotor.setPower(-1*motor_power);
                rightMotor.setPower(motor_power);
                backleftMotor.setPower(-1*motor_power);
                backrightMotor.setPower(motor_power);
            }
            else if (gamepad1.b)
            {
                //Strafe Right
                leftMotor.setPower(motor_power);
                rightMotor.setPower(-1*motor_power);
                backleftMotor.setPower(-1*motor_power);
                backrightMotor.setPower(motor_power);
            }
            else if (gamepad1.x)
            {
                //Strafe Left
                leftMotor.setPower(-1*motor_power);
                rightMotor.setPower(motor_power);
                backleftMotor.setPower(motor_power);
                backrightMotor.setPower(-1*motor_power);
            }
            else if (gamepad1.right_bumper)
            {
                //right diagonal forward
                leftMotor.setPower(motor_power);
                backrightMotor.setPower(motor_power);
            }
            else if (gamepad1.left_bumper)
            {
                //forward diagonal left
                rightMotor.setPower(motor_power);
                backleftMotor.setPower(motor_power);
            }
            else if (gamepad1.right_stick_button)
            {
                //right diagonal backward
                rightMotor.setPower(-1*motor_power);
                backleftMotor.setPower(-1*motor_power);
            }
            else if (gamepad1.left_stick_button)
            {
                //left diagonal backward
                leftMotor.setPower(-1*motor_power);
                backrightMotor.setPower(-1*motor_power);
            }
            else if (gamepad1.a)
            {
                count++;
                //Increase Power
                if (motor_power < 1.0) {
                    motor_power = motor_power + 0.1;
                }
            }
            else if (gamepad1.y)
            {
                //Reduce Power
                if (motor_power > 0.2) {
                    motor_power = motor_power - 0.1;
                }
            }
            else
            {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                backleftMotor.setPower(0);
                backrightMotor.setPower(0);
            }


            if (gamepad2.dpad_up)
            {
                tempMotor.setPower(0.2);
            }
            else if (gamepad2.dpad_down)
            {
                tempMotor.setPower(-0.2);
            } else {
                tempMotor.setPower(0);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "FL(%.1f),FR(%.1f),BL(%.1f),BR(%.1f), count(%d)", motor_power, motor_power, motor_power, motor_power, count);
            telemetry.update();
        }
    }
}
