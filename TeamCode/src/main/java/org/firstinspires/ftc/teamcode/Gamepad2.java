
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Gamepad2test Linear OpMode", group="Linear Opmode")
//@Disabled
public class Gamepad2test.java extends LinearOpMode {

// Declare OpMode members.
private ElapsedTime runtime = new ElapsedTime();
//  private DcMotor leftDrive = null;
// private DcMotor rightDrive = null;

private DcMotor leftMotor = null;
private DcMotor rightMotor = null;
private DcMotor backrightMotor = null;
private DcMotor backleftMotor = null;

@Override
public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor  = hardwareMap.get(DcMotor.class, "MotorOne");
        rightMotor = hardwareMap.get(DcMotor.class, "MotorTwo");
        backleftMotor  = hardwareMap.get(DcMotor.class, "MotorThree");
        backrightMotor  = hardwareMap.get(DcMotor.class, "MotorFour");


        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /* Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)*/

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();

        if (gamepad1.a
        )
        {
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        backleftMotor.setPower(1);
        backrightMotor.setPower(1);
        }
        else if (gamepad1.b)
        {
        leftMotor.setPower(-1);
        rightMotor.setPower(-1);
        backleftMotor.setPower(1);
        backrightMotor.setPower(1);
        }
        else if (gamepad1.left_bumper)
        {
        leftMotor.setPower(1);
        backrightMotor.setPower(1);
        }
        else if (gamepad1.left_trigger)
        {
        leftMotor.setPower(-1);
        backrightMotor.setPower(-1);
        }
        else if (gamepad1.right_bumper)
        {
        rightMotor.setPower(1);
        backleftMotor.setPower(1);
        }
        else if (gamepad1.right_trigger)
        {
        rightMotor.setPower(-1);
        backleftMotor.setPower(-1);
        }
        else
        {
        leftMotor.setPower(-gamepad1.left_stick_y);
        backleftMotor.setPower(-gamepad1.right_stick_y);
        rightMotor.setPower(-gamepad1.right_stick_y);
        backrightMotor.setPower(-gamepad1.right_stick_y);
        }











            /*

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            */
        }
        }
        }
