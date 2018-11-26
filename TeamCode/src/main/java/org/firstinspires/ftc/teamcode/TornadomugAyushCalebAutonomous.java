package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

 @Autonomous(name="TornadomugAyushCalebAutonomous")
//@Disabled
public class TornadomugAyushCalebAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareAutonomous         robot   = new HardwareAutonomous();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.4;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset;
        telemetry.addData("MotorTelemetry",  "Starting at %7d :%7d :%7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition(),
                robot.backleftMotor.getCurrentPosition(),
                robot.backrightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //0,2,1,3
        myEncoderDrive(1, DRIVE_SPEED, 60, 60,5.0);
        myEncoderDrive(2, DRIVE_SPEED, 24, -24,5.0);
        myEncoderDrive(0, DRIVE_SPEED, 24, 24,5.0);
        myEncoderDrive(2, DRIVE_SPEED, 24, -24,5.0);
        myEncoderDrive(0, DRIVE_SPEED, 24, 24,5.0);
        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);0
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void myEncoderDrive(int direction,
                               double speed,
                               double LeftInches,
                               double RightInches,
                               double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if (direction == 0)
            {
                //Go forward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int)(RightInches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int)(RightInches * COUNTS_PER_INCH);

            } else if (direction == 1)
            {
                //Go backward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int)(RightInches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int)(RightInches * COUNTS_PER_INCH);

            }
            else if (direction == 2)
            {
                //Strafe Right
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int)(RightInches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int)(RightInches * COUNTS_PER_INCH);

            }
            else if (direction == 3)
            {
                //Strafe Left
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int)(RightInches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int)(RightInches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);

            }
            else
            {
                LeftInches = 0;
                RightInches = 0;
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int)(RightInches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int)(LeftInches * COUNTS_PER_INCH);

            }

            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);
            robot.backleftMotor.setTargetPosition(newLeftBackTarget);
            robot.backrightMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
            robot.backleftMotor.setPower(Math.abs(speed));
            robot.backrightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.backleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);   // optional pause after each move
        }
    }

}
