package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A simple OpMode to reset all motor encoders to zero.
 * Run this before running the MotorTester for the first time or when you need to reset positions.
 */
@TeleOp(name = "Reset Encoders Lime", group = "Test")
public class ResetEncodersLime extends LinearOpMode {
    private dumbMapLime robot;

    @Override
    public void runOpMode() {
        robot = new dumbMapLime(this);
        robot.initMotors();
        
        telemetry.addData("Status", "Ready to reset encoders");
        telemetry.addData("Press Start", "to reset all encoders to zero");
        telemetry.update();

        waitForStart();

        // Reset all motors
        resetMotor(robot.spinner, "Spinner");
        resetMotor(robot.shooter, "Shooter");

        telemetry.addData("Status", "All encoders have been reset to zero");
        telemetry.addData("Spinner Position", robot.spinner.getCurrentPosition());
        telemetry.addData("Shooter Position", robot.shooter.getCurrentPosition());
        telemetry.update();

        // Keep the program running to show the final positions
        while (opModeIsActive()) {
            idle();
        }
    }

    private void resetMotor(DcMotor motor, String motorName) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData(motorName, "Encoder Reset");
        telemetry.update();
        
        // Small delay to ensure the motor has time to reset
        sleep(100);
        
        // Set back to using encoders
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
