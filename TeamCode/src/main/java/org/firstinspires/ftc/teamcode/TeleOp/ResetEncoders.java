package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dumbMap;

/**
 * A simple OpMode to reset all motor encoders to zero.
 * Run this before running tests or when you need to reset positions.
 */
@TeleOp(name = "Reset Encoders", group = "Test")
public class ResetEncoders extends LinearOpMode {
    private dumbMap robot;

    @Override
    public void runOpMode() {
        robot = new dumbMap(this);
        robot.init2();
        
        telemetry.addData("Status", "Ready to reset encoders");
        telemetry.addData("Press Start", "to reset all encoders to zero");
        telemetry.update();

        waitForStart();

        // Reset all motors
        if (robot.spinner != null) {
            resetMotor(robot.spinner, "Spinner");
        }
        if (robot.shooter != null) {
            resetMotor(robot.shooter, "Shooter");
        }
        if (robot.leftFront != null) {
            resetMotor(robot.leftFront, "Left Front");
        }
        if (robot.rightFront != null) {
            resetMotor(robot.rightFront, "Right Front");
        }
        if (robot.leftBack != null) {
            resetMotor(robot.leftBack, "Left Back");
        }
        if (robot.rightBack != null) {
            resetMotor(robot.rightBack, "Right Back");
        }

        telemetry.addLine();
        telemetry.addData("Status", "All encoders have been reset to zero");
        if (robot.spinner != null) {
            telemetry.addData("Spinner Position", robot.spinner.getCurrentPosition());
        }
        if (robot.shooter != null) {
            telemetry.addData("Shooter Position", robot.shooter.getCurrentPosition());
        }
        if (robot.leftFront != null) {
            telemetry.addData("Left Front Position", robot.leftFront.getCurrentPosition());
        }
        if (robot.rightFront != null) {
            telemetry.addData("Right Front Position", robot.rightFront.getCurrentPosition());
        }
        if (robot.leftBack != null) {
            telemetry.addData("Left Back Position", robot.leftBack.getCurrentPosition());
        }
        if (robot.rightBack != null) {
            telemetry.addData("Right Back Position", robot.rightBack.getCurrentPosition());
        }
        telemetry.update();

        // Keep the program running to show the final positions
        while (opModeIsActive()) {
            idle();
        }
    }

    private void resetMotor(DcMotor motor, String motorName) {
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData(motorName, "Encoder Reset");
            telemetry.update();
            
            // Small delay to ensure the motor has time to reset
            sleep(100);
            
            // Set back to using encoders
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
