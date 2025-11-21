package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A simple motor tester that allows controlling a motor's position using the gamepad.
 * X: +10 ticks
 * Y: -10 ticks
 * A: +100 ticks
 * B: -100 ticks
 */
@TeleOp(name = "Motor Tester", group = "Test")
public class MotorTester extends OpMode {
    private dumbMapLime robot;
    private DcMotor currentMotor;
    private int targetPosition = 0;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {
        robot = new dumbMapLime(this);
        robot.initMotors();
        
        currentMotor = robot.spinner; // Default to spinner
        
        // Set up motors
        setupMotor(robot.spinner);
        setupMotor(robot.shooter);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Current Motor", "Spinner");
        telemetry.addData("Controls", "X:+10  Y:-10  A:+100  B:-100");
        telemetry.addData("DPad Up/Down", "Switch between motors");
        telemetry.update();
    }

    private void setupMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5); // Set a safe power level
    }

    @Override
    public void loop() {
        // Toggle between motors with DPad
        if (gamepad1.dpad_up) {
            currentMotor = robot.spinner;
            telemetry.addData("Current Motor", "Spinner");
        } else if (gamepad1.dpad_down) {
            currentMotor = robot.shooter;
            telemetry.addData("Current Motor", "Shooter");
        }

        // Handle button presses for position control
        if (gamepad1.x && !lastX) {
            targetPosition += 10;
            updateMotorPosition();
        } else if (gamepad1.y && !lastY) {
            targetPosition -= 10;
            updateMotorPosition();
        } else if (gamepad1.a && !lastA) {
            targetPosition += 100;
            updateMotorPosition();
        } else if (gamepad1.b && !lastB) {
            targetPosition -= 100;
            updateMotorPosition();
        }

        // Update button states
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        // Update telemetry
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentMotor.getCurrentPosition());
        telemetry.addData("Is Busy", currentMotor.isBusy());
        telemetry.update();
    }

    private void updateMotorPosition() {
        currentMotor.setTargetPosition(targetPosition);
        currentMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
