package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Position Test", group = "Test")
public class ServoPositionTest extends OpMode {

    // --- Servo ---
    private Servo testServo;

    // --- Position tracking ---
    private double currentPosition = 0.5; // Start at middle position
    private double lastSetPosition = -1.0; // Track last position sent to servo
    private static final double POSITION_INCREMENT = 0.01; // Amount to change per button press
    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 1.0;

    // --- Button tracking ---
    private boolean aWasPressed = false; // Button to increase position
    private boolean bWasPressed = false; // Button to decrease position

    @Override
    public void init() {
        // Map the servo - change "servo" to match your hardware configuration name
        // Common names: "servo", "liftGate", "shooterServo", etc.
        // NOTE: This is for POSITIONAL servos (not CRServo/continuous rotation)
        try {
            testServo = hardwareMap.get(Servo.class, "servo");
            testServo.setPosition(currentPosition);
            lastSetPosition = currentPosition; // Track that we've set initial position
            telemetry.addData("Status", "Servo Initialized");
            telemetry.addData("Servo Name", "servo");
            telemetry.addData("Servo Type", "Positional (0.0 to 1.0)");
        } catch (Exception e) {
            testServo = null;
            telemetry.addData("Status", "ERROR: Servo 'servo' NOT FOUND!");
            telemetry.addData("Error", "Change servo name in code (line 28)");
            telemetry.addData("Common names", "servo, liftGate, shooterServo");
        }

        telemetry.addData("Instructions", "A = +0.1, B = -0.1");
        telemetry.addData("Position", "%.2f", currentPosition);
        telemetry.addLine();
        telemetry.addData("IMPORTANT", "Check Status above - if ERROR, change servo name!");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a; // Increase position
        boolean bPressed = gamepad1.b; // Decrease position

        // --- Increase position when A is pressed ---
        if (aPressed && !aWasPressed) {
            currentPosition += POSITION_INCREMENT;
            if (currentPosition > MAX_POSITION) {
                currentPosition = MAX_POSITION;
            }
        }

        // --- Decrease position when B is pressed ---
        if (bPressed && !bWasPressed) {
            currentPosition -= POSITION_INCREMENT;
            if (currentPosition < MIN_POSITION) {
                currentPosition = MIN_POSITION;
            }
        }

        // --- Only set servo position when it actually changes (prevents continuous updates) ---
        if (testServo != null && Math.abs(currentPosition - lastSetPosition) > 0.001) {
            testServo.setPosition(currentPosition);
            lastSetPosition = currentPosition; // Remember what we set
        }

        // --- Update previous button states ---
        aWasPressed = aPressed;
        bWasPressed = bPressed;

        // --- Telemetry ---
        telemetry.addData("Instructions", "A = +0.1, B = -0.1");
        telemetry.addData("Button A", aPressed ? "PRESSED" : "not pressed");
        telemetry.addData("Button B", bPressed ? "PRESSED" : "not pressed");
        telemetry.addData("Target Position", "%.2f", currentPosition);
        if (testServo != null) {
            telemetry.addData("Servo Position", "%.2f", testServo.getPosition());
            telemetry.addData("Status", "OK - Positional Servo");
            if (Math.abs(currentPosition - lastSetPosition) < 0.001) {
                telemetry.addData("Servo State", "HOLDING POSITION");
            } else {
                telemetry.addData("Servo State", "MOVING...");
            }
        } else {
            telemetry.addData("Status", "ERROR: Servo not found!");
            telemetry.addData("Fix", "Check servo name in config");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (testServo != null) {
            testServo.setPosition(currentPosition); // Keep current position
            telemetry.addData("Status", "Test Stopped");
            telemetry.addData("Final Position", "%.2f", currentPosition);
            telemetry.update();
        }
    }
}

