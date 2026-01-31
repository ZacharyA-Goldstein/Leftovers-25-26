package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoFlickTest", group = "TeleOp")
public class ServoFlickTest extends OpMode {

    // --- Servo ---
    private Servo shooterServo;

    // --- Servo positions ---
    private static final double POSITION_DOWN = 0.1;   // 0 degrees
    private static final double POSITION_UP = 0.45;     // 90 degrees (adjust as needed)

    // --- Timing variables ---
    private boolean flicking = false;
    private double flickStartTime = 0;
    private static final double FLICK_DURATION = 5.0; // seconds

    // --- Button tracking ---
    private boolean aWasPressed = false;

    @Override
    public void init() {
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(POSITION_DOWN);

        telemetry.addData("Status", "Servo Initialized");
        telemetry.addData("Position", "Down (%.2f)", POSITION_DOWN);
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a;

        // --- Start flick when A is pressed ---
        if (aPressed && !aWasPressed && !flicking) {
            flicking = true;
            flickStartTime = getRuntime();
            shooterServo.setPosition(POSITION_UP);
        }

        // --- After delay, return servo to down position ---
        if (flicking && (getRuntime() - flickStartTime >= FLICK_DURATION)) {
            shooterServo.setPosition(POSITION_DOWN);
            flicking = false;
        }

        // --- Update previous button state ---
        aWasPressed = aPressed;

        // --- Telemetry ---
        telemetry.addData("Servo State", flicking ? "UP (%.2f)" : "DOWN (%.2f)",
                flicking ? POSITION_UP : POSITION_DOWN);
        telemetry.addData("Time Since Flick (s)", getRuntime() - flickStartTime);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterServo.setPosition(POSITION_DOWN);
        telemetry.addData("Status", "Servo Reset to Down");
        telemetry.update();
    }
}
