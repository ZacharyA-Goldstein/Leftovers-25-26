package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "transfer", group = "TeleOp")
public class transfer extends OpMode {

    private Servo servo;
    private double targetPosition = 0.0;

    // Track previous button states
    private boolean prevA = false;
    private boolean prevB = false;

    // GoBilda 5-turn servo constants
    private static final double TOTAL_TURNS = 5.0;
    private static final double DEGREES_PER_TURN = 360.0;
    private static final double TOTAL_DEGREES = TOTAL_TURNS * DEGREES_PER_TURN; // 1800°
    private static final double DEGREES_TO_UNITS = 1.0 / TOTAL_DEGREES; // convert degrees → servo position

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        targetPosition = 0.0; // starting position
        servo.setPosition(targetPosition);
    }

    @Override
    public void loop() {
        boolean currentA = gamepad1.a;
        boolean currentB = gamepad1.b;

        // --- EDGE DETECTION ---
        // A button pressed (transition from not pressed → pressed)
        if (currentA && !prevA) {
            targetPosition += 120.0 * DEGREES_TO_UNITS; // move +120 degrees (1/3 turn)
        }

        // B button pressed (transition from not pressed → pressed)
        if (currentB && !prevB) {
            targetPosition -= 360.0 * DEGREES_TO_UNITS; // move -360 degrees (1 turn)
        }

        // Clamp within [0.0, 1.0] so servo never exceeds its range
        targetPosition = Math.max(0.0, Math.min(1.0, targetPosition));
        servo.setPosition(targetPosition);

        // Update previous button states
        prevA = currentA;
        prevB = currentB;

        telemetry.addData("Target Position", targetPosition);
        telemetry.update();
    }
}
