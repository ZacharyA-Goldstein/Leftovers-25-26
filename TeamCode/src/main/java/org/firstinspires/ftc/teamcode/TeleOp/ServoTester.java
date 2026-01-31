package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * ServoTester
 *
 * A utility TeleOp for testing servo positions and behavior.
 *
 * Controls (Gamepad 1):
 * - D-pad Up/Down: Increase/Decrease position by small step
 * - D-pad Left/Right: Increase/Decrease position by large step
 * - Left/Right Bumpers: Fine adjust (very small steps)
 * - A: Set position to MIN (0.0)
 * - B: Set position to MAX (1.0)
 * - X: Set position to MID (0.5)
 * - Y: Toggle between continuous and position mode (if servo supports it)
 * - Right trigger: Hold to enable continuous adjustment mode
 *
 * Telemetry shows:
 * - Current position
 * - Target position
 * - Position change per button press
 */
@TeleOp(name = "Servo Tester", group = "Test")
public class ServoTester extends LinearOpMode {

    // --- CONFIGURATION ---
    // Change this to match your servo's hardware name
    private static final String SERVO_NAME = "hood"; // Change to your servo name
    
    // Position adjustment steps
    private static final double SMALL_STEP = 0.01;   // D-pad Up/Down
    private static final double LARGE_STEP = 0.05;   // D-pad Left/Right
    private static final double FINE_STEP = 0.001;    // Bumpers
    
    // Position limits
    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 1.0;
    private static final double MID_POSITION = 0.5;
    
    // --- HARDWARE ---
    private Servo servo;
    
    // --- STATE ---
    private double currentPosition = 0.5; // Start at middle
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {
        // Initialize servo
        try {
            servo = hardwareMap.get(Servo.class, SERVO_NAME);
            if (servo != null) {
                servo.setPosition(currentPosition);
                telemetry.addLine("Servo '" + SERVO_NAME + "' found!");
            } else {
                telemetry.addLine("ERROR: Servo '" + SERVO_NAME + "' NOT FOUND!");
                telemetry.addLine("Check the SERVO_NAME constant at the top of the code.");
            }
        } catch (Exception e) {
            telemetry.addLine("ERROR initializing servo: " + e.getMessage());
        }
        
        telemetry.addLine("\n=== Servo Tester ===");
        telemetry.addLine("D-pad Up/Down: small step");
        telemetry.addLine("D-pad Left/Right: large step");
        telemetry.addLine("LB/RB: fine adjust");
        telemetry.addLine("A: MIN (0.0)");
        telemetry.addLine("B: MAX (1.0)");
        telemetry.addLine("X: MID (0.5)");
        telemetry.addLine("Y: Toggle direction");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleInput();
            updateServo();
            updateTelemetry();
            sleep(20);
        }
        
        // Stop servo on exit
        if (servo != null) {
            servo.setPosition(currentPosition);
        }
    }
    
    private void handleInput() {
        // Small steps: D-pad Up/Down
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadUp && !lastDpadUp) {
            currentPosition = clampPosition(currentPosition + SMALL_STEP);
        }
        if (dpadDown && !lastDpadDown) {
            currentPosition = clampPosition(currentPosition - SMALL_STEP);
        }
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        
        // Large steps: D-pad Left/Right
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        if (dpadRight && !lastDpadRight) {
            currentPosition = clampPosition(currentPosition + LARGE_STEP);
        }
        if (dpadLeft && !lastDpadLeft) {
            currentPosition = clampPosition(currentPosition - LARGE_STEP);
        }
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        
        // Fine adjust: Bumpers
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        if (rightBumper && !lastRightBumper) {
            currentPosition = clampPosition(currentPosition + FINE_STEP);
        }
        if (leftBumper && !lastLeftBumper) {
            currentPosition = clampPosition(currentPosition - FINE_STEP);
        }
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
        
        // Preset positions
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        if (a && !lastA) {
            currentPosition = MIN_POSITION;
        }
        if (b && !lastB) {
            currentPosition = MAX_POSITION;
        }
        if (x && !lastX) {
            currentPosition = MID_POSITION;
        }
        lastA = a;
        lastB = b;
        lastX = x;
        
        // Continuous adjustment mode (hold right trigger)
        double rightTrigger = gamepad1.right_trigger;
        if (rightTrigger > 0.1) {
            // Continuous adjustment based on trigger value
            double adjustment = (rightTrigger - 0.1) * 0.9; // Scale to 0-0.9 range
            adjustment *= LARGE_STEP; // Scale to step size
            currentPosition = clampPosition(currentPosition + adjustment);
        }
        
        // Continuous adjustment mode (hold left trigger for reverse)
        double leftTrigger = gamepad1.left_trigger;
        if (leftTrigger > 0.1) {
            double adjustment = (leftTrigger - 0.1) * 0.9;
            adjustment *= LARGE_STEP;
            currentPosition = clampPosition(currentPosition - adjustment);
        }
    }
    
    private void updateServo() {
        if (servo != null) {
            servo.setPosition(currentPosition);
        }
    }
    
    private double clampPosition(double position) {
        return Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
    }
    
    private void updateTelemetry() {
        telemetry.addLine("=== Servo Tester ===");
        telemetry.addData("Servo Name", SERVO_NAME);
        
        if (servo != null) {
            telemetry.addData("Position", "%.4f", currentPosition);
            telemetry.addData("Position %", "%.1f%%", currentPosition * 100.0);
            
            // Visual indicator
            int barLength = 40;
            int filled = (int) (currentPosition * barLength);
            StringBuilder bar = new StringBuilder();
            bar.append("[");
            for (int i = 0; i < barLength; i++) {
                if (i < filled) {
                    bar.append("=");
                } else {
                    bar.append(" ");
                }
            }
            bar.append("]");
            telemetry.addData("Visual", bar.toString());
            
            // Show which preset is closest
            double distToMin = Math.abs(currentPosition - MIN_POSITION);
            double distToMid = Math.abs(currentPosition - MID_POSITION);
            double distToMax = Math.abs(currentPosition - MAX_POSITION);
            if (distToMin < 0.01) {
                telemetry.addData("Preset", "MIN (0.0)");
            } else if (distToMax < 0.01) {
                telemetry.addData("Preset", "MAX (1.0)");
            } else if (distToMid < 0.01) {
                telemetry.addData("Preset", "MID (0.5)");
            } else {
                telemetry.addData("Preset", "Custom");
            }
        } else {
            telemetry.addLine("ERROR: Servo not found!");
            telemetry.addLine("Check SERVO_NAME constant.");
        }
        
        telemetry.addLine("\nControls:");
        telemetry.addLine("D-pad U/D: ±" + SMALL_STEP);
        telemetry.addLine("D-pad L/R: ±" + LARGE_STEP);
        telemetry.addLine("LB/RB: ±" + FINE_STEP);
        telemetry.addLine("A=MIN, B=MAX, X=MID");
        telemetry.addLine("Triggers: Continuous adjust");
        
        telemetry.update();
    }
}
