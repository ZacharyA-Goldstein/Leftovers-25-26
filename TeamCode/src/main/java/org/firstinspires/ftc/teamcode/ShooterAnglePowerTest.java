package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;

/**
 * Test class for adjustable vertical shooter angle (servo hood) and shooter power (motor).
 * Allows manual adjustment of both angle and power for tuning.
 */
@TeleOp(name = "Shooter Angle & Power Test", group = "Test")
public class ShooterAnglePowerTest extends LinearOpMode {
    private dumbMap robot;
    
    // Shooter power motor (outtake wheel)
    private DcMotor shooterMotor;
    
    // Shooter angle servo (hood)
    private Servo hoodServo;
    
    // Limelight + AprilTag detection
    private AprilTagDetector tagDetector;
    private Limelight3A limelight;
    
    // --- TUNING: Camera mounting ---
    private static final double CAMERA_HEIGHT_IN = 13.0; // Inches. Measure lens height from floor
    private static final double CAMERA_ANGLE_DEG = 0.0; // Degrees down from horizontal. Measure on robot
    private static final double MAX_SHOT_DISTANCE_IN = 144.0; // Maximum distance for tag detection
    
    // --- TUNING: Shooter power control ---
    private double currentPower = 0.0;
    private static final double POWER_INCREMENT = 0.05;  // Power change per button press
    private static final double MIN_POWER = 0.0;
    private static final double MAX_POWER = 1.0;
    
    // --- TUNING: Hood servo positions ---
    private double currentHoodPosition = 0;  // Start at middle position
    private static final double HOOD_POS_INCREMENT = 0.001;  // Position change per button press
    private static final double MIN_HOOD_POS = 0.00;  // Minimum servo position (hood down/flat)
    private static final double MAX_HOOD_POS = 0.5;  // Maximum servo position (hood up/angled)
    
    // Button state tracking
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    
    @Override
    public void runOpMode() {
        // Initialize the robot hardware via dumbMap
        robot = new dumbMap(this);
        robot.init2();
        
        // Initialize Limelight using the helper inside dumbMap and create our detector
        robot.initLimeLight();
        limelight = robot.getLimeLight();
        if (limelight != null) {
            tagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT_IN, CAMERA_ANGLE_DEG, MAX_SHOT_DISTANCE_IN);
            telemetry.addData("Limelight", "Initialized");
        } else {
            tagDetector = null;
            telemetry.addData("Limelight", "NOT FOUND");
        }
        
        // Map shooter power motor
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setPower(0.0);
            telemetry.addData("Shooter Motor", "Mapped");
        } catch (Exception e) {
            shooterMotor = null;
            telemetry.addData("Shooter Motor", "NOT found - check hardware name 'shooter'");
        }
        
        // Map hood servo
        try {
            hoodServo = hardwareMap.get(Servo.class, "hood");
            hoodServo.setPosition(currentHoodPosition);
            telemetry.addData("Hood Servo", "Mapped");
        } catch (Exception e) {
            hoodServo = null;
            telemetry.addData("Hood Servo", "NOT found - check hardware name 'hood'");
        }
        
        telemetry.addData("Status", "Initialized. Press START to begin.");
        telemetry.addData("---", "---");
        telemetry.addData("Controls", "A/B = Increase/Decrease Power");
        telemetry.addData("Controls", "X/Y = Increase/Decrease Hood Angle");
        telemetry.addData("Controls", "D-pad Up/Down = Fine adjust Hood");
        telemetry.addData("Controls", "Right Bumper = Toggle Power On/Off");
        telemetry.addData("Controls", "Left Bumper = Reset Hood to 0.0");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Power control with A/B buttons
            boolean aPressed = gamepad1.a;
            boolean bPressed = gamepad1.b;
            
            if (aPressed && !lastA) {
                // Increase power
                currentPower = Math.min(MAX_POWER, currentPower + POWER_INCREMENT);
            }
            if (bPressed && !lastB) {
                // Decrease power
                currentPower = Math.max(MIN_POWER, currentPower - POWER_INCREMENT);
            }
            lastA = aPressed;
            lastB = bPressed;
            
            // Toggle power on/off with right bumper
            boolean rightBumper = gamepad1.right_bumper;
            if (rightBumper && !lastRightBumper) {
                if (currentPower > 0.0) {
                    currentPower = 0.0;  // Turn off
                } else {
                    currentPower = 0.5;  // Turn on to 50%
                }
            }
            lastRightBumper = rightBumper;
            
            // Hood angle control with X/Y buttons (coarse)
            boolean xPressed = gamepad1.x;
            boolean yPressed = gamepad1.y;
            
            if (xPressed && !lastX) {
                // Increase hood angle (move up)
                currentHoodPosition = Math.min(MAX_HOOD_POS, currentHoodPosition + HOOD_POS_INCREMENT);
            }
            if (yPressed && !lastY) {
                // Decrease hood angle (move down)
                currentHoodPosition = Math.max(MIN_HOOD_POS, currentHoodPosition - HOOD_POS_INCREMENT);
            }
            lastX = xPressed;
            lastY = yPressed;
            
            // Fine hood adjustment with D-pad
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            
            if (dpadUp && !lastDpadUp) {
                // Fine increase hood angle
                currentHoodPosition = Math.min(MAX_HOOD_POS, currentHoodPosition + (HOOD_POS_INCREMENT * 0.5));
            }
            if (dpadDown && !lastDpadDown) {
                // Fine decrease hood angle
                currentHoodPosition = Math.max(MIN_HOOD_POS, currentHoodPosition - (HOOD_POS_INCREMENT * 0.5));
            }
            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
            
            // Reset hood to zero with left bumper
            boolean leftBumper = gamepad1.left_bumper;
            if (leftBumper && !lastLeftBumper) {
                currentHoodPosition = 0.0;
            }
            lastLeftBumper = leftBumper;
            
            // Apply power to motor (negative for correct direction)
            if (shooterMotor != null) {
                shooterMotor.setPower(-currentPower);
            }
            
            // Apply position to servo
            if (hoodServo != null) {
                hoodServo.setPosition(currentHoodPosition);
            }
            
            // Limelight telemetry
            if (tagDetector != null) {
                AprilTagDetector.AprilTagResult tag = tagDetector.getClosestTag();
                if (tag.isValid) {
                    telemetry.addData("---", "---");
                    telemetry.addData("Limelight", "Tag Detected");
                    telemetry.addData("Tag ID", tag.tagId);
                    telemetry.addData("Distance (in)", "%.1f", tag.distance);
                    telemetry.addData("Angle (deg)", "%.1f", tag.angle);
                    telemetry.addData("tx (deg)", "%.2f", tag.xDegrees);
                    telemetry.addData("ty (deg)", "%.2f", tag.yDegrees);
                } else {
                    telemetry.addData("---", "---");
                    telemetry.addData("Limelight", "No Tag Detected");
                }
            } else {
                telemetry.addData("---", "---");
                telemetry.addData("Limelight", "Not Available");
            }
            
            // Telemetry
            telemetry.addData("---", "---");
            telemetry.addData("Shooter Power", "%.3f", currentPower);
            if (shooterMotor != null) {
                telemetry.addData("Motor Power (read)", "%.3f", shooterMotor.getPower());
            } else {
                telemetry.addData("Motor Power (read)", "Motor not found");
            }
            
            telemetry.addData("---", "---");
            telemetry.addData("Hood Position", "%.3f", currentHoodPosition);
            if (hoodServo != null) {
                telemetry.addData("Servo Position (read)", "%.3f", hoodServo.getPosition());
            } else {
                telemetry.addData("Servo Position (read)", "Servo not found");
            }
            
            telemetry.addData("---", "---");
            telemetry.addData("A/B", "Power +/-");
            telemetry.addData("X/Y", "Hood +/-");
            telemetry.addData("D-pad", "Fine Hood");
            telemetry.addData("RB", "Toggle Power");
            telemetry.addData("LB", "Reset Hood");
            telemetry.update();
        }
        
        // Stop motor when opmode ends
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
    }
}

