package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;

import java.util.List;

/**
 * AutoAimShooter - Automatically adjusts turret, hood, and shooter power using Limelight AprilTag detection
 * 
 * Features:
 * - Automatically aligns turret horizontally to AprilTag
 * - Automatically adjusts hood position based on distance
 * - Automatically adjusts shooter RPM based on distance
 * - Turret limited to ±90 degrees from initialization
 * - Maintains lock as bot moves
 * 
 * Hardware:
 * - spinner: Turret motor (horizontal alignment)
 * - hood: Hood servo (vertical angle)
 * - shooter: Shooter motor (power/RPM)
 */
@TeleOp(name = "Auto Aim Shooter", group = "Test")
public class AutoAimShooter extends LinearOpMode {
    private dumbMapLime robot;
    private Limelight3A limelight;
    private AprilTagDetector aprilTagDetector;
    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    
    // --- TUNING: Camera mounting ---
    private static final double CAMERA_HEIGHT = 13.0; // Inches. Measure lens height from floor
    private static final double CAMERA_ANGLE = 0.0; // Degrees down from horizontal. Measure on robot
    private static final double MAX_DISTANCE = 144.0; // Maximum distance for tag detection
    
    // --- TUNING: Target AprilTag ID (change for different alliances) ---
    private static final int TARGET_TAG_ID = 24; // AprilTag ID to track (24 for red, 20 for blue, etc.)
    
    // --- TUNING: Turret limits (±90 degrees from initialization) ---
    private static final int SPINNER_MIN = -550;  // Encoder limit (min) - -90 degrees left
    private static final int SPINNER_MAX = 550;   // Encoder limit (max) - +90 degrees right
    
    // --- TUNING: Horizontal turret alignment ---
    private static final double TURRET_KP = 0.03;      // Proportional gain (deg -> power) - reduced to prevent overshoot
    private static final double TURRET_MIN_POWER = 0.15; // Minimum power to move turret - reduced
    private static final double TURRET_MAX_POWER = 0.35;  // Maximum alignment power - reduced to prevent overshoot
    private static final double TURRET_DEADBAND = 1.5;  // Deadband - no movement within this (degrees) - increased
    private static final double TURRET_SLOW_ZONE = 5.0; // Zone where turret slows down significantly (degrees)
    private static final double TURRET_SLOW_POWER = 0.2; // Power used in slow zone
    private static final double HORIZONTAL_OFFSET_DEG = 2.0; // Offset to compensate for shooting left/right (tune this)
    
    // --- TUNING: Hood adjustment for close distances ---
    private static final double CLOSE_DISTANCE_THRESHOLD = 80.0; // Inches - below this is "close"
    private static final double HOOD_CLOSE_ADJUSTMENT = -0.02; // Subtract this from hood when close (lowers hood)
    
    // --- TUNING: Hood servo limits ---
    private static final double HOOD_MIN = 0.217; // Minimum hood position
    private static final double HOOD_MAX = 0.282; // Maximum hood position
    
    // --- TUNING: Shooter motor constants ---
    private static final int TICKS_PER_REVOLUTION = 28; // PPR for GoBILDA 6000 RPM motor
    
    // --- TUNING: RPM scaling (distance-based adjustment) ---
    private static double calculateRPMScaleFactor(double distance) {
        if (distance <= 80.0) {
            return 0.90; // Close distances
        } else if (distance >= 120.0) {
            return 1.0; // Far distances
        } else {
            // Linear interpolation between 80" and 120"
            double t = (distance - 80.0) / (120.0 - 80.0);
            return 0.90 + (1.0 - 0.90) * t;
        }
    }
    
    // Runtime variables
    private AprilTagDetector.AprilTagResult cachedTagResult = null;
    private int loopCounter = 0;
    private static final int LIMELIGHT_CALL_INTERVAL = 1; // Call Limelight every loop for faster response
    private int lostDetectionCount = 0;
    private static final int MAX_LOST_DETECTIONS = 2; // Max loops without detection before giving up (very fast response)
    
    // Current settings
    private double currentHoodPosition = (HOOD_MIN + HOOD_MAX) / 2.0;
    private double targetRPM = 0.0;
    private boolean tagDetected = false;
    private double lastValidTx = 0.0; // Store last valid tx for reference
    
    // Shooter control
    private boolean shooterOn = false;
    private boolean lastAButton = false;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        robot = new dumbMapLime(this);
        
        // Initialize motors
        try {
            robot.initMotors();
            
            // Configure spinner (turret) motor
            if (robot.spinner != null) {
                robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(100);
                robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.spinner.setDirection(DcMotor.Direction.FORWARD);
                robot.spinner.setPower(0.0);
                telemetry.addData("Spinner Motor", "Initialized");
            } else {
                telemetry.addData("Spinner Motor", "NOT FOUND");
            }
        } catch (Exception e) {
            telemetry.addData("Motor Init Error", e.getMessage());
        }
        
        // Initialize shooter motor (as DcMotorEx for velocity control)
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            if (shooterMotor == null) {
                shooterMotor = hardwareMap.get(DcMotorEx.class, "outtake");
            }
            if (shooterMotor != null) {
                shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
                shooterMotor.setVelocity(0);
                telemetry.addData("Shooter Motor", "Initialized (Velocity Control)");
            } else {
                telemetry.addData("Shooter Motor", "NOT FOUND");
            }
        } catch (Exception e) {
            telemetry.addData("Shooter Motor Error", e.getMessage());
        }
        
        // Initialize hood servo
        try {
            hoodServo = hardwareMap.get(Servo.class, "hood");
            if (hoodServo != null) {
                currentHoodPosition = (HOOD_MIN + HOOD_MAX) / 2.0;
                hoodServo.setPosition(currentHoodPosition);
                telemetry.addData("Hood Servo", "Initialized");
            } else {
                telemetry.addData("Hood Servo", "NOT FOUND");
            }
        } catch (Exception e) {
            telemetry.addData("Hood Servo Error", e.getMessage());
        }
        
        telemetry.addData("Status", "Hardware initialized. Press START to begin.");
        telemetry.update();
        
        // Wait for start
        waitForStart();
        
        // Initialize Limelight
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();
        
        try {
            String[] possibleNames = {
                "limelight", "limelight3a", "limelight3A", 
                "limelight_3a", "limelight_3A", "limelight3a_1",
                "limelight3a_2", "limelight3a_3", "limelight3a_4"
            };
            
            for (String name : possibleNames) {
                try {
                    limelight = hardwareMap.get(Limelight3A.class, name);
                    if (limelight != null) {
                        telemetry.addData("Found Limelight", "Name: '%s'", name);
                        break;
                    }
                } catch (Exception e) {
                    // Try next name
                }
            }
            
            if (limelight == null) {
                try {
                    for (Limelight3A device : hardwareMap.getAll(Limelight3A.class)) {
                        limelight = device;
                        telemetry.addLine("Found Limelight device (unnamed)");
                        break;
                    }
                } catch (Exception e) {
                    telemetry.addData("Error", "Getting Limelight devices: " + e.getMessage());
                }
            }
            
            if (limelight != null) {
                limelight.start();
                limelight.pipelineSwitch(0); // Pipeline 0 for AprilTag
                sleep(500);
                
                LLResult testResult = limelight.getLatestResult();
                if (testResult != null) {
                    telemetry.addLine("✅ Limelight initialized successfully!");
                    telemetry.addData("Pipeline", "0 (AprilTag)");
                } else {
                    telemetry.addLine("⚠️ Limelight found but not returning data yet");
                }
                
                aprilTagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT, CAMERA_ANGLE, MAX_DISTANCE);
            } else {
                telemetry.addData("ERROR", "❌ Limelight not found!");
                telemetry.addLine("Check hardware configuration");
            }
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight: " + e.getMessage());
            e.printStackTrace();
        }
        
        telemetry.update();
        
        // Main loop
        while (opModeIsActive()) {
            try {
                // Handle A button toggle for shooter
                handleShooterToggle();
                
                // Detect AprilTag and update all settings
                detectAndUpdate();
                
                // Update telemetry
                updateTelemetry();
                
                sleep(20);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Exception in main loop: " + e.getMessage());
                telemetry.update();
                try {
                    if (robot.spinner != null) {
                        robot.spinner.setPower(0.0);
                    }
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(0);
                    }
                } catch (Exception e2) {
                    // Ignore
                }
                sleep(100);
            }
        }
        
        // Stop all motors when OpMode ends
        try {
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            if (shooterMotor != null) {
                shooterMotor.setPower(0.0);
            }
            if (hoodServo != null) {
                hoodServo.setPosition((HOOD_MIN + HOOD_MAX) / 2.0);
            }
        } catch (Exception e) {
            // Ignore
        }
    }
    
    /**
     * Handle A button toggle for shooter
     */
    private void handleShooterToggle() {
        boolean aPressed = gamepad1.a;
        
        if (aPressed && !lastAButton) {
            shooterOn = !shooterOn;
        }
        lastAButton = aPressed;
        
        // Auto-stop shooter if tag is lost
        if (!tagDetected) {
            shooterOn = false;
        }
    }
    
    /**
     * Detect AprilTag and automatically update turret, hood, and shooter
     */
    private void detectAndUpdate() {
        if (limelight == null || aprilTagDetector == null) {
            // No Limelight - stop everything
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            if (shooterMotor != null) {
                try {
                    shooterMotor.setVelocity(0);
                } catch (Exception e) {
                    // Ignore
                }
            }
            return;
        }
        
        try {
            loopCounter++;
            boolean shouldCallLimelight = (loopCounter % LIMELIGHT_CALL_INTERVAL == 0);
            
            boolean freshTagDetected = false;
            double currentTx = 0.0;
            double currentDistance = 0.0;
            
            if (shouldCallLimelight) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();
                    
                    double MIN_TAG_AREA = 0.003;
                    List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    boolean hasTargetTag = false;
                    
                    if (fiducials != null && !fiducials.isEmpty()) {
                        for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid : fiducials) {
                            if (fid.getFiducialId() == TARGET_TAG_ID) {
                                hasTargetTag = true;
                                break;
                            }
                        }
                    }
                    
                    if (ta > MIN_TAG_AREA && hasTargetTag) {
                        try {
                            cachedTagResult = aprilTagDetector.getTagById(TARGET_TAG_ID);
                            if (cachedTagResult != null && cachedTagResult.isValid && cachedTagResult.distance > 0) {
                                freshTagDetected = true;
                                lostDetectionCount = 0;
                                tagDetected = true;
                                // Use FRESH tx from Limelight, not cached result
                                currentTx = tx;
                                currentDistance = cachedTagResult.distance;
                                lastValidTx = tx; // Store for reference
                            }
                        } catch (Exception e) {
                            // Ignore
                        }
                    }
                }
                
                // If no fresh tag detected this loop, increment lost count
                if (!freshTagDetected) {
                    lostDetectionCount++;
                    if (lostDetectionCount > MAX_LOST_DETECTIONS) {
                        tagDetected = false;
                        cachedTagResult = null;
                        // Immediately stop turret when tag is lost
                        if (robot.spinner != null) {
                            robot.spinner.setPower(0.0);
                        }
                    }
                }
            }
            
            // Update all systems ONLY if we have a valid, FRESH tag detection THIS loop
            if (freshTagDetected && tagDetected && cachedTagResult != null && cachedTagResult.isValid && currentDistance > 0) {
                // 1. Update turret (horizontal alignment) - use FRESH tx from current detection
                updateTurret(currentTx);
                
                // 2. Update hood position
                updateHood(currentDistance);
                
                // 3. Update shooter RPM (only if A button is pressed)
                if (shooterOn) {
                    updateShooter(currentDistance);
                } else {
                    // Shooter off - stop motor
                    if (shooterMotor != null) {
                        try {
                            shooterMotor.setVelocity(0);
                        } catch (Exception e) {
                            // Ignore
                        }
                    }
                }
            } else {
                // No FRESH tag - IMMEDIATELY stop shooter and turret
                if (shooterMotor != null) {
                    try {
                        shooterMotor.setVelocity(0);
                    } catch (Exception e) {
                        // Ignore
                    }
                }
                if (robot.spinner != null) {
                    robot.spinner.setPower(0.0);
                }
            }
        } catch (Exception e) {
            telemetry.addData("Detection Error", e.getMessage());
            // On error, stop everything
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            if (shooterMotor != null) {
                try {
                    shooterMotor.setVelocity(0);
                } catch (Exception e2) {
                    // Ignore
                }
            }
        }
    }
    
    /**
     * Automatically adjust turret horizontally to align with AprilTag
     */
    private void updateTurret(double tx) {
        if (robot.spinner == null) {
            return;
        }
        
        try {
            // Apply horizontal offset to compensate for shooting left/right
            double adjustedTx = tx + HORIZONTAL_OFFSET_DEG;
            double absTx = Math.abs(adjustedTx);
            
            // Check if within deadband - stop turret immediately
            if (absTx <= TURRET_DEADBAND) {
                robot.spinner.setPower(0.0);
                return;
            }
            
            // Check encoder limits (±90 degrees) - MUST check before applying power
            int currentPos;
            try {
                currentPos = robot.spinner.getCurrentPosition();
            } catch (Exception e) {
                // Can't read position - stop turret for safety
                robot.spinner.setPower(0.0);
                return;
            }
            
            // Safety check: if already past limits, stop immediately
            if (currentPos >= SPINNER_MAX) {
                robot.spinner.setPower(0.0);
                telemetry.addData("Turret Limit", "At/past RIGHT limit (+90°)");
                return;
            } else if (currentPos <= SPINNER_MIN) {
                robot.spinner.setPower(0.0);
                telemetry.addData("Turret Limit", "At/past LEFT limit (-90°)");
                return;
            }
            
            // Progressive power control: slow down as we approach the target
            double cmd;
            double sign = Math.signum(adjustedTx);
            
            if (absTx <= TURRET_SLOW_ZONE) {
                // Close to target - use slow, gentle power
                cmd = TURRET_SLOW_POWER * sign;
            } else {
                // Far from target - use proportional control with limits
                cmd = adjustedTx * TURRET_KP;
                // Clamp to max power
                double absCmd = Math.abs(cmd);
                if (absCmd > TURRET_MAX_POWER) {
                    cmd = TURRET_MAX_POWER * sign;
                } else if (absCmd < TURRET_MIN_POWER) {
                    cmd = TURRET_MIN_POWER * sign;
                }
            }
            
            // Check if approaching limits - prevent movement beyond ±90 degrees
            if (cmd > 0 && currentPos >= SPINNER_MAX - 50) {
                cmd = 0.0; // Can't go right - approaching +90° limit
            } else if (cmd < 0 && currentPos <= SPINNER_MIN + 50) {
                cmd = 0.0; // Can't go left - approaching -90° limit
            }
            
            // Apply command to turret (only if cmd is valid)
            robot.spinner.setPower(cmd);
            
            // Debug telemetry
            telemetry.addData("Turret TX", "%.2f° (adj: %.2f°)", tx, adjustedTx);
            telemetry.addData("Turret Cmd", "%.3f", cmd);
            
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
            // On any error, stop turret immediately
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
        }
    }
    
    /**
     * Automatically adjust hood position based on distance
     */
    private void updateHood(double distance) {
        if (robot == null || hoodServo == null) {
            return;
        }
        
        try {
            // Calculate hood position using formula
            double calculatedHood = robot.calculateHoodPosition(distance, null);
            
            // Adjust hood for close distances (lower the hood)
            if (distance < CLOSE_DISTANCE_THRESHOLD) {
                calculatedHood += HOOD_CLOSE_ADJUSTMENT; // Negative adjustment lowers hood
            }
            
            // Clamp to servo limits
            currentHoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, calculatedHood));
            
            // Apply to servo
            hoodServo.setPosition(currentHoodPosition);
            
        } catch (Exception e) {
            telemetry.addData("Hood Error", e.getMessage());
        }
    }
    
    /**
     * Automatically adjust shooter RPM based on distance
     */
    private void updateShooter(double distance) {
        if (robot == null || shooterMotor == null) {
            return;
        }
        
        try {
            // Calculate target RPM using formula
            double calculatedRPM = robot.calculateShooterRPM(distance);
            
            // Apply distance-based scaling factor
            double scaleFactor = calculateRPMScaleFactor(distance);
            targetRPM = calculatedRPM * scaleFactor;
            
            // Convert RPM to ticks per second for velocity control
            double velocityTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;
            int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
            
            // Apply to shooter motor (positive velocity for forward direction)
            shooterMotor.setVelocity(velocityTicksPerSecInt);
            
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
            try {
                shooterMotor.setVelocity(0);
            } catch (Exception e2) {
                // Ignore
            }
        }
    }
    
    /**
     * Update telemetry with current status
     */
    private void updateTelemetry() {
        try {
            telemetry.addLine("=== Auto Aim Shooter ===");
            telemetry.addData("Limelight", limelight != null ? "Connected" : "Missing");
            telemetry.addData("Tag Detected", tagDetected ? "YES" : "NO");
            
            if (tagDetected && cachedTagResult != null && cachedTagResult.isValid) {
                telemetry.addLine("\n--- AprilTag " + TARGET_TAG_ID + " Detected ---");
                telemetry.addData("Distance", "%.1f inches", cachedTagResult.distance);
                telemetry.addData("X Angle (raw)", "%.2f°", cachedTagResult.xDegrees);
                telemetry.addData("X Angle (adjusted)", "%.2f° (offset: %.1f°)", 
                    cachedTagResult.xDegrees + HORIZONTAL_OFFSET_DEG, HORIZONTAL_OFFSET_DEG);
                telemetry.addData("Y Angle", "%.2f°", cachedTagResult.yDegrees);
                
                // Turret status
                telemetry.addLine("\n--- Turret Status ---");
                if (robot.spinner != null) {
                    try {
                        int pos = robot.spinner.getCurrentPosition();
                        double posDegrees = (pos / 550.0) * 90.0; // Approximate conversion
                        telemetry.addData("Turret Position", "%d ticks (%.1f°)", pos, posDegrees);
                        telemetry.addData("Turret Power", "%.3f", robot.spinner.getPower());
                        if (pos >= SPINNER_MAX - 50) {
                            telemetry.addData("Turret Limit", "⚠️ Near RIGHT limit (+90°)");
                        } else if (pos <= SPINNER_MIN + 50) {
                            telemetry.addData("Turret Limit", "⚠️ Near LEFT limit (-90°)");
                        }
                    } catch (Exception e) {
                        telemetry.addData("Turret", "Error reading");
                    }
                } else {
                    telemetry.addData("Turret", "NOT FOUND");
                }
                
                // Hood status
                telemetry.addLine("\n--- Hood Status ---");
                if (hoodServo != null) {
                    try {
                        telemetry.addData("Hood Position", "%.4f", hoodServo.getPosition());
                        if (cachedTagResult.distance < CLOSE_DISTANCE_THRESHOLD) {
                            telemetry.addData("Hood Adjustment", "%.4f (close distance)", HOOD_CLOSE_ADJUSTMENT);
                        }
                    } catch (Exception e) {
                        telemetry.addData("Hood", "Error reading");
                    }
                } else {
                    telemetry.addData("Hood Servo", "NOT FOUND");
                }
                
                // Shooter status
                telemetry.addLine("\n--- Shooter Status ---");
                telemetry.addData("Shooter", shooterOn ? "ON (Press A to toggle)" : "OFF (Press A to toggle)");
                if (shooterMotor != null) {
                    try {
                        double currentVelocity = shooterMotor.getVelocity();
                        int currentRPM = (int)Math.round((currentVelocity / TICKS_PER_REVOLUTION) * 60.0);
                        telemetry.addData("Shooter Velocity", "%.1f ticks/sec (%.0f RPM)", currentVelocity, (double)currentRPM);
                        telemetry.addData("Target RPM", "%.0f", targetRPM);
                    } catch (Exception e) {
                        telemetry.addData("Shooter", "Error reading: " + e.getMessage());
                    }
                } else {
                    telemetry.addData("Shooter Motor", "NOT FOUND");
                }
                
                // Formula values
                if (robot != null) {
                    try {
                        double calculatedRPM = robot.calculateShooterRPM(cachedTagResult.distance);
                        double calculatedHood = robot.calculateHoodPosition(cachedTagResult.distance, null);
                        telemetry.addLine("\n--- Formula Values ---");
                        telemetry.addData("Calculated RPM (raw)", "%.0f", calculatedRPM);
                        telemetry.addData("Calculated Hood (raw)", "%.4f", calculatedHood);
                    } catch (Exception e) {
                        telemetry.addData("Formula", "Error: " + e.getMessage());
                    }
                }
            } else {
                telemetry.addLine("\n❌ AprilTag " + TARGET_TAG_ID + " not detected");
                if (lostDetectionCount > 0) {
                    telemetry.addData("Lost Count", "%d / %d", lostDetectionCount, MAX_LOST_DETECTIONS);
                }
            }
            
            // Controls
            telemetry.addLine("\n--- Controls ---");
            telemetry.addData("A Button", "Toggle shooter ON/OFF");
            
            // Tuning values
            telemetry.addLine("\n--- Tuning Values ---");
            telemetry.addData("Horizontal Offset", "%.1f° (adjust if shooting left/right)", HORIZONTAL_OFFSET_DEG);
            telemetry.addData("Close Distance Threshold", "%.1f\"", CLOSE_DISTANCE_THRESHOLD);
            telemetry.addData("Hood Close Adjustment", "%.4f (lowers hood when close)", HOOD_CLOSE_ADJUSTMENT);
            telemetry.addData("Turret KP", "%.3f", TURRET_KP);
            telemetry.addData("Turret Deadband", "%.1f°", TURRET_DEADBAND);
            telemetry.addData("Max Lost Detections", "%d", MAX_LOST_DETECTIONS);
            
        } catch (Exception e) {
            telemetry.addData("Telemetry Error", e.getMessage());
        }
        
        telemetry.update();
    }
}

