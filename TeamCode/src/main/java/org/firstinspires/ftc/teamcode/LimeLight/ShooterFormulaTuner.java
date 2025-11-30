package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * ShooterFormulaTuner - Tuning class to find formula: formula = A * hoodPos + B * shooterEncoderRPM
 * 
 * Controls:
 * - D-pad Up/Down: Adjust hood servo position (0.001 increments)
 * - D-pad Left/Right: Adjust shooter encoder velocity/RPM (small increments)
 * - Right Bumper: Increase shooter RPM (larger increments)
 * - Left Bumper: Decrease shooter RPM (larger increments)
 * - A button: Shoot (run shooter for 2 seconds)
 * - B button: Reset shooter encoder
 * - X button: Record current settings as a data point
 * 
 * Formula goal: distance = A * hoodPos + B * shooterRPM
 * You'll test at different distances and record hoodPos + RPM combinations that work,
 * then use linear regression to find A and B.
 */
@TeleOp(name = "Shooter Formula Tuner", group = "Test")
public class ShooterFormulaTuner extends LinearOpMode {
    private dumbMapLime robot;
    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    private Limelight3A limelight;
    private AprilTagDetector tagDetector;
    
    // Limelight constants
    private static final double CAMERA_HEIGHT = 13.0; // Camera height in inches (matches ShooterAnglePowerTest and AutoSpinAndShoot)
    private static final double CAMERA_ANGLE = 0.0; // Camera angle in degrees
    private static final double MAX_DISTANCE = 144.0; // Max detection distance in inches (matches ShooterAnglePowerTest and AutoSpinAndShoot)
    private static final int TARGET_TAG_ID = 24; // AprilTag ID to use for distance (can be 24 or 20)
    
    // Tuning parameters
    private double hoodPosition = 0.2; // Current hood position
    private int targetRPM = 0; // Target shooter RPM
    
    // Constants
    private static final double HOOD_MIN = 0.206;
    private static final double HOOD_MAX = 0.295;
    private static final double HOOD_INCREMENT = 0.001; // Small increments for fine tuning
    private static final int RPM_SMALL_INCREMENT = 10; // Small RPM adjustment
    private static final int RPM_LARGE_INCREMENT = 50; // Large RPM adjustment
    private static final int MAX_RPM = 6000; // Maximum RPM (motor limit, positive direction)
    private static final int MIN_RPM = -6000; // Minimum RPM (motor limit, negative direction)
    // GoBILDA 5202 Series 6000 RPM motor (1:1 ratio):
    // - Encoder: 28 PPR (Pulses Per Revolution) at output shaft
    // Note: FTC SDK getVelocity()/setVelocity() use PPR, not quadrature counts
    // If RPM readings are incorrect, you can calibrate this value:
    // 1. Set target to a known RPM (e.g., 3000)
    // 2. Measure actual RPM with a tachometer
    // 3. Calculate: TICKS_PER_REVOLUTION = (actualVelocity * 60) / measuredRPM
    private static final int TICKS_PER_REVOLUTION = 28; // GoBILDA 6000 RPM motor encoder resolution (PPR)
    
    // Alternative: If you want to work directly with velocity (ticks/sec) instead of RPM,
    // you can set targetRPM to velocity values and set TICKS_PER_REVOLUTION = 1
    // But this makes the interface less intuitive
    
    // State
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastAButton = false;
    private boolean lastBButton = false;
    private boolean lastXButton = false;
    private boolean isShooting = false;
    
    // Data recording
    private int dataPointCount = 0;
    private static final int MAX_DATA_POINTS = 20;
    private double[] recordedDistances = new double[MAX_DATA_POINTS];
    private double[] recordedHoodPositions = new double[MAX_DATA_POINTS];
    private int[] recordedRPMs = new int[MAX_DATA_POINTS];
    
    // Limelight call throttling to prevent disconnects
    private int loopCounter = 0;
    private static final int LIMELIGHT_CALL_INTERVAL = 5; // Only call Limelight every 5 loops (~100ms)
    private AprilTagDetector.AprilTagResult cachedTagResult = null; // Cache tag result to avoid multiple calls
    
    @Override
    public void runOpMode() {
        robot = new dumbMapLime(this);
        robot.initMotors();
        
        // Get shooter motor (as DcMotorEx for velocity control)
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            if (shooterMotor == null) {
                shooterMotor = hardwareMap.get(DcMotorEx.class, "outtake");
            }
            if (shooterMotor != null) {
                shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                shooterMotor.setVelocity(0);
                targetRPM = 0;
                
                // Test if encoder is working
                sleep(100); // Give encoder time to initialize
                try {
                    double testVelocity = shooterMotor.getVelocity();
                    if (Math.abs(testVelocity) < 1000) { // Should be near 0 when stopped
                        telemetry.addData("Shooter Motor", "Initialized (DcMotorEx) - Encoder OK");
                    } else {
                        telemetry.addData("Shooter Motor", "⚠️ Encoder may not be connected!");
                    }
                } catch (Exception e) {
                    telemetry.addData("Shooter Motor", "⚠️ Encoder Error: " + e.getMessage());
                    telemetry.addLine("Check encoder cable connection!");
                }
            } else {
                telemetry.addData("Shooter Motor", "NOT FOUND");
            }
        } catch (Exception e) {
            telemetry.addData("Shooter Motor", "Error: " + e.getMessage());
        }
        
        // Get hood servo
        try {
            hoodServo = hardwareMap.get(Servo.class, "hood");
            if (hoodServo == null) {
                hoodServo = hardwareMap.get(Servo.class, "servo");
            }
            if (hoodServo != null) {
                hoodServo.setPosition(hoodPosition);
                telemetry.addData("Hood Servo", "Initialized");
            } else {
                telemetry.addData("Hood Servo", "NOT FOUND");
            }
        } catch (Exception e) {
            telemetry.addData("Hood Servo", "Error: " + e.getMessage());
        }
        
        // Initialize Limelight for distance detection
        robot.initLimeLight();
        limelight = robot.getLimeLight();
        if (limelight != null) {
            try {
                limelight.start();
                limelight.pipelineSwitch(0); // Pipeline 0 for AprilTag
                sleep(500); // Give Limelight time to initialize
                tagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT, CAMERA_ANGLE, MAX_DISTANCE);
                telemetry.addData("Limelight", "Initialized");
            } catch (Exception e) {
                telemetry.addData("Limelight", "Error: " + e.getMessage());
            }
        } else {
            telemetry.addData("Limelight", "NOT FOUND");
        }
        
        telemetry.addLine("=== Shooter Formula Tuner ===");
        telemetry.addLine("Controls:");
        telemetry.addLine("D-pad Up/Down: Hood position");
        telemetry.addLine("D-pad Left/Right: Target RPM (small)");
        telemetry.addLine("Right/Left Bumper: Target RPM (large)");
        telemetry.addLine("A: Toggle shooter (ON/OFF)");
        telemetry.addLine("B: Reset shooter encoder");
        telemetry.addLine("X: Record data point (uses AprilTag distance)");
        telemetry.addLine("\nPress START to begin");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            handleControls();
            updateShooter();
            updateHood();
            updateTelemetry();
            loopCounter++; // Increment counter for Limelight throttling
            sleep(20);
        }
        
        // Stop all motors
        try {
            if (shooterMotor != null) {
                shooterMotor.setPower(0.0);
            }
        } catch (Exception e) {
            // Ignore
        }
    }
    
    private void handleControls() {
        // Hood position controls (D-pad Up/Down)
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadUp && !lastDpadUp) {
            hoodPosition = Math.min(HOOD_MAX, hoodPosition + HOOD_INCREMENT);
        }
        if (dpadDown && !lastDpadDown) {
            hoodPosition = Math.max(HOOD_MIN, hoodPosition - HOOD_INCREMENT);
        }
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        
        // Target RPM controls (D-pad Left/Right - small increments)
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        if (dpadLeft && !lastDpadLeft) {
            targetRPM = Math.max(MIN_RPM, targetRPM - RPM_SMALL_INCREMENT);
        }
        if (dpadRight && !lastDpadRight) {
            targetRPM = Math.min(MAX_RPM, targetRPM + RPM_SMALL_INCREMENT);
        }
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        
        // Target RPM controls (Bumpers - large increments)
        boolean rightBumper = gamepad1.right_bumper;
        boolean leftBumper = gamepad1.left_bumper;
        if (rightBumper && !lastRightBumper) {
            targetRPM = Math.min(MAX_RPM, targetRPM + RPM_LARGE_INCREMENT);
        }
        if (leftBumper && !lastLeftBumper) {
            targetRPM = Math.max(MIN_RPM, targetRPM - RPM_LARGE_INCREMENT);
        }
        lastRightBumper = rightBumper;
        lastLeftBumper = leftBumper;
        
        // Shoot toggle button (A)
        boolean aButton = gamepad1.a;
        if (aButton && !lastAButton) {
            isShooting = !isShooting; // Toggle shooting state
        }
        lastAButton = aButton;
        
        // Reset encoder button (B)
        boolean bButton = gamepad1.b;
        if (bButton && !lastBButton) {
            try {
                if (shooterMotor != null) {
                    shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    targetRPM = 0;
                    isShooting = false;
                    telemetry.addLine("✅ Shooter encoder reset");
                }
            } catch (Exception e) {
                telemetry.addData("Reset Error", e.getMessage());
            }
        }
        lastBButton = bButton;
        
        // Record data point (X)
        boolean xButton = gamepad1.x;
        if (xButton && !lastXButton) {
            recordDataPoint();
        }
        lastXButton = xButton;
    }
    
    /**
     * Convert RPM to ticks per second for motor controller
     * Motor controller setVelocity() requires ticks/sec, but we work in RPM
     * GoBILDA 6000 RPM motor: 28 ticks per revolution
     */
    private int rpmToTicksPerSecond(int rpm) {
        return (int)Math.round((rpm / 60.0) * TICKS_PER_REVOLUTION);
    }
    
    /**
     * Convert ticks per second to RPM
     * GoBILDA 6000 RPM motor: 28 ticks per revolution
     */
    private int ticksPerSecondToRPM(double ticksPerSec) {
        return (int)Math.round((ticksPerSec / TICKS_PER_REVOLUTION) * 60.0);
    }
    
    private void updateShooter() {
        if (shooterMotor == null) return;
        
        try {
            if (isShooting) {
                // Set velocity in RPM (converted to ticks/sec internally)
                int velocityTicksPerSec = rpmToTicksPerSecond(targetRPM);
                shooterMotor.setVelocity(velocityTicksPerSec);
            } else {
                // When not shooting, set velocity to 0
                shooterMotor.setVelocity(0);
            }
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }
    }
    
    private void updateHood() {
        if (hoodServo != null) {
            try {
                hoodServo.setPosition(hoodPosition);
            } catch (Exception e) {
                telemetry.addData("Hood Error", e.getMessage());
            }
        }
    }
    
    // No longer needed - shooting is now a toggle
    
    private void recordDataPoint() {
        if (dataPointCount >= MAX_DATA_POINTS) {
            telemetry.addLine("⚠️ Maximum data points reached!");
            return;
        }
        
        // Get distance from AprilTag detector (use cached result if available to avoid extra Limelight call)
        double distance = 0.0;
        if (tagDetector != null) {
            // Try cached result first (faster, no network call)
            if (cachedTagResult != null && cachedTagResult.isValid && cachedTagResult.distance > 0) {
                distance = cachedTagResult.distance;
                telemetry.addLine("📊 Recording data point " + (dataPointCount + 1));
                telemetry.addData("Distance (from AprilTag " + cachedTagResult.tagId + ")", "%.1f\"", distance);
            } else {
                // Cache is stale or invalid - get fresh data (only when recording)
                try {
                    AprilTagDetector.AprilTagResult tag = tagDetector.getTagById(TARGET_TAG_ID);
                    if (tag != null && tag.isValid && tag.distance > 0) {
                        distance = tag.distance;
                        cachedTagResult = tag; // Update cache
                        telemetry.addLine("📊 Recording data point " + (dataPointCount + 1));
                        telemetry.addData("Distance (from AprilTag)", "%.1f\"", distance);
                    } else {
                        // Try to get closest tag if target ID not found
                        tag = tagDetector.getClosestTag();
                        if (tag != null && tag.isValid && tag.distance > 0) {
                            distance = tag.distance;
                            cachedTagResult = tag; // Update cache
                            telemetry.addLine("📊 Recording data point " + (dataPointCount + 1));
                            telemetry.addData("Distance (from AprilTag " + tag.tagId + ")", "%.1f\"", distance);
                        } else {
                            telemetry.addLine("⚠️ No AprilTag detected - distance set to 0");
                            telemetry.addLine("Point a Limelight at an AprilTag and try again");
                        }
                    }
                } catch (Exception e) {
                    telemetry.addData("Distance Error", e.getMessage());
                    distance = 0.0;
                }
            }
        } else {
            telemetry.addLine("⚠️ Limelight not available - distance set to 0");
            telemetry.addLine("You'll need to enter distance manually");
        }
        
        // Record the data point
        recordedHoodPositions[dataPointCount] = hoodPosition;
        recordedRPMs[dataPointCount] = getCurrentRPM();
        recordedDistances[dataPointCount] = distance;
        
        dataPointCount++;
        telemetry.addData("Hood Position", "%.4f", hoodPosition);
        telemetry.addData("Shooter RPM", "%d", getCurrentRPM());
        telemetry.addLine("✅ Data point " + dataPointCount + " recorded");
    }
    
    /**
     * Get current RPM from encoder velocity
     * Motor controller returns ticks/sec, we convert to RPM
     * GoBILDA 6000 RPM motor: 28 ticks per revolution
     */
    private int getCurrentRPM() {
        if (shooterMotor == null) return 0;
        try {
            double velocityTicksPerSec = shooterMotor.getVelocity();
            return ticksPerSecondToRPM(velocityTicksPerSec);
        } catch (Exception e) {
            // Fallback: return target RPM if encoder read fails
            return targetRPM;
        }
    }
    
    /**
     * Calculate formula coefficients using linear regression
     * Formula: distance = A * hoodPos + B * shooterRPM
     * Using multiple linear regression
     */
    private void calculateFormulaCoefficients() {
        if (dataPointCount < 3) {
            telemetry.addLine("Need at least 3 data points");
            return;
        }
        
        // Multiple linear regression: distance = A * hoodPos + B * RPM
        // Using least squares method
        double sumHood = 0, sumRPM = 0, sumDistance = 0;
        double sumHoodSq = 0, sumRPMSq = 0, sumHoodRPM = 0;
        double sumHoodDist = 0, sumRPMDist = 0;
        int n = dataPointCount;
        
        for (int i = 0; i < n; i++) {
            double hood = recordedHoodPositions[i];
            double rpm = recordedRPMs[i];
            double dist = recordedDistances[i];
            
            sumHood += hood;
            sumRPM += rpm;
            sumDistance += dist;
            sumHoodSq += hood * hood;
            sumRPMSq += rpm * rpm;
            sumHoodRPM += hood * rpm;
            sumHoodDist += hood * dist;
            sumRPMDist += rpm * dist;
        }
        
        // Calculate means
        double meanHood = sumHood / n;
        double meanRPM = sumRPM / n;
        double meanDistance = sumDistance / n;
        
        // Calculate coefficients using matrix method
        // [A]   [sum(hood²)  sum(hood*RPM)]^-1   [sum(hood*dist)]
        // [B] = [sum(hood*RPM) sum(RPM²)   ]    [sum(RPM*dist) ]
        
        double det = (sumHoodSq * sumRPMSq) - (sumHoodRPM * sumHoodRPM);
        
        if (Math.abs(det) < 0.0001) {
            telemetry.addLine("⚠️ Data points are collinear - need more varied data");
            return;
        }
        
        // Calculate A and B
        double A = ((sumHoodDist * sumRPMSq) - (sumRPMDist * sumHoodRPM)) / det;
        double B = ((sumHoodSq * sumRPMDist) - (sumHoodDist * sumHoodRPM)) / det;
        double C = meanDistance - (A * meanHood) - (B * meanRPM); // Intercept
        
        telemetry.addLine("Calculated Formula:");
        telemetry.addData("A (hood coefficient)", "%.6f", A);
        telemetry.addData("B (RPM coefficient)", "%.6f", B);
        if (Math.abs(C) > 0.01) {
            telemetry.addData("C (intercept)", "%.6f", C);
            telemetry.addLine(String.format("distance = %.6f * hoodPos + %.6f * RPM + %.6f", A, B, C));
        } else {
            telemetry.addLine(String.format("distance = %.6f * hoodPos + %.6f * RPM", A, B));
        }
        
        // Calculate R² (coefficient of determination) to show fit quality
        double ssRes = 0, ssTot = 0;
        for (int i = 0; i < n; i++) {
            double predicted = A * recordedHoodPositions[i] + B * recordedRPMs[i] + C;
            double actual = recordedDistances[i];
            ssRes += (actual - predicted) * (actual - predicted);
            ssTot += (actual - meanDistance) * (actual - meanDistance);
        }
        double rSquared = 1.0 - (ssRes / ssTot);
        telemetry.addData("R² (fit quality)", "%.4f (1.0 = perfect)", rSquared);
        
        // Show predicted vs actual for each point
        telemetry.addLine("\nPredicted vs Actual:");
        for (int i = 0; i < n; i++) {
            double predicted = A * recordedHoodPositions[i] + B * recordedRPMs[i] + C;
            double error = Math.abs(predicted - recordedDistances[i]);
            telemetry.addData(String.format("Point %d", i + 1), 
                "Pred: %.1f\", Actual: %.1f\", Error: %.1f\"", 
                predicted, recordedDistances[i], error);
        }
    }
    
    private void updateTelemetry() {
        telemetry.addLine("=== Current Settings ===");
        telemetry.addData("Hood Position", "%.4f [%.3f - %.3f]", hoodPosition, HOOD_MIN, HOOD_MAX);
        telemetry.addData("Target RPM", "%d", targetRPM);
        
        // Show AprilTag distance if available (throttled to prevent disconnects)
        if (tagDetector != null) {
            // Only call Limelight every N loops to reduce network traffic
            boolean shouldCallLimelight = (loopCounter % LIMELIGHT_CALL_INTERVAL == 0);
            
            if (shouldCallLimelight) {
                // Update cache with fresh Limelight data
                try {
                    AprilTagDetector.AprilTagResult tag = tagDetector.getTagById(TARGET_TAG_ID);
                    if (tag == null || !tag.isValid) {
                        tag = tagDetector.getClosestTag();
                    }
                    cachedTagResult = tag; // Cache the result
                } catch (Exception e) {
                    // Limelight call failed - keep using cached result
                    telemetry.addData("AprilTag", "Error (using cache)");
                }
            }
            
            // Use cached result for display (updated every N loops)
            if (cachedTagResult != null && cachedTagResult.isValid) {
                telemetry.addData("AprilTag Distance", "%.1f\" (ID: %d)", cachedTagResult.distance, cachedTagResult.tagId);
            } else {
                telemetry.addData("AprilTag", "Not detected");
            }
        }
        
        if (shooterMotor != null) {
            try {
                int actualRPM = getCurrentRPM();
                int rpmDifference = actualRPM - targetRPM;
                
                // Show what we're sending vs what we're getting
                int targetTicksPerSec = rpmToTicksPerSecond(targetRPM);
                double actualTicksPerSec = shooterMotor.getVelocity();
                
                // Show calculation breakdown for debugging
                double calculatedRPM = (actualTicksPerSec / TICKS_PER_REVOLUTION) * 60.0;
                
                telemetry.addData("Target RPM", "%d", targetRPM);
                telemetry.addData("Target Velocity", "%d ticks/sec", targetTicksPerSec);
                telemetry.addData("Actual Velocity", "%.1f ticks/sec", actualTicksPerSec);
                
                // Show both RPM calculation and direct velocity for comparison
                telemetry.addData("Calculation", "(%.1f / %d) * 60 = %.1f RPM", actualTicksPerSec, TICKS_PER_REVOLUTION, calculatedRPM);
                telemetry.addData("Actual RPM (rounded)", "%d", actualRPM);
                telemetry.addData("RPM Difference", "%d", rpmDifference);
                
                // Show alternative calculations to help determine correct TICKS_PER_REVOLUTION
                if (isShooting && Math.abs(actualTicksPerSec) > 100) {
                    telemetry.addLine("--- Calibration Helper ---");
                    double rpmWith28 = (actualTicksPerSec / 28.0) * 60.0;
                    double rpmWith112 = (actualTicksPerSec / 112.0) * 60.0;
                    telemetry.addData("If TICKS_PER_REV = 28", "RPM = %.0f", rpmWith28);
                    telemetry.addData("If TICKS_PER_REV = 112", "RPM = %.0f", rpmWith112);
                    telemetry.addLine("Compare with actual RPM to determine correct value");
                    
                    // If you know actual RPM, calculate what TICKS_PER_REVOLUTION should be
                    telemetry.addLine("To find correct value:");
                    telemetry.addData("Formula", "(%.1f * 60) / measuredRPM", actualTicksPerSec);
                }
                
                // Show direction
                if (targetRPM > 0) {
                    telemetry.addLine("Direction: Forward");
                } else if (targetRPM < 0) {
                    telemetry.addLine("Direction: Reverse");
                } else {
                    telemetry.addLine("Direction: Stopped");
                }
                
                // Check for velocity capping
                if (isShooting && Math.abs(targetRPM) > 0) {
                    double velocityMatch = Math.abs(actualTicksPerSec - targetTicksPerSec);
                    double velocityRatio = Math.abs(actualTicksPerSec) / Math.max(1, Math.abs(targetTicksPerSec));
                    
                    // Check if velocity is being capped (actual is significantly less than target)
                    if (velocityMatch > 100 && velocityRatio < 0.8) {
                        telemetry.addLine("⚠️ VELOCITY CAPPING DETECTED!");
                        telemetry.addData("Target Velocity", "%d ticks/sec (%.0f RPM)", targetTicksPerSec, (targetTicksPerSec / TICKS_PER_REVOLUTION) * 60.0);
                        telemetry.addData("Actual Velocity", "%.1f ticks/sec (%.0f RPM)", actualTicksPerSec, (actualTicksPerSec / TICKS_PER_REVOLUTION) * 60.0);
                        telemetry.addData("Velocity Ratio", "%.1f%%", velocityRatio * 100);
                        
                        // Calculate what RPM the actual velocity represents
                        int actualRPMFromVelocity = (int)Math.round((Math.abs(actualTicksPerSec) / TICKS_PER_REVOLUTION) * 60.0);
                        telemetry.addLine("Motor appears capped at ~" + actualRPMFromVelocity + " RPM");
                        telemetry.addLine("Possible causes:");
                        telemetry.addLine("1. Motor controller velocity limit (check REV Hub config)");
                        telemetry.addLine("2. Low battery voltage (<12V)");
                        telemetry.addLine("3. Physical load/resistance");
                        telemetry.addLine("4. Motor PID needs tuning");
                    } else if (velocityMatch > 50) {
                        telemetry.addLine("⚠️ Velocity mismatch detected!");
                        telemetry.addData("Sent", "%d ticks/sec", targetTicksPerSec);
                        telemetry.addData("Received", "%.1f ticks/sec", actualTicksPerSec);
                    }
                }
                
                // Check if motor is capped (if trying to go above 80% of max but actual is much lower)
                if (isShooting && Math.abs(targetRPM) > MAX_RPM * 0.8 && Math.abs(actualRPM) < Math.abs(targetRPM) * 0.7) {
                    telemetry.addLine("⚠️ WARNING: Motor may be capped!");
                    telemetry.addLine("Motor rated for " + MAX_RPM + " RPM but only reaching " + actualRPM);
                    telemetry.addLine("Check motor controller velocity limits in REV Hardware Client");
                }
                
                // Color code the difference (closer to 0 is better)
                if (Math.abs(rpmDifference) <= 10) {
                    telemetry.addLine("✅ RPM locked!");
                } else if (Math.abs(rpmDifference) <= 50) {
                    telemetry.addLine("⚠️ Close to target");
                } else {
                    telemetry.addLine("❌ Adjust RPM to minimize difference");
                }
            } catch (Exception e) {
                telemetry.addData("Velocity", "N/A");
            }
        }
        
        telemetry.addLine("\n=== Shooting Status ===");
        telemetry.addData("Shooting", isShooting ? "ON" : "OFF");
        
        telemetry.addLine("\n=== Data Points ===");
        telemetry.addData("Recorded", "%d / %d", dataPointCount, MAX_DATA_POINTS);
        if (dataPointCount > 0) {
            telemetry.addLine("Data Points:");
            for (int i = 0; i < dataPointCount; i++) {
                telemetry.addData(String.format("Point %d", i + 1), 
                    "Hood: %.4f, RPM: %d, Distance: %.1f\"", 
                    recordedHoodPositions[i], recordedRPMs[i], recordedDistances[i]);
            }
        }
        
        telemetry.addLine("\n=== Formula Analysis ===");
        telemetry.addLine("Target: distance = A * hoodPos + B * shooterRPM");
        
        if (dataPointCount >= 3) {
            // Calculate linear regression coefficients
            calculateFormulaCoefficients();
        } else {
            telemetry.addLine("Need at least 3 data points to calculate formula");
            telemetry.addLine("Record more data points using X button");
        }
        
        telemetry.update();
    }
}

