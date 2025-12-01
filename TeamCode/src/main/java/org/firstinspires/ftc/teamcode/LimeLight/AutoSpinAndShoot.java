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
 * AutoSpinAndShoot - Search and lock behavior for AprilTag 24, then shoot
 * 
 * Behavior:
 * - Spinner moves continuously searching for April Tag 24
 * - When tag is found within 5 degrees, stops and locks onto it
 * - After locking, automatically shoots the ball
 * - If tag not found within 2 full spins, stops and reports "no april tag found"
 */
@TeleOp(name = "Auto Spin and Shoot", group = "Test")
public class AutoSpinAndShoot extends LinearOpMode {
    private dumbMapLime robot;
    private Limelight3A limelight;
    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    
    // Hood servo limits (from HeleOpBase)
    private static final double HOOD_MIN = 0.217;
    private static final double HOOD_MAX = 0.282;
    
    // Shooter motor constants (GoBILDA 6000 RPM motor)
    private static final int TICKS_PER_REVOLUTION = 28; // PPR for GoBILDA 6000 RPM motor
    
    // Constants
    private static final int SPINNER_MIN = -550;  // Encoder limit (min) - roughly 90 degrees left
    private static final int SPINNER_MAX = 590;   // Encoder limit (max) - roughly 300 degrees right
    private static final int TARGET_TAG_ID = 24;   // AprilTag ID to track
    private static final double CAMERA_HEIGHT = 13.0; // Camera height in inches (matches ShooterAnglePowerTest)
    private static final double CAMERA_ANGLE = 0.0; // Camera angle in degrees
    private static final double MAX_DISTANCE = 144.0; // Max detection distance in inches (matches ShooterAnglePowerTest)
    private static final double TOLERANCE = 2.0;   // Degrees tolerance - start aligning when within this (wider to catch tag)
    private static final double DEADBAND = 0.5;    // Deadband - lock when within 0.5° (tight margin of error as requested)
    private static final double ALIGN_KP = 0.04;   // Proportional constant (reduced further to prevent overcorrection)
    private static final double MIN_ALIGN_POWER = 0.15; // Minimum power to move (reduced to prevent overcorrection)
    private static final double MAX_ALIGN_POWER = 0.3;  // Maximum alignment power (reduced to prevent overshooting)
    
    // RPM scaling - distance-based adjustment
    // Close distances need more reduction, far distances need less (or none)
    private static double calculateRPMScaleFactor(double distance) {
        // At close distances (50-80"), use lower multiplier (0.90)
        // At far distances (120-144"), use higher multiplier (0.95-1.0)
        // Linear interpolation between these ranges
        if (distance <= 80.0) {
            return 0.90; // Close distances
        } else if (distance >= 120.0) {
            return 1.0; // Far distances (less reduction needed)
        } else {
            // Linear interpolation between 80" and 120"
            // At 80": 0.90, at 120": 0.98
            double t = (distance - 80.0) / (120.0 - 80.0); // 0 to 1
            return 0.90 + (1.0 - 0.90) * t; // Interpolate
        }
    }
    private static final double SEARCH_POWER = 0.15; // Power for searching rotation (reduced to prevent overshooting)
    private static final int BRAKE_DURATION_MS = 150; // How long to stop (milliseconds) - allows momentum to dissipate
    private static final int MAX_SPINS = 2;        // Maximum number of full spins before stopping
    
    // Shooter constants
    private static final double SHOOTER_POWER_BASE = 1.0; // Base shooter power (will be adjusted by distance)
    
    // Runtime variables
    private boolean isLocked = false;   // Whether we've locked onto the target
    private boolean isTracking = false; // Whether we're tracking the tag (detected but not yet locked)
    private boolean searchStopped = false; // Whether search has stopped (timeout or locked)
    private int searchDirection = 1;     // 1 for right, -1 for left
    private int limitHitCount = 0;       // Count of limit hits (2 hits = 1 full spin)
    private boolean lastWasRightLimit = false; // Track last limit hit to count full cycles
    private boolean lastWasLeftLimit = false;
    private AprilTagDetector aprilTagDetector; // AprilTag detector instance
    private AprilTagDetector.AprilTagResult cachedTagResult = null; // Cache tag result to avoid multiple Limelight calls
    private int loopCounter = 0; // Counter to limit Limelight calls (only call every N loops)
    private int lostDetectionCount = 0; // Count of consecutive loops without detection (reset when detected)
    private static final int MAX_LOST_DETECTIONS = 5; // Max loops without detection before giving up tracking
    private long brakeStartTime = 0; // Time when braking started (0 = not braking)
    private long postBrakeStartTime = 0; // Time when braking ended (for gentle initial alignment)
    private static final int POST_BRAKE_DURATION_MS = 200; // Gentle alignment period after braking
    private static final int LIMELIGHT_CALL_INTERVAL_SEARCH = 2; // Call more frequently when searching (~40ms)
    private static final int LIMELIGHT_CALL_INTERVAL_LOCKED = 10; // Call less frequently when locked (~200ms)
    
    // Shooting state
    private long lockTime = 0; // Time when we locked onto the tag
    private boolean isShooting = false; // Whether we're currently shooting (toggled by A button)
    private boolean lastAButton = false; // Track A button state for toggle
    private double lockedDistance = 0.0; // Distance to tag when locked (for shooter RPM calculation)
    private double targetRPM = 0.0; // Calculated target RPM for shooter (set when locked)
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        robot = new dumbMapLime(this);
        
        // Initialize motors first
        try {
            robot.initMotors();
            
            // Explicitly reset encoders to ensure they're at zero
            if (robot.spinner != null) {
                robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(100); // Give time for reset to complete
                // Use RUN_WITHOUT_ENCODER for more direct power control (like drive motors)
                robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.spinner.setPower(0.0); // Ensure it starts stopped
            }
            
            // Initialize shooter motor (as DcMotorEx for velocity control)
            try {
                shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
                if (shooterMotor == null) {
                    shooterMotor = hardwareMap.get(DcMotorEx.class, "outtake");
                }
                if (shooterMotor != null) {
                    shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    shooterMotor.setVelocity(0); // Start at 0 velocity
                    targetRPM = 0.0;
                    telemetry.addData("Shooter Motor", "Initialized (Velocity Control)");
                } else {
                    telemetry.addData("Shooter Motor", "NOT FOUND");
                }
            } catch (Exception e) {
                telemetry.addData("Shooter Motor", "Error: " + e.getMessage());
            }
            
            // Initialize hood servo
            try {
                hoodServo = hardwareMap.get(Servo.class, "hood");
                if (hoodServo != null) {
                    // Start at middle position
                    hoodServo.setPosition((HOOD_MIN + HOOD_MAX) / 2.0);
                    telemetry.addData("Hood Servo", "Initialized");
                } else {
                    telemetry.addData("Hood Servo", "NOT FOUND");
                }
            } catch (Exception e) {
                telemetry.addData("Hood Servo", "Error: " + e.getMessage());
            }
            
            telemetry.addData("Status", "Motors initialized");
            telemetry.addData("Spinner Position", robot.spinner != null ? robot.spinner.getCurrentPosition() : "N/A");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize motors: " + e.getMessage());
            telemetry.update();
            sleep(1000);
            requestOpModeStop();
            return;
        }
        
        // Set up telemetry
        telemetry.addData("Status", "Initialized - Press START to begin");
        telemetry.addData("Max Spins", "%d", MAX_SPINS);
        telemetry.update();
        
        // Wait for start FIRST (like LimeLightTeleOpTest does)
        waitForStart();
        
        // NOW initialize Limelight AFTER waitForStart() (like LimeLightTeleOpTest does)
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();
        
        try {
            // Use the same initialization approach as LimeLightTeleOpTest
            // Try to get the Limelight from hardware map with common names
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
            
            // If still not found, try to get any Limelight device
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
                // Initialize the Limelight EXACTLY like LimeLightTeleOpTest does (lines 490-491)
                limelight.start();
                limelight.pipelineSwitch(0); // Pipeline 0 for AprilTag
                
                // Give Limelight time to initialize
                sleep(500);
                
                // Verify it's working
                LLResult testResult = limelight.getLatestResult();
                if (testResult != null) {
                    telemetry.addLine("✅ Limelight initialized successfully!");
                    telemetry.addData("Pipeline", "0 (AprilTag)");
                } else {
                    telemetry.addLine("⚠️ Limelight found but not returning data yet");
                }
                
                // Initialize AprilTag detector
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
                // Handle A button toggle for shooting
                handleShootingToggle();
                
                // Search for and lock onto AprilTag
                searchAndLock();
                
                // Handle shooting when locked and toggled on
                handleShooting();
            
            // Update telemetry
            updateTelemetry();
            
            // Small delay to prevent overwhelming the robot
            sleep(20);
            } catch (Exception e) {
                // Catch any exceptions to prevent OpMode from ending
                telemetry.addData("ERROR", "Exception in main loop: " + e.getMessage());
                telemetry.update();
                try {
                    if (robot != null && robot.spinner != null) {
                        robot.spinner.setPower(0.0);
                    }
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(0);
                    }
                } catch (Exception e2) {
                    // Ignore
                }
                sleep(100); // Wait before continuing
            }
        }
        
        // Stop all motors when OpMode ends
        try {
            if (robot != null && robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            if (shooterMotor != null) {
                shooterMotor.setPower(0.0);
            }
            if (hoodServo != null) {
                // Reset hood to middle position
                hoodServo.setPosition((HOOD_MIN + HOOD_MAX) / 2.0);
            }
        } catch (Exception e) {
            // Ignore
        }
    }
    
    /**
     * Handle A button toggle for shooting
     * Simple toggle: A button turns shooting ON/OFF (only works when locked)
     */
    private void handleShootingToggle() {
        boolean aPressed = gamepad1.a;
        
        // Toggle shooting on/off when A is pressed (only if locked)
        if (aPressed && !lastAButton) {
            if (isLocked) {
                isShooting = !isShooting;
            } else {
                // If not locked, can't shoot
                isShooting = false;
            }
        }
        lastAButton = aPressed;
        
        // Auto-stop shooting if we lose lock
        if (!isLocked) {
            isShooting = false;
        }
    }
    
    /**
     * Handle shooting when locked onto target and A button is toggled on
     * Uses 31-point formula to calculate optimal hood position and RPM
     */
    private void handleShooting() {
        // Calculate hood position and RPM when we first lock (only once)
        if (isLocked && lockTime == 0) {
            lockTime = System.currentTimeMillis();
            
            // Calculate hood position and RPM using 31-point formula
            if (robot != null && lockedDistance > 0) {
                try {
                    // Calculate optimal hood position from distance
                    double calculatedHood = robot.calculateHoodPosition(lockedDistance, null);
                    // Clamp to servo limits
                    double hoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, calculatedHood));
                    
                    // Set hood servo position immediately when locked
                    if (hoodServo != null) {
                        hoodServo.setPosition(hoodPosition);
                        telemetry.addLine("🎯 Hood adjusted to: " + String.format("%.4f", hoodPosition));
                    }
                    
                    // Calculate target RPM and store it for velocity control (with distance-based scaling)
                    double calculatedRPM = robot.calculateShooterRPM(lockedDistance);
                    double scaleFactor = calculateRPMScaleFactor(lockedDistance);
                    targetRPM = calculatedRPM * scaleFactor; // Apply distance-based scaling factor
                    telemetry.addData("Formula RPM (raw)", "%.0f", calculatedRPM);
                    telemetry.addData("RPM Scale Factor", "%.3f (distance-based)", scaleFactor);
                    telemetry.addData("Formula RPM (scaled)", "%.0f", targetRPM);
                    telemetry.addData("Formula Hood", "%.4f", hoodPosition);
                    telemetry.addData("Distance", "%.1f\"", lockedDistance);
                } catch (Exception e) {
                    telemetry.addData("Formula Error", e.getMessage());
                    e.printStackTrace();
                }
            } else {
                telemetry.addData("Formula Warning", "Robot or distance not available");
            }
        }
        
        // Control shooter based on toggle state
        if (shooterMotor == null) {
            return;
        }
        
        // Only shoot if locked AND toggled on
        if (isLocked && isShooting) {
            // If lockedDistance is 0, try to get it fresh (critical for shooting)
            if (lockedDistance <= 0 && aprilTagDetector != null) {
                try {
                    AprilTagDetector.AprilTagResult freshTag = aprilTagDetector.getTagById(TARGET_TAG_ID);
                    if (freshTag != null && freshTag.isValid && freshTag.distance > 0) {
                        lockedDistance = freshTag.distance;
                        cachedTagResult = freshTag; // Update cache
                        telemetry.addLine("⚠️ Retrieved distance on shoot: " + String.format("%.1f\"", lockedDistance));
                    }
                } catch (Exception e) {
                    telemetry.addData("Distance Retrieval Error", e.getMessage());
                }
            }
            
            // ALWAYS recalculate RPM when shooting (ensures it's always up to date)
            if (lockedDistance > 0 && robot != null) {
                try {
                    double newRPM = robot.calculateShooterRPM(lockedDistance);
                    if (newRPM != 0.0) {
                        double scaleFactor = calculateRPMScaleFactor(lockedDistance);
                        targetRPM = newRPM * scaleFactor; // Apply distance-based scaling factor
                        telemetry.addData("RPM Calculated (raw)", "%.0f", newRPM);
                        telemetry.addData("RPM Scale Factor", "%.3f (distance-based)", scaleFactor);
                        telemetry.addData("RPM Calculated (scaled)", "%.0f (from %.1f\")", targetRPM, lockedDistance);
                    } else {
                        telemetry.addData("RPM Warning", "Calculation returned 0 for distance %.1f\"", lockedDistance);
                    }
                } catch (Exception e) {
                    telemetry.addData("RPM Calc Error", e.getMessage());
                    e.printStackTrace();
                }
            } else {
                telemetry.addData("RPM Error", "lockedDistance=%.1f, robot=%s", lockedDistance, robot != null ? "OK" : "NULL");
                if (aprilTagDetector == null) {
                    telemetry.addLine("⚠️ AprilTagDetector is NULL!");
                }
            }
            
            if (targetRPM != 0.0) {
                try {
                    // Convert RPM to ticks per second for velocity control
                    // RPM to ticks/sec: (RPM / 60) * TICKS_PER_REVOLUTION
                    // Note: targetRPM is negative (reverse direction), so velocity will be negative
                    int velocityTicksPerSec = (int)Math.round((targetRPM / 60.0) * TICKS_PER_REVOLUTION);
                    shooterMotor.setVelocity(velocityTicksPerSec);
                    
                    // Debug telemetry
                    telemetry.addData("Shooter Target RPM", "%.0f", targetRPM);
                    telemetry.addData("Shooter Velocity", "%d ticks/sec", velocityTicksPerSec);
                } catch (Exception e) {
                    telemetry.addData("Shoot Error", e.getMessage());
                    e.printStackTrace();
                }
            } else {
                telemetry.addData("Shooter Warning", "targetRPM is 0 - cannot shoot");
                telemetry.addData("Debug Info", "lockedDistance=%.1f, robot=%s, isLocked=%s", 
                    lockedDistance, robot != null ? "OK" : "NULL", isLocked);
            }
        } else {
            // Stop shooting if not locked or not toggled on
            try {
                shooterMotor.setVelocity(0);
            } catch (Exception e) {
                telemetry.addData("Shoot Stop Error", e.getMessage());
            }
        }
    }
    
    /**
     * Simple search and lock behavior:
     * - Spinner moves continuously searching for April Tag 24
     * - When tag is found, aligns toward it
     * - When tag is found within 5 degrees, stop and lock
     * - Stops after 2 full spins if tag not found
     */
    private void searchAndLock() {
        if (limelight == null || aprilTagDetector == null || robot == null || robot.spinner == null) {
            return;
        }
        
        // If already locked or search stopped, don't move - ALWAYS set power to 0
        // Skip ALL Limelight calls when locked to prevent disconnects
        if (isLocked || searchStopped) {
            try {
                robot.spinner.setPower(0.0);
                // Add simple status to show OpMode is still active
                if (isLocked) {
                    telemetry.addLine("🔒 LOCKED - OpMode Active (No Limelight calls)");
                }
            } catch (Exception e) {
                // Ignore hardware errors when locked
                telemetry.addData("Lock Status", "Motor stopped (error ignored)");
            }
            return; // Exit early - NO Limelight calls when locked!
        }
        
        // Check if we've exceeded max spins
        if (limitHitCount >= MAX_SPINS * 2) {
            try {
                robot.spinner.setPower(0);
            } catch (Exception e) {
                // Ignore hardware errors
            }
            searchStopped = true;
            return;
        }
        
        try {
            // Call Limelight more frequently when searching (to catch tag), less when locked
            loopCounter++;
            int callInterval = isLocked ? LIMELIGHT_CALL_INTERVAL_LOCKED : LIMELIGHT_CALL_INTERVAL_SEARCH;
            boolean shouldCallLimelight = (loopCounter % callInterval == 0);
            
            // Use raw Limelight result like LimeLightTeleOpTest does (which works!)
            // Call more frequently when searching to catch the tag, less when locked to prevent disconnects
            if (limelight != null && shouldCallLimelight) {
                // Skip status check to reduce network calls - only check when needed
                // Skip status check to reduce network calls - only check occasionally
                // Removed frequent status checks to prevent disconnects
                
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    // Get raw values like LimeLightTeleOpTest does (lines 330-332)
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();
                    
                    // Check if we have a valid AprilTag detection using area (like LimeLightTeleOpTest line 336)
                    // MIN_TAG_AREA: ta is 0-1 range where 1.0 = 100%, so 0.003 = 0.3% (lowered to catch smaller detections)
                    double MIN_TAG_AREA = 0.003;
                    
                    // Also check fiducials to verify it's tag 24
                    List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    boolean hasTag24 = false;
                    
                    if (fiducials != null && !fiducials.isEmpty()) {
                        for (int i = 0; i < fiducials.size(); i++) {
                            com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid = fiducials.get(i);
                            int tagId = fid.getFiducialId();
                            if (tagId == TARGET_TAG_ID) {
                                hasTag24 = true;
                                break; // Found it, no need to continue
                            }
                        }
                    }
                    
                    // Reduced debug telemetry to prevent overload (only show key info)
                    if (hasTag24) {
                        telemetry.addData("DEBUG", "Tag 24 detected! ta=%.3f%%", ta * 100);
                    }
                    
                    if (ta > MIN_TAG_AREA && hasTag24) {
                        // Tag 24 found! Cache the tag result to avoid extra Limelight calls
                        try {
                            if (aprilTagDetector != null) {
                                cachedTagResult = aprilTagDetector.getTagById(TARGET_TAG_ID);
                            }
                        } catch (Exception e) {
                            // Ignore - will use raw tx/ty values instead
                        }
                        
                        // Reset lost detection counter - we found the tag!
                        lostDetectionCount = 0;
                        
                        // Start braking if we just started tracking (first detection)
                        if (!isTracking) {
                            isTracking = true;
                            brakeStartTime = System.currentTimeMillis(); // Start brake timer
                            telemetry.addLine("✅✅✅ TAG 24 DETECTED - BRAKING & ALIGNING ✅✅✅");
                        } else {
                            telemetry.addLine("✅✅✅ TAG 24 DETECTED - ALIGNING ✅✅✅");
                        }
                        
                        // tx is already set from raw result (like LimeLightTeleOpTest line 330)
                        double absTx = Math.abs(tx);
                        
                        // Check if we're still in braking phase
                        long brakeElapsed = System.currentTimeMillis() - brakeStartTime;
                        boolean isBraking = (brakeStartTime > 0 && brakeElapsed < BRAKE_DURATION_MS);
                        
                        // Horizontal alignment: drive spinner so tx -> 0°
                        double cmd = 0.0;
                        
                        if (isBraking) {
                            // Immediately stop the spinner (set power to 0) to stop momentum
                            // This is safer than reverse braking which might go the wrong direction
                            try {
                                robot.spinner.setPower(0.0);
                                telemetry.addData("Status", "STOPPING (%.0f ms)", (double)brakeElapsed);
                            } catch (Exception e) {
                                // Ignore
                            }
                            // Skip alignment during stopping phase - just stop, alignment will happen next loop
                            return; // Exit early, continue stopping next loop
                        } else {
                            // Braking complete - start post-brake gentle alignment phase
                            if (brakeStartTime > 0) {
                                // Just finished braking - start post-brake timer
                                postBrakeStartTime = System.currentTimeMillis();
                                brakeStartTime = 0; // Reset brake timer
                            }
                        }
                        
                        // Check deadband FIRST (like LimeLightTeleOpTest) - if within deadband, stop completely
                        // This matches LimeLightTeleOpTest line 341: if (Math.abs(tx) > TAG_DEADBAND)
                        if (absTx <= DEADBAND) {
                            // Within deadband - stop completely and lock (like LimeLightTeleOpTest stops rotation)
                            cmd = 0.0;
                            isLocked = true;
                            isTracking = false; // No longer tracking, we're locked
                            searchStopped = true;
                            lostDetectionCount = 0;
                            brakeStartTime = 0; // Reset brake timer
                            postBrakeStartTime = 0; // Reset post-brake timer
                            // Get distance - try cached first, then get fresh if needed (critical for shooting)
                            if (cachedTagResult != null && cachedTagResult.isValid && cachedTagResult.distance > 0) {
                                lockedDistance = cachedTagResult.distance;
                                telemetry.addLine("🔒 LOCKED - Distance: " + String.format("%.1f\"", lockedDistance));
                            } else {
                                // Cached result invalid - get fresh distance (one extra call is OK when locking)
                                try {
                                    if (aprilTagDetector != null) {
                                        AprilTagDetector.AprilTagResult freshTag = aprilTagDetector.getTagById(TARGET_TAG_ID);
                                        if (freshTag != null && freshTag.isValid && freshTag.distance > 0) {
                                            lockedDistance = freshTag.distance;
                                            cachedTagResult = freshTag; // Update cache
                                            telemetry.addLine("🔒 LOCKED - Distance: " + String.format("%.1f\"", lockedDistance));
                                        } else {
                                            lockedDistance = 60.0; // Fallback default
                                            telemetry.addLine("⚠️ LOCKED - Using default distance: 60.0\"");
                                        }
                                    } else {
                                        lockedDistance = 60.0; // Fallback default
                                        telemetry.addLine("⚠️ LOCKED - AprilTagDetector NULL, using default: 60.0\"");
                                    }
                                } catch (Exception e) {
                                    lockedDistance = 60.0; // Fallback default on error
                                    telemetry.addLine("⚠️ LOCKED - Error getting distance: " + e.getMessage());
                                }
                            }
                            // Immediately stop motor - don't wait for next loop
                            try {
                                robot.spinner.setPower(0.0);
                            } catch (Exception e) {
                                // Ignore
                            }
                        }
                        // Outside deadband - continue aligning (tolerance is just for reference, not locking)
                        // Only lock when within deadband (0.5°) for tight margin of error
                        else {
                            // Use POSITIVE multiplier like ShooterTurnTest (tx * KP)
                            cmd = tx * ALIGN_KP;
                            double sign = Math.signum(cmd);
                            
                            // Adjust power based on distance from target and time since braking:
                            // - Just after braking: Use very gentle correction to prevent overshooting
                            // - Far off (> TOLERANCE): Use moderate correction
                            // - Getting close (within TOLERANCE): Use normal correction for fine alignment
                            double maxPower = MAX_ALIGN_POWER;
                            
                            // Check if we're in post-brake gentle phase
                            boolean inPostBrakePhase = (postBrakeStartTime > 0 && 
                                (System.currentTimeMillis() - postBrakeStartTime) < POST_BRAKE_DURATION_MS);
                            
                            if (inPostBrakePhase) {
                                // Just after braking - use very gentle correction to prevent speeding up
                                maxPower = Math.min(0.15, SEARCH_POWER); // Same as search power or less
                                telemetry.addData("Status", "Post-brake gentle alignment");
                            } else if (absTx > TOLERANCE) {
                                // Far off - use moderate correction (not too strong to prevent overshooting)
                                maxPower = Math.min(0.25, MAX_ALIGN_POWER * 1.2); // Max 25% power when far
                                postBrakeStartTime = 0; // Clear post-brake timer
                            } else {
                                // Within tolerance - use normal correction for fine alignment toward 0.5° deadband
                                maxPower = MAX_ALIGN_POWER; // Normal max power (0.3)
                                postBrakeStartTime = 0; // Clear post-brake timer
                            }
                            
                            cmd = Math.min(maxPower, Math.max(MIN_ALIGN_POWER, Math.abs(cmd))) * sign;
                            
                            // Check encoder limits (with exception handling)
                            try {
                int currentPos = robot.spinner.getCurrentPosition();
                                if (cmd > 0 && currentPos >= SPINNER_MAX - 50) {
                                    cmd = 0.0; // Can't go right
                                }
                                if (cmd < 0 && currentPos <= SPINNER_MIN + 50) {
                                    cmd = 0.0; // Can't go left
                                }
                            } catch (Exception e) {
                                // Hardware access failed - stop motor
                                cmd = 0.0;
                            }
                        }
                        
                        // Apply command to motor - ALWAYS set power, even if 0.0
                        try {
                            robot.spinner.setPower(cmd);
                        } catch (Exception e) {
                            // Hardware access failed - log and continue
                            telemetry.addData("Error", "Motor setPower failed: " + e.getMessage());
                        }
                        
                        // DEBUG telemetry
                        telemetry.addData("DEBUG tx", "%.3f° (abs: %.3f°)", tx, absTx);
                        telemetry.addData("DEBUG cmd", "%.3f", cmd);
                        telemetry.addData("DEBUG locked", isLocked ? "YES" : "NO");
                    
                    } else {
                        // No tag 24 found this loop - immediately reset tracking and search (like AutoSpin.java)
                        // This ensures the spinner always moves when no tag is detected
                        isTracking = false;
                        lostDetectionCount = 0;
                        cachedTagResult = null;
                        brakeStartTime = 0; // Reset brake timer
                        postBrakeStartTime = 0; // Reset post-brake timer
                        continueSearch();
                    }
                } else {
                    // No result from Limelight - clear cache and keep searching
                    cachedTagResult = null;
                    isTracking = false; // Make sure tracking is off
                    brakeStartTime = 0; // Reset brake timer
                    postBrakeStartTime = 0; // Reset post-brake timer
                    continueSearch();
                }
            } else if (!shouldCallLimelight) {
                // Not calling Limelight this loop - if we have a valid cached result and are tracking, use it
                // Otherwise, just continue searching (simpler logic like AutoSpin.java)
                if (isTracking && cachedTagResult != null && cachedTagResult.isValid && cachedTagResult.tagId == TARGET_TAG_ID) {
                    // Use cached result for alignment (we're tracking, just waiting for next Limelight call)
                    double cachedTx = cachedTagResult.xDegrees;
                    double absTx = Math.abs(cachedTx);
                    
                    if (absTx <= DEADBAND) {
                        // Within deadband (0.5°) - lock!
                        isLocked = true;
                        isTracking = false;
                        searchStopped = true;
                        lostDetectionCount = 0;
                        try {
                            robot.spinner.setPower(0.0);
                        } catch (Exception e) {
                            // Ignore
                        }
                    } else {
                        // Still need to align - use cached tx with distance-based power
                        double cmd = cachedTx * ALIGN_KP;
                        double sign = Math.signum(cmd);
                        
                        // Adjust power based on distance from target and post-brake phase (same logic as main detection)
                        double maxPower = MAX_ALIGN_POWER;
                        boolean inPostBrakePhase = (postBrakeStartTime > 0 && 
                            (System.currentTimeMillis() - postBrakeStartTime) < POST_BRAKE_DURATION_MS);
                        
                        if (inPostBrakePhase) {
                            // Just after braking - use very gentle correction
                            maxPower = Math.min(0.15, SEARCH_POWER);
                        } else if (absTx > TOLERANCE) {
                            // Far off - use moderate correction
                            maxPower = Math.min(0.25, MAX_ALIGN_POWER * 1.2);
                            postBrakeStartTime = 0;
                        } else {
                            maxPower = MAX_ALIGN_POWER;
                            postBrakeStartTime = 0;
                        }
                        
                        cmd = Math.min(maxPower, Math.max(MIN_ALIGN_POWER, Math.abs(cmd))) * sign;
                        try {
                            robot.spinner.setPower(cmd);
                        } catch (Exception e) {
                            // Ignore
                        }
                    }
                } else {
                    // Not tracking or no valid cached result - immediately search (like AutoSpin.java)
                    isTracking = false;
                    brakeStartTime = 0; // Reset brake timer
                    postBrakeStartTime = 0; // Reset post-brake timer
                    continueSearch();
                }
            } else {
                // Limelight not initialized - keep searching
                continueSearch();
            }
            
        } catch (Exception e) {
            telemetry.addData("Error", "AprilTag detection failed: " + e.getMessage());
            continueSearch();
        }
    }
    
    
    /**
     * Continue searching by rotating the spinner
     * Handles encoder limits by reversing direction when limits are reached
     * Tracks limit hits to count full spins
     * Limits: -2300 (left, ~90°) to 4100 (right, ~300°)
     */
    private void continueSearch() {
        if (robot == null || robot.spinner == null) {
            return;
        }
        
        // CRITICAL: When searching, we should NOT be tracking or braking
        // Reset these states to ensure clean search behavior
        if (isTracking) {
            isTracking = false;
            brakeStartTime = 0;
            postBrakeStartTime = 0;
        }
        
        int currentPos;
        try {
            currentPos = robot.spinner.getCurrentPosition();
        } catch (Exception e) {
            // Hardware access failed - stop motor and return
            try {
                robot.spinner.setPower(0.0);
            } catch (Exception e2) {
                // Ignore
            }
            return;
        }
        
        // Strict limit enforcement - stop immediately if past limits
        try {
            if (currentPos >= SPINNER_MAX) {
                // At or past right limit - stop and reverse
                robot.spinner.setPower(0);
                if (searchDirection > 0 && !lastWasRightLimit) {
                    searchDirection = -1;
                    lastWasRightLimit = true;
                    lastWasLeftLimit = false;
                    limitHitCount++;
                }
                return;
            }
            if (currentPos <= SPINNER_MIN) {
                // At or past left limit - stop and reverse
                robot.spinner.setPower(0);
                if (searchDirection < 0 && !lastWasLeftLimit) {
                    searchDirection = 1;
                    lastWasLeftLimit = true;
                    lastWasRightLimit = false;
                    limitHitCount++;
                }
                return;
            }
        } catch (Exception e) {
            // Hardware access failed - return without moving
            return;
        }
        
        // Check if approaching limits and reverse before hitting them
        if (currentPos >= SPINNER_MAX - 50 && searchDirection > 0 && !lastWasRightLimit) {
            // Approaching right limit - reverse now
            searchDirection = -1;
            lastWasRightLimit = true;
            lastWasLeftLimit = false;
            limitHitCount++;
        } else if (currentPos <= SPINNER_MIN + 50 && searchDirection < 0 && !lastWasLeftLimit) {
            // Approaching left limit - reverse now
            searchDirection = 1;
            lastWasLeftLimit = true;
            lastWasRightLimit = false;
            limitHitCount++;
        } else if (currentPos > SPINNER_MIN + 50 && currentPos < SPINNER_MAX - 50) {
            // Not near a limit, reset flags so we can detect next limit hit
            lastWasRightLimit = false;
            lastWasLeftLimit = false;
        }
        
        // Move in search direction (direction already corrected if at limit)
        try {
            double searchPower = searchDirection * SEARCH_POWER;
            robot.spinner.setPower(searchPower);
            // Debug: confirm we're actually setting power
            telemetry.addData("Search Power", "%.3f (dir: %d)", searchPower, searchDirection);
        } catch (Exception e) {
            // Hardware access failed - stop motor
            try {
                robot.spinner.setPower(0.0);
            } catch (Exception e2) {
                // Ignore
            }
            telemetry.addData("Search Error", e.getMessage());
        }
    }
    
    private void updateTelemetry() {
        try {
        // Limelight status
            if (limelight != null && aprilTagDetector != null) {
            try {
                // Only get status occasionally to avoid blocking (every 10 loops = ~200ms)
                // For now, skip getStatus() to reduce network calls
                telemetry.addLine("=== Limelight Status ===");
                telemetry.addData("Pipeline", "0 (AprilTag)");
                
                // Use cached tag result instead of calling getTagById() again
                // This avoids multiple Limelight network calls per loop
                AprilTagDetector.AprilTagResult tagResult = cachedTagResult;
                
                // If no cached result, try to get one (but only if we haven't already this loop)
                // Skip Limelight calls when locked to prevent disconnects
                if (tagResult == null && !isLocked) {
                    try {
                        tagResult = aprilTagDetector.getTagById(TARGET_TAG_ID);
                        cachedTagResult = tagResult; // Cache it
                    } catch (Exception e) {
                        // Limelight call failed - use invalid result
                        tagResult = new AprilTagDetector.AprilTagResult();
                    }
                } else if (isLocked && tagResult == null) {
                    // When locked, use a dummy result to avoid Limelight calls
                    tagResult = new AprilTagDetector.AprilTagResult();
                }
                
                if (tagResult != null && tagResult.isValid) {
                    telemetry.addLine("\n--- AprilTag 24 Detected ---");
                    telemetry.addData("X Angle", "%.1f°", tagResult.xDegrees);
                    telemetry.addData("Distance", "%.1f inches", tagResult.distance);
                    telemetry.addData("Status", isLocked ? "🔒 LOCKED" : "🔍 SEARCHING");
                    try {
                        if (robot != null && robot.spinner != null) {
                            telemetry.addData("Spinner Power", "%.2f", robot.spinner.getPower());
                        } else {
                            telemetry.addData("Spinner Power", "N/A");
                        }
                    } catch (Exception e) {
                        telemetry.addData("Spinner Power", "Error");
                    }
                    
                    if (isLocked) {
                        telemetry.addLine("✅ Locked onto target!");
                        telemetry.addData("Shooting", isShooting ? "ON (Press A to toggle)" : "OFF (Press A to toggle)");
                    } else {
                        String direction = tagResult.xDegrees > 0 ? "right" : "left";
                        double absTx = Math.abs(tagResult.xDegrees);
                        telemetry.addData("Raw tx", "%.3f°", tagResult.xDegrees);
                        telemetry.addData("Abs tx", "%.3f°", absTx);
                        telemetry.addData("Deadband", "%.3f°", DEADBAND);
                        if (absTx <= DEADBAND) {
                            telemetry.addData("Action", "✅ WITHIN DEADBAND! Should STOP!");
                        } else {
                            telemetry.addData("Action", "Aligning %s (%.3f° from center, need < %.3f°)", 
                                direction, absTx, DEADBAND);
                        }
                        try {
                            if (robot != null && robot.spinner != null) {
                                telemetry.addData("Spinner Power", "%.3f", robot.spinner.getPower());
                            } else {
                                telemetry.addData("Spinner Power", "N/A");
                            }
                        } catch (Exception e) {
                            telemetry.addData("Spinner Power", "Error");
                        }
                        telemetry.addData("Is Locked?", isLocked ? "YES" : "NO");
                    }
                } else {
                    telemetry.addLine("\n❌ AprilTag 24 not detected");
                    if (searchStopped && !isLocked) {
                        telemetry.addLine("⚠️ NO APRIL TAG FOUND");
                        telemetry.addData("Status", "Stopped after %d spins", MAX_SPINS);
                    } else {
                        telemetry.addData("Status", "🔍 SEARCHING");
                    }
                    try {
                        if (robot != null && robot.spinner != null) {
                            telemetry.addData("Spinner Power", "%.2f", robot.spinner.getPower());
                        } else {
                            telemetry.addData("Spinner Power", "N/A");
                        }
                    } catch (Exception e) {
                        telemetry.addData("Spinner Power", "Error");
                    }
                }
                
                // Show search progress
                if (!isLocked && !searchStopped) {
                    double spinsCompleted = limitHitCount / 2.0;
                    telemetry.addData("Spins", "%.1f / %d", spinsCompleted, MAX_SPINS);
                }
                
                telemetry.addLine("\n--- Spinner Status ---");
                int pos = 0;
                try {
                    if (robot != null && robot.spinner != null) {
                        pos = robot.spinner.getCurrentPosition();
                        telemetry.addData("Position", "%d [%d - %d]", 
                            pos, SPINNER_MIN, SPINNER_MAX);
                        telemetry.addData("Direction", searchDirection > 0 ? "Right →" : "Left ←");
                        
                        // Warn if past limits
                        if (pos > SPINNER_MAX) {
                            telemetry.addData("Limit", "⚠️⚠️ PAST RIGHT LIMIT! (%d > %d)", pos, SPINNER_MAX);
                        } else if (pos < SPINNER_MIN) {
                            telemetry.addData("Limit", "⚠️⚠️ PAST LEFT LIMIT! (%d < %d)", pos, SPINNER_MIN);
                        } else if (pos >= SPINNER_MAX - 50) {
                            telemetry.addData("Limit", "⚠️ Near RIGHT limit");
                        } else if (pos <= SPINNER_MIN + 50) {
                            telemetry.addData("Limit", "⚠️ Near LEFT limit");
                        }
                    } else {
                        telemetry.addData("Position", "N/A (hardware not available)");
                    }
                } catch (Exception e) {
                    telemetry.addData("Position", "Error reading encoder: " + e.getMessage());
                }
                
                // Shooter status
                telemetry.addLine("\n--- Shooter Status ---");
                if (shooterMotor != null) {
                    try {
                        // Show velocity (since we're using velocity control, not power)
                        double currentVelocity = shooterMotor.getVelocity();
                        int currentRPM = (int)Math.round((currentVelocity / TICKS_PER_REVOLUTION) * 60.0);
                        telemetry.addData("Shooter Velocity", "%.1f ticks/sec (%.0f RPM)", currentVelocity, (double)currentRPM);
                        telemetry.addData("Target RPM", "%.0f", targetRPM);
                        telemetry.addData("Shooting", isShooting ? "ON (A toggled)" : "OFF");
                        telemetry.addData("Is Locked", isLocked ? "YES" : "NO");
                        if (!isLocked && isShooting) {
                            telemetry.addLine("⚠️ Must be locked to shoot");
                        }
                        if (isLocked && isShooting && targetRPM == 0.0) {
                            telemetry.addLine("⚠️ WARNING: targetRPM is 0!");
                        }
                    } catch (Exception e) {
                        telemetry.addData("Shooter", "Error reading: " + e.getMessage());
                    }
                } else {
                    telemetry.addData("Shooter", "NOT FOUND");
                }
                
                // Controls
                telemetry.addLine("\n--- Controls ---");
                telemetry.addData("A Button", "Toggle shooting (ON/OFF)");
                telemetry.addData("Note", "Must be locked to shoot");
                
                // Hood status
                if (isLocked && lockedDistance > 0) {
                    try {
                        if (robot != null) {
                            double calculatedRPM = robot.calculateShooterRPM(lockedDistance);
                            double calculatedHood = robot.calculateHoodPosition(lockedDistance, null);
                            telemetry.addLine("\n--- Formula Settings (31-point) ---");
                            telemetry.addData("Distance", "%.1f\"", lockedDistance);
                            telemetry.addData("Calculated RPM", "%.0f", calculatedRPM);
                            telemetry.addData("Calculated Hood", "%.4f", calculatedHood);
                        }
                    } catch (Exception e) {
                        telemetry.addData("Formula", "Error: " + e.getMessage());
                    }
                }
                
                if (hoodServo != null) {
                    try {
                        telemetry.addData("Hood Position", "%.4f", hoodServo.getPosition());
                    } catch (Exception e) {
                        telemetry.addData("Hood Position", "Error reading");
                    }
                } else {
                    telemetry.addData("Hood Servo", "NOT FOUND");
                }
                
            } catch (Exception e) {
                telemetry.addData("Error", "Limelight status error: " + e.getMessage());
            }
        }
        
            // Show final status if search stopped without finding tag
            if (searchStopped && !isLocked) {
                telemetry.addLine("\n=== SEARCH COMPLETE ===");
                telemetry.addLine("❌ NO APRIL TAG FOUND");
                telemetry.addData("Reason", "Completed %d full spins without finding tag", MAX_SPINS);
            }
        
        telemetry.update();
        } catch (Exception e) {
            // Catch any exceptions in telemetry to prevent OpMode from ending
            try {
                telemetry.addData("Telemetry Error", e.getMessage());
                telemetry.update();
            } catch (Exception e2) {
                // If even telemetry fails, just continue
            }
        }
    }
}

