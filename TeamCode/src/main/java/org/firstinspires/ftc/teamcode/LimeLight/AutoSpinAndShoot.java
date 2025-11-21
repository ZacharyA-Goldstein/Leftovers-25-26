package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import java.util.List;

/**
 * AutoSpinAndShoot - Simple search and lock behavior for AprilTag 24
 * 
 * Behavior:
 * - Spinner moves continuously searching for April Tag 24
 * - When tag is found within 5 degrees, stops and locks onto it
 * - If tag not found within 2 full spins, stops and reports "no april tag found"
 */
@TeleOp(name = "Auto Spin and Shoot", group = "Test")
public class AutoSpinAndShoot extends LinearOpMode {
    private dumbMapLime robot;
    private Limelight3A limelight;
    
    // Constants
    private static final int SPINNER_MIN = -1000;  // Encoder limit (min) - roughly 90 degrees left
    private static final int SPINNER_MAX = 3000;   // Encoder limit (max) - roughly 300 degrees right
    private static final int TARGET_TAG_ID = 24;   // AprilTag ID to track
    private static final double CAMERA_HEIGHT = 9.5; // Camera height in inches
    private static final double CAMERA_ANGLE = 0.0; // Camera angle in degrees
    private static final double MAX_DISTANCE = 150.0; // Max detection distance in inches
    private static final double TOLERANCE = 3.0;   // Degrees tolerance - stop correcting within this (like ShooterTurnTest)
    private static final double DEADBAND = 0.5;    // Deadband - no movement within this (like ShooterTurnTest)
    private static final double ALIGN_KP = 0.02;   // Proportional constant (like ShooterTurnTest TURRET_KP)
    private static final double MIN_ALIGN_POWER = 0.12; // Minimum power to move (like ShooterTurnTest)
    private static final double MAX_ALIGN_POWER = 0.5;  // Maximum alignment power (like ShooterTurnTest)
    private static final double SEARCH_POWER = 0.3; // Power for searching rotation
    private static final int MAX_SPINS = 2;        // Maximum number of full spins before stopping
    
    // Runtime variables
    private boolean isLocked = false;   // Whether we've locked onto the target
    private boolean searchStopped = false; // Whether search has stopped (timeout or locked)
    private int searchDirection = 1;     // 1 for right, -1 for left
    private int limitHitCount = 0;       // Count of limit hits (2 hits = 1 full spin)
    private boolean lastWasRightLimit = false; // Track last limit hit to count full cycles
    private boolean lastWasLeftLimit = false;
    private AprilTagDetector aprilTagDetector; // AprilTag detector instance
    private AprilTagDetector.AprilTagResult cachedTagResult = null; // Cache tag result to avoid multiple Limelight calls
    
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
            // Search for and lock onto AprilTag
            searchAndLock();
            
            // Update telemetry
            updateTelemetry();
            
            // Small delay to prevent overwhelming the robot
            sleep(20);
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
        if (isLocked || searchStopped) {
            robot.spinner.setPower(0.0);
            return;
        }
        
        // Check if we've exceeded max spins
        if (limitHitCount >= MAX_SPINS * 2) {
            robot.spinner.setPower(0);
            searchStopped = true;
            return;
        }
        
        try {
            // Use raw Limelight result like LimeLightTeleOpTest does (which works!)
            if (limelight != null) {
                // Check status first to see what pipeline we're on
                try {
                    LLStatus status = limelight.getStatus();
                    if (status != null) {
                        telemetry.addData("DEBUG Pipeline", "%d (%s)", 
                            status.getPipelineIndex(), 
                            status.getPipelineType() != null ? status.getPipelineType() : "Unknown");
                    }
                } catch (Exception e) {
                    telemetry.addData("DEBUG Status Error", e.getMessage());
                }
                
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    // Get raw values like LimeLightTeleOpTest does (lines 330-332)
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();
                    
                    // DEBUG: Show what we're detecting
                    telemetry.addData("DEBUG Result Valid", result.isValid() ? "YES" : "NO");
                    telemetry.addData("DEBUG Raw tx", "%.3f°", tx);
                    telemetry.addData("DEBUG Raw ty", "%.3f°", ty);
                    telemetry.addData("DEBUG Raw ta", "%.3f%%", ta * 100);
                    
                    // Check if we have a valid AprilTag detection using area (like LimeLightTeleOpTest line 336)
                    // MIN_TAG_AREA from LimeLightTeleOpTest is 0.5
                    double MIN_TAG_AREA = 0.5;
                    
                    // Also check fiducials to verify it's tag 24
                    List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    boolean hasTag24 = false;
                    
                    telemetry.addData("DEBUG Fiducial Count", fiducials != null ? fiducials.size() : 0);
                    
                    if (fiducials != null && !fiducials.isEmpty()) {
                        for (int i = 0; i < fiducials.size(); i++) {
                            com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid = fiducials.get(i);
                            int tagId = fid.getFiducialId();
                            telemetry.addData("DEBUG Fiducial " + i, "ID: %d", tagId);
                            if (tagId == TARGET_TAG_ID) {
                                hasTag24 = true;
                            }
                        }
                    }
                    
                    // Check if we have a valid detection (area check like LimeLightTeleOpTest)
                    // AND verify it's tag 24
                    if (ta > MIN_TAG_AREA && hasTag24) {
                        // Tag 24 found! Use logic similar to LimeLightTeleOpTest which actually stops
                        // tx is already set from raw result (like LimeLightTeleOpTest line 330)
                        double absTx = Math.abs(tx);
                        
                        // Horizontal alignment: drive spinner so tx -> 0°
                        double cmd = 0.0;
                        
                        // Check deadband FIRST (like LimeLightTeleOpTest) - if within deadband, stop completely
                        // This matches LimeLightTeleOpTest line 341: if (Math.abs(tx) > TAG_DEADBAND)
                        if (absTx <= DEADBAND) {
                            // Within deadband - stop completely and lock (like LimeLightTeleOpTest stops rotation)
                            cmd = 0.0;
                            isLocked = true;
                            searchStopped = true;
                            // Immediately stop motor - don't wait for next loop
                            try {
                                robot.spinner.setPower(0.0);
                            } catch (Exception e) {
                                // Ignore
                            }
                        }
                        // Check tolerance - if within tolerance, also stop (treat as aligned)
                        else if (absTx <= TOLERANCE) {
                            // Within tolerance - stop and lock
                            cmd = 0.0;
                            isLocked = true;
                            searchStopped = true;
                            // Immediately stop motor - don't wait for next loop
                            try {
                                robot.spinner.setPower(0.0);
                            } catch (Exception e) {
                                // Ignore
                            }
                        }
                        // Outside both deadzone and tolerance - calculate correction
                        else {
                            // Use POSITIVE multiplier like ShooterTurnTest (tx * KP)
                            cmd = tx * ALIGN_KP;
                            double sign = Math.signum(cmd);
                            cmd = Math.min(MAX_ALIGN_POWER, Math.max(MIN_ALIGN_POWER, Math.abs(cmd))) * sign;
                            
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
                        // No tag 24 found (either no detection or wrong tag) - keep searching
                        continueSearch();
                    }
                } else {
                    // No result from Limelight - keep searching
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
            robot.spinner.setPower(searchDirection * SEARCH_POWER);
        } catch (Exception e) {
            // Hardware access failed - stop motor
            try {
                robot.spinner.setPower(0.0);
            } catch (Exception e2) {
                // Ignore
            }
        }
    }
    
    private void updateTelemetry() {
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
                if (tagResult == null) {
                    try {
                        tagResult = aprilTagDetector.getTagById(TARGET_TAG_ID);
                        cachedTagResult = tagResult; // Cache it
                    } catch (Exception e) {
                        // Limelight call failed - use invalid result
                        tagResult = new AprilTagDetector.AprilTagResult();
                    }
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
    }
}
