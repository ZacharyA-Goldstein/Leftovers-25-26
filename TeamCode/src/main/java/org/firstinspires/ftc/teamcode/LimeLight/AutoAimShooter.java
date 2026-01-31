package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    
    // Transfer continuous servos
    private CRServo blueTunnel;
    private CRServo blueToilet;
    private CRServo blackTunnel;
    // Transfer motor
    private DcMotor orangeToilet;
    
    // --- TUNING: Camera mounting ---
    private static final double CAMERA_HEIGHT = 13.0; // Inches. Measure lens height from floor
    private static final double CAMERA_ANGLE = 0.0; // Degrees down from horizontal. Measure on robot
    private static final double MAX_DISTANCE = 144.0; // Maximum distance for tag detection
    
    // --- TUNING: Distance limits for formula ---
    private static final double MIN_DISTANCE = 30.0; // Minimum distance for formula calculation (tune this for close shots)
    private static final double MAX_DISTANCE_FORMULA = 160.0; // Maximum distance for formula calculation
    
    // --- TUNING: Target AprilTag ID (change for different alliances) ---
    private static final int TARGET_TAG_ID = 20; // AprilTag ID to track (24 for red, 20 for blue, etc.)
    
    // --- TUNING: Turret limits (±90 degrees from initialization) ---
    private static final int SPINNER_MIN = -550;  // Encoder limit (min) - -90 degrees left
    private static final int SPINNER_MAX = 550;   // Encoder limit (max) - +90 degrees right
    
    // --- TUNING: Horizontal turret alignment ---
    private static final double TURRET_KP = 0.02;      // Proportional gain (deg -> power) - reduced to prevent wobbling
    private static final double TURRET_MIN_POWER = 0.08; // Minimum power to move turret - reduced to prevent constant movement
    private static final double TURRET_MAX_POWER = 0.25;  // Maximum alignment power - reduced to prevent overshoot
    private static final double TURRET_DEADBAND = 2.5;  // Deadband - no movement within this (degrees) - increased to reduce wobbling
    private static final double TURRET_SLOW_ZONE = 6.0; // Zone where turret slows down significantly (degrees) - increased
    private static final double TURRET_SLOW_POWER = 0.12; // Power used in slow zone - reduced for smoother approach
    private static final double TURRET_VERY_SLOW_ZONE = 3.5; // Very close zone with even slower power
    private static final double TURRET_VERY_SLOW_POWER = 0.06; // Very slow power when very close to target
    private static final double HORIZONTAL_OFFSET_DEG = 0.0; // Offset to compensate for shooting left/right (tune this)
    
    // --- TUNING: Hood adjustment for close distances ---
    // NOTE: Removed close distance adjustment - new formula (R² = 0.972) handles all distances accurately
    // If fine-tuning is needed, adjust the formula coefficients in dumbMapLime.java
    
    // --- TUNING: Hood servo limits ---
    private static final double HOOD_MIN = 0.677; // Minimum hood position
    private static final double HOOD_MAX = 0.717; // Maximum hood position
    
    // --- TUNING: Shooter motor constants ---
    private static final int TICKS_PER_REVOLUTION = 28; // PPR for GoBILDA 6000 RPM motor
    
    // --- TUNING: Manual mode adjustments ---
    private static final double MANUAL_RPM_STEP = 50.0; // RPM adjustment step size
    private static final double MANUAL_HOOD_STEP = 0.001; // Hood adjustment step size
    
    // --- TUNING: Transfer servo control ---
    private static final double TRANSFER_SERVO_POWER = 1.0; // Power for transfer servos
    private static final double BLUE_TOILET_TOGGLE_DURATION = 0.2; // Toggle every 0.2 seconds
    private static final double BLACK_TUNNEL_DURATION = 5.0; // Duration in seconds for BlackTunnel in intake-only mode (adjustable)
    private static final double RPM_TOLERANCE = 50.0; // RPM tolerance for "good" RPM
    private ElapsedTime blueToiletToggleTimer = new ElapsedTime();
    private boolean blueToiletToggleState = false; // Current on/off state for BlueToilet toggle
    private ElapsedTime blackTunnelTimer = new ElapsedTime(); // Timer for BlackTunnel in intake-only mode
    
    // --- TUNING: RPM scaling (distance-based adjustment) ---
    // NOTE: Removed scaling factor - new formula (R² = 0.972) is accurate enough
    // If fine-tuning is needed, adjust the formula coefficients in dumbMapLime.java
    
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
    
    // Transfer control (manual override toggle)
    private boolean manualTransferOverride = false;
    private boolean lastXButton = false;
    
    // Manual mode for testing
    private boolean manualMode = false;
    private boolean lastBButton = false;
    private double manualRPM = 3000.0; // Starting manual RPM value
    private double manualHood = (HOOD_MIN + HOOD_MAX) / 2.0; // Starting manual hood value
    
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
                shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
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
        
        // Initialize transfer servos and motor
        initializeTransferServos();
        
        telemetry.addData("Status", "Hardware initialized. Press START to begin.");
        telemetry.update();
    }
    
    /**
     * Initialize transfer continuous servos
     */
    private void initializeTransferServos() {
        try {
            blueTunnel = hardwareMap.get(CRServo.class, "BlueTunnel");
            if (blueTunnel != null) {
                blueTunnel.setPower(0.0);
                telemetry.addData("BlueTunnel", "Initialized");
            } else {
                telemetry.addData("BlueTunnel", "NOT FOUND");
            }
        } catch (Exception e) {
            blueTunnel = null;
            telemetry.addData("BlueTunnel", "NOT FOUND");
            e.printStackTrace();
        }
        
        try {
            blueToilet = hardwareMap.get(CRServo.class, "BlueToilet");
            if (blueToilet != null) {
                blueToilet.setPower(0.0);
                telemetry.addData("BlueToilet", "Initialized");
            } else {
                telemetry.addData("BlueToilet", "NOT FOUND");
            }
        } catch (Exception e) {
            blueToilet = null;
            telemetry.addData("BlueToilet", "NOT FOUND");
            e.printStackTrace();
        }
        
        try {
            orangeToilet = hardwareMap.get(DcMotor.class, "OrangeToilet");
            if (orangeToilet != null) {
                orangeToilet.setPower(0.0);
                telemetry.addData("OrangeToilet", "Initialized");
            } else {
                telemetry.addData("OrangeToilet", "NOT FOUND");
            }
        } catch (Exception e) {
            orangeToilet = null;
            telemetry.addData("OrangeToilet", "NOT FOUND");
            e.printStackTrace();
        }
        
        try {
            blackTunnel = hardwareMap.get(CRServo.class, "BlackTunnel");
            if (blackTunnel != null) {
                blackTunnel.setPower(0.0);
                telemetry.addData("BlackTunnel", "Initialized");
            } else {
                telemetry.addData("BlackTunnel", "NOT FOUND");
            }
        } catch (Exception e) {
            blackTunnel = null;
            telemetry.addData("BlackTunnel", "NOT FOUND");
            e.printStackTrace();
        }
        
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
                limelight.pipelineSwitch(3); // Pipeline 3 for Blue Alliance AprilTag
                sleep(500);
                
                LLResult testResult = limelight.getLatestResult();
                if (testResult != null) {
                    telemetry.addLine("✅ Limelight initialized successfully!");
                    telemetry.addData("Pipeline", "3 (AprilTag - Blue Alliance)");
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
                
                // Handle X button toggle for transfer
                handleTransferToggle();
                
                // Handle B button toggle for manual mode
                handleManualModeToggle();
                
                // Handle manual adjustments (if in manual mode)
                handleManualAdjustments();
                
                // Update transfer motor
                updateTransfer();
                
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
            if (blueTunnel != null) {
                blueTunnel.setPower(0.0);
            }
            if (blueToilet != null) {
                blueToilet.setPower(0.0);
            }
            if (blackTunnel != null) {
                blackTunnel.setPower(0.0);
            }
            if (orangeToilet != null) {
                orangeToilet.setPower(0.0);
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
        
        // Toggle on button press (edge detection)
        if (aPressed && !lastAButton) {
            shooterOn = !shooterOn;
        }
        lastAButton = aPressed;
        
        // Auto-stop shooter if tag is lost (only in auto mode)
        // This should only run AFTER the toggle, so it doesn't interfere with manual toggle
        if (!tagDetected && !manualMode && shooterOn) {
            shooterOn = false;
        }
    }
    
    /**
     * Handle X button toggle for manual transfer override
     */
    private void handleTransferToggle() {
        boolean xPressed = gamepad1.x;
        
        if (xPressed && !lastXButton) {
            manualTransferOverride = !manualTransferOverride;
        }
        lastXButton = xPressed;
    }
    
    /**
     * Handle transfer servo controls
     * - Transfer ONLY runs when manual override (X button) is enabled
     * - A button controls shooter only, X button controls transfer only (no double binding)
     * 
     * Servo/Motor directions:
     * - BlueTunnel: reverse (negative power)
     * - BlueToilet: reverse (negative power)
     * - BlackTunnel: forward (positive power)
     * - OrangeToilet: negative direction (negative power)
     */
    private void updateTransfer() {
        // Transfer ONLY runs when manual override is enabled (X button)
        // This ensures A button only controls shooter, X button only controls transfer
        boolean transferActive = manualTransferOverride;
        
        // Determine servo states
        // No intake in AutoAimShooter, so intakeActive is always false
        boolean intakeActive = false;
        // Shooter active only for reference (not used for transfer logic)
        boolean shooterActive = transferActive;
        
        // BlueTunnel: on when transfer is active (reverse direction)
        double blueTunnelPower = 0.0;
        if (transferActive) {
            blueTunnelPower = -TRANSFER_SERVO_POWER; // Reverse direction
        }
        
        // BlackTunnel: on when transfer is active (forward direction)
        double blackTunnelPower = 0.0;
        if (transferActive) {
            blackTunnelPower = TRANSFER_SERVO_POWER; // Forward direction
        }
        
        // OrangeToilet: on when transfer is active (negative direction)
        double orangeToiletPower = 0.0;
        if (transferActive) {
            orangeToiletPower = -TRANSFER_SERVO_POWER; // Negative direction
        }
        
        // BlueToilet: on when transfer is active, toggles every 0.2s (reverse direction)
        double blueToiletPower = 0.0;
        if (transferActive) {
            // Toggle every 0.2 seconds
            if (blueToiletToggleTimer.seconds() >= BLUE_TOILET_TOGGLE_DURATION) {
                blueToiletToggleState = !blueToiletToggleState;
                blueToiletToggleTimer.reset();
            }
            
            if (blueToiletToggleState) {
                blueToiletPower = -TRANSFER_SERVO_POWER; // Reverse direction
            } else {
                blueToiletPower = 0.0;
            }
        } else {
            // Transfer off - reset toggle state
            blueToiletToggleState = false;
            blueToiletToggleTimer.reset();
        }
        
        // Apply powers to servos and motor
        try {
            if (blueTunnel != null) {
                blueTunnel.setPower(blueTunnelPower);
            }
            if (blackTunnel != null) {
                blackTunnel.setPower(blackTunnelPower);
            }
            if (blueToilet != null) {
                blueToilet.setPower(blueToiletPower);
            }
            if (orangeToilet != null) {
                orangeToilet.setPower(orangeToiletPower);
            }
        } catch (Exception e) {
            // Ignore errors
        }
    }
    
    /**
     * Handle B button toggle for manual mode
     */
    private void handleManualModeToggle() {
        boolean bPressed = gamepad1.b;
        
        if (bPressed && !lastBButton) {
            manualMode = !manualMode;
        }
        lastBButton = bPressed;
    }
    
    /**
     * Handle manual adjustments for RPM and hood (D-pad controls)
     * D-pad Up/Down: Adjust RPM
     * D-pad Left/Right: Adjust Hood
     */
    private void handleManualAdjustments() {
        if (!manualMode) {
            return;
        }
        
        // Adjust RPM with D-pad Up/Down
        if (gamepad1.dpad_up) {
            manualRPM += MANUAL_RPM_STEP;
        } else if (gamepad1.dpad_down) {
            manualRPM -= MANUAL_RPM_STEP;
        }
        
        // Clamp RPM to reasonable range
        manualRPM = Math.max(-6000, Math.min(-2000, manualRPM));
        
        // Adjust Hood with D-pad Left/Right
        if (gamepad1.dpad_right) {
            manualHood += MANUAL_HOOD_STEP;
        } else if (gamepad1.dpad_left) {
            manualHood -= MANUAL_HOOD_STEP;
        }
        
        // Clamp hood to servo limits
        manualHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, manualHood));
        
        // Apply manual values directly
        if (shooterMotor != null) {
            if (shooterOn) {
                try {
                    // Convert RPM to ticks per second
                    double velocityTicksPerSec = (manualRPM / 60.0) * TICKS_PER_REVOLUTION;
                    int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                    shooterMotor.setVelocity(velocityTicksPerSecInt);
                    targetRPM = manualRPM;
                } catch (Exception e) {
                    telemetry.addData("Manual Shooter Error", e.getMessage());
                }
            } else {
                // Shooter toggled off - stop motor
                try {
                    shooterMotor.setVelocity(0);
                } catch (Exception e) {
                    // Ignore
                }
            }
        }
        
        if (hoodServo != null) {
            try {
                hoodServo.setPosition(manualHood);
                currentHoodPosition = manualHood;
            } catch (Exception e) {
                telemetry.addData("Manual Hood Error", e.getMessage());
            }
        }
    }
    
    /**
     * Detect AprilTag and automatically update turret, hood, and shooter
     */
    private void detectAndUpdate() {
        // Note: When tag is detected, hood and RPM always adjust automatically
        // The shooterOn toggle only affects behavior when no tag is detected
        
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
                // 1. Update turret (horizontal alignment) - always active when tag detected
                updateTurret(currentTx);
                
                // 2. Update hood position - always active when tag detected (only in auto mode)
                if (!manualMode) {
                    updateHood(currentDistance);
                }
                
                // 3. Calculate shooter RPM - always calculate when tag detected (only in auto mode)
                // Motor only runs when shooterOn is true, but RPM is always calculated and ready
                if (!manualMode) {
                    // Calculate target RPM (prepares motor to run)
                    calculateShooterRPM(currentDistance);
                    
                    // Only actually run motor if shooter toggle is on
                    if (shooterOn) {
                        applyShooterRPM();
                    } else {
                        // Motor off - stop velocity
                        if (shooterMotor != null) {
                            try {
                                shooterMotor.setVelocity(0);
                            } catch (Exception e) {
                                // Ignore
                            }
                        }
                    }
                }
            } else {
                // No FRESH tag - handle turret, hood, and shooter separately (like TeleOp)
                if (!manualMode) {
                    // Turret: stop when no tag detected
                    if (robot.spinner != null) {
                        robot.spinner.setPower(0.0);
                    }
                    
                    // Hood and shooter respond to user input even when no tag detected (like TeleOp)
                    if (shooterOn) {
                        // Use last known distance if available, otherwise use default distance
                        double distanceToUse = (cachedTagResult != null && cachedTagResult.isValid && cachedTagResult.distance > 0) 
                            ? cachedTagResult.distance 
                            : 120.0; // Default distance when no tag detected
                        
                        // Update hood based on formula (even without tag) - always adjusts like TeleOp
                        updateHood(distanceToUse);
                        
                        // Update shooter RPM based on formula
                        updateShooter(distanceToUse);
                    } else {
                        // Shooter motor off - stop velocity
                        if (shooterMotor != null) {
                            try {
                                shooterMotor.setVelocity(0);
                            } catch (Exception e) {
                                // Ignore
                            }
                        }
                    }
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
            
            // Progressive power control: slow down as we approach the target (reduces wobbling)
            double cmd;
            double sign = Math.signum(adjustedTx);
            
            if (absTx <= TURRET_VERY_SLOW_ZONE) {
                // Very close to target - use very slow power to prevent wobbling
                cmd = TURRET_VERY_SLOW_POWER * sign;
            } else if (absTx <= TURRET_SLOW_ZONE) {
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
     * Automatically adjust hood position based on distance using new formula
     */
    private void updateHood(double distance) {
        if (robot == null || hoodServo == null) {
            telemetry.addData("Hood Error", "robot or hoodServo is null");
            return;
        }
        
        try {
            // Clamp distance to formula limits before calculation
            double clampedDistance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE_FORMULA, distance));
            
            // Calculate hood position using new formula (R² = 0.972)
            // Formula automatically finds optimal hood/RPM combination
            double calculatedHood = robot.calculateHoodPosition(clampedDistance, null);
            
            // Clamp to servo limits
            currentHoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, calculatedHood));
            
            // Apply to servo
            hoodServo.setPosition(currentHoodPosition);
            
            // Debug telemetry
            telemetry.addData("Hood Update", "Distance: %.1f -> Hood: %.4f", distance, currentHoodPosition);
            
        } catch (Exception e) {
            telemetry.addData("Hood Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Calculate shooter RPM based on distance using new formula (prepares motor but doesn't run it)
     */
    private void calculateShooterRPM(double distance) {
        if (robot == null) {
            return;
        }
        
        try {
            // Clamp distance to formula limits before calculation
            double clampedDistance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE_FORMULA, distance));
            
            // Calculate target RPM using new formula (R² = 0.972)
            // Formula automatically finds optimal hood/RPM combination
            targetRPM = robot.calculateShooterRPM(clampedDistance);
            
        } catch (Exception e) {
            telemetry.addData("Shooter Calc Error", e.getMessage());
        }
    }
    
    /**
     * Apply the calculated RPM to the shooter motor
     */
    private void applyShooterRPM() {
        if (shooterMotor == null) {
            return;
        }
        
        try {
            // Convert RPM to ticks per second for velocity control
            double velocityTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;
            int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
            
            // Apply to shooter motor (positive velocity for forward direction)
            shooterMotor.setVelocity(velocityTicksPerSecInt);
            
        } catch (Exception e) {
            telemetry.addData("Shooter Apply Error", e.getMessage());
            try {
                shooterMotor.setVelocity(0);
            } catch (Exception e2) {
                // Ignore
            }
        }
    }
    
    /**
     * Update shooter RPM (for backward compatibility with no-tag case)
     */
    private void updateShooter(double distance) {
        calculateShooterRPM(distance);
        if (shooterOn) {
            applyShooterRPM();
        }
    }
    
    /**
     * Update telemetry with current status
     */
    private void updateTelemetry() {
        try {
            telemetry.addLine("=== Auto Aim Shooter ===");
            telemetry.addData("Mode", manualMode ? "MANUAL (B to toggle)" : "AUTO (B to toggle)");
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("A Button", gamepad1.a ? "PRESSED" : "Released");
            telemetry.addData("Manual Transfer Override", manualTransferOverride ? "ON (X button)" : "OFF");
            telemetry.addData("Limelight", limelight != null ? "Connected" : "Missing");
            telemetry.addData("Tag Detected", tagDetected ? "YES" : "NO");
            
            // Manual mode values (always show when in manual mode)
            if (manualMode) {
                telemetry.addLine("\n--- Manual Mode (Testing) ---");
                telemetry.addData("Manual RPM", "%.0f (D-pad Up/Down)", manualRPM);
                telemetry.addData("Manual Hood", "%.4f (D-pad Left/Right)", manualHood);
                telemetry.addData("Shooter", shooterOn ? "ON (A to toggle)" : "OFF (A to toggle)");
            }
            
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
                        double currentHoodPos = hoodServo.getPosition();
                        telemetry.addData("Hood Position (current)", "%.4f", currentHoodPos);
                        telemetry.addData("Hood Position (target)", "%.4f", currentHoodPosition);
                        telemetry.addData("Hood Distance", "%.1f inches", cachedTagResult.distance);
                    } catch (Exception e) {
                        telemetry.addData("Hood", "Error reading: " + e.getMessage());
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
                
                // DATA POINT FOR FORMULA (easy to copy)
                telemetry.addLine("\n=== DATA POINT (Copy this) ===");
                if (shooterMotor != null && hoodServo != null) {
                    try {
                        double currentVelocity = shooterMotor.getVelocity();
                        int currentRPM = (int)Math.round((currentVelocity / TICKS_PER_REVOLUTION) * 60.0);
                        double hoodPos = hoodServo.getPosition();
                        telemetry.addData("Format", "{%.1f, %.4f, %d},", 
                            cachedTagResult.distance, hoodPos, currentRPM);
                        telemetry.addLine("Distance: " + String.format("%.1f", cachedTagResult.distance) + 
                            " | Hood: " + String.format("%.4f", hoodPos) + 
                            " | RPM: " + currentRPM);
                    } catch (Exception e) {
                        telemetry.addData("Data Point", "Error: " + e.getMessage());
                    }
                }
                
                // Transfer status
                telemetry.addLine("\n--- Transfer Status ---");
                boolean transferActive = manualTransferOverride;
                
                if (transferActive) {
                    telemetry.addData("Transfer", "ACTIVE (X button pressed)");
                    telemetry.addData("BlueToilet", blueToiletToggleState ? "ON" : "OFF");
                } else {
                    telemetry.addData("Transfer", "OFF (Press X to activate)");
                }
                
                telemetry.addData("Servo/Motor Powers", String.format("BT:%.2f BlT:%.2f BTo:%.2f OTo:%.2f",
                    blueTunnel != null ? (transferActive ? -1.0 : 0.0) : -999.0,
                    blackTunnel != null ? (transferActive ? 1.0 : 0.0) : -999.0,
                    blueToilet != null ? (transferActive && blueToiletToggleState ? -1.0 : 0.0) : -999.0,
                    orangeToilet != null ? (transferActive ? -1.0 : 0.0) : -999.0));
                
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
            
            // Controls - Show all controls clearly
            telemetry.addLine("\n=== CONTROLS (Gamepad 1) ===");
            telemetry.addLine("--- Main Controls ---");
            telemetry.addData("A Button", "Toggle shooter ON/OFF (shooter only, no transfer)");
            telemetry.addData("X Button", "Toggle transfer ON/OFF (transfer only, independent of shooter)");
            telemetry.addData("B Button", "Toggle manual mode ON/OFF");
            if (manualMode) {
                telemetry.addLine("Manual Mode Controls:");
                telemetry.addData("D-pad Up", "Increase RPM (+" + MANUAL_RPM_STEP + ")");
                telemetry.addData("D-pad Down", "Decrease RPM (-" + MANUAL_RPM_STEP + ")");
                telemetry.addData("D-pad Right", "Increase Hood (+" + MANUAL_HOOD_STEP + ")");
                telemetry.addData("D-pad Left", "Decrease Hood (-" + MANUAL_HOOD_STEP + ")");
            }
            
            // Tuning values
            telemetry.addLine("\n--- Tuning Values ---");
            telemetry.addData("Min Distance", "%.1f\" (tune for close shots)", MIN_DISTANCE);
            telemetry.addData("Max Distance", "%.1f\"", MAX_DISTANCE_FORMULA);
            telemetry.addData("Horizontal Offset", "%.1f° (adjust if shooting left/right)", HORIZONTAL_OFFSET_DEG);
            telemetry.addData("Formula R²", "0.972 (new formula - no scaling/adjustments)");
            telemetry.addData("Turret KP", "%.3f", TURRET_KP);
            telemetry.addData("Turret Deadband", "%.1f°", TURRET_DEADBAND);
            telemetry.addData("Max Lost Detections", "%d", MAX_LOST_DETECTIONS);
            
        } catch (Exception e) {
            telemetry.addData("Telemetry Error", e.getMessage());
        }
        
        telemetry.update();
    }
}

