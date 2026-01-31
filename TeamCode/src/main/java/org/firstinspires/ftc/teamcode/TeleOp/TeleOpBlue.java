package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import org.firstinspires.ftc.teamcode.LimeLight.dumbMapLime;

import java.util.List;

/**
 * TeleOpBlue - Full teleop with drive, intake, transfer, and auto-aim shooter for Blue Alliance
 * 
 * Controls:
 * Gamepad 1 (Driver) - HeleOpBase drive style:
 *   - Left stick Y: Drive forward/backward
 *   - Right stick X: Strafe left/right
 *   - Left stick X: Turn left/right
 *   - Y Button: Slow mode (0.5x speed)
 * 
 * Gamepad 2 (Operator):
 *   - Left trigger: Toggle intake on/off
 *   - Left bumper: Reverse intake (hold)
 *   - X button: Toggle shooter motor on/off
 *   - A button: Manual transfer override - bypasses RPM check (works independently, even when shooter is off)
 * 
 * Auto-Aim Features (from AutoAimShooter, always active):
 * - Automatically aligns turret horizontally to AprilTag when tag is detected
 * - Automatically adjusts hood position based on distance when tag is detected
 * - Automatically adjusts shooter RPM based on distance (only when shooter motor is on)
 * - Turret limited to ±90 degrees from initialization
 * - Auto-aim systems continuously track AprilTag regardless of shooter motor state
 */
@TeleOp(name = "TeleOp Blue", group = "TeleOp")
public class TeleOpBlue extends LinearOpMode {
    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    
    // Intake motor
    private DcMotor intake;
    
    // Transfer continuous servos
    private CRServo blueTunnel;
    private CRServo blueToilet;
    private CRServo blackTunnel;
    // Transfer motor
    private DcMotor orangeToilet;
    
    // Auto-aim shooter components
    private dumbMapLime robot;
    private Limelight3A limelight;
    private AprilTagDetector aprilTagDetector;
    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    
    // --- TUNING: Camera mounting ---
    private static final double CAMERA_HEIGHT = 13.0; // Inches. Measure lens height from floor
    private static final double CAMERA_ANGLE = 0.0; // Degrees down from horizontal. Measure on robot
    private static final double MAX_DISTANCE = 160.0; // Maximum distance for tag detection
    
    // --- TUNING: Distance limits for formula ---
    private static final double MIN_DISTANCE = 30.0; // Minimum distance for formula calculation (tune this for close shots)
    private static final double MAX_DISTANCE_FORMULA = 160.0; // Maximum distance for formula calculation
    
    // --- TUNING: Target AprilTag ID for Blue Alliance ---
    private static final int TARGET_TAG_ID = 20; // AprilTag ID to track (20 for blue alliance)
    
    // --- TUNING: Turret limits (±90 degrees from initialization) ---
    private static final int SPINNER_MIN = -550;  // Encoder limit (min) - -90 degrees left
    private static final int SPINNER_MAX = 550;   // Encoder limit (max) - +90 degrees right
    
    // --- TUNING: Manual turret control ---
    private static final double MANUAL_TURRET_POWER = 0.3; // Power for manual turret movement via D-pad
    
    // --- TUNING: Manual hood control ---
    private static final double MANUAL_HOOD_STEP = 0.0002; // Step size for manual hood adjustment (servo position increment)
    
    // --- TUNING: Horizontal turret alignment ---
    private static final double TURRET_KP = 0.02;      // Proportional gain (degrees -> power) - reduced to prevent wobbling
    private static final double TURRET_MIN_POWER = 0.08; // Minimum power to move turret - reduced to prevent constant movement
    private static final double TURRET_MAX_POWER = 0.15;  // Maximum alignment power - reduced to prevent overshoot
    private static final double TURRET_DEADBAND = 3.5;  // Deadband - no movement within this angle (degrees) - increased to reduce wobbling
    private static final double TURRET_SLOW_ZONE = 6.0; // Zone where turret slows down significantly (degrees) - increased
    private static final double TURRET_SLOW_POWER = 0.12; // Power used in slow zone - reduced for smoother approach
    private static final double TURRET_VERY_SLOW_ZONE = 5.5; // Very close zone with even slower power
    private static final double TURRET_VERY_SLOW_POWER = 0.06; // Very slow power when very close to target
    private static final double HORIZONTAL_OFFSET_DEG = 0.0; // Offset to compensate for shooting left/right (tune this)
    private static final double TURRET_DIRECTION_FLIP = -1.0; // Flip turret direction: -1.0 if turning wrong way, 1.0 if correct
    
    // --- TUNING: Hood adjustment ---
    // NOTE: Formula was trained on data with hood 0.681-0.709, but new minimum is 0.690
    // Add small offset to shift formula values up to match new servo range
    // This prevents all values from being clamped to minimum
    private static final double HOOD_OFFSET = 0.010; // Offset to shift hood values up (tune this if needed)
    
    // --- TUNING: Hood servo limits ---
    private static final double HOOD_MIN = 0.690; // Minimum hood position
    private static final double HOOD_MAX = 0.717; // Maximum hood position
    
    // --- TUNING: Shooter motor constants ---
    private static final int TICKS_PER_REVOLUTION = 28; // PPR for GoBILDA 6000 RPM motor
    
    // --- TUNING: RPM scaling (distance-based adjustment) ---
    // NOTE: Removed scaling factor - new formula (R² = 0.972) is accurate enough
    // If fine-tuning is needed, adjust the formula coefficients in dumbMapLime.java
    
    // Drive control (from HeleOpBase)
    private static final double DRIVE_DEADBAND = 0.3; // Increased to filter out stick drift
    
    // Intake control
    private boolean intakeOn = false;
    private boolean lastIntakeTrigger = false;
    private static final double INTAKE_POWER = 1.0;
    private static final double INTAKE_REVERSE_POWER = -1.0; // Reverse intake power
    
    // Transfer servo control
    private static final double TRANSFER_SERVO_POWER = 1.0; // Power for transfer servos
    private static final double BLUE_TOILET_TOGGLE_DURATION = 0.2; // Toggle every 1 second
    private static final double BLACK_TUNNEL_DURATION = 5.0; // Duration in seconds for BlackTunnel in intake-only mode (adjustable)
    private ElapsedTime blueToiletToggleTimer = new ElapsedTime();
    private boolean blueToiletToggleState = false; // Current on/off state for BlueToilet toggle
    private ElapsedTime blackTunnelTimer = new ElapsedTime(); // Timer for BlackTunnel in intake-only mode
    
    // Haptics for shooter RPM
    private static final double RPM_TOLERANCE = 50.0; // RPM tolerance for "good" RPM (tune this)
    private boolean lastHapticState = false; // Track if haptics were active last loop
    
    // Auto-aim runtime variables
    private AprilTagDetector.AprilTagResult cachedTagResult = null;
    private int loopCounter = 0;
    private static final int LIMELIGHT_CALL_INTERVAL = 1; // Call Limelight every loop for faster response
    private int lostDetectionCount = 0;
    private static final int MAX_LOST_DETECTIONS = 2; // Max consecutive loops without detection before resetting tag state
    
    // Current auto-aim settings
    private double currentHoodPosition = (HOOD_MIN + HOOD_MAX) / 2.0;
    private static final double FIXED_TARGET_RPM = -4500.0; // Fixed RPM (negative for reverse direction)
    private double targetRPM = FIXED_TARGET_RPM; // Always use fixed RPM value
    private boolean tagDetected = false;
    private double lastValidTx = 0.0; // Store last valid tx value for reference
    
    // Shooter control
    private boolean shooterOn = false;
    
    // Turret state tracking
    private boolean turretStopped = false; // Track if turret has been set to 0 when tag is lost
    
    // Manual mode tracking
    private boolean manualTurretActive = false; // Track if turret is being manually controlled
    private boolean manualHoodActive = false; // Track if hood is being manually controlled
    
    // Shooter toggle tracking
    private boolean lastXButton = false; // Track X button state for toggle detection
    
    // Manual transfer override (when shooter is on)
    private boolean manualTransferOverride = false; // Manual override to activate transfer servos
    private boolean lastAButton = false; // Track A button state for manual transfer toggle
    
    @Override
    public void runOpMode() {
        // Initialize drive motors
        initializeDriveMotors();
        
        // Initialize intake motor
        initializeIntake();
        
        // Initialize transfer servos
        initializeTransferServos();
        
        // Initialize auto-aim shooter hardware (only turret, no shooter/hood)
        initializeAutoAimShooter();
        
        telemetry.addData("Status", "Hardware initialized. Press START to begin.");
        telemetry.update();
        
        // Wait for start
        waitForStart();
        
        // Initialize Limelight
        initializeLimelight();
        
        // Main loop
        while (opModeIsActive()) {
            try {
                // Handle drive controls (gamepad 1)
                handleDrive();
                
                // Handle intake controls (gamepad 2)
                handleIntake();
                
                // Handle transfer servo controls (based on intake and shooter state)
                handleTransferServos();
                
                // Handle shooter toggle (gamepad 2 X button) - turns motor on/off
                handleShooterToggle();
                
                // Handle manual transfer override (gamepad 2 A button) - works independently
                handleManualTransferOverride();
                
                // Handle manual turret control (gamepad 2 D-pad) - only when no tag detected
                handleManualTurret();
                
                // Detect AprilTag and update auto-aim systems (only turret, always active)
                detectAndUpdate();
                
                // Apply shooter RPM if manually turned on (always 6000 RPM, independent of Limelight)
                if (shooterOn) {
                    // Ensure targetRPM is set to fixed 6000 RPM
                    targetRPM = FIXED_TARGET_RPM;
                    applyShooterRPM();
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
                
                // Check shooter RPM and trigger haptics if at target
                checkShooterRPMAndHaptics();
                
                // Update telemetry
                updateTelemetry();
                
                sleep(20);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Exception in main loop: " + e.getMessage());
                telemetry.update();
                try {
                    stopAllMotors();
                } catch (Exception e2) {
                    // Ignore
                }
                sleep(100);
            }
        }
        
        // Stop all motors when OpMode ends
        stopAllMotors();
    }
    
    /**
     * Initialize drive motors
     */
    private void initializeDriveMotors() {
        try {
            // Use exact dumbMap motor names
            leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
            rightFront = hardwareMap.get(DcMotor.class, "frontRight");
            leftBack = hardwareMap.get(DcMotor.class, "backLeft");
            rightBack = hardwareMap.get(DcMotor.class, "backRight");
            
            if (leftFront == null || rightFront == null || leftBack == null || rightBack == null) {
                telemetry.addData("Drive Motors", "ERROR: One or more motors not found!");
                return;
            }
            
            // Set motor directions (matching dumbMap.init2())
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            
            // Set motor modes - use RUN_WITHOUT_ENCODER for more reliable control
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            // Set zero power behavior
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Stop all motors
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            
            telemetry.addData("Drive Motors", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Drive Motors Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Initialize intake motor
     */
    private void initializeIntake() {
        try {
            // Use exact dumbMap motor name
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotor.Direction.FORWARD);
            // Set run mode (matching dumbMap.init2() - RUN_WITHOUT_ENCODER)
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setPower(0);
            telemetry.addData("Intake Motor", "Initialized");
        } catch (Exception e) {
            intake = null;
            telemetry.addData("Intake Motor", "NOT FOUND");
            e.printStackTrace();
        }
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
    }
    
    /**
     * Initialize auto-aim shooter hardware
     */
    private void initializeAutoAimShooter() {
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
            e.printStackTrace();
        }
        
        // Initialize shooter motor (as DcMotorEx for velocity control)
        try {
            // Use exact dumbMap motor name (with fallback to "outtake")
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
            e.printStackTrace();
        }
        
        // Initialize hood servo (matching dumbMap.init2())
        try {
            // Use exact dumbMap servo name (with fallback to "servo")
            hoodServo = hardwareMap.get(Servo.class, "hood");
            if (hoodServo == null) {
                hoodServo = hardwareMap.get(Servo.class, "servo");
            }
            if (hoodServo != null) {
                hoodServo.setPosition(HOOD_MIN); // Set to 0.677 initially
                telemetry.addData("Hood Servo", "Initialized");
            } else {
                telemetry.addData("Hood Servo", "NOT FOUND");
            }
        } catch (Exception e) {
            telemetry.addData("Hood Servo Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Initialize Limelight using dumbMapLime (same as AutoAimShooter)
     */
    private void initializeLimelight() {
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();
        
        try {
            // Use dumbMapLime's initLimeLight() method (same as AutoAimShooter)
            robot.initLimeLight();
            
            // Get the Limelight instance from robot
            limelight = robot.getLimeLight();
            
            if (limelight != null) {
                // Set pipeline to 3 for Blue Alliance AprilTag (matching HeleOpBlue)
                limelight.pipelineSwitch(3);
                sleep(500);
                
                LLResult testResult = limelight.getLatestResult();
                if (testResult != null) {
                    telemetry.addLine("✅ Limelight initialized successfully!");
                    telemetry.addData("Pipeline", "3 (AprilTag - Blue Alliance)");
                } else {
                    telemetry.addLine("⚠️ Limelight found but not returning data yet");
                }
                
                // Create AprilTagDetector (same as AutoAimShooter)
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
    }
    
    /**
     * Handle drive controls (gamepad 1) - from HeleOpBase
     */
    private void handleDrive() {
        // Check if motors are initialized
        if (leftFront == null || leftBack == null || rightFront == null || rightBack == null) {
            return; // Motors not initialized, skip drive
        }
        
        // Get inputs (HeleOpBase mapping)
        double forward = gamepad1.left_stick_y;      // Forward/backward
        double strafe = -gamepad1.left_stick_x;     // Left/right strafe (negative left stick x)
        double rotate = -gamepad1.right_stick_x;    // Rotation (right stick x)
        
        // Apply deadband FIRST (before scaling) to filter out stick drift
        if (Math.abs(forward) < DRIVE_DEADBAND) forward = 0;
        if (Math.abs(strafe) < DRIVE_DEADBAND) strafe = 0;
        if (Math.abs(rotate) < DRIVE_DEADBAND) rotate = 0;
        
        // Drive scale with Y button (0.5 when pressed, 1.0 otherwise)
        double driveScale = gamepad1.y ? 0.5 : 1.0;
        forward *= driveScale;
        strafe *= driveScale;
        rotate *= driveScale;
        
        // Mecanum drive math (HeleOpBase style)
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;
        
        // Clamp powers to [-1.0, 1.0] range to prevent motor issues
        frontLeftPower = Math.max(-1.0, Math.min(1.0, frontLeftPower));
        frontRightPower = Math.max(-1.0, Math.min(1.0, frontRightPower));
        backLeftPower = Math.max(-1.0, Math.min(1.0, backLeftPower));
        backRightPower = Math.max(-1.0, Math.min(1.0, backRightPower));
        
        // Apply powers with error handling
        try {
            if (leftFront != null) {
                leftFront.setPower(frontLeftPower);
            } else {
                telemetry.addData("Drive Error", "leftFront is null!");
            }
            if (rightFront != null) {
                rightFront.setPower(frontRightPower);
            } else {
                telemetry.addData("Drive Error", "rightFront is null!");
            }
            if (leftBack != null) {
                leftBack.setPower(backLeftPower);
            } else {
                telemetry.addData("Drive Error", "leftBack is null!");
            }
            if (rightBack != null) {
                rightBack.setPower(backRightPower);
            } else {
                telemetry.addData("Drive Error", "rightBack is null!");
            }
        } catch (Exception e) {
            telemetry.addData("Drive Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Handle intake controls (gamepad 2)
     * Intake automatically turns on when shooter is at target RPM (or manual override is active)
     */
    private void handleIntake() {
        // Check if shooter is at target RPM or manual override is active
        boolean shooterAtTargetRPM = false;
        if (shooterOn && shooterMotor != null && targetRPM != 0) {
            try {
                double currentVelocity = shooterMotor.getVelocity();
                double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                double rpmError = Math.abs(currentRPM - targetRPM);
                shooterAtTargetRPM = rpmError <= RPM_TOLERANCE;
            } catch (Exception e) {
                // Ignore errors
            }
        }
        // Transfer only activates with manual override (no automatic activation)
        boolean shooterTransferActive = manualTransferOverride;
        
        // Reverse intake on left bumper (gamepad 2) - takes priority over normal intake
        boolean leftBumper = gamepad2.left_bumper;
        
        if (intake != null) {
            try {
                if (leftBumper) {
                    // Left bumper pressed - reverse intake
                    intake.setPower(INTAKE_REVERSE_POWER);
                } else {
                    // Normal intake toggle on left trigger (gamepad 2)
                    boolean currentIntakeTrigger = gamepad2.left_trigger > 0.5;
                    if (currentIntakeTrigger && !lastIntakeTrigger) {
                        intakeOn = !intakeOn;
                        if (intakeOn) {
                            // Reset timer when intake turns on
                            blackTunnelTimer.reset();
                        }
                    }
                    lastIntakeTrigger = currentIntakeTrigger;
                    
                    // Intake runs if manually toggled on OR if manual transfer override is active
                    boolean intakeShouldRun = intakeOn || shooterTransferActive;
                    intake.setPower(intakeShouldRun ? INTAKE_POWER : 0.0);
                }
            } catch (Exception e) {
                // Ignore - motor may have disconnected
            }
        } else {
            // Intake not available, still update trigger state
            boolean currentIntakeTrigger = gamepad2.left_trigger > 0.5;
            if (currentIntakeTrigger && !lastIntakeTrigger) {
                intakeOn = !intakeOn;
                if (intakeOn) {
                    // Reset timer when intake turns on
                    blackTunnelTimer.reset();
                }
            }
            lastIntakeTrigger = currentIntakeTrigger;
        }
        
    }
    
    /**
     * Handle transfer servo controls (same logic as TransferServoTest)
     * - When intake is on: BlueTunnel and BlackTunnel turn on
     * - When shooter is on AND at target RPM: All servos/motors turn on, BlueToilet toggles every 1.0s
     * 
     * Servo/Motor directions:
     * - BlueTunnel: reverse (negative power)
     * - BlueToilet: reverse (negative power)
     * - BlackTunnel: forward (positive power)
     * - OrangeToilet: negative direction (negative power)
     * 
     * Note: BlackTunnel stops after 5 seconds when only intake is active (not when shooter is active)
     */
    private void handleTransferServos() {
        // Check if shooter is at target RPM
        boolean shooterAtTargetRPM = false;
        if (shooterOn && shooterMotor != null && targetRPM != 0) {
            try {
                double currentVelocity = shooterMotor.getVelocity();
                double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                double rpmError = Math.abs(currentRPM - targetRPM);
                shooterAtTargetRPM = rpmError <= RPM_TOLERANCE;
            } catch (Exception e) {
                // Ignore errors
            }
        }
        
        // Determine servo states
        boolean intakeActive = intakeOn;
        // Shooter active ONLY when manual override is enabled (no automatic activation)
        // Manual override works independently - bypasses RPM check and shooter requirement
        boolean shooterActive = manualTransferOverride;
        
        // BlueTunnel: on when intake OR shooter is active (reverse direction)
        double blueTunnelPower = 0.0;
        if (intakeActive || shooterActive) {
            blueTunnelPower = -TRANSFER_SERVO_POWER; // Reverse direction
        }
        
        // BlackTunnel: on when intake OR shooter is active (forward direction)
        // In intake-only mode, stops after 5 seconds
        double blackTunnelPower = 0.0;
        if (shooterActive) {
            // Shooter active - always on
            blackTunnelPower = TRANSFER_SERVO_POWER; // Forward direction
        } else if (intakeActive) {
            // Intake active - check timer for intake-only mode
            if (blackTunnelTimer.seconds() < BLACK_TUNNEL_DURATION) {
                // Within 5 seconds - turn on
                blackTunnelPower = TRANSFER_SERVO_POWER; // Forward direction
            } else {
                // Past 5 seconds - turn off
                blackTunnelPower = 0.0;
            }
        }
        
        // OrangeToilet: on when shooter is active (negative direction)
        double orangeToiletPower = 0.0;
        if (shooterActive) {
            orangeToiletPower = -TRANSFER_SERVO_POWER; // Negative direction
        }
        
        // BlueToilet: on when shooter is active, toggles every 1.0s (reverse direction)
        double blueToiletPower = 0.0;
        if (shooterActive) {
            // Toggle every 1.0 seconds
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
            // Shooter off - reset toggle state
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
     * Handle right trigger toggle for shooter (gamepad 2)
     */
    private void handleShooterToggle() {
        // X button (gamepad 2) toggles shooter motor on/off
        boolean xPressed = gamepad2.x;
        
        if (xPressed && !lastXButton) {
            shooterOn = !shooterOn;
            // Reset manual transfer override when shooter turns off
            if (!shooterOn) {
                manualTransferOverride = false;
            }
        }
        lastXButton = xPressed;
    }
    
    /**
     * Handle manual transfer override (gamepad 2 A button)
     * A button can manually activate all transfer servos, bypassing RPM check
     * Works independently - can be used even when shooter is off
     */
    private void handleManualTransferOverride() {
        // A button (gamepad 2) toggles manual transfer override
        // Works independently of shooter state - allows manual control anytime
        boolean aPressed = gamepad2.a;
        
        if (aPressed && !lastAButton) {
            manualTransferOverride = !manualTransferOverride;
        }
        lastAButton = aPressed;
    }
    
    /**
     * Handle manual turret and hood control via D-pad (gamepad 2)
     * Only active when no tag is detected - allows human player to manually position turret and hood
     * D-pad left: rotate turret left (counter-clockwise)
     * D-pad right: rotate turret right (clockwise)
     * D-pad up: raise hood (increase servo position)
     * D-pad down: lower hood (decrease servo position)
     */
    private void handleManualTurret() {
        // Reset manual mode flags
        manualTurretActive = false;
        manualHoodActive = false;
        
        // Only allow manual control when no tag is detected
        if (tagDetected) {
            return;
        }
        
        try {
            // Manual turret control (D-pad left/right)
            if (robot != null && robot.spinner != null) {
                // Get current turret position
                int currentPos = robot.spinner.getCurrentPosition();
                
                // Check D-pad inputs for turret
                boolean dpadLeft = gamepad2.dpad_left;
                boolean dpadRight = gamepad2.dpad_right;
                
                double turretPower = 0.0;
                
                if (dpadLeft) {
                    // Rotate left (negative direction) - check limit
                    if (currentPos > SPINNER_MIN) {
                        turretPower = -MANUAL_TURRET_POWER;
                        manualTurretActive = true;
                    } else {
                        // At limit - stop
                        turretPower = 0.0;
                    }
                } else if (dpadRight) {
                    // Rotate right (positive direction) - check limit
                    if (currentPos < SPINNER_MAX) {
                        turretPower = MANUAL_TURRET_POWER;
                        manualTurretActive = true;
                    } else {
                        // At limit - stop
                        turretPower = 0.0;
                    }
                } else {
                    // No D-pad input - set to 0 to maintain belt tension
                    turretPower = 0.0;
                }
                
                // Apply power to turret
                robot.spinner.setPower(turretPower);
                turretStopped = false; // Reset stopped flag since we're controlling it
            }
            
            // Manual hood control (D-pad up/down)
            if (hoodServo != null) {
                // Check D-pad inputs for hood
                boolean dpadUp = gamepad2.dpad_up;
                boolean dpadDown = gamepad2.dpad_down;
                
                if (dpadUp || dpadDown) {
                    manualHoodActive = true;
                    
                    // Get current hood position
                    double currentHood = currentHoodPosition;
                    
                    if (dpadUp) {
                        // Raise hood (increase position) - check limit
                        currentHood = Math.min(HOOD_MAX, currentHood + MANUAL_HOOD_STEP);
                    } else if (dpadDown) {
                        // Lower hood (decrease position) - check limit
                        currentHood = Math.max(HOOD_MIN, currentHood - MANUAL_HOOD_STEP);
                    }
                    
                    // Clamp to limits
                    currentHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, currentHood));
                    
                    // Update and apply hood position
                    currentHoodPosition = currentHood;
                    hoodServo.setPosition(currentHoodPosition);
                }
            }
            
        } catch (Exception e) {
            telemetry.addData("Manual Control Error", e.getMessage());
            // On error, stop turret
            if (robot != null && robot.spinner != null) {
                try {
                    robot.spinner.setPower(0.0);
                } catch (Exception e2) {
                    // Ignore
                }
            }
        }
    }
    
    /**
     * Detect AprilTag and automatically update turret (horizontal alignment only)
     * This method is always active and continuously tracks the AprilTag.
     * Only turret updates - no hood or shooter updates.
     */
    private void detectAndUpdate() {
        if (limelight == null || aprilTagDetector == null) {
            // No Limelight - stop turret
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            return;
        }
        
        try {
            loopCounter++;
            boolean shouldCallLimelight = (loopCounter % LIMELIGHT_CALL_INTERVAL == 0);
            
            boolean freshTagDetected = false;
            double currentTx = 0.0;
            double currentDistance = 0.0;
            
            // Always get tx from Limelight for continuous turret alignment (even if not doing full detection)
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                
                // Always update currentTx from Limelight for continuous alignment
                currentTx = tx;
                
                // Only do full detection cycle (AprilTag detector) when throttled
                if (shouldCallLimelight) {
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
                                currentDistance = cachedTagResult.distance;
                                lastValidTx = tx; // Store for reference
                            }
                        } catch (Exception e) {
                            // Ignore
                        }
                    }
                    
                    // If no fresh tag detected this loop, increment lost count
                    if (!freshTagDetected) {
                        lostDetectionCount++;
                        if (lostDetectionCount > MAX_LOST_DETECTIONS) {
                            tagDetected = false;
                            cachedTagResult = null;
                            // Set turret to 0 power once to maintain belt tension (BRAKE mode holds position)
                            if (!turretStopped && robot.spinner != null) {
                                robot.spinner.setPower(0.0);
                                turretStopped = true;
                            }
                        }
                    } else {
                        // Tag detected again - reset the stopped flag
                        turretStopped = false;
                    }
                }
            }
            
            // Update turret based on tag detection status - CONTINUOUSLY every loop when tag is detected
            if (tagDetected && cachedTagResult != null && cachedTagResult.isValid) {
                // Reset stopped flag when tag is detected (auto mode takes over)
                turretStopped = false;
                
                // Always use current tx from Limelight for continuous alignment
                // This ensures the turret updates every loop, not just when doing full detection
                // NOTE: Use currentTx even if it's 0.0, as updateTurret() handles the deadband check
                if (currentTx != 0.0) {
                    updateTurret(currentTx);
                    lastValidTx = currentTx; // Keep lastValidTx updated
                } else if (lastValidTx != 0.0) {
                    // Fallback to last valid tx if current is 0
                    updateTurret(lastValidTx);
                } else {
                    // Both are 0 - stop turret
                    if (robot.spinner != null) {
                        robot.spinner.setPower(0.0);
                    }
                }
            } else {
                // No tag detected - manual turret control is handled in handleManualTurret() method
                // (called before detectAndUpdate() in main loop)
                telemetry.addData("Turret Status", "No tag detected (tagDetected: %s)", tagDetected);
            }
        } catch (Exception e) {
            telemetry.addData("Detection Error", e.getMessage());
            // On error, stop turret
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
        }
    }
    
    /**
     * Automatically adjust turret horizontally to align with AprilTag
     * Uses progressive power control: slower when close to target, faster when far away.
     * Respects encoder limits of ±90 degrees from initialization.
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
            
            // Check encoder limits (±90 degrees) - must check before applying power
            int currentPos;
            try {
                currentPos = robot.spinner.getCurrentPosition();
            } catch (Exception e) {
                // Cannot read encoder position - stop turret for safety
                robot.spinner.setPower(0.0);
                return;
            }
            
            // Safety check: if already at or past limits, stop immediately
            if (currentPos >= SPINNER_MAX) {
                robot.spinner.setPower(0.0);
                return;
            } else if (currentPos <= SPINNER_MIN) {
                robot.spinner.setPower(0.0);
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
                // Far from target - use proportional control with power limits
                cmd = adjustedTx * TURRET_KP;
                // Clamp power to min/max range
                double absCmd = Math.abs(cmd);
                if (absCmd > TURRET_MAX_POWER) {
                    cmd = TURRET_MAX_POWER * sign;
                } else if (absCmd < TURRET_MIN_POWER) {
                    cmd = TURRET_MIN_POWER * sign;
                }
            }
            
            // Apply direction flip if motor is wired backwards
            cmd = cmd * TURRET_DIRECTION_FLIP;
            
            // Check if approaching limits - prevent movement beyond ±90 degrees
            // After direction flip: if TURRET_DIRECTION_FLIP is -1.0, positive cmd turns left, negative turns right
            // So we need to check limits based on the ORIGINAL direction (before flip)
            // Original: positive = right (toward MAX), negative = left (toward MIN)
            // After flip: positive = left (toward MIN), negative = right (toward MAX)
            if (TURRET_DIRECTION_FLIP < 0) {
                // Direction is flipped: positive cmd = left, negative cmd = right
                if (cmd > 0 && currentPos <= SPINNER_MIN + 50) {
                    cmd = 0.0; // Cannot go further left - approaching MIN limit
                } else if (cmd < 0 && currentPos >= SPINNER_MAX - 50) {
                    cmd = 0.0; // Cannot go further right - approaching MAX limit
                }
            } else {
                // Direction is normal: positive cmd = right, negative cmd = left
                if (cmd > 0 && currentPos >= SPINNER_MAX - 50) {
                    cmd = 0.0; // Cannot go further right - approaching MAX limit
                } else if (cmd < 0 && currentPos <= SPINNER_MIN + 50) {
                    cmd = 0.0; // Cannot go further left - approaching MIN limit
                }
            }
            
            // Apply command to turret motor
            robot.spinner.setPower(cmd);
            
            // Debug telemetry - show turret alignment status
            telemetry.addData("Turret TX", "%.2f° (raw)", tx);
            telemetry.addData("Turret TX Adj", "%.2f° (offset: %.1f°)", adjustedTx, HORIZONTAL_OFFSET_DEG);
            telemetry.addData("Turret Cmd", "%.3f", cmd);
            telemetry.addData("Turret Deadband", "%.1f° (within: %s)", TURRET_DEADBAND, absTx <= TURRET_DEADBAND ? "YES" : "NO");
            telemetry.addData("Turret Pos", "%d (min: %d, max: %d)", currentPos, SPINNER_MIN, SPINNER_MAX);
            telemetry.addData("Turret At Limit", "%s", 
                (currentPos >= SPINNER_MAX) ? "MAX" : 
                (currentPos <= SPINNER_MIN) ? "MIN" : 
                (cmd > 0 && currentPos >= SPINNER_MAX - 50) ? "Near MAX" :
                (cmd < 0 && currentPos <= SPINNER_MIN + 50) ? "Near MIN" : "OK");
            
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
            // On any error, stop turret immediately
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
        }
    }
    
    /**
     * Automatically adjust hood position based on distance to AprilTag
     * Uses formula-based calculation with additional adjustment for close distances.
     * Hood position is clamped to servo limits.
     */
    /**
     * Update both hood and RPM together
     * RPM is fixed at 6000, hood still uses formula based on distance
     */
    private void updateHoodAndRPM(double distance) {
        if (robot == null) {
            telemetry.addData("Formula Error", "robot is null");
            return;
        }
        
        try {
            // Set RPM to fixed 6000 (negative for reverse direction)
            targetRPM = FIXED_TARGET_RPM;
            
            // Clamp distance to formula limits before calculation
            double clampedDistance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE_FORMULA, distance));
            
            // Calculate hood position using formula with fixed 6000 RPM
            // Pass the fixed RPM so hood is calculated for that specific RPM value
            double calculatedHood = robot.calculateHoodPosition(clampedDistance, FIXED_TARGET_RPM);
            
            // Debug telemetry - show values
            telemetry.addData("Shooter Settings", String.format("Dist:%.1f\" Hood:%.3f RPM:%.0f (FIXED)", 
                clampedDistance, calculatedHood, targetRPM));
            
            // Apply offset to shift formula values up to match new servo range (0.690-0.717)
            // Formula was trained on data with hood 0.681-0.709, new min is 0.690
            calculatedHood += HOOD_OFFSET;
            
            // Debug telemetry - show after offset
            telemetry.addData("Hood After Offset", String.format("Hood:%.3f (offset:%.3f)", 
                calculatedHood, HOOD_OFFSET));
            
            // Clamp to servo limits (should rarely be needed now with offset)
            currentHoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, calculatedHood));
            
            // Debug telemetry - show if hood was clamped
            if (calculatedHood < HOOD_MIN) {
                telemetry.addData("Hood Clamp", String.format("⚠️ CLAMPED UP: %.3f -> %.3f", calculatedHood, currentHoodPosition));
            } else if (calculatedHood > HOOD_MAX) {
                telemetry.addData("Hood Clamp", String.format("⚠️ CLAMPED DOWN: %.3f -> %.3f", calculatedHood, currentHoodPosition));
            } else {
                telemetry.addData("Hood Clamp", "✓ No clamp needed");
            }
            
            // Apply hood to servo
            if (hoodServo != null) {
                hoodServo.setPosition(currentHoodPosition);
            }
            
            // Debug telemetry - show final values
            telemetry.addData("Final Settings", String.format("Dist:%.1f\" Hood:%.3f RPM:%.0f (FIXED)", 
                clampedDistance, currentHoodPosition, targetRPM));
            
        } catch (Exception e) {
            telemetry.addData("Formula Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Update hood position (kept for backward compatibility, but now uses updateHoodAndRPM internally)
     */
    private void updateHood(double distance) {
        updateHoodAndRPM(distance);
    }
    
    /**
     * Calculate shooter RPM (kept for backward compatibility, but now uses updateHoodAndRPM internally)
     */
    private void calculateShooterRPM(double distance) {
        updateHoodAndRPM(distance);
    }

     //Apply the calculated RPM to the shooter motor
    private void applyShooterRPM() {
        if (shooterMotor == null) {
            return;
        }
        try {
            // Convert RPM to ticks per second for velocity control
            // Note: targetRPM is negative (reverse direction), so velocity will be negative
            double velocityTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;
            int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
            
            // Debug telemetry
            telemetry.addData("Shooter Velocity", String.format("RPM:%.0f -> %d ticks/sec", 
                targetRPM, velocityTicksPerSecInt));
            
            // Apply to shooter motor (negative velocity for reverse direction)
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
     * Stop all motors
     */
    private void stopAllMotors() {
        try {
            if (leftFront != null) leftFront.setPower(0);
            if (leftBack != null) leftBack.setPower(0);
            if (rightFront != null) rightFront.setPower(0);
            if (rightBack != null) rightBack.setPower(0);
            if (intake != null) intake.setPower(0);
            if (blueTunnel != null) blueTunnel.setPower(0.0);
            if (blueToilet != null) blueToilet.setPower(0.0);
            if (blackTunnel != null) blackTunnel.setPower(0.0);
            if (orangeToilet != null) orangeToilet.setPower(0.0);
            if (robot != null && robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            if (shooterMotor != null) {
                try {
                    shooterMotor.setVelocity(0);
                } catch (Exception e) {
                    shooterMotor.setPower(0.0);
                }
            }
            if (hoodServo != null) {
                hoodServo.setPosition(HOOD_MIN); // Set to 0.677
            }
        } catch (Exception e) {
            // Ignore
        }
    }
    
    /**
     * Check shooter RPM and trigger haptics when at target RPM
     */
    private void checkShooterRPMAndHaptics() {
        if (shooterMotor == null || gamepad2 == null) {
            return;
        }
        
        try {
            // Only check if shooter is running and target RPM is valid (RPMs are negative, so check != 0)
            if (shooterOn && targetRPM != 0) {
                double currentVelocity = shooterMotor.getVelocity();
                double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                
                // Check if within tolerance
                // Both currentRPM and targetRPM are negative, so compare directly
                double rpmError = Math.abs(currentRPM - targetRPM);
                boolean atTargetRPM = rpmError <= RPM_TOLERANCE;
                
                // Debug telemetry
                telemetry.addData("RPM Check", String.format("Cur:%.0f Tgt:%.0f Err:%.0f OK:%s", 
                    currentRPM, targetRPM, rpmError, atTargetRPM ? "YES" : "NO"));
                
                // Trigger haptics when reaching target (only once when first reaching target)
                if (atTargetRPM && !lastHapticState) {
                    // Just reached target - trigger haptics on gamepad 2
                    gamepad2.rumble(200); // 200ms rumble
                }
                
                lastHapticState = atTargetRPM;
            } else {
                // Shooter not running or no valid target RPM - reset haptic state
                lastHapticState = false;
            }
        } catch (Exception e) {
            // Ignore errors
        }
    }
    
    /**
     * Update telemetry with current status (simplified)
     */
    private void updateTelemetry() {
        try {
            telemetry.addLine("=== TeleOp Blue ===");
            
            // Debug: Motor initialization status
            telemetry.addData("Motors", String.format("LF:%s RF:%s LB:%s RB:%s", 
                leftFront != null ? "OK" : "NULL", 
                rightFront != null ? "OK" : "NULL",
                leftBack != null ? "OK" : "NULL",
                rightBack != null ? "OK" : "NULL"));
            telemetry.addData("Intake Motor", intake != null ? "OK" : "NULL");
            telemetry.addData("Transfer Servos/Motors", String.format("BT:%s BTo:%s BlT:%s OTo:%s",
                blueTunnel != null ? "OK" : "NULL",
                blueToilet != null ? "OK" : "NULL",
                blackTunnel != null ? "OK" : "NULL",
                orangeToilet != null ? "OK" : "NULL"));
            telemetry.addData("Shooter Motor", shooterMotor != null ? "OK" : "NULL");
            
            // System status
            telemetry.addData("Tag", tagDetected ? "DETECTED" : "NOT DETECTED");
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            // Transfer servo status
            if (shooterOn) {
                if (manualTransferOverride) {
                    telemetry.addData("Transfer", "ACTIVE (Manual Override)");
                    telemetry.addData("BlueToilet", blueToiletToggleState ? "ON" : "OFF");
                } else if (shooterMotor != null && targetRPM != 0) {
                    try {
                        double currentVelocity = shooterMotor.getVelocity();
                        double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                        double rpmError = Math.abs(currentRPM - targetRPM);
                        boolean atTarget = rpmError <= RPM_TOLERANCE;
                        telemetry.addData("Transfer", atTarget ? "ACTIVE (At RPM)" : "WAITING (Not at RPM)");
                        if (atTarget) {
                            telemetry.addData("BlueToilet", blueToiletToggleState ? "ON" : "OFF");
                        }
                    } catch (Exception e) {
                        // Ignore
                    }
                } else {
                    telemetry.addData("Transfer", "WAITING (No RPM target)");
                }
            } else if (intakeOn) {
                telemetry.addData("Transfer", "ACTIVE (Intake)");
                double timeRemaining = BLACK_TUNNEL_DURATION - blackTunnelTimer.seconds();
                if (timeRemaining > 0) {
                    telemetry.addData("BlackTunnel Timer", "%.1fs remaining", timeRemaining);
                } else {
                    telemetry.addData("BlackTunnel Timer", "STOPPED (5s elapsed)");
                }
            } else {
                telemetry.addData("Transfer", "OFF");
            }
            
            // Debug: Show actual servo/motor powers being applied
            telemetry.addLine();
            // Calculate active states for telemetry (matching TransferServoTest logic)
            boolean intakeActive = intakeOn;
            boolean shooterAtTargetRPM = false;
            if (shooterOn && shooterMotor != null && targetRPM != 0) {
                try {
                    double currentVelocity = shooterMotor.getVelocity();
                    double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                    double rpmError = Math.abs(currentRPM - targetRPM);
                    shooterAtTargetRPM = rpmError <= RPM_TOLERANCE;
                } catch (Exception e) {
                    // Ignore
                }
            }
            // Transfer only activates with manual override (no automatic activation)
            boolean shooterActive = manualTransferOverride;
            
            telemetry.addData("Servo/Motor Powers", String.format("BT:%.2f BlT:%.2f BTo:%.2f OTo:%.2f",
                blueTunnel != null ? (intakeActive || shooterActive ? -1.0 : 0.0) : -999.0,
                blackTunnel != null ? (intakeActive || shooterActive ? 1.0 : 0.0) : -999.0,
                blueToilet != null ? (shooterActive && blueToiletToggleState ? -1.0 : 0.0) : -999.0,
                orangeToilet != null ? (shooterActive ? -1.0 : 0.0) : -999.0));
            // Intake state (shows if manually on or from manual transfer override)
            // Transfer only activates with manual override (no automatic activation)
            boolean shooterTransferActive = manualTransferOverride;
            boolean intakeActuallyRunning = intakeOn || shooterTransferActive;
            
            if (shooterTransferActive && !intakeOn) {
                telemetry.addData("Intake State", "ON (Auto from Shooter)");
            } else {
                telemetry.addData("Intake State", intakeActuallyRunning ? "ON" : "OFF");
            }
            telemetry.addData("Shooter State", shooterOn ? "ON" : "OFF");
            telemetry.addData("Manual Transfer Override", manualTransferOverride ? "ON (Gamepad 2 A)" : "OFF (Gamepad 2 A)");
            if (manualTransferOverride) {
                telemetry.addData("Transfer Status", "ACTIVE (Manual Override - bypasses RPM check)");
            }
            
            // Debug: Drive inputs
            telemetry.addData("Drive", String.format("F:%.2f S:%.2f R:%.2f", 
                gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            
            // Manual mode (only show if active)
            if (manualTurretActive || manualHoodActive) {
                if (manualTurretActive) telemetry.addData("Manual Turret", "ACTIVE");
                if (manualHoodActive) telemetry.addData("Manual Hood", "ACTIVE");
            }
            
            // AprilTag info (when detected)
            if (tagDetected && cachedTagResult != null && cachedTagResult.isValid) {
                telemetry.addData("Distance", "%.1f\"", cachedTagResult.distance);
                telemetry.addData("X Angle", "%.1f°", cachedTagResult.xDegrees);
            }
            
            // Turret position
            if (robot != null && robot.spinner != null) {
                try {
                    int pos = robot.spinner.getCurrentPosition();
                    double posDegrees = (pos / 550.0) * 90.0;
                    telemetry.addData("Turret", "%.1f°", posDegrees);
                } catch (Exception e) {
                    // Ignore
                }
            }
            
            // Hood position
            if (hoodServo != null) {
                try {
                    telemetry.addData("Hood", "%.3f", hoodServo.getPosition());
                } catch (Exception e) {
                    // Ignore
                }
            }
            
            // Shooter RPM (when on)
            if (shooterOn && shooterMotor != null) {
                try {
                    double currentVelocity = shooterMotor.getVelocity();
                    int currentRPM = (int)Math.round((currentVelocity / TICKS_PER_REVOLUTION) * 60.0);
                    telemetry.addData("RPM", "%d / %.0f", currentRPM, targetRPM);
                } catch (Exception e) {
                    // Ignore
                }
            }
            
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        }
        
        telemetry.update();
    }
}

