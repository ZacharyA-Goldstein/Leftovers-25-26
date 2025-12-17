package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
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
 *   - Right Bumper: Slow mode (0.5x speed)
 * 
 * Gamepad 2 (Operator):
 *   - Left trigger: Toggle intake on/off
 *   - A button: Toggle transfer on/off
 *   - Right trigger: Toggle shooter motor on/off
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
    
    // Intake and transfer motors
    private DcMotor intake;
    private DcMotor transfer; // Transfer wheels motor
    
    // Auto-aim shooter components
    private dumbMapLime robot;
    private Limelight3A limelight;
    private AprilTagDetector aprilTagDetector;
    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    
    // --- TUNING: Camera mounting ---
    private static final double CAMERA_HEIGHT = 13.0; // Inches. Measure lens height from floor
    private static final double CAMERA_ANGLE = 0.0; // Degrees down from horizontal. Measure on robot
    private static final double MAX_DISTANCE = 144.0; // Maximum distance for tag detection
    
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
    private static final double TURRET_MAX_POWER = 0.25;  // Maximum alignment power - reduced to prevent overshoot
    private static final double TURRET_DEADBAND = 2.5;  // Deadband - no movement within this angle (degrees) - increased to reduce wobbling
    private static final double TURRET_SLOW_ZONE = 6.0; // Zone where turret slows down significantly (degrees) - increased
    private static final double TURRET_SLOW_POWER = 0.12; // Power used in slow zone - reduced for smoother approach
    private static final double TURRET_VERY_SLOW_ZONE = 3.5; // Very close zone with even slower power
    private static final double TURRET_VERY_SLOW_POWER = 0.06; // Very slow power when very close to target
    private static final double HORIZONTAL_OFFSET_DEG = 2.0; // Offset to compensate for shooting left/right (tune this)
    
    // --- TUNING: Hood adjustment ---
    // NOTE: Removed close distance adjustment - new formula (R² = 0.972) handles all distances accurately
    // If fine-tuning is needed, adjust the formula coefficients in dumbMapLime.java
    private static final double HOOD_MULTIPLIER = 0.95; // Multiplier applied to calculated hood position (tune this if needed)
    
    // --- TUNING: Hood servo limits ---
    private static final double HOOD_MIN = 0.677; // Minimum hood position
    private static final double HOOD_MAX = 0.717; // Maximum hood position
    
    // --- TUNING: Shooter motor constants ---
    private static final int TICKS_PER_REVOLUTION = 28; // PPR for GoBILDA 6000 RPM motor
    
    // --- TUNING: RPM scaling (distance-based adjustment) ---
    // NOTE: Removed scaling factor - new formula (R² = 0.972) is accurate enough
    // If fine-tuning is needed, adjust the formula coefficients in dumbMapLime.java
    
    // Drive control (from HeleOpBase)
    private static final double DRIVE_DEADBAND = 0.3; // Increased to filter out stick drift
    
    // Intake and transfer control
    private boolean intakeOn = false;
    private boolean transferOn = false;
    private boolean lastIntakeTrigger = false;
    private boolean lastAButton = false;
    private static final double INTAKE_POWER = 1.0;
    private static final double INTAKE_REVERSE_POWER = -1.0; // Reverse intake power
    private static final double TRANSFER_POWER = 1.0;
    
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
    private double targetRPM = 0.0; // Calculated by formula based on distance
    private boolean tagDetected = false;
    private double lastValidTx = 0.0; // Store last valid tx value for reference
    
    // Shooter control
    private boolean shooterOn = false;
    private boolean lastRightTrigger = false;
    
    // Turret state tracking
    private boolean turretStopped = false; // Track if turret has been set to 0 when tag is lost
    
    // Manual mode tracking
    private boolean manualTurretActive = false; // Track if turret is being manually controlled
    private boolean manualHoodActive = false; // Track if hood is being manually controlled
    
    // Simple shooter control
    private boolean simpleShooterOn = false; // Toggle state for simple shooter
    private boolean lastXButton = false; // Track X button state for toggle detection
    
    @Override
    public void runOpMode() {
        // Initialize drive motors
        initializeDriveMotors();
        
        // Initialize intake and transfer motors
        initializeIntakeAndTransfer();
        
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
                
                // Handle intake and transfer controls (gamepad 2)
                handleIntakeAndTransfer();
                
                // Handle simple shooter control (gamepad 2 X button)
                handleSimpleShooter();
                
                // Handle shooter toggle (gamepad 2 right trigger)
                handleShooterToggle();
                
                // Handle manual turret control (gamepad 2 D-pad) - only when no tag detected
                handleManualTurret();
                
                // Detect AprilTag and update auto-aim systems (only turret, always active)
                detectAndUpdate();
                
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
     * Initialize intake and transfer motors
     */
    private void initializeIntakeAndTransfer() {
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
        
        // Initialize transfer motor
        try {
            transfer = hardwareMap.get(DcMotor.class, "transfer");
            if (transfer != null) {
                transfer.setDirection(DcMotor.Direction.FORWARD);
                // Set run mode (matching intake - RUN_WITHOUT_ENCODER)
                transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                transfer.setPower(0);
                telemetry.addData("Transfer Motor", "Initialized");
            } else {
                telemetry.addData("Transfer Motor", "NOT FOUND");
            }
        } catch (Exception e) {
            transfer = null;
            telemetry.addData("Transfer Motor", "NOT FOUND");
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
                shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
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
        
        // Drive scale with right bumper (0.5 when pressed, 1.0 otherwise)
        double driveScale = gamepad1.right_bumper ? 0.5 : 1.0;
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
     * Handle intake and transfer controls (gamepad 2)
     */
    private void handleIntakeAndTransfer() {
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
                    }
                    lastIntakeTrigger = currentIntakeTrigger;
                    
                    intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
                }
            } catch (Exception e) {
                // Ignore - motor may have disconnected
            }
        } else {
            // Intake not available, still update trigger state
            boolean currentIntakeTrigger = gamepad2.left_trigger > 0.5;
            if (currentIntakeTrigger && !lastIntakeTrigger) {
                intakeOn = !intakeOn;
            }
            lastIntakeTrigger = currentIntakeTrigger;
        }
        
        // Transfer toggle on A button (gamepad 2)
        boolean aPressed = gamepad2.a;
        if (aPressed && !lastAButton) {
            transferOn = !transferOn;
        }
        lastAButton = aPressed;
        
        if (transfer != null) {
            try {
                transfer.setPower(transferOn ? TRANSFER_POWER : 0.0);
            } catch (Exception e) {
                // Ignore - motor may have disconnected
            }
        }
    }
    
    /**
     * Simple shooter control: Toggle shooter on/off with X button (gamepad 2)
     * When on, shooter runs at -6000 RPM and hood is at 0.677
     */
    private void handleSimpleShooter() {
        // Keep hood at 0.677 always
        if (hoodServo != null) {
            try {
                hoodServo.setPosition(HOOD_MIN); // HOOD_MIN is 0.677
            } catch (Exception e) {
                // Ignore
            }
        }
        
        // Toggle shooter on/off with X button (gamepad 2)
        boolean xPressed = gamepad2.x;
        
        // Detect button press (edge detection)
        if (xPressed && !lastXButton) {
            // Button just pressed - toggle shooter state
            simpleShooterOn = !simpleShooterOn;
        }
        lastXButton = xPressed;
        
        // Run shooter based on toggle state
        if (simpleShooterOn && shooterMotor != null) {
            try {
                // Convert -6000 RPM to ticks per second (negative for reverse direction)
                double velocityTicksPerSec = (-6000.0 / 60.0) * TICKS_PER_REVOLUTION;
                int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                shooterMotor.setVelocity(velocityTicksPerSecInt);
            } catch (Exception e) {
                // If velocity control fails, try power control at -1.0
                try {
                    shooterMotor.setPower(-1.0);
                } catch (Exception e2) {
                    // Ignore
                }
            }
        } else if (shooterMotor != null) {
            // Shooter off - stop motor
            try {
                shooterMotor.setVelocity(0);
            } catch (Exception e) {
                try {
                    shooterMotor.setPower(0.0);
                } catch (Exception e2) {
                    // Ignore
                }
            }
        }
    }
    
    /**
     * Handle right trigger toggle for shooter (gamepad 2)
     */
    private void handleShooterToggle() {
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;
        
        if (rightTriggerPressed && !lastRightTrigger) {
            shooterOn = !shooterOn;
        }
        lastRightTrigger = rightTriggerPressed;
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
     * Detect AprilTag and automatically update turret, hood, and shooter
     * This method is always active and continuously tracks the AprilTag.
     * Turret and hood update whenever a tag is detected.
     * Shooter motor RPM only updates when shooter is turned on via right trigger.
     */
    private void detectAndUpdate() {
        if (limelight == null || aprilTagDetector == null) {
            // No Limelight - stop everything
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            if (shooterMotor != null && !simpleShooterOn) {
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
                                // Use fresh tx value from current Limelight result (not cached)
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
            
            // Update all systems only if we have a valid, fresh tag detection in this loop
            if (freshTagDetected && tagDetected && cachedTagResult != null && cachedTagResult.isValid && currentDistance > 0) {
                // Reset stopped flag when tag is detected (auto mode takes over)
                turretStopped = false;
                
                // 1. Update turret (horizontal alignment) - always active when tag detected
                updateTurret(currentTx);
                
                // 2. Update hood position - only if simple shooter is off (simple shooter keeps hood at 0.677)
                if (!simpleShooterOn) {
                    updateHood(currentDistance);
                }
                
                // 3. Calculate shooter RPM - always calculate when tag detected (but don't apply if simple shooter is on)
                // Motor only runs when shooterOn is true, but RPM is always calculated and ready
                if (!simpleShooterOn) {
                    calculateShooterRPM(currentDistance);
                }
                
                // Only actually run motor if shooter toggle is on (and simple shooter is off)
                if (shooterOn && !simpleShooterOn) {
                    applyShooterRPM();
                } else if (!simpleShooterOn) {
                    // Motor off - stop velocity (only if simple shooter is also off)
                    if (shooterMotor != null) {
                        try {
                            shooterMotor.setVelocity(0);
                        } catch (Exception e) {
                            // Ignore
                        }
                    }
                }
            } else {
                // No fresh tag detected - handle turret, hood, and shooter separately
                
                // Manual turret control is handled in handleManualTurret() method
                // (called before detectAndUpdate() in main loop)
                
                // Hood and shooter respond to user input even when no tag detected (only if simple shooter is off)
                if (shooterOn && !simpleShooterOn) {
                    // Use last known distance if available, otherwise use default distance
                    double distanceToUse = (cachedTagResult != null && cachedTagResult.isValid && cachedTagResult.distance > 0) 
                        ? cachedTagResult.distance 
                        : 120.0; // Default distance when no tag detected
                    
                    // Update hood based on formula (even without tag)
                    updateHood(distanceToUse);
                    
                    // Calculate and apply shooter RPM based on formula
                    calculateShooterRPM(distanceToUse);
                    applyShooterRPM();
                } else if (!simpleShooterOn) {
                    // Shooter motor off - stop velocity (only if simple shooter is also off)
                    if (shooterMotor != null) {
                        try {
                            shooterMotor.setVelocity(0);
                        } catch (Exception e) {
                            // Ignore
                        }
                    }
                    // Hood stays at last position when shooter is off
                }
            }
        } catch (Exception e) {
            telemetry.addData("Detection Error", e.getMessage());
            // On error, stop shooter but keep turret stationary
            if (shooterMotor != null && !simpleShooterOn) {
                try {
                    shooterMotor.setVelocity(0);
                } catch (Exception e2) {
                    // Ignore
                }
            }
            // Turret stays stationary on error - don't change its power
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
            
            // Check if approaching limits - prevent movement beyond ±90 degrees
            if (cmd > 0 && currentPos >= SPINNER_MAX - 50) {
                cmd = 0.0; // Cannot go right - approaching +90° limit
            } else if (cmd < 0 && currentPos <= SPINNER_MIN + 50) {
                cmd = 0.0; // Cannot go left - approaching -90° limit
            }
            
            // Apply command to turret motor
            robot.spinner.setPower(cmd);
            
        } catch (Exception e) {
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
    private void updateHood(double distance) {
        if (robot == null) {
            telemetry.addData("Hood Error", "robot is null");
            return;
        }
        if (hoodServo == null) {
            telemetry.addData("Hood Error", "hoodServo is null");
            return;
        }
        
        try {
            // Clamp distance to formula limits before calculation
            double clampedDistance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE_FORMULA, distance));
            
            // Calculate hood position using new formula (R² = 0.972)
            // Formula automatically finds optimal hood/RPM combination
            double calculatedHood = robot.calculateHoodPosition(clampedDistance, null);
            
            // Apply hood multiplier (if still needed for fine-tuning)
            calculatedHood *= HOOD_MULTIPLIER;
            
            // Clamp to servo limits
            currentHoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, calculatedHood));
            
            // Apply to servo
            hoodServo.setPosition(currentHoodPosition);
            
        } catch (Exception e) {
            telemetry.addData("Hood Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Automatically adjust shooter RPM based on distance to AprilTag
     * Uses formula-based calculation with distance-based scaling factor.
     * Only called when shooter motor is turned on via right trigger.
     */
    /**
     * Calculate shooter RPM based on distance using new formula (prepares motor but doesn't run it)
     * Target RPM is calculated by the formula based on distance
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
     * Stop all motors
     */
    private void stopAllMotors() {
        try {
            if (leftFront != null) leftFront.setPower(0);
            if (leftBack != null) leftBack.setPower(0);
            if (rightFront != null) rightFront.setPower(0);
            if (rightBack != null) rightBack.setPower(0);
            if (intake != null) intake.setPower(0);
            if (transfer != null) transfer.setPower(0);
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
        if (shooterMotor == null) {
            return;
        }
        
        try {
            // Only check if shooter is running (simple shooter or auto-aim shooter)
            boolean shooterRunning = simpleShooterOn || shooterOn;
            
            if (shooterRunning) {
                double currentVelocity = shooterMotor.getVelocity();
                double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                
                // For simple shooter, target is -6000 RPM
                double targetRPMToCheck = simpleShooterOn ? -6000.0 : targetRPM;
                
                // Check if within tolerance
                double rpmError = Math.abs(currentRPM - targetRPMToCheck);
                boolean atTargetRPM = rpmError <= RPM_TOLERANCE;
                
                // Trigger haptics when reaching target (only once when first reaching target)
                if (atTargetRPM && !lastHapticState) {
                    // Just reached target - trigger haptics
                    gamepad2.rumble(200); // 200ms rumble
                }
                
                lastHapticState = atTargetRPM;
            } else {
                // Shooter not running - reset haptic state
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
            telemetry.addData("Transfer Motor", transfer != null ? "OK" : "NULL");
            telemetry.addData("Shooter Motor", shooterMotor != null ? "OK" : "NULL");
            
            // System status
            telemetry.addData("Tag", tagDetected ? "DETECTED" : "NOT DETECTED");
            if (simpleShooterOn) {
                telemetry.addData("Shooter", "ON (-6000 RPM)");
            } else {
                telemetry.addData("Shooter", shooterOn ? "ON (Auto)" : "OFF");
            }
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
            
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
            
            // Shooter RPM (when on - either simple shooter or auto-aim shooter)
            if ((simpleShooterOn || shooterOn) && shooterMotor != null) {
                try {
                    double currentVelocity = shooterMotor.getVelocity();
                    int currentRPM = (int)Math.round((currentVelocity / TICKS_PER_REVOLUTION) * 60.0);
                    if (simpleShooterOn) {
                        telemetry.addData("RPM", "%d / -6000", currentRPM);
                    } else {
                        telemetry.addData("RPM", "%d / %.0f", currentRPM, targetRPM);
                    }
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

