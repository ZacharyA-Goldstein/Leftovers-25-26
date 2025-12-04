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
    
    // --- TUNING: Target AprilTag ID for Blue Alliance ---
    private static final int TARGET_TAG_ID = 20; // AprilTag ID to track (20 for blue alliance)
    
    // --- TUNING: Turret limits (±90 degrees from initialization) ---
    private static final int SPINNER_MIN = -550;  // Encoder limit (min) - -90 degrees left
    private static final int SPINNER_MAX = 550;   // Encoder limit (max) - +90 degrees right
    
    // --- TUNING: Horizontal turret alignment ---
    private static final double TURRET_KP = 0.03;      // Proportional gain (degrees -> power)
    private static final double TURRET_MIN_POWER = 0.15; // Minimum power to move turret
    private static final double TURRET_MAX_POWER = 0.35;  // Maximum alignment power
    private static final double TURRET_DEADBAND = 10.0;  // Deadband - no movement within this angle (degrees) - increased for stability
    private static final double TURRET_SLOW_ZONE = 10.0; // Zone where turret slows down significantly (degrees)
    private static final double TURRET_SLOW_POWER = 0.2; // Power used in slow zone for gentle approach
    private static final double HORIZONTAL_OFFSET_DEG = 2.0; // Offset to compensate for shooting left/right (tune this)
    
    // --- TUNING: Hood adjustment for close distances ---
    private static final double CLOSE_DISTANCE_THRESHOLD = 80.0; // Inches - below this distance is considered "close"
    private static final double HOOD_CLOSE_ADJUSTMENT = -0.02; // Adjustment added to hood position when close (negative value lowers hood)
    private static final double HOOD_MULTIPLIER = 0.95; // Multiplier applied to calculated hood position (tune this)
    
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
    
    // Drive control (from HeleOpBase)
    private static final double DRIVE_DEADBAND = 0.05;
    
    // Intake and transfer control
    private boolean intakeOn = false;
    private boolean transferOn = false;
    private boolean lastIntakeTrigger = false;
    private boolean lastAButton = false;
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;
    
    // Auto-aim runtime variables
    private AprilTagDetector.AprilTagResult cachedTagResult = null;
    private int loopCounter = 0;
    private static final int LIMELIGHT_CALL_INTERVAL = 1; // Call Limelight every loop for faster response
    private int lostDetectionCount = 0;
    private static final int MAX_LOST_DETECTIONS = 2; // Max consecutive loops without detection before resetting tag state
    
    // Current auto-aim settings
    private double currentHoodPosition = (HOOD_MIN + HOOD_MAX) / 2.0;
    private double targetRPM = 0.0;
    private boolean tagDetected = false;
    private double lastValidTx = 0.0; // Store last valid tx value for reference
    
    // Shooter control
    private boolean shooterOn = false;
    private boolean lastRightTrigger = false;
    
    // Turret state tracking
    private boolean turretStopped = false; // Track if turret has been set to 0 when tag is lost
    
    @Override
    public void runOpMode() {
        // Initialize drive motors
        initializeDriveMotors();
        
        // Initialize intake and transfer motors
        initializeIntakeAndTransfer();
        
        // Initialize auto-aim shooter hardware
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
                
                // Handle shooter toggle (gamepad 2 right trigger)
                handleShooterToggle();
                
                // Detect AprilTag and update auto-aim systems (always active)
                detectAndUpdate();
                
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
            
            // Set motor modes (matching dumbMap.init2() - RUN_USING_ENCODER)
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
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
                currentHoodPosition = (HOOD_MIN + HOOD_MAX) / 2.0;
                hoodServo.setPosition(currentHoodPosition);
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
        double strafe = -gamepad1.left_stick_x;     // Left/right strafe (negative right stick x)
        double rotate = -gamepad1.right_stick_x;       // Rotation (left stick x)
        
        // Drive scale with right bumper (0.5 when pressed, 1.0 otherwise)
        double driveScale = gamepad1.right_bumper ? 0.5 : 1.0;
        forward *= driveScale;
        strafe *= driveScale;
        rotate *= driveScale;
        
        // Apply deadband
        if (Math.abs(forward) < DRIVE_DEADBAND) forward = 0;
        if (Math.abs(strafe) < DRIVE_DEADBAND) strafe = 0;
        if (Math.abs(rotate) < DRIVE_DEADBAND) rotate = 0;
        
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
            }
            if (rightFront != null) {
                rightFront.setPower(frontRightPower);
            }
            if (leftBack != null) {
                leftBack.setPower(backLeftPower);
            }
            if (rightBack != null) {
                rightBack.setPower(backRightPower);
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
        // Intake toggle on left trigger (gamepad 2)
        boolean currentIntakeTrigger = gamepad2.left_trigger > 0.5;
        if (currentIntakeTrigger && !lastIntakeTrigger) {
            intakeOn = !intakeOn;
        }
        lastIntakeTrigger = currentIntakeTrigger;
        
        if (intake != null) {
            try {
                intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
            } catch (Exception e) {
                // Ignore - motor may have disconnected
            }
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
                // Reset stopped flag when tag is detected
                turretStopped = false;
                
                // 1. Update turret (horizontal alignment) - always active when tag detected
                updateTurret(currentTx);
                
                // 2. Update hood position - always active when tag detected
                updateHood(currentDistance);
                
                // 3. Update shooter RPM - only when shooter motor is turned on via right trigger
                if (shooterOn) {
                    updateShooter(currentDistance);
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
            } else {
                // No fresh tag detected - stop shooter motor and set turret to 0 power to maintain belt tension
                if (shooterMotor != null) {
                    try {
                        shooterMotor.setVelocity(0);
                    } catch (Exception e) {
                        // Ignore
                    }
                }
                // Set turret to 0 power once to maintain belt tension (BRAKE mode holds position)
                if (!turretStopped && robot.spinner != null) {
                    robot.spinner.setPower(0.0);
                    turretStopped = true;
                }
            }
        } catch (Exception e) {
            telemetry.addData("Detection Error", e.getMessage());
            // On error, stop shooter but keep turret stationary
            if (shooterMotor != null) {
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
            
            // Progressive power control: slow down as we approach the target
            double cmd;
            double sign = Math.signum(adjustedTx);
            
            if (absTx <= TURRET_SLOW_ZONE) {
                // Close to target - use slow, gentle power to prevent overshoot
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
            // Calculate hood position using formula (matching AutoAimShooter)
            double calculatedHood = robot.calculateHoodPosition(distance, null);
            
            // Apply hood multiplier
            calculatedHood *= HOOD_MULTIPLIER;
            
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
            e.printStackTrace();
        }
    }
    
    /**
     * Automatically adjust shooter RPM based on distance to AprilTag
     * Uses formula-based calculation with distance-based scaling factor.
     * Only called when shooter motor is turned on via right trigger.
     */
    private void updateShooter(double distance) {
        if (robot == null || shooterMotor == null) {
            return;
        }
        
        try {
            // Calculate target RPM using distance formula
            double calculatedRPM = robot.calculateShooterRPM(distance);
            
            // Apply distance-based scaling factor (adjusts for close/far distances)
            double scaleFactor = calculateRPMScaleFactor(distance);
            targetRPM = calculatedRPM * scaleFactor;
            
            // Convert RPM to ticks per second for velocity control
            double velocityTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;
            int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
            
            // Apply velocity to shooter motor (positive velocity for forward direction)
            shooterMotor.setVelocity(velocityTicksPerSecInt);
            
        } catch (Exception e) {
            // On error, stop shooter motor
            try {
                shooterMotor.setVelocity(0);
            } catch (Exception e2) {
                // Ignore
            }
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
                hoodServo.setPosition((HOOD_MIN + HOOD_MAX) / 2.0);
            }
        } catch (Exception e) {
            // Ignore
        }
    }
    
    /**
     * Update telemetry with current status
     */
    private void updateTelemetry() {
        try {
            telemetry.addLine("=== TeleOp Blue ===");
            
            // Drive status
            telemetry.addLine("\n--- Drive Status ---");
            double rawForward = gamepad1.left_stick_y;
            double rawStrafe = -gamepad1.right_stick_x;
            double rawRotate = gamepad1.left_stick_x;
            double driveScale = gamepad1.right_bumper ? 0.5 : 1.0;
            double forward = rawForward * driveScale;
            double strafe = rawStrafe * driveScale;
            double rotate = rawRotate * driveScale;
            
            // Apply deadband for display
            if (Math.abs(forward) < DRIVE_DEADBAND) forward = 0;
            if (Math.abs(strafe) < DRIVE_DEADBAND) strafe = 0;
            if (Math.abs(rotate) < DRIVE_DEADBAND) rotate = 0;
            
            telemetry.addData("Raw Forward", "%.2f", rawForward);
            telemetry.addData("Raw Strafe", "%.2f", rawStrafe);
            telemetry.addData("Raw Rotate", "%.2f", rawRotate);
            telemetry.addData("Forward (scaled)", "%.2f", forward);
            telemetry.addData("Strafe (scaled)", "%.2f", strafe);
            telemetry.addData("Rotate (scaled)", "%.2f", rotate);
            telemetry.addData("Drive Scale", "%.1f", driveScale);
            
            // Show calculated powers
            double flPower = forward + strafe + rotate;
            double frPower = forward - strafe - rotate;
            double blPower = forward - strafe + rotate;
            double brPower = forward + strafe - rotate;
            telemetry.addData("Calc FL", "%.3f", Math.max(-1.0, Math.min(1.0, flPower)));
            telemetry.addData("Calc FR", "%.3f", Math.max(-1.0, Math.min(1.0, frPower)));
            telemetry.addData("Calc BL", "%.3f", Math.max(-1.0, Math.min(1.0, blPower)));
            telemetry.addData("Calc BR", "%.3f", Math.max(-1.0, Math.min(1.0, brPower)));
            
            // Motor status
            telemetry.addData("Motors Initialized", 
                (leftFront != null && leftBack != null && rightFront != null && rightBack != null) ? "YES" : "NO");
            if (leftFront != null) {
                try {
                    telemetry.addData("LeftFront Power", "%.3f (cmd: %.3f)", leftFront.getPower(), 
                        gamepad1.left_stick_y + (-gamepad1.right_stick_x) + gamepad1.left_stick_x);
                } catch (Exception e) {
                    telemetry.addData("LeftFront", "Error reading");
                }
            }
            if (rightFront != null) {
                try {
                    telemetry.addData("RightFront Power", "%.3f", rightFront.getPower());
                } catch (Exception e) {
                    telemetry.addData("RightFront", "Error reading");
                }
            }
            if (leftBack != null) {
                try {
                    telemetry.addData("LeftBack Power", "%.3f", leftBack.getPower());
                } catch (Exception e) {
                    telemetry.addData("LeftBack", "Error reading");
                }
            }
            if (rightBack != null) {
                try {
                    telemetry.addData("RightBack Power", "%.3f", rightBack.getPower());
                } catch (Exception e) {
                    telemetry.addData("RightBack", "Error reading");
                }
            }
            
            // Intake and transfer status
            telemetry.addLine("\n--- Intake & Transfer ---");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Intake Motor", intake != null ? "Found" : "NOT FOUND");
            if (intake != null) {
                telemetry.addData("Intake Power", "%.3f", intake.getPower());
            }
            telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
            telemetry.addData("Transfer Motor", transfer != null ? "Found" : "NOT FOUND");
            if (transfer != null) {
                telemetry.addData("Transfer Power", "%.3f", transfer.getPower());
            }
            
            // Shooter status
            telemetry.addLine("\n--- Shooter Status ---");
            telemetry.addData("Shooter", shooterOn ? "ON (Right Trigger)" : "OFF");
            telemetry.addData("Shooter Motor", shooterMotor != null ? "Found" : "NOT FOUND");
            telemetry.addData("Spinner Motor", (robot != null && robot.spinner != null) ? "Found" : "NOT FOUND");
            telemetry.addData("Hood Servo", hoodServo != null ? "Found" : "NOT FOUND");
            if (hoodServo != null) {
                telemetry.addData("Hood Position", "%.4f", hoodServo.getPosition());
            }
            telemetry.addData("Limelight", limelight != null ? "Connected" : "Missing");
            telemetry.addData("Tag Detected", tagDetected ? "YES" : "NO");
            
            if (tagDetected && cachedTagResult != null && cachedTagResult.isValid) {
                telemetry.addLine("\n--- AprilTag " + TARGET_TAG_ID + " Detected ---");
                telemetry.addData("Distance", "%.1f inches", cachedTagResult.distance);
                telemetry.addData("X Angle", "%.2f°", cachedTagResult.xDegrees);
                
                // Turret status
                if (robot.spinner != null) {
                    try {
                        int pos = robot.spinner.getCurrentPosition();
                        double posDegrees = (pos / 550.0) * 90.0; // Approximate encoder to degrees conversion
                        telemetry.addData("Turret Position", "%d ticks (%.1f°)", pos, posDegrees);
                    } catch (Exception e) {
                        // Ignore
                    }
                }
                
                // Hood status
                telemetry.addLine("\n--- Hood Status ---");
                if (hoodServo != null) {
                    try {
                        telemetry.addData("Hood Position (current)", "%.4f", hoodServo.getPosition());
                        telemetry.addData("Hood Position (calculated)", "%.4f", currentHoodPosition);
                        if (cachedTagResult.distance < CLOSE_DISTANCE_THRESHOLD) {
                            telemetry.addData("Hood Adjustment", "%.4f (close distance)", HOOD_CLOSE_ADJUSTMENT);
                        }
                        // Show calculated hood from formula
                        if (robot != null) {
                            try {
                                double calculatedHood = robot.calculateHoodPosition(cachedTagResult.distance, null);
                                telemetry.addData("Hood (formula)", "%.4f", calculatedHood);
                            } catch (Exception e) {
                                telemetry.addData("Hood Formula Error", e.getMessage());
                            }
                        }
                    } catch (Exception e) {
                        telemetry.addData("Hood", "Error reading: " + e.getMessage());
                    }
                } else {
                    telemetry.addData("Hood Servo", "NOT FOUND");
                }
                
                // Shooter RPM
                if (shooterMotor != null) {
                    try {
                        double currentVelocity = shooterMotor.getVelocity();
                        int currentRPM = (int)Math.round((currentVelocity / TICKS_PER_REVOLUTION) * 60.0);
                        telemetry.addData("Shooter RPM", "%.0f / %.0f", (double)currentRPM, targetRPM);
                    } catch (Exception e) {
                        // Ignore
                    }
                }
            } else {
                telemetry.addLine("\n❌ AprilTag " + TARGET_TAG_ID + " not detected");
            }
            
            // Controls
            telemetry.addLine("\n--- Controls ---");
            telemetry.addLine("Gamepad 1: Left stick Y = Forward, Right stick X = Strafe, Left stick X = Turn");
            telemetry.addLine("Gamepad 1: Right Bumper = Slow mode (0.5x speed)");
            telemetry.addLine("Gamepad 2: Left Trigger = Intake, A = Transfer, Right Trigger = Shooter");
            
        } catch (Exception e) {
            telemetry.addData("Telemetry Error", e.getMessage());
        }
        
        telemetry.update();
    }
}

