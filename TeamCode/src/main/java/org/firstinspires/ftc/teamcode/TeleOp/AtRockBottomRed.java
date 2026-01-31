package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import org.firstinspires.ftc.teamcode.LimeLight.dumbMapLime;

import java.util.List;

/**
 * AtRockBottomRed
 *
 * Single‑controller TeleOp that controls the whole robot and uses Limelight
 * to auto‑aim at the RED goal (red AprilTag) while you tune distance vs RPM.
 *
 * Gamepad 1 controls:
 * - Left stick: drive (forward/back & strafe)
 * - Right stick X: rotate
 * - A: toggle transfer (feeds AND runs intake when on)
 * - Left trigger: toggle intake forward
 * - Left bumper: intake reverse (hold)
 * - Right trigger: toggle shooter ON/OFF
 * - B: toggle limelight auto-aim ON/OFF (manual adjustments always work)
 * - D‑pad Left/Right: manual turret adjust (spin left/right) - always available
 * - D‑pad Up/Down: manual RPM adjust (±RPM_STEP) - always available
 *
 * Limelight:
 * - Configured for RED tag ID (TARGET_TAG_ID_RED)
 * - When a valid tag is seen and manual mode is OFF:
 *     - Turret auto‑aligns using tx
 *     - Shooter runs at AUTO_SHOOTER_RPM
 *
 * All major tuning constants are collected at the top for easy changes.
 */
@TeleOp(name = "At Rock Bottom Red", group = "TeleOp")
public class AtRockBottomRed extends LinearOpMode {

    // --- HARDWARE NAMES (CHANGE HERE IF WIRING CHANGES) ---
    private static final String MOTOR_FRONT_LEFT  = "frontLeft";
    private static final String MOTOR_FRONT_RIGHT = "frontRight";
    private static final String MOTOR_BACK_LEFT   = "backLeft";
    private static final String MOTOR_BACK_RIGHT  = "backRight";
    private static final String MOTOR_INTAKE      = "intake";
    private static final String MOTOR_TRANSFER    = "transfer";  // or "flicker"
    private static final String MOTOR_SHOOTER     = "shooter";   // or "outtake"
    private static final String MOTOR_TURRET      = "spinner";   // from dumbMapLime
    private static final String SERVO_TRANSFER_CR = "toiletBottom";

    // --- DRIVE TUNING ---
    private static final double DRIVE_DEADBAND = 0.3;   // Stick deadband
    private static final double DRIVE_SLOW_SCALE = 0.5; // Not used yet, reserved for future

    // --- INTAKE / TRANSFER TUNING ---
    private static final double INTAKE_POWER_FWD = 1.0;
    private static final double INTAKE_POWER_REV = -1.0;
    private static final double TRANSFER_MOTOR_POWER = 1.0;
    private static final double TRANSFER_SERVO_POWER = 1.0; // matches CrunchBotBlue: negative to spin correct way

    // --- SHOOTER / RPM TUNING ---
    private static final int TICKS_PER_REV = 28;       // GoBILDA 6000RPM encoder
    private static final int AUTO_SHOOTER_RPM = -4650; // Default auto‑aim RPM
    private static final int MANUAL_RPM_STEP = 100;    // D‑pad Up/Down increments in manual mode
    private static final int SHOOTER_RPM_MIN = -6000;  // Minimum RPM (negative = reverse direction)
    private static final int SHOOTER_RPM_MAX = 0;       // Maximum RPM (0 = stopped, negative only)

    // --- LIMELIGHT TUNING (RED GOAL) ---
    private static final double CAMERA_HEIGHT = 13.0;    // inches
    private static final double CAMERA_ANGLE  = 17.0;    // degrees
    private static final double MAX_DISTANCE  = 160.0;   // inches
    private static final int TARGET_TAG_ID_RED = 24;     // RED alliance tag
    private static final int LIMELIGHT_CALL_INTERVAL = 1; // Always call limelight every loop for better far detection
    private static final int MAX_LOST_DETECTIONS = 20;   // Increased for better far detection

    // --- TURRET TUNING ---
    private static final double TURRET_MIN_POWER = 0.08;
    private static final double TURRET_MAX_POWER = 0.15;
    private static final double TURRET_DEADBAND = 3.5;
    private static final double TURRET_SLOW_ZONE = 6.0;
    private static final double TURRET_SLOW_ZONE_POWER = 0.12;
    private static final double TURRET_VERY_SLOW_ZONE = 5.0;
    private static final double TURRET_VERY_SLOW_ZONE_POWER = 0.06;
    private static final double TURRET_HORIZONTAL_OFFSET = 0.0; // left = -, right = +
    private static final double TURRET_DIRECTION_FLIP = 1.0;   // flip sign if turret runs backwards (changed from -1.0 to fix turning away from tag)
    private static final double MANUAL_TURRET_POWER = 0.3;      // D‑pad manual spin power

    // --- STATE: MOTORS / SERVOS ---
    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx shooter;
    private DcMotor spinner; // turret (from robot.spinner, which is DcMotor)
    private CRServo transferServo;

    // --- STATE: LIMELIGHT / AUTO‑AIM ---
    private dumbMapLime robot;
    private Limelight3A limelight;
    private AprilTagDetector aprilTagDetector;
    private AprilTagDetector.AprilTagResult cachedTagResult = null;
    private boolean tagDetected = false;
    private double lastValidTx = 0.0;
    private int loopCounter = 0;
    private int lostDetectionCount = 0;
    
    // --- STATE: BATTERY / POWER MANAGEMENT ---
    private VoltageSensor voltageSensor;
    private double batteryVoltage = 12.6;
    private double targetShooterRPM = 0; // For ramp-up
    private static final double BATTERY_LOW_THRESHOLD = 11.5; // V - warn below this
    private static final double BATTERY_CRITICAL_THRESHOLD = 11.0; // V - reduce power below this
    private static final double FLYWHEEL_RAMP_RATE = 0.95; // Multiplier per loop for smooth ramp-up (0.95 = 5% reduction per loop)

    // --- STATE: CONTROL FLAGS ---
    private boolean shooterOn = false;
    private boolean transferOn = false;
    private boolean intakeOn = false; // Toggle state for intake forward
    private boolean intakeTurnedOnByTransfer = false; // Track if intake was turned on by transfer
    private boolean limelightEnabled = true; // B button toggles this - when false, limelight auto-aim is disabled
    private int manualTargetRPM = AUTO_SHOOTER_RPM;

    // Button edge tracking
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;

    @Override
    public void runOpMode() {
        initHardware();
        initLimelight();

        telemetry.addData("Status", "At Rock Bottom Red ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            try {
                // Update battery voltage
                updateBatteryVoltage();
                
                handleDrive();
                handleIntakeAndTransfer();
                handleShooterAndModes();
                handleTurretAndLimelight();
                updateTelemetry();
                sleep(20);
            } catch (Exception e) {
                telemetry.addData("ERROR", e.getMessage());
                telemetry.update();
            }
        }

        stopAllMotors();
    }

    // --- INITIALIZATION ---

    private void initHardware() {
        // Drive
        leftFront  = hardwareMap.get(DcMotor.class, MOTOR_FRONT_LEFT);
        rightFront = hardwareMap.get(DcMotor.class, MOTOR_FRONT_RIGHT);
        leftRear   = hardwareMap.get(DcMotor.class, MOTOR_BACK_LEFT);
        rightRear  = hardwareMap.get(DcMotor.class, MOTOR_BACK_RIGHT);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake + transfer
        intake = hardwareMap.get(DcMotor.class, MOTOR_INTAKE);
        if (intake != null) {
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        transfer = hardwareMap.get(DcMotor.class, MOTOR_TRANSFER);
        if (transfer != null) {
            transfer.setDirection(DcMotor.Direction.FORWARD);
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        transferServo = hardwareMap.get(CRServo.class, SERVO_TRANSFER_CR);

        // Shooter + turret via dumbMapLime
        robot = new dumbMapLime(this);
        robot.initMotors();

        if (robot.spinner != null) {
            robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.spinner.setDirection(DcMotor.Direction.FORWARD);
            spinner = robot.spinner;
        }

        shooter = hardwareMap.get(DcMotorEx.class, MOTOR_SHOOTER);
        if (shooter == null) {
            shooter = hardwareMap.get(DcMotorEx.class, "outtake");
        }
        if (shooter != null) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter.setDirection(DcMotorEx.Direction.REVERSE);
        }
        
        // Initialize voltage sensor for battery monitoring
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            if (voltageSensor != null) {
                batteryVoltage = voltageSensor.getVoltage();
            }
        } catch (Exception e) {
            telemetry.addData("Voltage Sensor", "Not available");
        }
    }

    private void initLimelight() {
        robot.initLimeLight();
        limelight = robot.getLimeLight();
        if (limelight != null) {
            limelight.pipelineSwitch(0); // Pipeline 0 for RED alliance (matches CrunchBotRed)
            sleep(500);
            aprilTagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT, CAMERA_ANGLE, MAX_DISTANCE);
        }
    }

    // --- MAIN CONTROL HANDLERS ---

    private void handleDrive() {
        double forward = gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

        if (Math.abs(forward) < DRIVE_DEADBAND) forward = 0;
        if (Math.abs(strafe)  < DRIVE_DEADBAND) strafe  = 0;
        if (Math.abs(rotate)  < DRIVE_DEADBAND) rotate  = 0;

        // Mecanum drive math - matches CrunchBotBlue exactly
        double fl = forward - strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward + strafe + rotate;
        double br = forward + strafe - rotate;

        if (leftFront  != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftRear   != null) leftRear.setPower(bl);
        if (rightRear  != null) rightRear.setPower(br);
    }

    private void handleIntakeAndTransfer() {
        // Intake: left trigger toggles forward, left bumper reverse (hold)
        boolean lt = gamepad1.left_trigger > 0.5;
        boolean lb = gamepad1.left_bumper;
        
        // Toggle intake on left trigger press
        if (lt && !lastLeftTrigger) {
            intakeOn = !intakeOn;
            // If user manually toggles intake, clear the flag that tracks if transfer turned it on
            intakeTurnedOnByTransfer = false;
        }
        lastLeftTrigger = lt;

        if (intake != null) {
            if (lb) {
                // Left bumper: reverse intake (hold, overrides toggle)
                intake.setPower(INTAKE_POWER_REV);
            } else {
                // Forward intake when on (either manually toggled or turned on by transfer)
                intake.setPower(intakeOn ? INTAKE_POWER_FWD : 0.0);
            }
        }

        // Transfer: A toggles transferOn (matches CrunchBotBlue logic)
        boolean a = gamepad1.a;
        if (a && !lastA) {
            transferOn = !transferOn;
            if (transferOn) {
                // When transfer turns on, also turn on intake
                intakeOn = true;
                intakeTurnedOnByTransfer = true;
            } else {
                // When transfer turns off, turn off intake if it was turned on by transfer
                if (intakeTurnedOnByTransfer) {
                    intakeOn = false;
                    intakeTurnedOnByTransfer = false;
                }
            }
        }
        lastA = a;

        // Transfer control (matches CrunchBotBlue)
        // Transfer servo spins when intake is on
        if (transferServo != null) {
            transferServo.setPower(intakeOn ? TRANSFER_SERVO_POWER : 0.0);
        }
        // Transfer motor only spins when transfer is explicitly on (independent of shooter)
        if (transfer != null) {
            transfer.setPower(transferOn ? TRANSFER_MOTOR_POWER : 0.0);
        }
    }

    private void handleShooterAndModes() {
        // B: toggle limelight enabled/disabled
        boolean b = gamepad1.b;
        if (b && !lastB) {
            limelightEnabled = !limelightEnabled;
        }
        lastB = b;

        // Right trigger: toggle shooter on/off
        boolean rt = gamepad1.right_trigger > 0.5;
        if (rt && !lastRightTrigger) {
            shooterOn = !shooterOn;
        }
        lastRightTrigger = rt;

        // Manual RPM adjust (Up/Down) - always available, not just in manual mode
        if (gamepad1.dpad_up) {
            manualTargetRPM = clampShooterRPM(manualTargetRPM + MANUAL_RPM_STEP);
        } else if (gamepad1.dpad_down) {
            manualTargetRPM = clampShooterRPM(manualTargetRPM - MANUAL_RPM_STEP);
        }

        // Apply shooter RPM with power optimization
        if (shooter == null) return;
        try {
            if (!shooterOn) {
                shooter.setVelocity(0);
                targetShooterRPM = 0;
                return;
            }

            // Use manual RPM if it's been adjusted, otherwise use auto RPM
            int desiredRPM = (manualTargetRPM != AUTO_SHOOTER_RPM) ? manualTargetRPM : AUTO_SHOOTER_RPM;
            
            // Clamp RPM to valid range (0 to -6000)
            desiredRPM = clampShooterRPM(desiredRPM);
            
            // Smooth ramp-up to reduce inrush current (helps battery)
            if (Math.abs(targetShooterRPM) < Math.abs(desiredRPM)) {
                // Ramping up: gradually increase
                targetShooterRPM = (int)(targetShooterRPM + (desiredRPM - targetShooterRPM) * (1.0 - FLYWHEEL_RAMP_RATE));
            } else {
                // Ramping down or at target: set immediately
                targetShooterRPM = desiredRPM;
            }
            
            // Power limiting based on battery voltage
            double powerMultiplier = 1.0;
            if (batteryVoltage < BATTERY_CRITICAL_THRESHOLD) {
                // Critical: reduce power by 20%
                powerMultiplier = 0.8;
            } else if (batteryVoltage < BATTERY_LOW_THRESHOLD) {
                // Low: reduce power by 10%
                powerMultiplier = 0.9;
            }
            
            int finalRPM = (int)(targetShooterRPM * powerMultiplier);
            finalRPM = clampShooterRPM(finalRPM);

            int velTicksPerSec = (int) Math.round((finalRPM / 60.0) * TICKS_PER_REV);
            shooter.setVelocity(velTicksPerSec);
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }
    }

    private void handleTurretAndLimelight() {
        if (spinner == null) return;

        // Manual turret: D‑pad left/right - always available
        double manualPower = 0.0;
        if (gamepad1.dpad_left) {
            manualPower = -MANUAL_TURRET_POWER;
        } else if (gamepad1.dpad_right) {
            manualPower = MANUAL_TURRET_POWER;
        }

        // If manual input is active, use it and skip auto-aim
        if (manualPower != 0.0) {
            spinner.setPower(manualPower);
            return;
        }

        // Auto‑aim with Limelight only if enabled and no manual input
        if (!limelightEnabled || limelight == null || aprilTagDetector == null) {
            spinner.setPower(0.0);
            return;
        }

        try {
            loopCounter++;
            // Always call limelight for better far detection
            boolean shouldCallLimelight = (loopCounter % LIMELIGHT_CALL_INTERVAL == 0);

            boolean freshTagDetected = false;
            double currentTx = 0.0;
            boolean hasTag = false; // Declare outside the if block so it's accessible later

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                double tx = result.getTx();

                double targetTagTx = tx;
                List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();
                if (fiducials != null) {
                    for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid : fiducials) {
                        if (fid.getFiducialId() == TARGET_TAG_ID_RED) {
                            hasTag = true;
                            targetTagTx = fid.getTargetXDegrees();
                            break;
                        }
                    }
                }

                currentTx = targetTagTx;

                // Always try to get distance when tag is detected, not just on interval
                if (hasTag) {
                    try {
                        cachedTagResult = aprilTagDetector.getTagById(TARGET_TAG_ID_RED);
                        if (cachedTagResult != null && cachedTagResult.isValid) {
                            freshTagDetected = true;
                            lostDetectionCount = 0;
                            tagDetected = true;
                            lastValidTx = targetTagTx;
                            // Fix distance if it seems reversed (use our corrected calculation)
                            if (cachedTagResult.yDegrees != 0.0) {
                                double correctedDistance = calculateDistanceFromTy(cachedTagResult.yDegrees);
                                cachedTagResult.distance = correctedDistance;
                            }
                        } else {
                            // Fallback: If Limelight sees the tag but AprilTagDetector fails,
                            // still use it for alignment using direct tx from Limelight
                            double ty = 0.0;
                            if (fiducials != null) {
                                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid : fiducials) {
                                    if (fid.getFiducialId() == TARGET_TAG_ID_RED) {
                                        ty = fid.getTargetYDegrees();
                                        break;
                                    }
                                }
                            }
                            freshTagDetected = true;
                            lostDetectionCount = 0;
                            tagDetected = true;
                            lastValidTx = targetTagTx;
                            // Create a minimal valid result for tracking using direct Limelight data
                            cachedTagResult = new AprilTagDetector.AprilTagResult();
                            cachedTagResult.tagId = TARGET_TAG_ID_RED;
                            cachedTagResult.xDegrees = targetTagTx;
                            cachedTagResult.yDegrees = ty;
                            cachedTagResult.angle = targetTagTx;
                            // Calculate distance using ty directly with corrected formula
                            cachedTagResult.distance = calculateDistanceFromTy(ty);
                            cachedTagResult.family = "36h11";
                            cachedTagResult.section = "RED";
                            cachedTagResult.isValid = true;
                        }
                    } catch (Exception e) {
                        // If AprilTagDetector throws an error but Limelight sees the tag, still use it
                        double ty = 0.0;
                        if (fiducials != null) {
                            for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid : fiducials) {
                                if (fid.getFiducialId() == TARGET_TAG_ID_RED) {
                                    ty = fid.getTargetYDegrees();
                                    break;
                                }
                            }
                        }
                        freshTagDetected = true;
                        lostDetectionCount = 0;
                        tagDetected = true;
                        lastValidTx = targetTagTx;
                        cachedTagResult = new AprilTagDetector.AprilTagResult();
                        cachedTagResult.tagId = TARGET_TAG_ID_RED;
                        cachedTagResult.xDegrees = targetTagTx;
                        cachedTagResult.yDegrees = ty;
                        cachedTagResult.angle = targetTagTx;
                        // Calculate distance using ty directly with corrected formula
                        cachedTagResult.distance = calculateDistanceFromTy(ty);
                        cachedTagResult.family = "36h11";
                        cachedTagResult.section = "RED";
                        cachedTagResult.isValid = true;
                    }
                }
            }

            if (!freshTagDetected) {
                lostDetectionCount++;
                if (lostDetectionCount > MAX_LOST_DETECTIONS) {
                    tagDetected = false;
                    cachedTagResult = null;
                }
            }

            // Update turret continuously when tag is detected (use currentTx which is always fresh)
            if (tagDetected && cachedTagResult != null && cachedTagResult.isValid) {
                // Always use currentTx from Limelight for continuous alignment
                updateTurret(currentTx);
                lastValidTx = currentTx;
            } else if (hasTag && currentTx != 0.0) {
                // If Limelight sees tag but detection state isn't set, still align using current tx
                updateTurret(currentTx);
                lastValidTx = currentTx;
            } else {
                // No tag detected - stop turret
                spinner.setPower(0.0);
            }
        } catch (Exception e) {
            telemetry.addData("Limelight Error", e.getMessage());
            spinner.setPower(0.0);
        }
    }

    private void updateTurret(double tx) {
        if (spinner == null) return;
        
        double adjustedTx = tx + TURRET_HORIZONTAL_OFFSET;
        double absTx = Math.abs(adjustedTx);

        if (absTx <= TURRET_DEADBAND) {
            spinner.setPower(0.0);
            return;
        }

        double sign = Math.signum(adjustedTx);
        double cmd;

        if (absTx <= TURRET_VERY_SLOW_ZONE) {
            cmd = TURRET_VERY_SLOW_ZONE_POWER * sign;
        } else if (absTx <= TURRET_SLOW_ZONE) {
            cmd = TURRET_SLOW_ZONE_POWER * sign;
        } else {
            cmd = adjustedTx * 0.02;
            if (Math.abs(cmd) > TURRET_MAX_POWER) cmd = TURRET_MAX_POWER * sign;
            if (Math.abs(cmd) < TURRET_MIN_POWER) cmd = TURRET_MIN_POWER * sign;
        }

        cmd *= TURRET_DIRECTION_FLIP;
        spinner.setPower(cmd);
    }

    private int clampShooterRPM(int rpm) {
        // Clamp between -6000 (max reverse) and 0 (stopped) - shooter only runs in reverse
        return Math.max(SHOOTER_RPM_MIN, Math.min(SHOOTER_RPM_MAX, rpm));
    }
    
    /**
     * Calculate distance from tag using ty (vertical angle) with corrected formula.
     * This fixes the reversed distance issue where close = high numbers, far = low numbers.
     * 
     * @param ty Vertical angle to tag in degrees (positive = above center, negative = below)
     * @return Distance in inches
     */
    private double calculateDistanceFromTy(double ty) {
        // Tag height and camera height
        double tagHeight = 30.0; // inches - height of AprilTag center from ground
        double heightDifference = tagHeight - CAMERA_HEIGHT; // inches
        
        // Convert to radians
        double tyRadians = Math.toRadians(ty);
        double cameraAngleRadians = Math.toRadians(CAMERA_ANGLE);
        
        // Corrected formula: distance = heightDifference / tan(cameraAngle + ty)
        // When close: ty is larger (more vertical), tan is larger, distance is smaller ✓
        // When far: ty is smaller (closer to horizontal), tan is smaller, distance is larger ✓
        // But if ty is negative (tag below center), we need to handle that
        double totalAngle = cameraAngleRadians + tyRadians;
        
        // Ensure we don't divide by zero or get invalid results
        if (Math.abs(totalAngle) < 0.01) {
            return MAX_DISTANCE; // Too far to calculate accurately
        }
        
        double distance = heightDifference / Math.tan(totalAngle);
        
        // Apply bounds
        distance = Math.max(12.0, Math.min(distance, MAX_DISTANCE));
        
        return distance;
    }

    private void stopAllMotors() {
        if (leftFront != null) leftFront.setPower(0.0);
        if (rightFront != null) rightFront.setPower(0.0);
        if (leftRear != null) leftRear.setPower(0.0);
        if (rightRear != null) rightRear.setPower(0.0);
        if (intake != null) intake.setPower(0.0);
        if (transfer != null) transfer.setPower(0.0);
        if (transferServo != null) transferServo.setPower(0.0);
        if (spinner != null) spinner.setPower(0.0);
        if (shooter != null) shooter.setVelocity(0);
    }

    private void updateBatteryVoltage() {
        if (voltageSensor != null) {
            try {
                batteryVoltage = voltageSensor.getVoltage();
            } catch (Exception e) {
                // Keep previous reading
            }
        }
    }
    
    private void updateTelemetry() {
        telemetry.addLine("=== At Rock Bottom Red ===");
        
        // Battery voltage with warnings
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        if (batteryVoltage < BATTERY_CRITICAL_THRESHOLD) {
            telemetry.addLine("⚠️ CRITICAL: Battery very low! Power reduced.");
        } else if (batteryVoltage < BATTERY_LOW_THRESHOLD) {
            telemetry.addLine("⚠️ WARNING: Battery low! Power reduced.");
        }
        
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Limelight", limelightEnabled ? "ENABLED" : "DISABLED");
        
        // Display current RPM (always show, even when off)
        if (shooter != null) {
            try {
                double currentVelocity = shooter.getVelocity();
                double currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;
                // Handle negative RPMs correctly (shooter runs in reverse)
                telemetry.addData("Current RPM", "%.0f", currentRPM);
            } catch (Exception e) {
                telemetry.addData("Current RPM", "ERROR: " + e.getMessage());
            }
        } else {
            telemetry.addData("Current RPM", "N/A");
        }
        
        telemetry.addData("Target RPM", manualTargetRPM != AUTO_SHOOTER_RPM ? manualTargetRPM : AUTO_SHOOTER_RPM);

        if (cachedTagResult != null && cachedTagResult.isValid) {
            telemetry.addData("Tag", "DETECTED (ID %d)", cachedTagResult.tagId);
            telemetry.addData("Distance", "%.1f in", cachedTagResult.distance);
            telemetry.addData("tx", "%.2f deg", cachedTagResult.xDegrees);
        } else {
            telemetry.addData("Tag", "NOT DETECTED");
        }

        telemetry.update();
    }
}

