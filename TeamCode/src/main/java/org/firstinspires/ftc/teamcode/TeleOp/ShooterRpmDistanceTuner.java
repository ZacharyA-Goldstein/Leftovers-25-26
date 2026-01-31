package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import org.firstinspires.ftc.teamcode.LimeLight.dumbMapLime;

import java.util.List;

/**
 * ShooterRpmDistanceTuner
 *
 * Simple TeleOp to collect data for a distance ↔ RPM formula using the Limelight.
 *
 * What it does:
 * - Uses Limelight + AprilTagDetector to measure distance to the speaker tag each loop
 * - Lets you adjust shooter RPM with the gamepad
 * - Records (distance, RPM) pairs when you press X
 * - At the bottom of telemetry it estimates a linear model:
 *      distance ≈ A * RPM + B
 *   using simple linear regression over the collected points
 *
 * Controls (gamepad 1):
 * - D-pad Left/Right: small RPM change (-/+ 25)
 * - Left/Right Bumper: large RPM change (-/+ 100)
 * - A: toggle shooter ON/OFF
 * - B: reset shooter encoder and stop shooter
 * - X: record a data point (current distance + actual RPM)
 *
 * Usage:
 * 1. Place the robot at a chosen distance from the AprilTag and aim the turret.
 * 2. Adjust RPM until shots are good at that distance.
 * 3. Press X to record (distance, RPM).
 * 4. Repeat at multiple distances.
 * 5. Use the printed A, B values for:
 *        distance ≈ A * RPM + B    or    RPM ≈ (distance - B) / A
 */
@TeleOp(name = "Shooter RPM Distance Tuner", group = "Test")
public class ShooterRpmDistanceTuner extends LinearOpMode {

    // Hardware / LL
    private dumbMapLime robot;
    private DcMotorEx shooterMotor;
    private DcMotor transferMotor;
    private DcMotor intakeMotor;
    private CRServo transferServo; // toilet bottom - transfer servo
    private DcMotor spinner; // turret motor
    private Limelight3A limelight;
    private AprilTagDetector tagDetector;

    // Limelight / geometry constants (match other LL code)
    private static final double CAMERA_HEIGHT = 13.0;   // inches
    private static final double CAMERA_ANGLE = 17.0;     // degrees
    private static final double MAX_DISTANCE = 160.0;   // inches
    // Use your alliance tag; 20 = blue, 24 = red. Change if needed.
    private static final int TARGET_TAG_ID = 24;
    
    // --- TURRET TUNING ---
    private static final double TURRET_MIN_POWER = 0.08;
    private static final double TURRET_MAX_POWER = 0.15;
    private static final double TURRET_DEADBAND = 3.5;
    private static final double TURRET_SLOW_ZONE = 6.0;
    private static final double TURRET_SLOW_ZONE_POWER = 0.12;
    private static final double TURRET_VERY_SLOW_ZONE = 5.0;
    private static final double TURRET_VERY_SLOW_ZONE_POWER = 0.06;
    private static final double TURRET_HORIZONTAL_OFFSET = 0.0; // left = -, right = +
    private static final double TURRET_DIRECTION_FLIP = 1.0;   // flip sign if turret runs backwards

    // Shooter tuning
    private int targetRPM = 0;
    private static final int SMALL_RPM_STEP = 25;
    private static final int LARGE_RPM_STEP = 100;
    private static final int MAX_RPM = 6000;
    private static final int MIN_RPM = -6000;
    private static final int TICKS_PER_REVOLUTION = 28; // GoBILDA 6000 RPM motor

    // State
    private boolean shooterOn = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean transferOn = false;
    private boolean lastY = false;

    // Data recording
    private static final int MAX_POINTS = 40;
    private final double[] distances = new double[MAX_POINTS];
    private final int[] rpms = new int[MAX_POINTS];
    private int pointCount = 0;

    // Limelight throttling
    private int loopCounter = 0;
    private static final int LL_INTERVAL = 3;
    private AprilTagDetector.AprilTagResult cachedTag = null;

    @Override
    public void runOpMode() {
        robot = new dumbMapLime(this);
        robot.initMotors();

        // Shooter motor
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
            }
        } catch (Exception e) {
            telemetry.addData("Shooter Init Error", e.getMessage());
        }

        // Turret motor (spinner) - from robot.spinner
        if (robot.spinner != null) {
            robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.spinner.setDirection(DcMotor.Direction.FORWARD);
            spinner = robot.spinner;
        }

        // Limelight / AprilTag detector
        robot.initLimeLight();
        limelight = robot.getLimeLight();
        if (limelight != null) {
            try {
                limelight.start();
                limelight.pipelineSwitch(0); // AprilTag pipeline
                sleep(500);
                tagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT, CAMERA_ANGLE, MAX_DISTANCE);
            } catch (Exception e) {
                telemetry.addData("Limelight Error", e.getMessage());
            }
        } else {
            telemetry.addData("Limelight", "NOT FOUND");
        }

        // Transfer motor (for feeding while tuning)
        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            if (transferMotor == null) {
                transferMotor = hardwareMap.get(DcMotor.class, "flicker");
            }
            if (transferMotor != null) {
                transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                transferMotor.setPower(0.0);
            }
        } catch (Exception e) {
            telemetry.addData("Transfer Init Error", e.getMessage());
        }

        // Intake motor (run with transfer so you can fully feed the shooter)
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            if (intakeMotor != null) {
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intakeMotor.setPower(0.0);
            }
        } catch (Exception e) {
            telemetry.addData("Intake Init Error", e.getMessage());
        }

        // Transfer servo (toilet bottom)
        try {
            transferServo = hardwareMap.get(CRServo.class, "toiletBottom");
            if (transferServo != null) {
                transferServo.setPower(0.0);
            }
        } catch (Exception e) {
            telemetry.addData("Transfer Servo Init Error", e.getMessage());
        }

        telemetry.addLine("Shooter RPM Distance Tuner");
        telemetry.addLine("D-pad L/R: small RPM step");
        telemetry.addLine("LB/RB: big RPM step");
        telemetry.addLine("A: toggle shooter");
        telemetry.addLine("B: reset encoder");
        telemetry.addLine("Y: toggle transfer+intake (feed)");
        telemetry.addLine("X: record (distance, RPM)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleInput();
            updateShooter();
            updateTransfer();
            updateDistanceCache();
            updateTurretAutoAlign();
            updateTelemetry();
            loopCounter++;
            sleep(20);
        }

        // Stop all motors on exit
        if (shooterMotor != null) {
            shooterMotor.setVelocity(0);
        }
        if (transferMotor != null) {
            transferMotor.setPower(0.0);
        }
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        if (transferServo != null) {
            transferServo.setPower(0.0);
        }
        if (spinner != null) {
            spinner.setPower(0.0);
        }
    }

    private void handleInput() {
        // Small RPM steps
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        if (dpadLeft && !lastDpadLeft) {
            targetRPM = clampRPM(targetRPM - SMALL_RPM_STEP);
        }
        if (dpadRight && !lastDpadRight) {
            targetRPM = clampRPM(targetRPM + SMALL_RPM_STEP);
        }
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;

        // Large RPM steps
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        if (leftBumper && !lastLeftBumper) {
            targetRPM = clampRPM(targetRPM - LARGE_RPM_STEP);
        }
        if (rightBumper && !lastRightBumper) {
            targetRPM = clampRPM(targetRPM + LARGE_RPM_STEP);
        }
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;

        // Toggle shooter
        boolean a = gamepad1.a;
        if (a && !lastA) {
            shooterOn = !shooterOn;
        }
        lastA = a;

        // Reset encoder
        boolean b = gamepad1.b;
        if (b && !lastB && shooterMotor != null) {
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetRPM = 0;
            shooterOn = false;
        }
        lastB = b;

        // Record data point
        boolean x = gamepad1.x;
        if (x && !lastX) {
            recordPoint();
        }
        lastX = x;

        // Toggle transfer (feed) on Y
        boolean y = gamepad1.y;
        if (y && !lastY) {
            transferOn = !transferOn;
        }
        lastY = y;
    }

    private int clampRPM(int rpm) {
        return Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
    }

    private void updateShooter() {
        if (shooterMotor == null) return;
        try {
            if (shooterOn) {
                shooterMotor.setVelocity(rpmToTicksPerSec(targetRPM));
            } else {
                shooterMotor.setVelocity(0);
            }
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }
    }

    private void updateTransfer() {
        double power = transferOn ? 1.0 : 0.0;
        try {
            if (transferMotor != null) {
                transferMotor.setPower(power);
            }
            if (intakeMotor != null) {
                intakeMotor.setPower(power);
            }
            // Transfer servo (toilet bottom) spins when transfer is on
            if (transferServo != null) {
                transferServo.setPower(transferOn ? 1.0 : 0.0);
            }
        } catch (Exception e) {
            telemetry.addData("Feed Error", e.getMessage());
        }
    }

    private int rpmToTicksPerSec(int rpm) {
        return (int) Math.round((rpm / 60.0) * TICKS_PER_REVOLUTION);
    }

    private int ticksPerSecToRpm(double tps) {
        return (int) Math.round((tps / TICKS_PER_REVOLUTION) * 60.0);
    }

    private void updateDistanceCache() {
        if (tagDetector == null) return;
        if (loopCounter % LL_INTERVAL != 0) return;

        try {
            AprilTagDetector.AprilTagResult tag = tagDetector.getTagById(TARGET_TAG_ID);
            if (tag == null || !tag.isValid) {
                tag = tagDetector.getClosestTag();
            }
            if (tag != null && tag.isValid && tag.distance > 0) {
                // Fix distance using corrected formula
                if (tag.yDegrees != 0.0) {
                    double correctedDistance = calculateDistanceFromTy(tag.yDegrees);
                    tag.distance = correctedDistance;
                }
                cachedTag = tag;
            }
        } catch (Exception e) {
            // keep previous cachedTag
        }
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
            // Angle too small - return a very large distance (no max limit)
            return 10000.0; // Very far, but still a valid number
        }
        
        double distance = heightDifference / Math.tan(totalAngle);
        
        // Only apply minimum bound to avoid invalid close distances
        // No maximum limit - calculate as far as tag can be detected
        distance = Math.max(12.0, distance);
        
        return distance;
    }
    
    /**
     * Auto-align turret to target tag using Limelight tx
     */
    private void updateTurretAutoAlign() {
        if (spinner == null || limelight == null) {
            if (spinner != null) spinner.setPower(0.0);
            return;
        }
        
        try {
            LLResult result = limelight.getLatestResult();
            if (result == null) {
                spinner.setPower(0.0);
                return;
            }
            
            // Get tx from the target tag's fiducial result
            double currentTx = 0.0;
            boolean hasTag = false;
            
            List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();
            if (fiducials != null) {
                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid : fiducials) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        hasTag = true;
                        currentTx = fid.getTargetXDegrees();
                        break;
                    }
                }
            }
            
            // If we found the target tag, align to it
            if (hasTag && currentTx != 0.0) {
                updateTurret(currentTx);
            } else {
                // No tag detected - stop turret
                spinner.setPower(0.0);
            }
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
            if (spinner != null) spinner.setPower(0.0);
        }
    }
    
    /**
     * Update turret power based on tx angle
     */
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

    private double getCurrentDistance() {
        if (cachedTag != null && cachedTag.isValid && cachedTag.distance > 0) {
            return cachedTag.distance;
        }
        return 0.0;
    }

    private int getActualRpm() {
        if (shooterMotor == null) return 0;
        try {
            double vel = shooterMotor.getVelocity();
            return ticksPerSecToRpm(vel);
        } catch (Exception e) {
            return targetRPM;
        }
    }

    private void recordPoint() {
        if (pointCount >= MAX_POINTS) {
            telemetry.addLine("⚠ Max data points reached");
            return;
        }
        double dist = getCurrentDistance();
        int rpm = getActualRpm();

        distances[pointCount] = dist;
        rpms[pointCount] = rpm;
        pointCount++;
    }

    private void updateTelemetry() {
        telemetry.addLine("=== Shooter RPM Distance Tuner ===");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
        telemetry.addData("Actual RPM", getActualRpm());

        if (cachedTag != null && cachedTag.isValid) {
            telemetry.addData("Distance (in)", "%.1f (Tag %d)", cachedTag.distance, cachedTag.tagId);
        } else {
            telemetry.addData("Distance", "No AprilTag detected");
        }

        telemetry.addLine("\nData Points (distance in, RPM):");
        for (int i = 0; i < pointCount; i++) {
            telemetry.addData("P" + (i + 1), "%.1f\", %d RPM", distances[i], rpms[i]);
        }

        if (pointCount >= 2) {
            // Simple linear regression: distance ≈ A * RPM + B
            double sumR = 0, sumD = 0, sumRSq = 0, sumRD = 0;
            for (int i = 0; i < pointCount; i++) {
                double r = rpms[i];
                double d = distances[i];
                sumR += r;
                sumD += d;
                sumRSq += r * r;
                sumRD += r * d;
            }
            int n = pointCount;
            double denom = (n * sumRSq - sumR * sumR);
            if (Math.abs(denom) > 1e-6) {
                double A = (n * sumRD - sumR * sumD) / denom;
                double B = (sumD - A * sumR) / n;
                telemetry.addLine("\nEstimated formula:");
                telemetry.addData("distance ≈ A * RPM + B", "A=%.6f, B=%.3f", A, B);
                telemetry.addLine("So RPM ≈ (distance - B) / A");
            } else {
                telemetry.addLine("\nNot enough RPM variation for regression");
            }
        } else {
            telemetry.addLine("\nRecord at least 2 points with X to fit a line");
        }

        // Controls information
        telemetry.addLine("\n=== CONTROLS ===");
        telemetry.addLine("D-pad L/R: RPM ±" + SMALL_RPM_STEP);
        telemetry.addLine("LB/RB: RPM ±" + LARGE_RPM_STEP);
        telemetry.addLine("A: Toggle shooter");
        telemetry.addLine("B: Reset encoder");
        telemetry.addLine("Y: Toggle transfer+intake");
        telemetry.addLine("X: Record data point");
        telemetry.addLine("Turret: Auto-aligns to tag");

        telemetry.update();
    }
}

