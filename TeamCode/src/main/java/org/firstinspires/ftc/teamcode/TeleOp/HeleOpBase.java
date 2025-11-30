package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.LimeLight.dumbMapLime;

public abstract class HeleOpBase extends LinearOpMode {
    private static final double DRIVE_DEADBAND = 0.05;
    private static final double HOOD_UP = 0.206;
    private static final double HOOD_DOWN = 0.295;
    private static final double HOOD_MIN = 0.206;
    private static final double HOOD_MAX = 0.295;
    private static final double TRANSFER_UP = 0.66;
    private static final double TRANSFER_DOWN = 1;
    private static final double INTAKE_POWER = 1.0;
    private static final double ALIGN_KP = 0.02;
    private static final double MIN_ALIGN_POWER = 0.12;
    private static final double MAX_ALIGN_POWER = 0.5;
    private static final double DEADBAND = 0.5;
    private static final double TOLERANCE = 3.0;
    private static final double SEARCH_POWER = 0.25;
    private static final double CAMERA_HEIGHT = 9.5;
    private static final double CAMERA_ANGLE = 0.0;
    private static final double MAX_TAG_DISTANCE = 150.0;
    private static final double SHOOTER_POWER = -1.0;

    // Ball tracking constants
    private static final double BALL_KP = 0.04;
    private static final double BALL_DEADBAND = 0.5;
    private static final double BALL_TOLERANCE = 1.0;
    private static final double MIN_BALL_ROTATION = 0.4;
    private static final double MAX_BALL_ROTATION = 0.8;
    private static final double AUTO_ASSIST_AREA = 0.5;
    private static final double SPEED_SCALING = 0.7;
    private static final int BALL_PIPELINE_GREEN = 1;
    private static final int BALL_PIPELINE_PURPLE = 2;

    private dumbMap robot;
    private dumbMapLime robotLime;
    private Limelight3A limelight;
    private AprilTagDetector tagDetector;

    private Servo hoodServo;

    private boolean intakeActive = false;
    private boolean lastIntakeButton = false;
    private boolean shootingMode = false;
    private boolean shootingTogglePressed = false;
    private boolean spinnerLocked = false;
    private boolean hapticSent = false;
    private boolean spinnerZeroRequested = false;
    private int searchDirection = 1;
    private double autoRotation = 0.0;
    private boolean manualMode = false;
    private boolean lastBButton = false;
    private double manualHoodTarget = HOOD_UP;
    private boolean lastAButton = false;
    private boolean lastManualDpadUp = false;
    private boolean lastManualDpadDown = false;
    private Servo flickerServo;
    private boolean isFlicking = false;
    private long flickStartTime = 0;
    private static final double FLICK_UP = 0.66;
    private static final double FLICK_DOWN = 1.0;
    private static final long FLICK_DURATION = 175;
    private boolean manualShooterOn = false;
    private int currentPipeline = -1;
    private double ballTrackingRotation = 0.0;

    @Override
    public void runOpMode() {
        robot = new dumbMap(this);
        robot.init2();
        robot.initLimeLight();
        
        // Initialize dumbMapLime for shooter formula calculations
        robotLime = new dumbMapLime(this);

        limelight = robot.getLimeLight();
        if (limelight != null) {
            tagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT, CAMERA_ANGLE, MAX_TAG_DISTANCE);
            // Set initial pipeline to AprilTag pipeline (will be set to correct alliance when shooting starts)
            // Note: dumbMapLime.initLimeLight() already sets pipeline 0, so we need to switch to the correct one
            int initialPipeline = getAprilTagPipelineIndex();
            try {
                limelight.pipelineSwitch(initialPipeline);
                currentPipeline = initialPipeline;
                // Give Limelight time to process the pipeline switch
                sleep(200);
                telemetry.addData("Limelight", "Initialized - Pipeline " + initialPipeline);
            } catch (Exception e) {
                telemetry.addData("Limelight", "Error setting pipeline: " + e.getMessage());
            }
        } else {
            telemetry.addData("Limelight", "NOT FOUND");
        }

        if (robot.spinner != null) {
            robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.spinner.setPower(0.0);
        }

        hoodServo = robot.hood;
        flickerServo = robot.flicker;
        if (flickerServo != null) {
            flickerServo.setPosition(FLICK_DOWN);
        }

        telemetry.addLine("HeleOp ready. Controller 2: LT=Intake, RT=Shoot");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveRobot();
            handleIntakeToggle();
            handleManualModeToggle();
            handleManualControls();
            updateFlicker(gamepad2.a);
            if (!manualMode) {
                handleShootingToggle();
            } else {
                if (robot.shooter != null && !manualShooterOn) {
                    robot.shooter.setPower(0.0);
                }
            }
            zeroSpinnerIfRequested();
            if (!manualMode && shootingMode) {
                alignRobotToTag();
            } else if (!manualMode && intakeActive && !shootingMode) {
                trackBall();
            } else {
                ballTrackingRotation = 0.0;
                if (!intakeActive && !shootingMode) {
                    ensureBallPipelineOff();
                }
            }
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            updateHood();
            updateTelemetry();
            if (!manualMode && robot.shooter != null && !shootingMode) {
                robot.shooter.setPower(0.0);
            }
            sleep(20);
        }
    }

    private void driveRobot() {
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;
        double driveScale = gamepad1.right_bumper ? 0.5 : 1.0;
        forward *= driveScale;
        strafe *= driveScale;
        rotate *= driveScale;

        if (Math.abs(forward) < DRIVE_DEADBAND) forward = 0;
        if (Math.abs(strafe) < DRIVE_DEADBAND) strafe = 0;
        if (Math.abs(rotate) < DRIVE_DEADBAND) rotate = 0;

        double applyRotation;
        if (shootingMode) {
            applyRotation = autoRotation;
        } else if (intakeActive && !manualMode && !shootingMode) {
            // Use ball tracking rotation if no manual rotation input
            boolean isManualRotating = Math.abs(rotate) > 0.1;
            applyRotation = isManualRotating ? rotate : ballTrackingRotation;
        } else {
            applyRotation = rotate;
        }

        if (robot.leftFront != null) robot.leftFront.setPower(forward + strafe + applyRotation);
        if (robot.rightFront != null) robot.rightFront.setPower(forward - strafe - applyRotation);
        if (robot.leftBack != null) robot.leftBack.setPower(forward - strafe + applyRotation);
        if (robot.rightBack != null) robot.rightBack.setPower(forward + strafe - applyRotation);
    }

    private void handleIntakeToggle() {
        boolean wasIntakeActive = intakeActive;
        intakeActive = gamepad2.left_trigger > 0.1;
        if (intakeActive) {
            spinnerZeroRequested = true;
            if (!wasIntakeActive && limelight != null) {
                // Switch to ball detection pipeline when intake starts
                ensureBallPipeline();
            }
        }
        if (robot.intake != null) {
            robot.intake.setPower(intakeActive ? INTAKE_POWER : 0.0);
        }
    }

    private void handleShootingToggle() {
        boolean triggerNow = gamepad2.right_trigger > 0.1;
        if (triggerNow && !shootingTogglePressed) {
            shootingMode = !shootingMode;
            spinnerZeroRequested = !shootingMode;
            spinnerLocked = false;
            hapticSent = false;
            if (!shootingMode && robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            if (shootingMode) {
                ensureAprilTagPipeline();
            }
        }
        shootingTogglePressed = triggerNow;

        if (robot.shooter != null) {
            robot.shooter.setPower(shootingMode ? SHOOTER_POWER : 0.0);
        }
    }

    private void handleManualModeToggle() {
        boolean bNow = gamepad2.b;
        if (bNow && !lastBButton) {
            manualMode = !manualMode;
            if (manualMode) {
                shootingMode = false;
                spinnerZeroRequested = true;
                autoRotation = 0.0;
            }
        }
        lastBButton = bNow;
    }

    private void handleManualControls() {
        if (!manualMode) {
            manualHoodTarget = HOOD_UP;
            manualShooterOn = false;
            lastManualDpadUp = false;
            lastManualDpadDown = false;
            return;
        }

        boolean dpadUp = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;
        if (dpadUp && !lastManualDpadUp) {
            manualHoodTarget = clamp(manualHoodTarget + 0.01, HOOD_MIN, HOOD_MAX);
        }
        if (dpadDown && !lastManualDpadDown) {
            manualHoodTarget = clamp(manualHoodTarget - 0.01, HOOD_MIN, HOOD_MAX);
        }
        lastManualDpadUp = dpadUp;
        lastManualDpadDown = dpadDown;

        manualShooterOn = gamepad2.right_bumper;
        if (robot.shooter != null) {
            robot.shooter.setPower(manualShooterOn ? SHOOTER_POWER : 0.0);
        }
    }

    private void updateFlicker(boolean aPressed) {
        if (flickerServo == null) {
            return;
        }

        if (aPressed && !lastAButton && !isFlicking) {
            isFlicking = true;
            flickStartTime = System.currentTimeMillis();
            flickerServo.setPosition(FLICK_UP);
        }
        if (!aPressed) {
            lastAButton = false;
        } else {
            lastAButton = true;
        }

        if (isFlicking) {
            long elapsed = System.currentTimeMillis() - flickStartTime;
            if (elapsed >= FLICK_DURATION) {
                flickerServo.setPosition(FLICK_DOWN);
                isFlicking = false;
            }
        }
    }

    private void zeroSpinnerIfRequested() {
        if (!spinnerZeroRequested || robot.spinner == null) {
            return;
        }
        int position = robot.spinner.getCurrentPosition();
        if (Math.abs(position) <= 5) {
            spinnerZeroRequested = false;
            robot.spinner.setPower(0.0);
            return;
        }
        double power = position > 0 ? -0.2 : 0.2;
        robot.spinner.setPower(power);
    }

    private void alignRobotToTag() {
        ensureAprilTagPipeline();
        if (tagDetector == null) {
            spinSearch();
            return;
        }
        AprilTagDetector.AprilTagResult tag = findTargetTag();
        if (tag == null || !tag.isValid) {
            spinnerLocked = false;
            hapticSent = false;
            spinSearch();
            return;
        }

        double tx = tag.xDegrees;
        double absTx = Math.abs(tx);
        if (absTx <= DEADBAND) {
            spinnerLocked = true;
            if (!hapticSent) {
                gamepad2.rumble(100);
                hapticSent = true;
            }
            autoRotation = 0.0;
            return;
        }

        spinnerLocked = false;
        hapticSent = false;
        double cmd = -tx * ALIGN_KP;
        double sign = Math.signum(cmd);
        cmd = Math.min(MAX_ALIGN_POWER, Math.max(MIN_ALIGN_POWER, Math.abs(cmd))) * sign;
        autoRotation = cmd;
    }

    private void spinSearch() {
        autoRotation = searchDirection * SEARCH_POWER;
    }

    private void ensureAprilTagPipeline() {
        if (limelight == null) {
            return;
        }
        int desiredPipeline = getAprilTagPipelineIndex();
        if (desiredPipeline == currentPipeline) {
            return;
        }
        try {
            limelight.pipelineSwitch(desiredPipeline);
            currentPipeline = desiredPipeline;
            // Give Limelight time to switch pipelines (critical for detection)
            sleep(100);
        } catch (Exception e) {
            telemetry.addData("Pipeline Switch Error", e.getMessage());
        }
    }

    private void ensureBallPipeline() {
        if (limelight == null) {
            return;
        }
        // Default to green ball detection (pipeline 1)
        // You can modify this to use BALL_PIPELINE_PURPLE (2) if needed
        int desiredPipeline = BALL_PIPELINE_GREEN;
        if (desiredPipeline == currentPipeline) {
            return;
        }
        try {
            limelight.pipelineSwitch(desiredPipeline);
            currentPipeline = desiredPipeline;
        } catch (Exception ignored) {
        }
    }

    private void ensureBallPipelineOff() {
        // Reset to AprilTag pipeline when not tracking balls
        if (limelight == null || currentPipeline == getAprilTagPipelineIndex()) {
            return;
        }
        try {
            int aprilTagPipeline = getAprilTagPipelineIndex();
            limelight.pipelineSwitch(aprilTagPipeline);
            currentPipeline = aprilTagPipeline;
        } catch (Exception ignored) {
        }
    }

    private void trackBall() {
        if (limelight == null) {
            ballTrackingRotation = 0.0;
            return;
        }

        ensureBallPipeline();

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            ballTrackingRotation = 0.0;
            return;
        }

        double tx = result.getTx();
        double ty = result.getTy();
        double ta = result.getTa();

        // Check for valid data
        if (Double.isNaN(tx) || Double.isNaN(ty) || Double.isNaN(ta)) {
            ballTrackingRotation = 0.0;
            return;
        }

        // Check if ball is close enough for auto-assist
        boolean needsAutoAssist = ta >= AUTO_ASSIST_AREA;

        if (needsAutoAssist) {
            // Calculate rotation correction
            double error = (Math.abs(tx) < BALL_DEADBAND) ? 0 : tx;
            double rotationCorrection = -BALL_KP * error;

            // Calculate speed factor based on how fast we're moving
            double forward = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double speed = Math.hypot(forward, strafe);
            double speedFactor = 1.0 - (SPEED_SCALING * speed);
            speedFactor = Math.max(0.3, speedFactor); // Don't go below 30% power

            // Scale rotation correction by speed factor
            rotationCorrection *= speedFactor;

            // Apply minimum and maximum rotation power
            double absCorrection = Math.abs(rotationCorrection);
            if (absCorrection > 0) {
                if (absCorrection < MIN_BALL_ROTATION) {
                    rotationCorrection = Math.copySign(MIN_BALL_ROTATION, rotationCorrection);
                } else if (absCorrection > MAX_BALL_ROTATION) {
                    rotationCorrection = Math.copySign(MAX_BALL_ROTATION, rotationCorrection);
                }
            }

            ballTrackingRotation = rotationCorrection;
        } else {
            // Ball detected but not close enough for auto-assist
            ballTrackingRotation = 0.0;
        }
    }

    private void updateHood() {
        double hoodTarget;
        if (manualMode) {
            hoodTarget = manualHoodTarget;
        } else {
            hoodTarget = shootingMode ? HOOD_DOWN : HOOD_UP;
            if (shootingMode && robotLime != null) {
                AprilTagDetector.AprilTagResult tag = findTargetTag();
                if (tag != null && tag.isValid && tag.distance > 0) {
                    // Use 15-point formula: calculate hood position from distance
                    double calculatedHood = robotLime.calculateHoodPosition(tag.distance, null);
                    hoodTarget = clamp(calculatedHood, HOOD_MIN, HOOD_MAX);
                }
            }
        }
        if (hoodServo != null) {
            hoodServo.setPosition(hoodTarget);
        }
        // No transfer servo anymore
    }

    private void updateTelemetry() {
        telemetry.addLine("=== HeleOp Status ===");
        telemetry.addData("Intake", intakeActive ? "ON (-1)" : "OFF");
        telemetry.addData("Shooting", shootingMode ? "ALIGNING" : "OFF");
        telemetry.addData("Manual Mode", manualMode ? "YES" : "NO");
        if (manualMode) {
            telemetry.addData("Manual Hood", "%.3f", manualHoodTarget);
        }
        telemetry.addData("Spinner Locked", spinnerLocked);
        telemetry.addData("Spinner Pos", robot.spinner != null ? robot.spinner.getCurrentPosition() : "N/A");
        telemetry.addData("Limelight", limelight != null ? "Connected" : "Missing");
        telemetry.addData("Pipeline", currentPipeline >= 0 ? String.valueOf(currentPipeline) : "Not set");
        if (shootingMode && robotLime != null) {
            AprilTagDetector.AprilTagResult tag = findTargetTag();
            if (tag != null && tag.isValid && tag.distance > 0) {
                double calculatedRPM = robotLime.calculateShooterRPM(tag.distance);
                double calculatedHood = robotLime.calculateHoodPosition(tag.distance, null);
                telemetry.addData("Formula RPM", "%.0f", calculatedRPM);
                telemetry.addData("Formula Hood", "%.4f", calculatedHood);
            }
        }
        if (shootingMode) {
            if (limelight != null) {
                // Add raw Limelight data for debugging
                try {
                    LLResult rawResult = limelight.getLatestResult();
                    if (rawResult != null) {
                        telemetry.addData("Raw TX", String.format("%.2f", rawResult.getTx()));
                        telemetry.addData("Raw TY", String.format("%.2f", rawResult.getTy()));
                        telemetry.addData("Raw TA", String.format("%.2f", rawResult.getTa()));
                        telemetry.addData("Raw Valid", rawResult.isValid());
                        if (rawResult.getFiducialResults() != null) {
                            telemetry.addData("Fiducial Count", rawResult.getFiducialResults().size());
                            if (!rawResult.getFiducialResults().isEmpty()) {
                                for (int i = 0; i < rawResult.getFiducialResults().size() && i < 3; i++) {
                                    int tagId = rawResult.getFiducialResults().get(i).getFiducialId();
                                    telemetry.addData("Tag " + i, "ID: " + tagId);
                                }
                            }
                        }
                    }
                } catch (Exception e) {
                    telemetry.addData("Raw Data Error", e.getMessage());
                }
            }
            AprilTagDetector.AprilTagResult tag = findTargetTag();
            if (tag != null && tag.isValid) {
                telemetry.addData("Target Tag", String.format("ID %d", tag.tagId));
                telemetry.addData("Tag Distance", String.format("%.1f\"", tag.distance));
                telemetry.addData("Tag TX", String.format("%.2f°", tag.xDegrees));
            } else {
                telemetry.addData("Target Tag", "none");
                telemetry.addData("TagDetector", tagDetector != null ? "OK" : "NULL");
            }
        } else if (intakeActive && !manualMode) {
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    double tx = result.getTx();
                    double ta = result.getTa();
                    if (!Double.isNaN(tx) && !Double.isNaN(ta)) {
                        telemetry.addData("Ball Tracking", String.format("TX: %.1f° Area: %.1f%%", tx, ta));
                    } else {
                        telemetry.addData("Ball Tracking", "No ball detected");
                    }
                }
            }
        }
        telemetry.update();
    }

    private AprilTagDetector.AprilTagResult findTargetTag() {
        if (tagDetector == null) {
            return new AprilTagDetector.AprilTagResult();
        }
        for (int id : getTargetTagIds()) {
            AprilTagDetector.AprilTagResult tag = tagDetector.getTagById(id);
            if (tag != null && tag.isValid) {
                return tag;
            }
        }
        return new AprilTagDetector.AprilTagResult();
    }

    protected abstract int[] getTargetTagIds();

    protected abstract int getAprilTagPipelineIndex();

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

