package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import org.firstinspires.ftc.teamcode.dumbMap;

public abstract class HeleOpBase extends LinearOpMode {
    private static final double DRIVE_DEADBAND = 0.05;
    private static final double HOOD_UP = 0.08;
    private static final double HOOD_DOWN = 0.14;
    private static final double TRANSFER_UP = 0.23;
    private static final double TRANSFER_DOWN = 0.6;
    private static final double INTAKE_POWER = -1.0;
    private static final double ALIGN_KP = 0.02;
    private static final double MIN_ALIGN_POWER = 0.12;
    private static final double MAX_ALIGN_POWER = 0.5;
    private static final double DEADBAND = 0.5;
    private static final double TOLERANCE = 3.0;
    private static final double SEARCH_POWER = 0.25;
    private static final int SPINNER_MIN = -1000;
    private static final int SPINNER_MAX = 3000;
    private static final double CAMERA_HEIGHT = 9.5;
    private static final double CAMERA_ANGLE = 0.0;
    private static final double MAX_TAG_DISTANCE = 150.0;
    private static final double SHOOTER_POWER = -1.0;

    private dumbMap robot;
    private Limelight3A limelight;
    private AprilTagDetector tagDetector;

    private Servo hoodServo;
    private Servo transferServo;

    private boolean intakeActive = false;
    private boolean intakeTogglePressed = false;
    private boolean shootingMode = false;
    private boolean shootingTogglePressed = false;
    private boolean spinnerLocked = false;
    private boolean hapticSent = false;
    private boolean spinnerZeroRequested = false;
    private int searchDirection = 1;
    private double autoRotation = 0.0;

    @Override
    public void runOpMode() {
        robot = new dumbMap(this);
        robot.init2();
        robot.initLimeLight();

        limelight = robot.getLimeLight();
        if (limelight != null) {
            tagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT, CAMERA_ANGLE, MAX_TAG_DISTANCE);
        }

        if (robot.spinner != null) {
            robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.spinner.setPower(0.0);
        }

        hoodServo = robot.hood;
        transferServo = robot.transfer;
        if (transferServo != null) {
            transferServo.setPosition(TRANSFER_DOWN);
        }

        telemetry.addLine("HeleOp ready. Controller 2: LT=Intake, RT=Shoot");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveRobot();
            handleIntakeToggle();
            handleShootingToggle();
            zeroSpinnerIfRequested();
            if (shootingMode) {
                alignRobotToTag();
            } else if (!spinnerZeroRequested && robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            updateHoodAndTransfer();
            updateTelemetry();
            sleep(20);
        }
    }

    private void driveRobot() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double driveScale = gamepad1.right_bumper ? 0.5 : 1.0;
        forward *= driveScale;
        strafe *= driveScale;
        rotate *= driveScale;

        if (Math.abs(forward) < DRIVE_DEADBAND) forward = 0;
        if (Math.abs(strafe) < DRIVE_DEADBAND) strafe = 0;
        if (Math.abs(rotate) < DRIVE_DEADBAND) rotate = 0;

        double applyRotation = shootingMode ? autoRotation : rotate;

        if (robot.leftFront != null) robot.leftFront.setPower(forward + strafe + applyRotation);
        if (robot.rightFront != null) robot.rightFront.setPower(forward - strafe - applyRotation);
        if (robot.leftBack != null) robot.leftBack.setPower(forward - strafe + applyRotation);
        if (robot.rightBack != null) robot.rightBack.setPower(forward + strafe - applyRotation);
    }

    private void handleIntakeToggle() {
        boolean triggerNow = gamepad2.left_trigger > 0.1;
        if (triggerNow && !intakeTogglePressed) {
            intakeActive = !intakeActive;
            spinnerZeroRequested = true;
        }
        intakeTogglePressed = triggerNow;

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
        }
        shootingTogglePressed = triggerNow;

        if (robot.shooter != null) {
            robot.shooter.setPower(shootingMode ? SHOOTER_POWER : 0.0);
        }
    }

    private void zeroSpinnerIfRequested() {
        if (!spinnerZeroRequested || shootingMode || robot.spinner == null) {
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

    private void updateHoodAndTransfer() {
        if (hoodServo != null) {
            hoodServo.setPosition(shootingMode ? HOOD_DOWN : HOOD_UP);
        }
        if (transferServo != null) {
            double position = (!intakeActive && shootingMode) ? TRANSFER_UP : TRANSFER_DOWN;
            transferServo.setPosition(position);
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== HeleOp Status ===");
        telemetry.addData("Intake", intakeActive ? "ON (-1)" : "OFF");
        telemetry.addData("Shooting", shootingMode ? "ALIGNING" : "OFF");
        telemetry.addData("Spinner Locked", spinnerLocked);
        telemetry.addData("Spinner Pos", robot.spinner != null ? robot.spinner.getCurrentPosition() : "N/A");
        telemetry.addData("Limelight", limelight != null ? "Connected" : "Missing");
        AprilTagDetector.AprilTagResult tag = findTargetTag();
        if (tag != null && tag.isValid) {
            telemetry.addData("Target Tag", String.format("ID %d", tag.tagId));
            telemetry.addData("Tag Distance", String.format("%.1f\"", tag.distance));
        } else {
            telemetry.addData("Target Tag", "none");
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
}

