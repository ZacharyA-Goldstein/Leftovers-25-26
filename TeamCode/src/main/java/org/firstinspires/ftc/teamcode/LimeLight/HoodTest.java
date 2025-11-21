package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.dumbMap;

@TeleOp(name = "Hood Test", group = "LimeLight")
public class HoodTest extends LinearOpMode {
    private static final double HOOD_MIN = 0.08;
    private static final double HOOD_MAX = 0.14;
    private static final double HOOD_SMALL_STEP = 0.01;
    private static final double HOOD_LARGE_STEP = 0.05;
    private static final double SHOOTER_POWER = -1.0;
    private static final int[] TARGET_TAG_IDS = {24, 20};

    private dumbMap robot;
    private Limelight3A limelight;
    private AprilTagDetector tagDetector;
    private Servo hoodServo;

    private double hoodPosition = 0.08;
    private boolean lastA, lastB, lastX, lastY;

    @Override
    public void runOpMode() {
        robot = new dumbMap(this);
        robot.init2();
        robot.initLimeLight();

        limelight = robot.getLimeLight();
        if (limelight != null) {
            limelight.pipelineSwitch(0);
            tagDetector = new AprilTagDetector(limelight, 9.5, 0.0, 150.0);
        }

        hoodServo = robot.hood;
        if (hoodServo != null) {
            hoodPosition = HOOD_MIN;
            hoodServo.setPosition(hoodPosition);
        }

        if (robot.shooter != null) {
            robot.shooter.setZeroPowerBehavior(robot.shooter.getZeroPowerBehavior());
            robot.shooter.setPower(0.0);
        }

        telemetry.addLine("Hood Test ready");
        telemetry.addLine("A/B: fine adjust ±0.05  |  X/Y: fine adjust ±0.01");
        telemetry.addLine("RT: hold to run shooter (-1)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleHoodButtons();
            updateServo();
            updateShooter();
            updateTelemetry();
            sleep(20);
        }

        if (robot.shooter != null) {
            robot.shooter.setPower(0.0);
        }
    }

    private void handleHoodButtons() {
        boolean aNow = gamepad1.a;
        boolean bNow = gamepad1.b;
        boolean xNow = gamepad1.x;
        boolean yNow = gamepad1.y;

        if (aNow && !lastA) {
            hoodPosition = clamp(hoodPosition - HOOD_LARGE_STEP, HOOD_MIN, HOOD_MAX);
        }
        if (bNow && !lastB) {
            hoodPosition = clamp(hoodPosition + HOOD_LARGE_STEP, HOOD_MIN, HOOD_MAX);
        }
        if (xNow && !lastX) {
            hoodPosition = clamp(hoodPosition - HOOD_SMALL_STEP, HOOD_MIN, HOOD_MAX);
        }
        if (yNow && !lastY) {
            hoodPosition = clamp(hoodPosition + HOOD_SMALL_STEP, HOOD_MIN, HOOD_MAX);
        }

        lastA = aNow;
        lastB = bNow;
        lastX = xNow;
        lastY = yNow;
    }

    private void updateServo() {
        if (hoodServo != null) {
            hoodServo.setPosition(hoodPosition);
        }
    }

    private void updateShooter() {
        if (robot.shooter == null) {
            return;
        }
        if (gamepad1.right_trigger > 0.1) {
            robot.shooter.setPower(SHOOTER_POWER);
        } else {
            robot.shooter.setPower(0.0);
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== Hood Test ===");
        telemetry.addData("Hood Position", "%.3f", hoodPosition);
        telemetry.addData("Limits", "%.2f - %.2f", HOOD_MIN, HOOD_MAX);

        String distanceLine = "No tag";
        if (tagDetector != null) {
            AprilTagDetector.AprilTagResult tag = findTargetTag();
            if (tag != null && tag.isValid) {
                distanceLine = String.format("ID %d: %.1f\"", tag.tagId, tag.distance);
            }
        }
        telemetry.addData("Distance to Tag", distanceLine);
        telemetry.addData("Right Trigger", gamepad1.right_trigger > 0.1 ? "SHOOTING" : "IDLE");
        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private AprilTagDetector.AprilTagResult findTargetTag() {
        if (tagDetector == null) {
            return new AprilTagDetector.AprilTagResult();
        }

        for (int id : TARGET_TAG_IDS) {
            AprilTagDetector.AprilTagResult tag = tagDetector.getTagById(id);
            if (tag != null && tag.isValid) {
                return tag;
            }
        }
        return new AprilTagDetector.AprilTagResult();
    }
}

