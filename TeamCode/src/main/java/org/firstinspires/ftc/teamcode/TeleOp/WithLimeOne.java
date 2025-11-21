/*package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Driver Control + Limelight (1C)", group = "TeleOp")
public class bot1teleLimeOne extends LinearOpMode {
    private dumbMap robot;

    // Drive
    private double drivePower = 1.0;

    // Intake / Outtake toggles (same behavior as bot1teleLime, but on gamepad1)
    private boolean outtakeOn = false;
    private boolean lastOuttakeTrigger = false;
    private boolean intakeOn = false;
    private boolean lastIntakeTrigger = false;

    // Spinner step (angle adjust) flags (left/right disabled; up/down for servo still available)
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private static final int SPINNER_INCREMENT = 100; // kept for parity (not used for left/right)

    // Limelight + shooter assist
    // Toggle auto-aim with X on gamepad1
    private AprilTagDetector tagDetector;
    private Limelight3A limelight;
    private boolean autoAimEnabled = false;
    private boolean lastX = false;

    // --- TUNING: Camera mounting ---
    private static final double CAMERA_HEIGHT_IN = 12.0; // Inches. Measure lens height from floor
    private static final double CAMERA_ANGLE_DEG = 15.0; // Degrees down from horizontal. Measure on robot
    private static final double MAX_SHOT_DISTANCE_IN = 144.0; // Cap used for power/angle mapping

    // --- TUNING: Horizontal turret control (uses shooterTurn motor)
    // This aligns the shooter horizontally so Limelight tx -> 0°
    private static final double TURRET_KP = 0.02;      // deg -> power
    private static final double TURRET_MIN = 0.12;     // minimum power to move
    private static final double TURRET_MAX = 0.5;      // power cap
    private static final double TURRET_TOL_DEG = 1.2;  // within this, treat as aligned

    // Optional turret motor (not part of dumbMap)
    private DcMotor shooterTurn;
    private CRServo transferServo;
    private double transferPower = 0.0;

    @Override
    public void runOpMode() {
        // Initialize the robot hardware via dumbMap
        robot = new dumbMap(this);
        robot.init2();

        // Initialize Limelight using the helper inside dumbMap and create our detector
        robot.initLimeLight();
        limelight = robot.getLimeLight();
        if (limelight != null) {
            tagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT_IN, CAMERA_ANGLE_DEG, MAX_SHOT_DISTANCE_IN);
        }

        // Map turret motor locally (optional - only if present in config as "shooterTurn")
        try {
            shooterTurn = hardwareMap.get(DcMotor.class, "shooterTurn");
            shooterTurn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterTurn.setDirection(DcMotorSimple.Direction.FORWARD);
            telemetry.addData("Turret", "shooterTurn mapped");
        } catch (Exception e) {
            shooterTurn = null; // leave null if not present
            telemetry.addData("Turret", "shooterTurn NOT found");
        }

        // Map the continuous transfer servo (vertical sorter)
        try {
            transferServo = hardwareMap.get(CRServo.class, "transfer");
            transferServo.setPower(0.0);
            telemetry.addData("Transfer", "CRServo mapped");
        } catch (Exception e) {
            transferServo = null;
            telemetry.addData("Transfer", "CRServo NOT found");
        }

        telemetry.addData("Status", "Initialized (Driver + Limelight 1C)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Driving on gamepad1
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Right bumper for slow mode
            drivePower = gamepad1.right_bumper ? 0.5 : 1.0;
            forward *= drivePower;
            strafe *= drivePower;
            rotate *= drivePower;

            // Set mecanum powers
            robot.leftFront.setPower(forward + strafe + rotate);
            robot.rightFront.setPower(forward - strafe - rotate);
            robot.leftBack.setPower(forward - strafe + rotate);
            robot.rightBack.setPower(forward + strafe - rotate);

            // Auto-aim toggle (X on gamepad1)
            boolean xNow = gamepad1.x;
            if (xNow && !lastX) {
                autoAimEnabled = !autoAimEnabled;

                if (autoAimEnabled) {
                    outtakeOn = true;
                    setTransferPower(1.0);
                } else {
                    outtakeOn = false;
                    setTransferPower(0.0);
                    if (shooterTurn != null) shooterTurn.setPower(0.0);
                }
            }
            lastX = xNow;

            // Intake toggle on left trigger (gamepad1)
            boolean currentIntakeTrigger = gamepad1.left_trigger > 0.5;
            if (currentIntakeTrigger && !lastIntakeTrigger) {
                intakeOn = !intakeOn;
            }
            lastIntakeTrigger = currentIntakeTrigger;
            robot.intake.setPower(intakeOn ? 1.0 : 0.0);

            // Outtake toggle on right trigger (gamepad1)
            boolean currentOuttakeTrigger = gamepad1.right_trigger > 0.5;
            if (currentOuttakeTrigger && !lastOuttakeTrigger) {
                outtakeOn = !outtakeOn;
            }
            lastOuttakeTrigger = currentOuttakeTrigger;

            // Emergency stop with B button (stops both intake and outtake)
            if (gamepad1.b) {
                outtakeOn = false;
                intakeOn = false;
                if (!autoAimEnabled) {
                    setTransferPower(0.0);
                }
            }

            // When auto-aim is enabled and we have a valid tag,
            // compute distance and set shooter wheel + angle automatically
            if (autoAimEnabled && tagDetector != null) {
                AprilTagDetector.AprilTagResult tag = tagDetector.getClosestTag();
                if (tag.isValid) {
                    double distanceIn = tag.distance;
                    double wheelPower = shooterPowerForDistance(distanceIn);
                    int angleTicks = spinnerTicksForDistance(distanceIn);

                    // Set shooter wheel
                    robot.shooter.setPower(wheelPower);

                    // Aim shooter angle with spinner RUN_TO_POSITION
                    robot.spinner.setTargetPosition(angleTicks);
                    robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.spinner.setPower(0.6);

                    // Horizontal alignment: drive shooterTurn so tx -> 0°
                    if (shooterTurn != null) {
                        double tx = tag.xDegrees; // horizontal error in degrees
                        double cmd;
                        if (Math.abs(tx) <= TURRET_TOL_DEG) {
                            cmd = 0.0;
                        } else {
                            cmd = tx * TURRET_KP;
                            double sign = Math.signum(cmd);
                            cmd = Math.min(TURRET_MAX, Math.max(TURRET_MIN, Math.abs(cmd))) * sign;
                        }
                        // Flip sign here if turret runs opposite direction:
                        shooterTurn.setPower(cmd);
                        telemetry.addData("Turret Cmd", "%.2f", cmd);
                    } else {
                        telemetry.addData("Turret", "not configured");
                    }

                    telemetry.addData("LL Distance (in)", String.format("%.1f", distanceIn));
                    telemetry.addData("LL Angle (deg)", String.format("%.1f", tag.angle));
                    telemetry.addData("AutoAim", "ON");
                } else {
                    telemetry.addData("AutoAim", "ON - No Tag");
                    if (shooterTurn != null) shooterTurn.setPower(0);
                }
                if (autoAimEnabled && transferServo != null) {
                    transferServo.setPower(transferPower);
                }
            }

            // If auto-aim is OFF, set outtake by toggle; if ON, wheel power is set above
            if (!autoAimEnabled) {
                robot.shooter.setPower(outtakeOn ? 1.0 : 0.0);
            }

            // D-pad left/right DISABLED for manual angle; auto-aim handles horizontal alignment

            // Transfer manual control with D-pad up/down (only when auto-aim is off)
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;
            if (currentDpadUp && !lastDpadUp) {
                if (!autoAimEnabled) {
                    setTransferPower(1.0);
                }
            }
            if (currentDpadDown && !lastDpadDown) {
                if (!autoAimEnabled) {
                    setTransferPower(0.0);
                }
            }
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // Telemetry
            telemetry.addData("Drive Power", "%.0f%%", drivePower * 100);
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Outtake", autoAimEnabled ? "AUTO" : (outtakeOn ? "ON" : "OFF"));
            telemetry.addData("Spinner Pos", robot.spinner.getCurrentPosition());
            telemetry.addData("Spinner Busy", robot.spinner.isBusy());
            telemetry.addData("Transfer Power", "%.2f", transferPower);
            telemetry.addData("AutoAim Enabled", autoAimEnabled);
            telemetry.update();
        }

        // Stop all motors when opmode ends
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.intake.setPower(0);
        robot.shooter.setPower(0);
        setTransferPower(0.0);
        if (shooterTurn != null) shooterTurn.setPower(0.0);
    }

    // Map distance (in) to shooter wheel power [0..1]
    // TUNING GUIDE:
    // - Shoot from two or three distances (near/mid/far) and record the minimum power
    //   that scores reliably. Fit a simple curve or adjust the linear endpoints below.
    private double shooterPowerForDistance(double distanceIn) {
        double power = 0.35 + (distanceIn / MAX_SHOT_DISTANCE_IN) * 0.65;
        return Math.max(0.4, Math.min(1.0, power));
    }

    // Map distance to spinner encoder ticks for shooter angle
    // TUNING GUIDE:
    // - Measure encoder ticks that hit from various distances and fit a line or curve.
    // - Start by adjusting 'a' (offset) to get mid-field correct, then 'b' (slope) for near/far.
    private int spinnerTicksForDistance(double distanceIn) {
        double a = 1200;   // base angle offset
        double b = -4.0;   // ticks per inch
        int ticks = (int)Math.round(a + b * distanceIn);
        int minTicks = 200;
        int maxTicks = 1800;
        return Math.max(minTicks, Math.min(maxTicks, ticks));
    }

    private void setTransferPower(double power) {
        transferPower = power;
        if (transferServo != null) {
            transferServo.setPower(power);
        }
    }
}*/


