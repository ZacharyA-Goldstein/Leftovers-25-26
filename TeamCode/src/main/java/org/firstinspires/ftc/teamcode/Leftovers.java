package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/**
 * Full scrimmage code for Leftovers robot.
 * - Controller 1: Driver only movement
 * - Controller 2: Left trigger toggles intake, right trigger cycles/shoots balls
 * - Auto-aim runs only during shooting (full-bot turn)
 */
@TeleOp(name = "Leftovers", group = "TeleOp")
public class Leftovers extends LinearOpMode {
    private dumbMap robot;

    // Drive
    private double drivePower = 1.0;

    // Intake / Outtake controls
    private boolean intakeOn = false;
    private boolean lastIntakeTrigger = false;
    private boolean shootingActive = false;
    private boolean lastShootTrigger = false;

    // Limelight + auto-aim (always active)
    private AprilTagDetector tagDetector;
    private Limelight3A limelight;

    // ============================================================================
    // TUNING VALUES - CHANGE THESE AFTER TESTING WITH ShooterAnglePowerTest
    // ============================================================================
    
    // --- Camera mounting ---
    private static final double CAMERA_HEIGHT_IN = 13.0; // Inches. Measure lens height from floor
    private static final double CAMERA_ANGLE_DEG = 0.0; // Degrees down from horizontal. Measure on robot
    private static final double MAX_SHOT_DISTANCE_IN = 144.0; // Maximum distance for tag detection
    
    // --- Auto rotation tuning ---
    private static final double ALIGN_KP = 0.02;
    private static final double MIN_ALIGN_POWER = 0.12;
    private static final double MAX_ALIGN_POWER = 0.5;
    private static final double ALIGN_DEADBAND = 0.5;
    private static final double SEARCH_POWER = 0.25;
    private static final long SEARCH_TIMEOUT_MS = 5000;
    
    // --- Shooter power control (shooter motor) ---
    // TUNE THESE VALUES based on ShooterAnglePowerTest results:
    private static final double SHOOTER_POWER_MIN = 0.4;  // Minimum power (for close shots)
    private static final double SHOOTER_POWER_MAX = 1.0;  // Maximum power (for far shots)
    private static final double SHOOTER_POWER_BASE = 0.35; // Base power offset
    private static final double SHOOTER_POWER_SCALE = 0.65; // Power scaling factor
    
    // --- Shooter angle control (hood servo) ---
    // TUNE THESE VALUES based on ShooterAnglePowerTest results:
    private static final double HOOD_POS_MIN = 0.0;   // Minimum hood position (flat/down)
    private static final double HOOD_POS_MAX = 0.5;  // Maximum hood position (up/angled)
    
    // --- Lift servo positions ---
    private static final double LIFT_UP_POSITION = 0.59;   // Lift top position (servo value)
    private static final double LIFT_DOWN_POSITION = 1.0;  // Lift bottom position (servo value)
    
    // --- Lift gate servo positions ---
    
    // --- 3-Ball Cycle System ---
    private static final int MAX_BALLS = 3;                    // Number of balls to cycle through
    private static final double INTAKE_WHEEL_POWER = 0.8;      // TUNE THIS - power for intake wheel to push balls
    private static final double BALL_LOAD_TIME = 0.5;          // TUNE THIS - time to run intake wheel to load ball (seconds)
    private static final double SHOT_WAIT_TIME = 0.8;          // TUNE THIS - time to wait after raising lift before next cycle (seconds)
    private static final double LIFT_MOVE_DELAY = 0.3;         // TUNE THIS - time to wait for lift servo to move (seconds)

    // Ball cycle state machine
    private enum BallCycleState {
        IDLE,               // Not cycling
        LOWERING_LIFT,      // Lowering lift to bottom to load next ball
        LOADING_BALL,       // Running intake wheel to push ball into lift
        RAISING_LIFT,       // Raising lift to top for shooting
        SHOOTING            // Lift at top, shooting ball
    }
    private BallCycleState cycleState = BallCycleState.IDLE;
    private int currentBall = 0;                               // Current ball number (1-3)
    private ElapsedTime stateTimer = new ElapsedTime();        // Timer for state transitions
    private boolean cycleActive = false;                       // Whether ball cycle is active

    // Hardware components
    private Servo transferLift;       // Lift servo
    private Servo hoodServo;          // Hood servo for shooter angle
    private double hoodTarget = HOOD_POS_MIN;
    private double manualLiftPosition = LIFT_DOWN_POSITION;
    private double manualShooterPower = 0.8;
    private DcMotor shooterMotor;     // Shooter wheel motor
    private DcMotor spinnerMotor;     // Encoder we keep near zero
    private double autoRotation = 0.0;
    private boolean manualMode = false;
    private boolean lastBButton = false;
    private boolean searchingForTag = false;
    private long searchStartTime = 0;

    @Override
    public void runOpMode() {
        // Initialize the robot hardware via dumbMap
        robot = new dumbMap(this);
        robot.init2();

        // Initialize Limelight
        robot.initLimeLight();
        limelight = robot.getLimeLight();
        if (limelight != null) {
            tagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT_IN, CAMERA_ANGLE_DEG, MAX_SHOT_DISTANCE_IN);
            limelight.pipelineSwitch(0);
            telemetry.addData("Limelight", "Initialized");
        } else {
            tagDetector = null;
            telemetry.addData("Limelight", "NOT FOUND");
        }

        // Map shooter motor
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setPower(0.0);
            telemetry.addData("Shooter Motor", "Mapped");
        } catch (Exception e) {
            shooterMotor = null;
            telemetry.addData("Shooter Motor", "NOT found");
        }

        // Map spinner encoder motor
        try {
            spinnerMotor = hardwareMap.get(DcMotor.class, "spinner");
            spinnerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spinnerMotor.setPower(0.0);
            telemetry.addData("Spinner Motor", "Mapped");
        } catch (Exception e) {
            spinnerMotor = null;
            telemetry.addData("Spinner Motor", "NOT found");
        }

        // Map hood servo
        try {
            hoodServo = hardwareMap.get(Servo.class, "hood");
            hoodServo.setPosition(HOOD_POS_MIN);
            telemetry.addData("Hood Servo", "Mapped");
        } catch (Exception e) {
            hoodServo = null;
            telemetry.addData("Hood Servo", "NOT found");
        }

        // Map transfer lift servo
        try {
            transferLift = hardwareMap.get(Servo.class, "transferLift");
            transferLift.setPosition(LIFT_DOWN_POSITION); // Start with lift down
            telemetry.addData("Transfer Lift", "Mapped");
        } catch (Exception e) {
            transferLift = null;
            telemetry.addData("Transfer Lift", "NOT found");
        }

        telemetry.addData("Status", "Initialized. Press START to begin.");
        telemetry.addData("Controller 1", "Driver only");
        telemetry.addData("Controller 2", "Left Trigger = Intake, Right Trigger = Shoot, A = Gate");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ========================================================================
            // CONTROLLER 1: DRIVER ONLY
            // ========================================================================
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double manualRotate = applyDeadband(gamepad1.right_stick_x, 0.1);

            // Right bumper for slow mode
            drivePower = gamepad1.right_bumper ? 0.5 : 1.0;
            forward *= drivePower;
            strafe *= drivePower;
            manualRotate *= drivePower;

            double autoCorrection;
            AprilTagDetector.AprilTagResult tagResult = null;
            if (manualMode) {
                autoCorrection = 0.0;
            } else if (shootingActive) {
                tagResult = updateAutoRotation();
                autoCorrection = autoRotation;
            } else {
                autoCorrection = 0.0;
                tagResult = null;
            }
            autoRotation = autoCorrection;
            double rotate = manualRotate + autoCorrection;

            maintainSpinnerZero();

            // Set mecanum powers
            robot.leftFront.setPower(forward + strafe + rotate);
            robot.rightFront.setPower(forward - strafe - rotate);
            robot.leftBack.setPower(forward - strafe + rotate);
            robot.rightBack.setPower(forward + strafe - rotate);

            // ========================================================================
            // CONTROLLER 2: INTAKE, SHOOTING, GATE CONTROL
            // ========================================================================
            
            // Intake toggle on left trigger (gamepad2)
            boolean currentIntakeTrigger = gamepad2.left_trigger > 0.5;
            if (currentIntakeTrigger && !lastIntakeTrigger) {
                intakeOn = !intakeOn;
                if (intakeOn) {
                    // Make sure lift is at bottom and gate is open to receive first ball
                    if (transferLift != null && !cycleActive) {
                        setLiftPosition(LIFT_DOWN_POSITION);
                    }
                } else {
                    // Stop intake - close gate to hold ball in lift
                }
            }
            lastIntakeTrigger = currentIntakeTrigger;
            robot.intake.setPower(intakeOn ? 1.0 : 0.0);
            
            // When intake is on, keep gate open and run intake wheel
            if (intakeOn && !cycleActive) {
                // Intake wheel runs to push balls through (connected to intake motor)
                // The intake motor already runs when intakeOn is true
            }

            // Shoot toggle on right trigger (gamepad2) - starts 3-ball cycle
            boolean currentShootTrigger = gamepad2.right_trigger > 0.5;
            if (currentShootTrigger && !lastShootTrigger) {
                if (!shootingActive && !cycleActive) {
                    // Start shooting cycle
                    shootingActive = true;
                    cycleActive = true;
                    currentBall = 1; // Start with ball 1 (already in lift)
                    cycleState = BallCycleState.RAISING_LIFT; // First ball is already in lift, just raise it
                    stateTimer.reset();
                    setLiftPosition(LIFT_UP_POSITION);
                } else if (shootingActive) {
                    // Stop shooting
                    shootingActive = false;
                    cycleActive = false;
                    cycleState = BallCycleState.IDLE;
                    currentBall = 0;
                    if (shooterMotor != null) shooterMotor.setPower(0.0);
                    setLiftPosition(LIFT_DOWN_POSITION);
                }
            }
            lastShootTrigger = currentShootTrigger;

            boolean bPressed = gamepad2.b;
            if (bPressed && !lastBButton) {
                manualMode = !manualMode;
            }
            lastBButton = bPressed;

            if (manualMode) {
                if (gamepad2.dpad_up) {
                    hoodTarget = Math.min(HOOD_POS_MAX, hoodTarget + 0.01);
                }
                if (gamepad2.dpad_down) {
                    hoodTarget = Math.max(HOOD_POS_MIN, hoodTarget - 0.01);
                }
                if (gamepad2.dpad_left) {
                    manualLiftPosition = Math.min(LIFT_DOWN_POSITION, manualLiftPosition + 0.01);
                }
                if (gamepad2.dpad_right) {
                    manualLiftPosition = Math.max(LIFT_UP_POSITION, manualLiftPosition - 0.01);
                }
                if (gamepad2.right_bumper) {
                    manualShooterPower = Math.min(1.0, manualShooterPower + 0.02);
                }
                if (gamepad2.left_bumper) {
                    manualShooterPower = Math.max(0.2, manualShooterPower - 0.02);
                }
            }

            // ========================================================================
            // AUTO-AIM: Shooting-only adjustments
            // ========================================================================
            double hoodPosition = hoodTarget;
            if (shootingActive && tagResult != null && tagResult.isValid) {
                double distanceIn = tagResult.distance;

                double wheelPower = manualMode ? manualShooterPower : shooterPowerForDistance(distanceIn);
                if (shooterMotor != null) {
                    shooterMotor.setPower(-wheelPower);
                }

                if (!manualMode) {
                    hoodPosition = calculateHoodPosition(distanceIn);
                    hoodTarget = hoodPosition;
                }
            } else if (shooterMotor != null) {
                shooterMotor.setPower(0.0);
            }

            if (hoodServo != null) {
                hoodServo.setPosition(hoodPosition);
            }

            // ========================================================================
            // 3-BALL CYCLE STATE MACHINE
            // ========================================================================
            if (cycleActive && shootingActive) {
                updateBallCycle();
            }

            // ========================================================================
            // TELEMETRY
            // ========================================================================
            telemetry.addData("Drive Power", "%.0f%%", drivePower * 100);
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Shooting", shootingActive ? "ACTIVE" : "OFF");
            
            if (tagDetector != null) {
                AprilTagDetector.AprilTagResult tag = tagDetector.getClosestTag();
                if (tag.isValid) {
                    telemetry.addData("Limelight", "Tag ID: %d", tag.tagId);
                    telemetry.addData("Distance (in)", "%.1f", tag.distance);
                    telemetry.addData("tx (deg)", "%.2f", tag.xDegrees);
                } else {
                    telemetry.addData("Limelight", "No Tag");
                }
            }
            
            if (cycleActive) {
                telemetry.addData("Ball Cycle", "Active - Ball %d/%d", currentBall, MAX_BALLS);
                telemetry.addData("Cycle State", cycleState.toString());
            } else {
                telemetry.addData("Ball Cycle", "Inactive");
            }
            
            if (transferLift != null) {
                telemetry.addData("Lift Pos", "%.3f", transferLift.getPosition());
            }
            
            if (shooterMotor != null) {
                telemetry.addData("Shooter Power", "%.2f", Math.abs(shooterMotor.getPower()));
            }
            
            telemetry.update();
        }

            // Stop all motors when opmode ends
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.intake.setPower(0);
            if (shooterMotor != null) shooterMotor.setPower(0.0);
            if (transferLift != null) {
                setLiftPosition(LIFT_DOWN_POSITION);
            }
        }

    // ============================================================================
    // TUNING FUNCTIONS - ADJUST THESE AFTER TESTING
    // ============================================================================
    
    /**
     * Map distance (in) to shooter wheel power [0..1]
     * TUNE THESE VALUES after testing with ShooterAnglePowerTest:
     * - SHOOTER_POWER_BASE: Base power offset (typically 0.35)
     * - SHOOTER_POWER_SCALE: Power scaling factor (typically 0.65)
     * - SHOOTER_POWER_MIN: Minimum power (typically 0.4)
     * - SHOOTER_POWER_MAX: Maximum power (typically 1.0)
     */
    private double shooterPowerForDistance(double distanceIn) {
        return 1.0;
    }

    /**
     * Map distance to hood servo position.
     * Uses the tuned formula hood ≈ 187.75452 / distance² and clamps to the servo range.
     */
    private double calculateHoodPosition(double distanceIn) {
        // Formula: hood position ≈ 187.75452 / (distance^2), clamped to servo range
        if (distanceIn <= 0.0) {
            return HOOD_POS_MIN;
        }
        double position = 187.75452 / (distanceIn * distanceIn);
        return Math.max(HOOD_POS_MIN, Math.min(HOOD_POS_MAX, position));
    }

    private AprilTagDetector.AprilTagResult updateAutoRotation() {
        if (tagDetector == null) {
            autoRotation = 0.0;
            searchingForTag = false;
            return new AprilTagDetector.AprilTagResult();
        }
        AprilTagDetector.AprilTagResult tag = tagDetector.getClosestTag();
        if (tag.isValid) {
            double tx = tag.xDegrees;
            double absTx = Math.abs(tx);
            if (absTx <= ALIGN_DEADBAND) {
                autoRotation = 0.0;
            } else {
                double cmd = -tx * ALIGN_KP;
                double sign = Math.signum(cmd);
                autoRotation = Math.min(MAX_ALIGN_POWER, Math.max(MIN_ALIGN_POWER, Math.abs(cmd))) * sign;
            }
            searchingForTag = false;
        } else {
            if (!searchingForTag) {
                searchingForTag = true;
                searchStartTime = System.currentTimeMillis();
            }
            long elapsed = System.currentTimeMillis() - searchStartTime;
            if (elapsed < SEARCH_TIMEOUT_MS) {
                autoRotation = SEARCH_POWER;
            } else {
                autoRotation = 0.0;
                searchingForTag = false;
            }
        }
        return tag;
    }

    private void maintainSpinnerZero() {
        if (spinnerMotor == null) {
            return;
        }
        int position = spinnerMotor.getCurrentPosition();
        if (Math.abs(position) > 10) {
            double power = Math.copySign(0.1, position);
            spinnerMotor.setPower(-power);
        } else {
            spinnerMotor.setPower(0.0);
        }
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }

    // ============================================================================
    // 3-BALL CYCLE STATE MACHINE
    // Assumes: Ball 1 starts in lift, Balls 2-3 wait in tunnel
    // ============================================================================
    private void updateBallCycle() {
        // Check if we've completed all 3 balls
        if (currentBall > MAX_BALLS) {
            cycleActive = false;
            shootingActive = false;
            cycleState = BallCycleState.IDLE;
            if (shooterMotor != null) shooterMotor.setPower(0.0);
            return;
        }

        switch (cycleState) {
            case LOWERING_LIFT:
                // Lower lift to bottom to load next ball from tunnel
                setLiftPosition(LIFT_DOWN_POSITION);
                
                // Wait for lift to move down
                if (stateTimer.seconds() >= LIFT_MOVE_DELAY) {
                    currentBall++;
                    cycleState = BallCycleState.LOADING_BALL;
                    stateTimer.reset();
                    // Gate removed; rely on lift delay instead
                    // Run intake wheel to push ball from tunnel (intake motor is already running)
                    robot.intake.setPower(INTAKE_WHEEL_POWER);
                }
                break;

            case LOADING_BALL:
                // Run intake wheel to push ball from tunnel into lift
                if (stateTimer.seconds() >= BALL_LOAD_TIME) {
                    cycleState = BallCycleState.RAISING_LIFT;
                    stateTimer.reset();
                    robot.intake.setPower(0.0); // Stop intake wheel
                    setLiftPosition(LIFT_UP_POSITION); // Start raising lift
                }
                break;

            case RAISING_LIFT:
                // Raise lift to top for shooting
                setLiftPosition(LIFT_UP_POSITION);
                
                // Wait for lift to move up
                if (stateTimer.seconds() >= LIFT_MOVE_DELAY) {
                    cycleState = BallCycleState.SHOOTING;
                    stateTimer.reset();
                }
                break;

            case SHOOTING:
                // Keep lift at top, ball exits into shooter
                setLiftPosition(LIFT_UP_POSITION);
                
                if (stateTimer.seconds() >= SHOT_WAIT_TIME) {
                    if (currentBall < MAX_BALLS) {
                        // More balls to shoot, lower lift for next ball from tunnel
                        cycleState = BallCycleState.LOWERING_LIFT;
                        stateTimer.reset();
                        setLiftPosition(LIFT_DOWN_POSITION);
                    } else {
                        // All 3 balls shot, stop cycle and open gate for next intake
                        cycleActive = false;
                        shootingActive = false;
                        cycleState = BallCycleState.IDLE;
                        if (shooterMotor != null) shooterMotor.setPower(0.0);
                    }
                }
                break;

            case IDLE:
                // Do nothing
                break;
        }
    }

    // Helper methods
    private void setLiftPosition(double targetPosition) {
        if (transferLift != null) {
            transferLift.setPosition(targetPosition);
        }
    }

}

