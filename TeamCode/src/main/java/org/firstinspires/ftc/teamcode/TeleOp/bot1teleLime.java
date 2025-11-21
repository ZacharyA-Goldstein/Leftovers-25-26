package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Driver Control + Limelight", group = "TeleOp")
public class bot1teleLime extends LinearOpMode {
    private dumbMap robot;

    // Drive
    private double drivePower = 1.0;

    // Intake / Outtake toggles (same pattern as bot1tele)
    private boolean outtakeOn = false;
    private boolean lastOuttakeTrigger = false;
    private boolean intakeOn = false;
    private boolean lastIntakeTrigger = false;

    // Spinner step (angle adjust) like bot1tele
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Limelight + shooter assist
    // Toggle auto-aim with X on gamepad2 (actions controller), just like our other TeleOps
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

    // --- TUNING: Lift system positions (encoder ticks)
    // 0 = top of lift, bottom position needs to be measured (encoder ticks)
    private static final int LIFT_UP_POSITION = 0;        // Encoder ticks for lift top position
    private static final int LIFT_DOWN_POSITION = 2000;   // Encoder ticks for lift bottom position (TUNE THIS - measure when lift is at bottom)
    private static final double LIFT_POWER = 0.8;         // Power for lift movement

    // --- TUNING: Lift gate servo positions
    // 0 = closed/down (holds ball), 0.37 = open/up (allows ball in)
    private static final double LIFT_GATE_CLOSED = 0.0;   // Position for gate closed/down (holds ball)
    private static final double LIFT_GATE_OPEN = 0.37;    // Position for gate open/up (allows ball in)

    // --- TUNING: 3-Ball Cycle System
    private static final int MAX_BALLS = 3;                    // Number of balls to cycle through
    private static final double INTAKE_WHEEL_POWER = 0.8;      // Power for intake wheel to push ball into lift
    private static final double BALL_LOAD_TIME = 0.5;          // Time to run intake wheel to load ball (seconds)
    private static final double SHOT_WAIT_TIME = 0.8;          // Time to wait after raising lift before next cycle (seconds)
    private static final double LIFT_MOVE_TOLERANCE = 50;      // Encoder tolerance for lift position

    // Ball cycle state machine
    private enum BallCycleState {
        IDLE,               // Not cycling, normal operation
        LOWERING_LIFT,      // Lowering lift to bottom to load next ball
        LOADING_BALL,       // Running intake wheel to push ball into lift
        RAISING_LIFT,       // Raising lift to top for shooting
        SHOOTING            // Lift at top, shooting ball
    }
    private BallCycleState cycleState = BallCycleState.IDLE;
    private int currentBall = 0;                               // Current ball number (1-3)
    private ElapsedTime stateTimer = new ElapsedTime();        // Timer for state transitions
    private boolean cycleActive = false;                       // Whether ball cycle is active

    // Optional turret motor (not part of dumbMap)
    private DcMotor shooterTurn;
    private DcMotor transferLift;
    private Servo liftGateServo;
    private DcMotor intakeWheel;                               // Small wheel in intake tunnel to push balls into lift

    @Override
    public void runOpMode() {
        // Initialize the robot hardware via dumbMap (same style as bot1tele)
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

        // Map the transfer lift motor
        try {
            transferLift = hardwareMap.get(DcMotor.class, "transferLift");
            transferLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            transferLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            transferLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            transferLift.setDirection(DcMotorSimple.Direction.FORWARD);
            // Set initial target to down position (will move after waitForStart)
            transferLift.setTargetPosition(LIFT_DOWN_POSITION);
            telemetry.addData("Transfer Lift", "DcMotor mapped");
        } catch (Exception e) {
            transferLift = null;
            telemetry.addData("Transfer Lift", "DcMotor NOT found");
        }

        // Map the lift gate servo (holds/releases balls in lift - only transfer mechanism)
        try {
            liftGateServo = hardwareMap.get(Servo.class, "liftGate");
            // Start with gate open - ready for intake (first ball goes into lift, others wait in tunnel)
            liftGateServo.setPosition(LIFT_GATE_OPEN);
            telemetry.addData("Lift Gate", "Servo mapped");
        } catch (Exception e) {
            liftGateServo = null;
            telemetry.addData("Lift Gate", "Servo NOT found");
        }

        // Map the intake wheel motor (small wheel in intake tunnel to push balls into lift)
        try {
            intakeWheel = hardwareMap.get(DcMotor.class, "intakeWheel");
            intakeWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeWheel.setPower(0.0);
            telemetry.addData("Intake Wheel", "DcMotor mapped");
        } catch (Exception e) {
            intakeWheel = null;
            telemetry.addData("Intake Wheel", "DcMotor NOT found");
        }

        telemetry.addData("Status", "Initialized (Driver + Limelight)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Driving on gamepad1 (same style as bot1tele)
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

            // Actions on gamepad2 (intake/outtake like bot1tele)
            // Auto-aim toggle (X on gamepad2)
            boolean xNow = gamepad2.x;
            if (xNow && !lastX) {
                autoAimEnabled = !autoAimEnabled;

                if (autoAimEnabled) {
                    // Auto-aim ON -> start 3-ball cycle system
                    // Assumes: 1 ball already in lift, 2 balls waiting in tunnel
                    outtakeOn = true;
                    cycleActive = true;
                    currentBall = 1; // Start with ball 1 (already in lift)
                    cycleState = BallCycleState.RAISING_LIFT; // First ball is already in lift, just raise it
                    stateTimer.reset();
                    // First ball is already in lift, gate should be closed, just raise lift
                    setLiftPosition(LIFT_UP_POSITION);
                    setLiftGate(LIFT_GATE_CLOSED);
                } else {
                    // Auto-aim OFF -> stop cycle, open gate for next intake cycle
                    outtakeOn = false;
                    cycleActive = false;
                    cycleState = BallCycleState.IDLE;
                    currentBall = 0;
                    setLiftPosition(LIFT_DOWN_POSITION); // Lower lift to bottom
                    setLiftGate(LIFT_GATE_OPEN);         // Open gate so first ball can enter on next intake
                    if (intakeWheel != null) intakeWheel.setPower(0.0);
                    if (shooterTurn != null) shooterTurn.setPower(0.0);
                }
            }
            lastX = xNow;

            // Intake toggle on left trigger (gamepad2)
            boolean currentIntakeTrigger = gamepad2.left_trigger > 0.5;
            if (currentIntakeTrigger && !lastIntakeTrigger) {
                intakeOn = !intakeOn;
                // Intake logic: First ball goes into lift, other 2 wait in tunnel
                if (intakeOn) {
                    // Make sure lift is at bottom and gate is open to receive first ball
                    if (!autoAimEnabled && transferLift != null) {
                        setLiftPosition(LIFT_DOWN_POSITION);
                    }
                    setLiftGate(LIFT_GATE_OPEN);  // Open gate at bottom to let first ball enter
                    // Run intake wheel to push first ball into lift
                    if (intakeWheel != null) {
                        intakeWheel.setPower(INTAKE_WHEEL_POWER);
                    }
                } else {
                    // Stop intake - close gate to hold ball in lift, stop intake wheel
                    setLiftGate(LIFT_GATE_CLOSED); // Close gate at bottom to hold ball in lift
                    if (intakeWheel != null) {
                        intakeWheel.setPower(0.0); // Stop intake wheel (other balls wait in tunnel)
                    }
                }
            }
            lastIntakeTrigger = currentIntakeTrigger;
            robot.intake.setPower(intakeOn ? 1.0 : 0.0);
            
            // When intake is on, keep gate open and intake wheel running
            // This allows first ball to enter lift, others wait in tunnel
            if (intakeOn && !autoAimEnabled) {
                setLiftGate(LIFT_GATE_OPEN);
                if (intakeWheel != null) {
                    intakeWheel.setPower(INTAKE_WHEEL_POWER);
                }
            } else if (!intakeOn && !autoAimEnabled) {
                // When intake stops, close gate to hold ball in lift
                setLiftGate(LIFT_GATE_CLOSED);
                if (intakeWheel != null) {
                    intakeWheel.setPower(0.0);
                }
            }

            // Outtake toggle on right trigger (gamepad2)
            boolean currentOuttakeTrigger = gamepad2.right_trigger > 0.5;
            if (currentOuttakeTrigger && !lastOuttakeTrigger) {
                outtakeOn = !outtakeOn;
            }
            lastOuttakeTrigger = currentOuttakeTrigger;

            // Emergency stop with B button (stops both intake and outtake)
            if (gamepad2.b) {
                outtakeOn = false;
                intakeOn = false;
                if (!autoAimEnabled) {
                    setLiftPosition(LIFT_DOWN_POSITION);
                    setLiftGate(LIFT_GATE_OPEN); // Open gate for next intake cycle
                }
                if (intakeWheel != null) intakeWheel.setPower(0.0);
                // Stop cycle if active
                if (cycleActive) {
                    cycleActive = false;
                    cycleState = BallCycleState.IDLE;
                    setLiftGate(LIFT_GATE_OPEN); // Open gate for next intake cycle
                }
            }

            // 3-Ball Cycle State Machine (runs when auto-aim is enabled)
            if (cycleActive && autoAimEnabled) {
                updateBallCycle();
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
                    // If you want to stop outtake when no tag is found, uncomment:
                    // outtakeOn = false;
                    if (shooterTurn != null) shooterTurn.setPower(0);
                }
                // Ball cycle state machine handles lift and gate positions
                // Only override if not in active cycle (shouldn't happen, but safety check)
                if (autoAimEnabled && !cycleActive && transferLift != null) {
                    setLiftPosition(LIFT_UP_POSITION);  // Keep lift at top - lifts ball up into shooter
                    setLiftGate(LIFT_GATE_CLOSED);      // Gate at bottom closed (not intaking)
                }
            }

            // If auto-aim is OFF, set outtake by toggle; if ON, wheel power is set above
            if (!autoAimEnabled) {
                robot.shooter.setPower(outtakeOn ? 1.0 : 0.0);
            }

            // D-pad left/right DISABLED for manual angle; auto-aim handles horizontal alignment

            // Lift manual control with D-pad up/down when auto-aim is off
            boolean currentDpadUp = gamepad2.dpad_up;
            boolean currentDpadDown = gamepad2.dpad_down;
            if (currentDpadUp && !lastDpadUp) {
                if (!autoAimEnabled && transferLift != null) {
                    setLiftPosition(LIFT_UP_POSITION);
                    setLiftGate(LIFT_GATE_CLOSED); // Close gate when raising lift
                }
            }
            if (currentDpadDown && !lastDpadDown) {
                if (!autoAimEnabled && transferLift != null) {
                    setLiftPosition(LIFT_DOWN_POSITION);
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
            if (transferLift != null) {
                telemetry.addData("Lift Pos", transferLift.getCurrentPosition());
                telemetry.addData("Lift Target", transferLift.getTargetPosition());
                telemetry.addData("Lift Busy", transferLift.isBusy());
            } else {
                telemetry.addData("Lift", "not configured");
            }
            if (liftGateServo != null) {
                telemetry.addData("Lift Gate", liftGateServo.getPosition() < 0.5 ? "OPEN" : "CLOSED");
            }
            telemetry.addData("AutoAim Enabled", autoAimEnabled);
            if (cycleActive) {
                telemetry.addData("Ball Cycle", "Active - Ball %d/%d", currentBall, MAX_BALLS);
                telemetry.addData("Cycle State", cycleState.toString());
            } else {
                telemetry.addData("Ball Cycle", "Inactive");
            }
            telemetry.update();
        }

        // Stop all motors when opmode ends
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.intake.setPower(0);
        robot.shooter.setPower(0);
        if (transferLift != null) {
            setLiftPosition(LIFT_DOWN_POSITION);
        }
        setLiftGate(LIFT_GATE_OPEN); // Open gate for next intake cycle
        if (intakeWheel != null) intakeWheel.setPower(0.0);
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

    // 3-Ball Cycle State Machine
    // Assumes: Ball 1 starts in lift, Balls 2-3 wait in tunnel
    private void updateBallCycle() {
        // Check if we've completed all 3 balls
        if (currentBall > MAX_BALLS) {
            cycleActive = false;
            cycleState = BallCycleState.IDLE;
            if (intakeWheel != null) intakeWheel.setPower(0.0);
            // Open gate when cycle completes so next intake can load first ball
            setLiftGate(LIFT_GATE_OPEN);
            return;
        }

        switch (cycleState) {
            case LOWERING_LIFT:
                // Lower lift to bottom to load next ball from tunnel
                setLiftPosition(LIFT_DOWN_POSITION);
                setLiftGate(LIFT_GATE_CLOSED);
                
                // Check if lift has reached bottom position
                if (transferLift != null && 
                    Math.abs(transferLift.getCurrentPosition() - LIFT_DOWN_POSITION) <= LIFT_MOVE_TOLERANCE &&
                    !transferLift.isBusy()) {
                    // Move to loading state
                    currentBall++;
                    cycleState = BallCycleState.LOADING_BALL;
                    stateTimer.reset();
                    setLiftGate(LIFT_GATE_OPEN); // Open gate to allow ball from tunnel into lift
                    if (intakeWheel != null) {
                        intakeWheel.setPower(INTAKE_WHEEL_POWER); // Start intake wheel to push ball from tunnel
                    }
                }
                break;

            case LOADING_BALL:
                // Run intake wheel to push ball from tunnel into lift
                if (stateTimer.seconds() >= BALL_LOAD_TIME) {
                    // Ball should be loaded from tunnel, close gate and move to raising
                    cycleState = BallCycleState.RAISING_LIFT;
                    stateTimer.reset();
                    setLiftGate(LIFT_GATE_CLOSED); // Close gate to hold ball
                    if (intakeWheel != null) {
                        intakeWheel.setPower(0.0); // Stop intake wheel (other balls wait in tunnel)
                    }
                    setLiftPosition(LIFT_UP_POSITION); // Start raising lift
                }
                break;

            case RAISING_LIFT:
                // Raise lift to top for shooting
                setLiftPosition(LIFT_UP_POSITION);
                
                // Check if lift has reached top position
                if (transferLift != null && 
                    Math.abs(transferLift.getCurrentPosition() - LIFT_UP_POSITION) <= LIFT_MOVE_TOLERANCE &&
                    !transferLift.isBusy()) {
                    // Move to shooting state
                    cycleState = BallCycleState.SHOOTING;
                    stateTimer.reset();
                }
                break;

            case SHOOTING:
                // Keep lift at top, ball exits into shooter
                setLiftPosition(LIFT_UP_POSITION);
                
                // Wait for shot to complete, then move to next ball
                if (stateTimer.seconds() >= SHOT_WAIT_TIME) {
                    if (currentBall < MAX_BALLS) {
                        // More balls to shoot, lower lift for next ball from tunnel
                        cycleState = BallCycleState.LOWERING_LIFT;
                        stateTimer.reset();
                        setLiftPosition(LIFT_DOWN_POSITION);
                    } else {
                        // All 3 balls shot, stop cycle and open gate for next intake
                        cycleActive = false;
                        cycleState = BallCycleState.IDLE;
                        setLiftGate(LIFT_GATE_OPEN); // Open gate so next intake can load first ball
                    }
                }
                break;

            case IDLE:
                // Do nothing
                break;
        }
    }

    // Set lift to a specific encoder position
    private void setLiftPosition(int targetPosition) {
        if (transferLift != null) {
            transferLift.setTargetPosition(targetPosition);
            transferLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            transferLift.setPower(LIFT_POWER);
        }
    }

    // Set lift gate servo position (open/closed)
    private void setLiftGate(double position) {
        if (liftGateServo != null) {
            liftGateServo.setPosition(position);
        }
    }
}


