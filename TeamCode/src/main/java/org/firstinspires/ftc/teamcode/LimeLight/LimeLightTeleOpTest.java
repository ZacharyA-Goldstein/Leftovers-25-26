package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.dumbMapLime;
import org.firstinspires.ftc.teamcode.pathing.Follower;
import org.firstinspires.ftc.teamcode.pathing.Pose;

/**
 * TeleOp mode with ball tracking that disables rotation when driver moves forward
 * This allows the driver to approach the ball without the robot spinning
 */
@TeleOp(name = "LimeLight TeleOp Test", group = "Test")
public class LimeLightTeleOpTest extends LinearOpMode {
    private Follower follower;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private final Pose startPose = new Pose(0, 0, 0);
    private dumbMapLime dumbBot;

    // Limelight variables
    private Limelight3A limelight;
    private AprilTagDetector aprilTagDetector;
    private static final int TARGET_TAG_ID = 24;

    // Camera parameters
    private static final double CAMERA_HEIGHT = 9.5;
    private static final double CAMERA_ANGLE = 0.0;
    private static final double MAX_DISTANCE = 150.0;

    // Control parameters
    private static final double TARGET_X = 0.0;
    private static final double ROTATION_KP = 0.025;  // Reduced from 0.04 for smoother AprilTag tracking
    
    // Ball detection thresholds - low threshold to detect early, manual control until auto-assist threshold
    private static final double MIN_GREEN_BALL_AREA = 0.1;  // Detect ball early at 0.1%
    private static final double MIN_PURPLE_BALL_AREA = 0.1; // Detect ball early at 0.1%
    
    // Ball tracking parameters - reduced KP for smoother tracking
    private static final double GREEN_BALL_KP = 0.025;  // Reduced from 0.04 for smoother rotation
    private static final double GREEN_BALL_KI = 0.0;
    private static final double GREEN_BALL_KD = 0.0;
    
    private static final double PURPLE_BALL_KP = 0.025;  // Reduced from 0.04 for smoother rotation
    private static final double PURPLE_BALL_KI = 0.0;
    private static final double PURPLE_BALL_KD = 0.0;
    
    private static final double MIN_ROTATION_POWER = 0.10;  // Reduced from 0.15 for gentler movements
    private static final double TOLERANCE = 1.0;
    private static final double SLOW_ZONE = 3.0;
    private static final double DEADBAND = 0.5;
    private static final double SEARCH_ROTATION_POWER = 0.20;  // Reduced from 0.35 for more controlled search
    private static final int MAX_LOST_FRAMES = 5;
    
    // Threshold for forward movement to disable rotation
    private static final double FORWARD_THRESHOLD = 0.15; // 15% forward stick = disable rotation
    
    // Auto-assist threshold - only auto-align when ball is very close
    private static final double AUTO_ASSIST_AREA = 0.5; // When ball area > 0.5%, provide auto-alignment assist
    
    // Close-range detection to handle blind spot
    private static final double CLOSE_RANGE_AREA = 1.75; // When ball area > 1.75%, we're very close (blind spot range)
    private static final long BLIND_SPOT_DRIVE_MS = 3000; // Continue driving for 3 seconds after losing ball at close range

    // PID control state
    private double lastXAngle = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime = 0;
    private final double MAX_INTEGRAL = 0.5;
    private final double SMOOTHING_FACTOR = 0.3;
    private double lastRotationPower = 0;
    private int lostFrames = 0;
    private boolean targetLocked = false;
    
    // NEW: Track close-range state
    private boolean wasCloseRange = false;
    private long lostAtCloseRangeTime = 0;
    private double lastKnownArea = 0;

    // Toggle and debounce
    private boolean alignmentActive = false;
    private boolean ballDetectionActive = false;
    private int currentPipeline = 0;
    private boolean xButtonPreviousState = false;
    private boolean yButtonPreviousState = false;
    private long lastToggleTime = 0;
    private long lastGreenToggleTime = 0;
    private static final long DEBOUNCE_DELAY_MS = 150;
    private double searchStartAngle = 0;
    private boolean isSearching = false;

    @Override
    public void runOpMode() {
        initializeMotors();

        dumbBot = new dumbMapLime(this);
        dumbBot.init2();
        limelight = dumbBot.getLimeLight();

        if (limelight != null) {
            aprilTagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT, CAMERA_ANGLE, MAX_DISTANCE);
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Target Tag", "ID: %d", TARGET_TAG_ID);
            
            LLStatus status = limelight.getStatus();
            telemetry.addLine("\n=== Limelight Status ===");
            telemetry.addData("Pipeline", "%d (%s)",
                    status.getPipelineIndex(),
                    status.getPipelineType() != null ? status.getPipelineType() : "Unknown");
            telemetry.addData("Pipeline Name", status.getName() != null ? status.getName() : "Unnamed");
            
            LLResult result = limelight.getLatestResult();
            telemetry.addData("Has Valid Result", result != null && result.isValid());
        } else {
            telemetry.addData("Warning", "LimeLight not found!");
        }

        telemetry.update();
        waitForStart();
        dumbBot.drivePower = 0.5;

        while (opModeIsActive()) {
            // Get gamepad inputs
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Apply drive power scaling
            forward *= dumbBot.drivePower;
            strafe *= dumbBot.drivePower;
            rotate *= Math.abs(dumbBot.drivePower);

            // Handle X button (AprilTag)
            boolean xButtonCurrentState = gamepad1.x;
            if (xButtonCurrentState && !xButtonPreviousState &&
                    (System.currentTimeMillis() - lastToggleTime > DEBOUNCE_DELAY_MS)) {
                alignmentActive = !alignmentActive;
                
                if (alignmentActive) {
                    ballDetectionActive = false;
                    currentPipeline = 0;
                    if (limelight != null) {
                        limelight.pipelineSwitch(0);
                    }
                }
                
                lastToggleTime = System.currentTimeMillis();
            }
            xButtonPreviousState = xButtonCurrentState;

            // Handle Y button (Ball detection toggle)
            boolean yButtonCurrentState = gamepad1.y;
            if (yButtonCurrentState && !yButtonPreviousState && 
                (System.currentTimeMillis() - lastGreenToggleTime > DEBOUNCE_DELAY_MS)) {
                
                if (!ballDetectionActive && currentPipeline == 0) {
                    // First press - green ball (passive detection, no auto-rotation)
                    ballDetectionActive = true;
                    currentPipeline = 1;
                    alignmentActive = false;
                    isSearching = false; // Don't search - just detect passively
                } else if (currentPipeline == 1) {
                    // Second press - purple ball (passive detection, no auto-rotation)
                    ballDetectionActive = true;
                    currentPipeline = 2;
                    isSearching = false; // Don't search - just detect passively
                } else {
                    // Third press - off
                    ballDetectionActive = false;
                    currentPipeline = 0;
                    alignmentActive = false;
                    isSearching = false;
                    setMotorPowers(0, 0, 0);
                }
                
                if (ballDetectionActive) {
                    limelight.pipelineSwitch(currentPipeline);
                    sleep(50);
                }
                
                lastGreenToggleTime = System.currentTimeMillis();
            }
            yButtonPreviousState = yButtonCurrentState;
            
            // Show mode
            String mode = "Manual";
            if (alignmentActive) {
                mode = "AprilTag Alignment";
            } else if (ballDetectionActive) {
                String ballColor = (currentPipeline == 1) ? "Green" : "Purple";
                mode = ballColor + " Ball Detection (Passive)";
            }
            telemetry.addData("🎮 Mode", mode);
            telemetry.addData("Press Y to cycle", "Green → Purple → Off");

            // Execute mode
            if (alignmentActive) {
                alignToTarget();
            } else if (ballDetectionActive) {
                // NEW: Pass forward value to check if driver is approaching
                trackBallWithDrive(forward, strafe);
            } else {
                setMotorPowers(forward, strafe, rotate);
            }

            telemetry.addData("Drive Power", "%.2f", dumbBot.drivePower);
            telemetry.addData("Forward/Back", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Rotate", "%.2f", rotate);
            telemetry.update();
        }
    }

    /**
     * Track ball with manual driving - rotation disabled when moving forward
     */
    private void trackBallWithDrive(double forward, double strafe) {
        LLResult result = limelight.getLatestResult();
        boolean targetFound = false;
        
        double txDeg = 0.0;
        double tyDeg = 0.0;
        double ta = 0.0;
        
        if (result != null) {
            txDeg = result.getTx();
            tyDeg = result.getTy();
            ta = result.getTa();
        }
        
        double minArea = (currentPipeline == 1) ? MIN_GREEN_BALL_AREA : MIN_PURPLE_BALL_AREA;
        telemetry.addData("Ball ta (area %)", "%.2f (min: %.2f)", ta, minArea);
        
        if (ta >= minArea) {
            isSearching = false;
            lostFrames = 0;
            lastKnownArea = ta;
            
            // NEW: Check if we're at close range (ball is very large in view)
            if (ta >= CLOSE_RANGE_AREA) {
                wasCloseRange = true;
                telemetry.addData("🎯 CLOSE RANGE", "Ball area: %.1f%%", ta);
            }
            
            double xAngle = txDeg;
            xAngle = (0.5 * xAngle) + (0.5 * lastXAngle);
            lastXAngle = xAngle;
            
            telemetry.addLine("✅ " + (currentPipeline == 1 ? "Green" : "Purple") + " ball detected!");
            telemetry.addData("Ball tx (deg)", "%.2f", txDeg);
            telemetry.addData("Ball ty (deg)", "%.2f", tyDeg);
            telemetry.addData("Ball ta (area %)", "%.2f", ta);
            
            // NEW: Driver-assist mode - only auto-align when ball is very close
            boolean needsAutoAssist = ta >= AUTO_ASSIST_AREA;
            boolean atCloseRange = ta >= CLOSE_RANGE_AREA;
            
            if (needsAutoAssist) {
                // Ball is close enough - provide AUTO-ASSIST to help driver align
                telemetry.addLine("🎯 AUTO-ASSIST ACTIVE - helping align!");
                telemetry.addData("Ball area", "%.2f%% (> %.2f%% threshold)", ta, AUTO_ASSIST_AREA);
                
                // Calculate subtle auto-alignment correction
                double error = (Math.abs(xAngle) < DEADBAND) ? 0 : xAngle;
                double kp = (currentPipeline == 1) ? GREEN_BALL_KP : PURPLE_BALL_KP;
                
                // Calculate rotation correction (subtle, not full power)
                double rotationCorrection = -kp * error;
                
                // Limit correction power to be subtle (max 30% power)
                rotationCorrection = Math.max(-0.3, Math.min(0.3, rotationCorrection));
                
                // Apply driver's manual control + subtle auto-correction
                // Driver has full forward/strafe control, robot adds rotation correction
                setMotorPowers(forward, strafe, rotationCorrection);
                
                telemetry.addData("Driver Control", "Forward: %.2f, Strafe: %.2f", forward, strafe);
                telemetry.addData("Auto-Correction", "Rotation: %.2f (error: %.1f°)", rotationCorrection, error);
                
                if (Math.abs(error) < TOLERANCE) {
                    telemetry.addLine("✅ Ball centered!");
                    if (opModeIsActive() && !isStopRequested()) {
                        gamepad1.rumble(50);
                    }
                }
                
                targetLocked = true;
            } else {
                // Ball detected but not close enough - FULL MANUAL CONTROL
                setMotorPowers(forward, strafe, 0);
                telemetry.addLine("🎮 MANUAL CONTROL - drive closer for auto-assist");
                telemetry.addData("Ball area", "%.2f%% (need %.2f%% for assist)", ta, AUTO_ASSIST_AREA);
                targetLocked = false;
                integralSum = 0;
                lastRotationPower = 0;
            }
            
            targetFound = true;
        } else {
            handleBallLost(forward, strafe);
        }
        
        if (!targetFound) {
            handleBallLost(forward, strafe);
        }
    }

    private void initializeMotors() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void alignToTarget() {
        AprilTagDetector.AprilTagResult tagResult = aprilTagDetector.getTagById(TARGET_TAG_ID);

        if (tagResult != null && tagResult.isValid) {
            telemetry.addLine("--- April Tag Detected ---");
            telemetry.addData("Tag ID", tagResult.tagId);
            telemetry.addData("Distance", "%.1f inches", tagResult.distance);
            telemetry.addData("Angle", "%.1f degrees", tagResult.angle);
            telemetry.addData("X Degrees", "%.1f°", tagResult.xDegrees);
            telemetry.addData("Y Degrees", "%.1f°", tagResult.yDegrees);
            telemetry.addLine("------------------------");

            double tx = tagResult.xDegrees;
            double rotation = Math.max(Math.abs(tx) * ROTATION_KP, MIN_ROTATION_POWER);

            if (Math.abs(tx) < TOLERANCE) {
                rotation = 0;
                telemetry.addData("Action", "Aligned! Distance: %.1f\"", tagResult.distance);
            } else {
                String direction = tx > 0 ? "right" : "left";
                telemetry.addData("Action", "Turn %s %.1f°", direction, Math.abs(tx));

                if (tx < 0) {
                    rotation = -rotation;
                }
            }

            setMotorPowers(0, -rotation, 0);

        } else {
            telemetry.addLine("April Tag 24 not detected");
            telemetry.addLine("Searching... (rotating slowly)");

            if (limelight != null) {
                LLStatus status = limelight.getStatus();
                telemetry.addLine("\n=== Limelight Status ===");
                telemetry.addData("Pipeline", "%d (%s)",
                        status.getPipelineIndex(),
                        status.getPipelineType() != null ? status.getPipelineType() : "Unknown");
                telemetry.addData("Pipeline Name", status.getName() != null ? status.getName() : "Unnamed");
                telemetry.addData("Is Valid", limelight.getLatestResult() != null && limelight.getLatestResult().isValid());
            }

            setMotorPowers(0, -SEARCH_ROTATION_POWER, 0);
        }
    }

    private void handleBallLost(double forward, double strafe) {
        lostFrames++;
        integralSum = 0;
        lastError = 0;
        targetLocked = false;
        lastRotationPower = 0;
        
        // NEW: Check if we lost the ball at close range (blind spot)
        if (wasCloseRange && lastKnownArea >= CLOSE_RANGE_AREA) {
            // Ball was very close - likely in blind spot below camera
            if (lostAtCloseRangeTime == 0) {
                lostAtCloseRangeTime = System.currentTimeMillis();
            }
            
            long timeSinceLost = System.currentTimeMillis() - lostAtCloseRangeTime;
            
            if (timeSinceLost < BLIND_SPOT_DRIVE_MS) {
                // Apply manual control - driver can drive to pick up ball
                setMotorPowers(forward, strafe, 0);
                telemetry.addLine("🎯 BLIND SPOT - Ball likely below camera!");
                telemetry.addData("Last known area", "%.1f%%", lastKnownArea);
                telemetry.addData("Time in blind spot", "%d ms", timeSinceLost);
                telemetry.addLine("💡 Drive forward to pick up ball!");
                return;
            } else {
                // Been too long, reset
                wasCloseRange = false;
                lostAtCloseRangeTime = 0;
            }
        }
        
        // NO AUTO-SEARCH! Apply manual control inputs
        isSearching = false;
        setMotorPowers(forward, strafe, 0); // Full manual control with no rotation
        telemetry.addLine("⚠️ Ball not detected - manual control");
        telemetry.addData("Ball area needed", ">= %.1f%% for detection", MIN_GREEN_BALL_AREA);
        telemetry.addData("Manual inputs", "Forward: %.2f, Strafe: %.2f", forward, strafe);
    }

    private void setMotorPowers(double forward, double strafe, double rotate) {
        double leftFrontPower = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightBackPower = forward + strafe - rotate;

        telemetry.addData("LF Power", "%4.2f", leftFrontPower);
        telemetry.addData("RF Power", "%4.2f", rightFrontPower);
        telemetry.addData("LB Power", "%4.2f", leftBackPower);
        telemetry.addData("RB Power", "%4.2f", rightBackPower);

        double maxPower = Math.max(Math.max(
                Math.abs(leftFrontPower),
                Math.abs(rightFrontPower)),
                Math.max(
                        Math.abs(leftBackPower),
                        Math.abs(rightBackPower)
                )
        );

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
}
