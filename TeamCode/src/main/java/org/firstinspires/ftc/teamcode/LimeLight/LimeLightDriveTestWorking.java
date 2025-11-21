//limelight drive works: //green and purple detection with PID and manual driving
package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import org.firstinspires.ftc.teamcode.dumbMap;
//import org.firstinspires.ftc.teamcode.pathing.Pose;

import java.util.List;

/**
 * Test OpMode for LimeLight driving with PID ball tracking and manual driving
 */
@TeleOp(name = "LimeLight Drive Test Working", group = "Test")
public class LimeLightDriveTestWorking extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    //private final Pose startPose = new Pose(0, 0, 0);
    private dumbMap dumbBot;

    // Limelight variables
    private Limelight3A limelight;
    private AprilTagDetector aprilTagDetector;
    private static final int TARGET_TAG_ID = 24; // AprilTag ID to detect

    // Camera parameters for distance calculation
    private static final double CAMERA_HEIGHT = 9.5; // inches - height of camera above ground
    private static final double CAMERA_ANGLE = 0.0;  // degrees - downward angle of camera
    private static final double MAX_DISTANCE = 150.0; // Maximum valid distance in inches

    // Control parameters
    private static final double TARGET_X = 0.0; // Target X position (center of screen)
    private static final double ROTATION_KP = 0.02; // Proportional constant for rotation (AprilTag)

    // Minimum area (in pixels^2) for ball detection to be considered valid
    private static final double MIN_GREEN_BALL_AREA = 0.5;  // Require at least 2% of screen
    private static final double MIN_PURPLE_BALL_AREA = 0.5;  // Require at least 2% of screen

    // Green ball tracking parameters - matched to reference code for faster response
    private static final double GREEN_BALL_KP = 0.02;     // Matches reference code kPBall_g
    private static final double GREEN_BALL_KI = 0.0;      // No integral term (like reference)
    private static final double GREEN_BALL_KD = 0.0;      // No derivative term (like reference)

    // Purple ball tracking parameters - matched to reference code for faster response
    private static final double PURPLE_BALL_KP = 0.02;    // Matches reference code kPBall_p
    private static final double PURPLE_BALL_KI = 0.0;     // No integral term (like reference)
    private static final double PURPLE_BALL_KD = 0.0;     // No derivative term (like reference)

    private static final double MIN_ROTATION_POWER = 0.10; // Matches reference minPowerBall
    private static final double TOLERANCE = 1.0;          // Degrees from target to consider aligned
    private static final double SLOW_ZONE = 3.0;           // Degrees where we start reducing speed
    private static final double DEADBAND = 0.5;           // Matches reference epsDriveDegBall
    private static final double SEARCH_ROTATION_POWER = 0.20; // Slow rotation speed when searching for tag
    private static final int MAX_LOST_FRAMES = 5; // Number of frames to wait before considering target lost

    // PID control state variables
    private double lastXAngle = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime = 0;
    private final double MAX_INTEGRAL = 0.5; // Prevent integral windup
    private final double SMOOTHING_FACTOR = 0.3; // Increased from 0.2 for more smoothing
    private double lastRotationPower = 0;
    private int lostFrames = 0;
    private boolean targetLocked = false;

    // Toggle and debounce variables
    private boolean alignmentActive = false;
    private boolean ballDetectionActive = false; // Whether ball detection is active
    private int currentPipeline = 0; // 0 = AprilTag, 1 = Green Ball, 2 = Purple Ball
    private boolean xButtonPreviousState = false;
    private boolean yButtonPreviousState = false;
    private long lastToggleTime = 0;
    private long lastGreenToggleTime = 0;
    private static final long DEBOUNCE_DELAY_MS = 150;
    private double searchStartAngle = 0;
    private boolean isSearching = false;

    @Override
    public void runOpMode() {
        // Initialize motors and components
        initializeMotors();

        // Initialize robot hardware (including Limelight with device discovery)
        dumbBot = new dumbMap(this);
        dumbBot.init2();  // This initializes the LimeLight with proper device discovery

        // Get the Limelight from dumbMap
        limelight = dumbBot.getLimeLight();

        // Initialize AprilTag detector if limelight is available
        if (limelight != null) {
            aprilTagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT, CAMERA_ANGLE, MAX_DISTANCE);

            // Display status
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Target Tag", "ID: %d", TARGET_TAG_ID);

            // Show pipeline information
            LLStatus status = limelight.getStatus();
            telemetry.addLine("\n=== Limelight Status ===");
            telemetry.addData("Pipeline", "%d (%s)",
                    status.getPipelineIndex(),
                    status.getPipelineType() != null ? status.getPipelineType() : "Unknown");
            telemetry.addData("Pipeline Name", status.getName() != null ? status.getName() : "Unnamed");

            // Check if we have a valid result
            LLResult result = limelight.getLatestResult();
            telemetry.addData("Has Valid Result", result != null && result.isValid());
        } else {
            telemetry.addData("Warning", "LimeLight not found!");
            telemetry.addData("Info", "Check hardware configuration and USB connection");
        }

        telemetry.update();

        // Wait for start - MUST BE CALLED BEFORE CHECKING opModeIsActive()
        waitForStart();

        // Set initial drive power
        dumbBot.drivePower = 0.5;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get gamepad inputs - EXACTLY as in reference code
            double forward = -gamepad1.left_stick_y;  // Invert Y axis of left stick
            double strafe = gamepad1.right_stick_x;    // RIGHT stick X for strafing
            double rotate = gamepad1.left_stick_x;    // LEFT stick X for rotation

            // Apply drive power scaling
            forward *= dumbBot.drivePower;
            strafe *= dumbBot.drivePower;
            rotate *= Math.abs(dumbBot.drivePower);

            // Handle X button toggle with edge detection and debounce
            boolean xButtonCurrentState = gamepad1.x;
            if (xButtonCurrentState && !xButtonPreviousState &&
                    (System.currentTimeMillis() - lastToggleTime > DEBOUNCE_DELAY_MS)) {
                alignmentActive = !alignmentActive; // Toggle state

                // Switch to AprilTag pipeline when enabling alignment
                if (alignmentActive) {
                    ballDetectionActive = false; // Turn off ball detection
                    currentPipeline = 0; // AprilTag pipeline
                    if (limelight != null) {
                        limelight.pipelineSwitch(0);
                    }
                }

                lastToggleTime = System.currentTimeMillis();
            }
            xButtonPreviousState = xButtonCurrentState;

            // Toggle ball detection mode with Y button (with debounce)
            // Cycle: Off -> Green Ball -> Purple Ball -> Off
            boolean yButtonCurrentState = gamepad1.y;
            if (yButtonCurrentState && !yButtonPreviousState &&
                    (System.currentTimeMillis() - lastGreenToggleTime > DEBOUNCE_DELAY_MS)) {

                if (!ballDetectionActive && currentPipeline == 0) {
                    // First press - enable green ball detection
                    ballDetectionActive = true;
                    currentPipeline = 1; // Green ball
                    alignmentActive = false; // Disable AprilTag alignment
                    isSearching = true;
                    searchStartAngle = 0;
                } else if (currentPipeline == 1) {
                    // Second press - switch to purple ball
                    ballDetectionActive = true; // Keep ball detection active!
                    currentPipeline = 2; // Purple ball
                    isSearching = true;
                } else {
                    // Third press - turn off ball detection
                    ballDetectionActive = false;
                    currentPipeline = 0; // AprilTag pipeline
                    alignmentActive = false;
                    isSearching = false;
                    setMotorPowers(0, 0, 0); // Stop any movement
                }

                // Switch pipeline if we're in a ball detection mode
                if (ballDetectionActive) {
                    limelight.pipelineSwitch(currentPipeline);
                    sleep(50); // Let the pipeline switch
                }

                lastGreenToggleTime = System.currentTimeMillis();
            }
            yButtonPreviousState = yButtonCurrentState;

            // Show current detection status
            String mode = "Manual";
            if (alignmentActive) {
                mode = "AprilTag";
            } else if (ballDetectionActive) {
                mode = (currentPipeline == 1) ? "Green Ball" : "Purple Ball";
            }
            telemetry.addData("Mode", mode);
            telemetry.addData("Press Y to cycle", "Current: " + mode);

            // If alignment is active, use Limelight to align
            if (alignmentActive) {
                alignToTarget();
            }
            // If ball detection is active, track the ball WITH MANUAL DRIVING
            else if (ballDetectionActive) {
                trackBallWithDrive(forward, strafe);
            }
            // Otherwise, use normal drive controls
            else {
                setMotorPowers(forward, strafe, rotate);
            }

            // Update telemetry
            telemetry.addData("Drive Power", "%.2f", dumbBot.drivePower);
            telemetry.addData("Forward/Back", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Rotate", "%.2f", rotate);
            telemetry.update();
        }
    }

    /**
     * Track ball with manual driving capability
     */
    private void trackBallWithDrive(double forward, double strafe) {
        // Process ball detection if active
        LLResult result = limelight.getLatestResult();
        boolean targetFound = false;

        // Get tx, ty, ta directly from LLResult (like working DriveTest.java)
        double txDeg = 0.0;
        double tyDeg = 0.0;
        double ta = 0.0;

        if (result != null) {
            txDeg = result.getTx();
            tyDeg = result.getTy();
            ta = result.getTa();
        }

        // Check if we have a valid detection with sufficient area
        double minArea = (currentPipeline == 1) ? MIN_GREEN_BALL_AREA : MIN_PURPLE_BALL_AREA;

        // Always show what area is being detected for debugging
        telemetry.addData("Ball ta (area %)", "%.2f (min: %.2f)", ta, minArea);

        if (ta >= minArea) {
            // Reset search state when target is found
            isSearching = false;
            lostFrames = 0;

            double xAngle = txDeg;

            // Stronger low-pass filter to reduce jitter
            xAngle = (0.5 * xAngle) + (0.5 * lastXAngle);
            lastXAngle = xAngle;

            telemetry.addLine("✅ " + (currentPipeline == 1 ? "Green" : "Purple") + " ball detected!");
            telemetry.addData("Ball tx (deg)", "%.2f", txDeg);
            telemetry.addData("Ball ty (deg)", "%.2f", tyDeg);
            telemetry.addData("Ball ta (area %)", "%.2f", ta);

            // Calculate time delta for integral and derivative terms
            long currentTime = System.currentTimeMillis();
            double deltaTime = (lastTime > 0) ? (currentTime - lastTime) / 1000.0 : 0.02; // Default to 50Hz if first run
            lastTime = currentTime;

            // Calculate error and apply deadband
            double error = (Math.abs(xAngle) < DEADBAND) ? 0 : xAngle;

            // Get PID constants based on ball color
            double kp = (currentPipeline == 1) ? GREEN_BALL_KP : PURPLE_BALL_KP;
            double ki = (currentPipeline == 1) ? GREEN_BALL_KI : PURPLE_BALL_KI;
            double kd = (currentPipeline == 1) ? GREEN_BALL_KD : PURPLE_BALL_KD;

            // Calculate integral term with anti-windup
            if (Math.abs(error) > 0.1) { // Only integrate when error is significant
                integralSum += error * deltaTime;
                // Anti-windup: limit the integral term
                integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
            } else {
                integralSum = 0; // Reset integral when close to target
            }

            // Calculate derivative term with smoothing
            double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
            derivative = lastXAngle * (1 - SMOOTHING_FACTOR) + derivative * SMOOTHING_FACTOR;

            // Calculate base PID output
            double basePower = (kp * error) + (ki * integralSum) + (kd * derivative);

            // Apply speed reduction in the slow zone (close to target)
            double rotationPower = basePower;
            if (Math.abs(error) < SLOW_ZONE) {
                // Scale down the power as we get closer to the target
                double speedFactor = 0.3 + (0.7 * (Math.abs(error) / SLOW_ZONE));
                rotationPower *= speedFactor;
            }

            // Apply minimum power threshold with sign preservation
            if (Math.abs(rotationPower) > 0 && Math.abs(rotationPower) < MIN_ROTATION_POWER) {
                rotationPower = Math.copySign(MIN_ROTATION_POWER, rotationPower);
            }

            // Limit max rotation power (matches reference code max of 0.7)
            rotationPower = Math.max(-0.7, Math.min(0.7, rotationPower));

            // Store values for next iteration
            lastError = error;

            // Only rotate if we're not already centered (within tolerance)
            if (Math.abs(xAngle) > TOLERANCE) {
                // Rotate to center the ball (inverted because of coordinate system)
                double finalPower = -rotationPower;
                // Allow manual driving: forward in pos 1, rotation in pos 2 (strafe param), strafe in pos 3 (rotate param)
                setMotorPowers(forward, finalPower, strafe);
                lastRotationPower = finalPower;
                targetLocked = true;
                telemetry.addData("Action", "Rotating %.2f power (P:%.3f, I:%.3f, D:%.3f)",
                        finalPower, kp*error, ki*integralSum, kd*derivative);
            } else {
                // Stop rotating when centered, but allow manual driving
                setMotorPowers(forward, 0, strafe);
                lastRotationPower = 0;
                integralSum = 0; // Reset integral when centered
                targetLocked = false;
                telemetry.addLine("✅ Centered on ball!");

                // Gentle rumble when locked on
                if (opModeIsActive() && !isStopRequested()) {
                    gamepad1.rumble(50);
                }
            }

            targetFound = true;
        } else {
            // Ball area too small, handle as lost
            handleBallLost();
        }

        // Handle case when no target is found
        if (!targetFound) {
            handleBallLost();
        }
    }

    /**
     * Initializes all drive motors with proper configuration
     */
    private void initializeMotors() {
        // Initialize motors with the correct names from your configuration
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions for mecanum drive
        // Flipping the back motors to match front direction
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);  // Flipped
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD); // Flipped

        // Set motor modes
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Aligns the robot to the target using Limelight and AprilTag detection
     */
    private void alignToTarget() {
        // Look specifically for the target tag
        AprilTagDetector.AprilTagResult tagResult = aprilTagDetector.getTagById(TARGET_TAG_ID);

        if (tagResult != null && tagResult.isValid) {
            // Display tag information
            telemetry.addLine("--- April Tag Detected ---");
            telemetry.addData("Tag ID", tagResult.tagId);
            telemetry.addData("Distance", "%.1f inches", tagResult.distance);
            telemetry.addData("Angle", "%.1f degrees", tagResult.angle);
            telemetry.addData("X Degrees", "%.1f°", tagResult.xDegrees);
            telemetry.addData("Y Degrees", "%.1f°", tagResult.yDegrees);
            telemetry.addLine("------------------------");

            // Calculate rotation power (proportional control)
            double tx = tagResult.xDegrees;
            double rotation = Math.max(Math.abs(tx) * ROTATION_KP, MIN_ROTATION_POWER);

            if (Math.abs(tx) < TOLERANCE) {
                rotation = 0; // Close enough to centered
                telemetry.addData("Action", "Aligned! Distance: %.1f\"", tagResult.distance);
            } else {
                // Determine direction to turn
                // Positive X = tag is to the right, so turn RIGHT (positive rotation)
                // Negative X = tag is to the left, so turn LEFT (negative rotation)
                String direction = tx > 0 ? "right" : "left";
                telemetry.addData("Action", "Turn %s %.1f°", direction, Math.abs(tx));

                // Apply correct rotation direction
                if (tx < 0) {
                    rotation = -rotation; // Turn left (negative rotation)
                } // else turn right (positive rotation)
            }

            // Set motor powers to rotate towards target
            // NOTE: Based on the motor configuration, strafe and rotate params are swapped
            // So we pass rotation as the 2nd parameter (strafe) to actually rotate
            // Invert rotation to match correct direction: negative X = left, positive X = right
            setMotorPowers(0, -rotation, 0);

        } else {
            // No target found - rotate slowly to search for it
            telemetry.addLine("April Tag 24 not detected");
            telemetry.addLine("Searching... (rotating slowly)");

            // Show LimeLight status for debugging
            if (limelight != null) {
                LLStatus status = limelight.getStatus();
                telemetry.addLine("\n=== Limelight Status ===");
                telemetry.addData("Pipeline", "%d (%s)",
                        status.getPipelineIndex(),
                        status.getPipelineType() != null ? status.getPipelineType() : "Unknown");
                telemetry.addData("Pipeline Name", status.getName() != null ? status.getName() : "Unnamed");
                telemetry.addData("Is Valid", limelight.getLatestResult() != null && limelight.getLatestResult().isValid());
            }

            // Rotate slowly to search for the AprilTag
            // Negative value rotates clockwise (to the right) to search
            setMotorPowers(0, -SEARCH_ROTATION_POWER, 0);
        }
    }

    /**
     * Handles the behavior when the ball is lost from view
     * - Gradually reduces rotation power
     * - Provides feedback in telemetry
     * - Resets state if ball isn't found after timeout
     */
    private void handleBallLost() {
        lostFrames++;

        // Reset PID terms when target is lost
        integralSum = 0;
        lastError = 0;

        // Initialize search if this is the first frame we lost the target
        if (!isSearching) {
            isSearching = true;
            searchStartAngle = 0; // This would ideally get the current robot heading
            telemetry.addLine("🔍 Starting 360° search...");
        }

        // If we're in search mode, rotate slowly
        if (isSearching) {
            // Use different search speeds for green vs purple to reduce overshooting
            double searchPower = (currentPipeline == 1) ? 0.15 : 0.12; // Reduced from 0.2/0.15

            // Add a small oscillation to help with detection
            double oscillation = 0.02 * Math.sin(System.currentTimeMillis() * 0.005);
            searchPower += oscillation;

            setMotorPowers(0, searchPower, 0);
            telemetry.addLine("🔍 Searching for " + (currentPipeline == 1 ? "green" : "purple") + " ball...");
        } else {
            // If we just lost the target, continue last rotation briefly with decay
            if (lostFrames < MAX_LOST_FRAMES && targetLocked) {
                // Apply exponential decay to the last rotation power
                double decayedPower = lastRotationPower * 0.7; // Faster decay (was 0.8)
                setMotorPowers(0, decayedPower, 0);
                lastRotationPower = decayedPower;
                telemetry.addLine("⚠️  Ball lost - continuing last rotation...");
            } else {
                // Stop after a few frames or if we weren't tracking
                setMotorPowers(0, 0, 0);
                targetLocked = false;
                lastRotationPower = 0;
                telemetry.addLine("⚠️  Ball lost - stopped");
            }
        }

        // If we've been searching too long, reset state
        if (lostFrames > MAX_LOST_FRAMES * 3) { // Increased from 2x to 3x
            lastXAngle = 0;
            lostFrames = 0;
            isSearching = false;
            telemetry.addLine("🔄 Resetting search state");
        }
    }

    /**
     * Sets the power for all drive motors using mecanum drive
     * @param forward Power for forward/backward movement (-1.0 to 1.0)
     * @param strafe Power for left/right strafing (-1.0 to 1.0)
     * @param rotate Power for rotation (-1.0 to 1.0)
     */
    private void setMotorPowers(double forward, double strafe, double rotate) {
        // Standard mecanum drive equations
        // For pure rotation (forward=0, strafe=0, rotate≠0):
        //   Left motors get -rotate, Right motors get +rotate
        // This makes the robot spin in place without translating
        double leftFrontPower = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightBackPower = forward + strafe - rotate;

        // Debug output for motor powers
        telemetry.addData("LF Power", "%4.2f", leftFrontPower);
        telemetry.addData("RF Power", "%4.2f", rightFrontPower);
        telemetry.addData("LB Power", "%4.2f", leftBackPower);
        telemetry.addData("RB Power", "%4.2f", rightBackPower);

        // Normalize the wheel speeds so no value exceeds 1.0
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

        // Set power for each motor
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
}

