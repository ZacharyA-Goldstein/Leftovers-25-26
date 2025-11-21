package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.dumbMap;
//import org.firstinspires.ftc.teamcode.pathing.Follower;
//import org.firstinspires.ftc.teamcode.pathing.Pose;

import java.util.List;

/**
 * Test OpMode for LimeLight driving
 */

@TeleOp(name = "LimeLight Drive Test", group = "Test")
public class LimeLightDriveTest extends LinearOpMode {
    //private Follower follower;
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
    private static final double BALL_ROTATION_KP = 0.01; // Proportional constant for ball tracking
    private static final double BALL_ROTATION_KD = 0.002; // Derivative constant for smoother tracking
    private static final double MIN_ROTATION_POWER = 0.15; // Minimum power to overcome friction
    private static final double TOLERANCE = 1.5; // Degrees tolerance for alignment
    private static final double SEARCH_ROTATION_POWER = 0.20; // Slow rotation speed when searching for tag
    private static final int MAX_LOST_FRAMES = 5; // Number of frames to wait before considering target lost
    
    // Tracking state variables
    private double lastXAngle = 0;
    private double lastRotationPower = 0;
    private int lostFrames = 0;
    private boolean targetLocked = false;
    
    // Toggle and debounce variables
    private boolean alignmentActive = false;
    private boolean greenBallMode = false;  // Toggle for green ball detection
    private boolean xButtonPreviousState = false; // Track previous button state for edge detection
    private boolean yButtonPreviousState = false; // Track Y button state for toggling green ball mode
    private long lastToggleTime = 0;
    private long lastGreenToggleTime = 0; // Track last time green ball mode was toggled
    private static final long DEBOUNCE_DELAY_MS = 150; // 150ms debounce delay
    
    @Override
    public void runOpMode() {
        // Initialize motors and components
        initializeMotors();
        
        // Initialize robot hardware (including Limelight with device discovery)
        dumbBot = new dumbMap(this);
        dumbBot.init2();  // This initializes the LimeLight with proper device discovery
        
        // Get the Limelight from dumbMap
        limelight = dumbBot.getLimeLight();
        
        // Initialize telemetry
        telemetry.setMsTransmissionInterval(50); // 20Hz update rate
        telemetry.addLine("LimeLight Drive Test");
        telemetry.addLine("Press X to toggle AprilTag alignment");
        telemetry.addLine("Left stick: Forward/Back, Right stick X: Strafe, Right stick Y: Rotate");
        
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
        
        // Now check if we can continue
        if (limelight == null) {
            telemetry.addData("Error", "Cannot run without LimeLight");
            telemetry.update();
            // Keep OpMode running so error is visible
            while (opModeIsActive()) {
                idle();
            }
            return;
        }
        
        // Set initial drive power
        dumbBot.drivePower = 0.5;
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get gamepad inputs
            // Invert forward direction to match expected behavior
            double forward = -gamepad1.left_stick_y;  // Negative because of motor directions
            double strafe = gamepad1.right_stick_x;     // is strafe but for some reason on the right stick??? Is actually on left
            double rotate = gamepad1.left_stick_x;    // IS rotation but for some reason left stick, is on right stick in actuality
            
            // Apply drive power scaling
            forward *= dumbBot.drivePower;
            strafe *= dumbBot.drivePower;
            rotate *= Math.abs(dumbBot.drivePower);
            
            // Handle X button toggle with edge detection and debounce
            // Only toggle when button transitions from not pressed to pressed
            boolean xButtonCurrentState = gamepad1.x;
            if (xButtonCurrentState && !xButtonPreviousState && 
                (System.currentTimeMillis() - lastToggleTime > DEBOUNCE_DELAY_MS)) {
                alignmentActive = !alignmentActive; // Toggle state
                lastToggleTime = System.currentTimeMillis();
            }
            xButtonPreviousState = xButtonCurrentState; // Update previous state
            
            // ALWAYS allow manual rotation when right stick X is used, regardless of alignment state
            if (Math.abs(rotate) > 0.1) {
                // Manual rotation takes full control - disable alignment during manual rotation
                setMotorPowers(forward, strafe, rotate);
                targetLocked = false; // Reset alignment state
                lastRotationPower = 0;
            } 
            // If not manually rotating and alignment is active, use Limelight alignment
            else if (alignmentActive) {
                alignToTarget();
            } 
            // If not manually rotating and alignment is not active, just drive normally
            else {
                setMotorPowers(forward, strafe, 0);
            }
            
            // Display alignment status
            telemetry.addData("Alignment", alignmentActive ? "ACTIVE" : "INACTIVE");
            
            // Toggle green ball detection mode with Y button (with debounce)
            boolean yButtonCurrentState = gamepad1.y;
            if (yButtonCurrentState && !yButtonPreviousState && 
                (System.currentTimeMillis() - lastGreenToggleTime > DEBOUNCE_DELAY_MS)) {
                greenBallMode = !greenBallMode; // Toggle state
                lastGreenToggleTime = System.currentTimeMillis();
                
                // Switch pipeline based on mode
                limelight.pipelineSwitch(greenBallMode ? 1 : 0);
                
                // Give the camera time to switch pipelines
                sleep(50);
            }
            yButtonPreviousState = yButtonCurrentState;
            
            // Show green ball detection status
            telemetry.addData("Green Ball Mode", greenBallMode ? "ACTIVE" : "INACTIVE");
            
            // Process green ball detection if active AND not manually rotating
            if (greenBallMode && Math.abs(rotate) < 0.1) {  // Only track ball if not manually rotating
                LLResult result = limelight.getLatestResult();
                
                if (result != null && result.isValid()) {
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    
                    if (!colorResults.isEmpty()) {
                        // Reset lost frames counter when we see the target
                        lostFrames = 0;
                        
                        // Get the largest detected object (most likely the ball)
                        LLResultTypes.ColorResult colorObj = colorResults.get(0);
                        double xAngle = colorObj.getTargetXDegrees();
                        
                        // Simple low-pass filter to smooth angle readings
                        xAngle = (0.7 * xAngle) + (0.3 * lastXAngle);
                        lastXAngle = xAngle;
                        
                        telemetry.addLine("✅ Color object detected!");
                        telemetry.addData("X Angle", "%.1f°", xAngle);
                        telemetry.addData("Y Angle", "%.1f°", colorObj.getTargetYDegrees());
                        
                        // Calculate rotation power (proportional + derivative control)
                        double error = xAngle;
                        double derivative = (error - lastXAngle) / 0.02; // 50Hz loop = ~20ms per cycle
                        double rotationPower = (error * BALL_ROTATION_KP) + (derivative * BALL_ROTATION_KD);
                        
                        // Apply minimum power if we need to move
                        if (Math.abs(rotationPower) > 0 && Math.abs(rotationPower) < MIN_ROTATION_POWER) {
                            rotationPower = Math.copySign(MIN_ROTATION_POWER, rotationPower);
                        }
                        
                        // Limit max rotation power
                        rotationPower = Math.max(-0.5, Math.min(0.5, rotationPower));
                        
                        // Only rotate if we're not already centered (within tolerance)
                        if (Math.abs(xAngle) > TOLERANCE) {
                            // Use negative because positive X means turn right (clockwise)
                            setMotorPowers(0, -rotationPower, 0);
                            lastRotationPower = -rotationPower;
                            targetLocked = true;
                            telemetry.addData("Rotating", "%.2f power", -rotationPower);
                        } else {
                            // Stop rotating when centered
                            setMotorPowers(0, 0, 0);
                            lastRotationPower = 0;
                            targetLocked = false;
                            telemetry.addLine("✅ Centered on ball!");
                        }
                        
                        // Visual feedback - gentle rumble when locked on
                        if (opModeIsActive() && !isStopRequested() && Math.abs(xAngle) <= TOLERANCE) {
                            gamepad1.rumble(50);
                        }
                        
                    } else {
                        handleBallLost();
                    }
                } else {
                    handleBallLost();
                }
            }
            
            // Add telemetry for debugging
            telemetry.addData("Drive Power", "%4.2f", dumbBot.drivePower);
            telemetry.addData("Forward/Back", "%4.2f", forward);
            telemetry.addData("Strafe", "%4.2f", strafe);
            telemetry.addData("Rotate", "%4.2f", rotate);
            telemetry.update();
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
        telemetry.addLine(lostFrames > MAX_LOST_FRAMES ? 
            "❌ Ball lost - waiting..." : "⚠️  Ball lost - searching...");
        
        // If we just lost the target, continue last rotation briefly
        if (lostFrames < MAX_LOST_FRAMES && targetLocked) {
            // Apply slight decay to the last rotation power
            double decayedPower = lastRotationPower * 0.8;
            setMotorPowers(0, decayedPower, 0);
        } else {
            // Stop after a few frames or if we weren't tracking
            setMotorPowers(0, 0, 0);
            targetLocked = false;
            lastRotationPower = 0;
        }
        
        // If we've been searching too long, reset state
        if (lostFrames > MAX_LOST_FRAMES * 2) {
            lastXAngle = 0;
            lostFrames = 0;
        }
    }
    
    private void initializeMotors() {
        // Initialize motors with the correct names from your configuration
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        
        // Initialize follower with hardware map
        //follower = new Follower(hardwareMap);
        
        // Set motor directions for mecanum drive
        // Flipping the back motors to match front direction
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);  // Flipped
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD); // Flipped

        // Set motor modes
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets the power for all drive motors using mecanum drive
     * @param forward Power for forward/backward movement (-1.0 to 1.0)
     * @param strafe Power for left/right strafing (-1.0 to 1.0)
     * @param rotate Power for rotation (-1.0 to 1.0)
     */
    private void setMotorPowers(double forward, double strafe, double rotate) {
        // Standard mecanum drive equations
        // For a standard mecanum drive:
        // - Forward/backward: All motors get +forward
        // - Strafe left/right: Left front/right back get +strafe, right front/left back get -strafe
        // - Rotate: Left side gets -rotate, right side gets +rotate
        double leftFrontPower = forward + strafe - rotate;  // Was: forward + strafe + rotate
        double rightFrontPower = forward - strafe + rotate; // Was: forward - strafe - rotate
        double leftBackPower = forward - strafe - rotate;   // Was: forward - strafe + rotate
        double rightBackPower = forward + strafe + rotate;  // Was: forward + strafe - rotate
        
        // Apply motor direction corrections if needed (uncomment and adjust if necessary)
        // leftFrontPower *= 1.0;
        // rightFrontPower *= 1.0;
        // leftBackPower *= 1.0;
        // rightBackPower *= 1.0;
        
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


    /**
     * Aligns the robot to the target using Limelight and AprilTag detection
     */
    private void alignToTarget() {
        // Look specifically for the target tag
        AprilTagDetector.AprilTagResult tagResult = aprilTagDetector.getTagById(TARGET_TAG_ID);
        
        if (tagResult.isValid) {
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
}