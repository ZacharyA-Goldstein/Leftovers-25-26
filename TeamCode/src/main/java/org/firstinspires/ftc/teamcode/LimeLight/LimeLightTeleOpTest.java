package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * TeleOp mode with ball tracking that disables rotation when driver moves forward
 * This allows the driver to approach the ball without the robot spinning
 */
@TeleOp(name = "LimeLight TeleOp Test", group = "Test")
public class LimeLightTeleOpTest extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private Limelight3A limelight;
    
    // Detection constants
    private static final double MIN_BALL_AREA = 0.1;    // Minimum area % to detect ball (very sensitive)
    private static final double AUTO_ASSIST_AREA = 0.5;  // Area % to start auto-assist
    private static final double CLOSE_RANGE_AREA = 1.75; // Ball is very close (blind spot range)
    private static final long BALL_LOCK_TIME_MS = 500;   // Time to keep tracking same ball
    
    // Ball tracking state
    private double lastBallX = 0;
    private double lastBallY = 0;
    private double lastBallArea = 0;
    private long lastBallTime = 0;
    private boolean wasCloseRange = false;
    private long lostAtCloseRangeTime = 0;
    private static final long BLIND_SPOT_DRIVE_MS = 3000; // Continue driving for 3s after losing ball at close range
    
    // Ball tracking parameters
    private static final double BALL_KP = 0.025;        // Reduced for smoother rotation
    private static final double BALL_DEADBAND = 0.5;     // Degrees of deadband for centering
    private static final double TOLERANCE = 1.0;         // Degrees of tolerance for centered
    
    // AprilTag constants
    private static final double TAG_TARGET_DISTANCE = 12.0; // Target distance in inches
    private static final double TAG_DISTANCE_KP = 0.01;    // Forward speed based on distance
    private static final double TAG_ALIGN_KP = 0.008;      // Reduced rotation sensitivity for AprilTag
    private static final double TAG_MAX_ROTATION = 0.4;    // Maximum rotation power for AprilTag
    private static final double TAG_DEADBAND = 0.5;        // Degrees of deadband for AprilTag centering
    private static final double MIN_TAG_AREA = 0.5;        // Minimum area to consider a valid tag
    
    // Pipeline states
    private enum BallPipeline {
        GREEN(1),  // Green ball detection
        PURPLE(2), // Purple ball detection
        OFF(0);    // AprilTag detection
        
        final int pipelineIndex;
        BallPipeline(int index) { this.pipelineIndex = index; }
    }
    
    private BallPipeline currentPipeline = BallPipeline.OFF;
    private boolean wasYButtonPressed = false;
    private boolean wasXButtonPressed = false;
    private boolean aprilTagMode = false;
    private static final int APRILTAG_PIPELINE = 0;
    
    // Search mode
    private boolean isSearching = false;
    private static final double SEARCH_POWER = 0.75;
    private static final long SEARCH_TIMEOUT_MS = 5000;
    private long searchStartTime = 0;

    private void handleXToggle() {
        boolean xButton = gamepad1.x;
        
        // Check for X button press (with debounce)
        if (xButton && !wasXButtonPressed) {
            aprilTagMode = !aprilTagMode;
            
            if (aprilTagMode) {
                // Switch to AprilTag pipeline
                if (limelight != null) {
                    limelight.pipelineSwitch(APRILTAG_PIPELINE);
                    telemetry.addLine("Switched to AprilTag mode");
                }
            } else {
                // Return to previous ball detection mode
                if (limelight != null) {
                    limelight.pipelineSwitch(currentPipeline.pipelineIndex);
                    telemetry.addLine("Returned to " + currentPipeline + " ball detection");
                }
            }
            
            // Reset search state when changing modes
            isSearching = false;
        }
        wasXButtonPressed = xButton;
    }
    
    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing...");
        
        // Initialize motors first
        initializeMotors();
        
        telemetry.addLine("Controls:");
        telemetry.addLine("- Left Stick: Drive/Strafe");
        telemetry.addLine("- Right Stick: Manual Rotate");
        telemetry.addLine("- Y: Toggle Ball Detection (Green/Purple/Off)");
        telemetry.addLine("- X: Toggle AprilTag Mode");
        telemetry.addLine("\nWaiting for start...");
        telemetry.update();
        
        // Wait for the start button to be pressed
        waitForStart();
        
        // Now initialize the Limelight after start is pressed
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();
        
        initLimelight();
        
        if (limelight == null) {
            telemetry.addData("ERROR", "❌ Limelight not found!");
            telemetry.addLine("Please check the following:");
            telemetry.addLine("1. Is the Limelight connected to the Robot Controller?");
            telemetry.addLine("2. Is the Limelight powered on?");
            telemetry.addLine("3. Is the correct device name configured in the hardware map?");
            telemetry.update();
            
            // Keep the OpMode running to show the error message
            while (opModeIsActive()) {
                idle();
            }
            return;
        }
        
        while (opModeIsActive()) {
            // Get gamepad inputs
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            
            // Handle button presses
            handleYToggle();
            handleXToggle();
            
            // Handle tracking based on current mode
            if (aprilTagMode) {
                trackAprilTag(forward, strafe, rotate);
            } else {
                trackBall(forward, strafe, rotate);
            }
            
            // Update telemetry
            updateTelemetry();
            
            telemetry.update();
        }
    }
    
    private void handleYToggle() {
        boolean yButton = gamepad1.y;
        
        // Check for Y button press (with debounce)
        if (yButton && !wasYButtonPressed && limelight != null) {
            // Cycle through the pipeline states
            switch (currentPipeline) {
                case OFF:
                    currentPipeline = BallPipeline.GREEN;
                    limelight.pipelineSwitch(BallPipeline.GREEN.pipelineIndex);
                    telemetry.addLine("Switched to GREEN ball detection");
                    break;
                case GREEN:
                    currentPipeline = BallPipeline.PURPLE;
                    limelight.pipelineSwitch(BallPipeline.PURPLE.pipelineIndex);
                    telemetry.addLine("Switched to PURPLE ball detection");
                    break;
                case PURPLE:
                    currentPipeline = BallPipeline.OFF;
                    limelight.pipelineSwitch(APRILTAG_PIPELINE);
                    telemetry.addLine("Ball detection OFF (AprilTag mode)");
                    break;
            }
            
            // Reset search state when changing modes
            isSearching = false;
        }
        wasYButtonPressed = yButton;
    }
    
    private void trackBall(double forward, double strafe, double manualRotate) {
        // Get the current time for tracking
        long currentTime = System.currentTimeMillis();
        
        // If ball detection is off, just use manual controls
        if (currentPipeline == BallPipeline.OFF || limelight == null) {
            setMotorPowers(forward, strafe, manualRotate);
            telemetry.addLine("Ball detection OFF - using manual controls");
            return;
        }
        
        // Check for manual rotation input (right stick X axis)
        boolean isManualRotating = Math.abs(manualRotate) > 0.1;  // Small deadband
        
        // If user is manually rotating, use that input and skip ball tracking
        if (isManualRotating) {
            setMotorPowers(forward, strafe, manualRotate);
            telemetry.addLine("🎮 MANUAL ROTATION ACTIVE");
            telemetry.addData("Rotation Power", "%.2f", manualRotate);
            return;
        }
        
        // If we get here, no manual rotation - try to track the ball
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            telemetry.addLine("No ball detected - using manual controls");
            setMotorPowers(forward, strafe, 0);  // No rotation
            return;
        }
        
        double tx = result.getTx();
        double ty = result.getTy();
        double ta = result.getTa();
        
        // Make sure we have valid data
        if (Double.isNaN(tx) || Double.isNaN(ty) || Double.isNaN(ta)) {
            telemetry.addLine("Waiting for valid ball detection...");
            setMotorPowers(forward, strafe, 0);  // No rotation
            return;
        }
        
        // Update ball tracking state
        lastBallX = tx;
        lastBallY = ty;
        lastBallArea = ta;
        lastBallTime = currentTime;
        
        // Ball detection information
        telemetry.addLine("✅ Ball detected!");
        telemetry.addData("Ball tx (deg)", "%.2f", tx);
        telemetry.addData("Ball area", "%.2f%%", ta);
        
        // Check if we're at close range (ball is very large in view)
        if (ta >= CLOSE_RANGE_AREA) {
            wasCloseRange = true;
            telemetry.addLine("🎯 CLOSE RANGE - Ball area: " + String.format("%.1f%%", ta));
        }
        
        // Check if we should provide auto-assist (only when ball is close enough)
        boolean needsAutoAssist = ta >= AUTO_ASSIST_AREA;
        
        if (needsAutoAssist) {
            // Ball is close enough - provide auto-assist
            telemetry.addLine("🎯 AUTO-ASSIST ACTIVE - helping align!");
            
            // Calculate rotation correction
            double error = (Math.abs(tx) < BALL_DEADBAND) ? 0 : tx;
            double rotationCorrection = -BALL_KP * error;
            
            // Limit correction power to be subtle (max 30% power)
            rotationCorrection = Math.max(-0.3, Math.min(0.3, rotationCorrection));
            
            // Apply driver's forward/strafe with auto-rotation
            setMotorPowers(forward, strafe, rotationCorrection);
            
            telemetry.addData("Auto-Correction", "Rotation: %.2f (error: %.1f°)", rotationCorrection, error);
            
            if (Math.abs(error) < TOLERANCE) {
                telemetry.addLine("✅ Ball centered!");
            }
        } else {
            // Ball detected but not close enough for auto-assist
            // Still allow forward/strafe but no auto-rotation
            setMotorPowers(forward, strafe, 0);
            telemetry.addLine("🎮 MANUAL CONTROL - drive closer for auto-assist");
            telemetry.addData("Ball area", "%.2f%% (need %.2f%% for assist)", ta, AUTO_ASSIST_AREA);
        }
    }
    
    private void updateTelemetry() {
        telemetry.addLine("=== Current Mode ===");
        if (aprilTagMode) {
            telemetry.addLine("APRILTAG DETECTION");
            telemetry.addLine("Looking for AprilTag 24");
        } else {
            telemetry.addLine("BALL DETECTION: " + currentPipeline);
        }
        
        telemetry.addLine("\n=== Controls ===");
        telemetry.addLine("Left Stick: Drive/Strafe");
        telemetry.addLine("Right Stick: Manual Rotate");
        telemetry.addLine("Y: Toggle Ball Detection (Green/Purple/Off)");
        telemetry.addLine("X: Toggle AprilTag Mode");
    }
    
    private void trackAprilTag(double forward, double strafe, double manualRotate) {
        // Check if limelight is initialized
        if (limelight == null) {
            telemetry.addData("Error", "Limelight not initialized!");
            setMotorPowers(forward, strafe, manualRotate);
            return;
        }
        
        // Get AprilTag detection data with null check
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            telemetry.addData("Error", "No result from Limelight!");
            setMotorPowers(forward, strafe, manualRotate);
            return;
        }
        
        double tx = result.getTx();
        double ty = result.getTy();
        double ta = result.getTa();
        
        // Check if we have a valid AprilTag detection 100% of image)
        
        if (ta > MIN_TAG_AREA) {
            // We have a valid AprilTag detection
            double rotation = 0;
            
            // Only rotate if we're not centered (with deadband)
            if (Math.abs(tx) > TAG_DEADBAND) {
                // Calculate base rotation
                rotation = -tx * TAG_ALIGN_KP;
                
                // Apply non-linear scaling for better control
                double scaleFactor = Math.min(1.0, Math.abs(tx) / 15.0);
                rotation *= scaleFactor;
                
                // Clamp rotation to maximum
                rotation = Math.max(-TAG_MAX_ROTATION, Math.min(TAG_MAX_ROTATION, rotation));
                
                // Apply minimum power when rotating (but less than the minimum for ball tracking)
                if (Math.abs(rotation) > 0 && Math.abs(rotation) < 0.05) {
                    rotation = Math.copySign(0.05, rotation);
                }
            }
            
            // Set motor powers with auto-rotation (no manual override for AprilTag)
            setMotorPowers(forward, strafe, rotation);
            
            // Telemetry
            telemetry.addLine("=== AprilTag Tracking ===");
            telemetry.addData("Status", "LOCKED - TX: %.1f° Area: %.1f%%", tx, ta);
            telemetry.addData("Rotation", "Power: %.2f (Max: %.2f)", rotation, TAG_MAX_ROTATION);
            telemetry.addData("Controls", "Drive: (%.2f, %.2f)", forward, strafe);
        } else {
            // No AprilTag detected - spin to search for tag (no manual control)
            telemetry.addLine("🔍 Searching for AprilTag... (No tag detected)");
            
            // Start search mode if not already searching
            if (!isSearching) {
                isSearching = true;
                searchStartTime = System.currentTimeMillis();
            }
            
            // Check if search timeout has been reached
            long currentTime = System.currentTimeMillis();
            if (currentTime - searchStartTime < SEARCH_TIMEOUT_MS) {
                // Continue searching (rotate slowly)
                double searchDirection = Math.signum(SEARCH_POWER);
                setMotorPowers(0, 0, SEARCH_POWER * 0.75);
                
                // Calculate search progress
                double progress = (double)(currentTime - searchStartTime) / SEARCH_TIMEOUT_MS * 100.0;
                telemetry.addData("Searching...", "%.1f%% complete", progress);
            } else {
                // Search timeout reached, stop searching
                isSearching = false;
                telemetry.addLine("⚠️  Search complete - no AprilTag found");
                setMotorPowers(0, 0, 0);
            }
        }
    }
    
    private void initLimelight() {
        telemetry.addLine("🔍 Initializing Limelight...");
        telemetry.update();
        
        try {
            // List all connected devices for debugging
            telemetry.addLine("🔧 Checking hardware map for Limelight devices...");
            boolean foundDevices = false;
            
            // Get all device names of type Limelight3A
            for (String deviceName : hardwareMap.getAllNames(Limelight3A.class)) {
                telemetry.addData("✅ Found Limelight device", "Name: '%s'", deviceName);
                foundDevices = true;
            }
            
            if (!foundDevices) {
                telemetry.addLine("❌ No Limelight devices found in hardware map");
                telemetry.addLine("Checking for any connected USB/Network devices...");
                
                // Try to list all device names regardless of type
                telemetry.addLine("\n📋 List of all connected devices:");
                
                // List all devices by category
                telemetry.addLine("\n🔌 DC Motors:");
                for (String name : hardwareMap.dcMotor.entrySet().stream()
                        .map(entry -> entry.getKey() + " (" + entry.getValue().getDeviceName() + ")")
                        .toArray(String[]::new)) {
                    telemetry.addData("-", name);
                }
                
                telemetry.addLine("\n📷 Cameras:");
                for (String name : hardwareMap.getAllNames(WebcamName.class)) {
                    telemetry.addData("-", name);
                }
                
                telemetry.addLine("\n📡 Other devices:");
                telemetry.addLine("Use the Robot Controller app to verify Limelight configuration");
            }
            
            // Try to get the Limelight from hardware map with common names
            String[] possibleNames = {
                "limelight", "limelight3a", "limelight3A", 
                "limelight_3a", "limelight_3A", "limelight3a_1",
                "limelight3a_2", "limelight3a_3", "limelight3a_4"
            };
            
            telemetry.addLine("\n🔄 Trying to initialize Limelight...");
            
            for (String name : possibleNames) {
                try {
                    telemetry.addData("Trying name", "'%s'...", name);
                    telemetry.update();
                    
                    // Small delay to allow telemetry to update
                    sleep(100);
                    
                    limelight = hardwareMap.get(Limelight3A.class, name);
                    if (limelight != null) {
                        telemetry.addData("✅ Success!", "Found Limelight with name: '%s'", name);
                        telemetry.update();
                        sleep(500); // Give time to read the success message
                        break;
                    }
                } catch (Exception e) {
                    telemetry.addData("❌ Failed", "Name '%s': %s", name, e.getMessage());
                    telemetry.update();
                    sleep(100);
                }
            }
            
            // If still not found, try to get any Limelight device (last resort)
            if (limelight == null) {
                telemetry.addLine("\n⚠️  No Limelight found with standard names. Trying any available device...");
                telemetry.update();
                sleep(500);
                
                try {
                    // Try to get the first device of type Limelight3A
                    for (Limelight3A device : hardwareMap.getAll(Limelight3A.class)) {
                        limelight = device;
                        telemetry.addLine("✅ Found Limelight device (unnamed)");
                        break;
                    }
                } catch (Exception e) {
                    telemetry.addData("❌ Error", "Getting Limelight devices: %s", e.getMessage());
                }
            }

            // If we found a Limelight, initialize it
            if (limelight != null) {
                try {
                    telemetry.addLine("\n🚀 Initializing Limelight...");
                    telemetry.update();
                    
                    // Initialize the Limelight
                    limelight.start();
                    limelight.pipelineSwitch(APRILTAG_PIPELINE); // Start with AprilTag pipeline
                    
                    // Try to get a result to verify it's working
                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        telemetry.addLine("✅ Limelight initialized successfully!");
                        telemetry.addData("Pipeline", "%d (AprilTag)", APRILTAG_PIPELINE);
                        telemetry.addData("Connection", "Active");
                    } else {
                        telemetry.addLine("⚠️  Limelight found but not returning data");
                        telemetry.addLine("Check if the camera is properly connected and powered");
                    }
                } catch (Exception e) {
                    telemetry.addData("❌ Error initializing Limelight", e.getMessage());
                    telemetry.addLine("Check the following:");
                    telemetry.addLine("1. Is the Limelight powered on?");
                    telemetry.addLine("2. Is the Ethernet cable properly connected?");
                    telemetry.addLine("3. Check the Limelight web interface at 172.29.0.26");
                    telemetry.addData("Error details", e.toString());
                }
            } else {
                telemetry.addLine("\n❌ No Limelight devices could be found or initialized!");
                telemetry.addLine("\nTROUBLESHOOTING:");
                telemetry.addLine("1. Check the physical connection:");
                telemetry.addLine("   - Is the Limelight powered (LED is on)?");
                telemetry.addLine("   - Is the Ethernet cable connected to the Control Hub?");
                telemetry.addLine("2. Check the configuration:");
                telemetry.addLine("   - Is the Limelight configured in the Robot Controller app?");
                telemetry.addLine("   - Try restarting the Robot Controller");
                telemetry.addLine("3. Check the network:");
                telemetry.addLine("   - Can you access the Limelight web interface at 172.29.0.26?");
                telemetry.addLine("   - Try pinging 172.29.0.26 from the Robot Controller");
            }
            
            if (limelight == null) {
                telemetry.addData("Error", "❌ No Limelight detected. Check connection and configuration.");
                telemetry.addLine("Make sure the Limelight is connected and configured in the Robot Controller");
            }
            
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight: " + e.getMessage());
            e.printStackTrace();
        }
        
        telemetry.update();
        sleep(1000); // Give time to read the messages
    }
    
    private void initializeMotors() {
        // Initialize motors with the correct hardware map names
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        
        // Set motor directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        
        telemetry.addData("Motors", "Initialized");
    }
    
    private void setMotorPowers(double forward, double strafe, double rotate) {
        // Standard mecanum drive equations with proper sign corrections
        double leftFrontPower = forward + strafe - rotate;
        double rightFrontPower = forward - strafe + rotate;
        double leftBackPower = forward - strafe - rotate;
        double rightBackPower = forward + strafe + rotate;
        
        // Apply deadband to small values to prevent motor twitching
        double deadband = 0.05;
        if (Math.abs(leftFrontPower) < deadband) leftFrontPower = 0;
        if (Math.abs(rightFrontPower) < deadband) rightFrontPower = 0;
        if (Math.abs(leftBackPower) < deadband) leftBackPower = 0;
        if (Math.abs(rightBackPower) < deadband) rightBackPower = 0;
        
        // Normalize wheel speeds if any exceeds 1.0
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
        
        // Set motor powers with error handling
        try {
            if (leftFront != null) leftFront.setPower(leftFrontPower);
            if (rightFront != null) rightFront.setPower(rightFrontPower);
            if (leftBack != null) leftBack.setPower(leftBackPower);
            if (rightBack != null) rightBack.setPower(rightBackPower);
        } catch (Exception e) {
            telemetry.addData("Motor Error", e.getMessage());
        }
    }
}
