package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.LimeLight.dumbMapLime;

@Autonomous(name = "pathingCloseTestBlue")
public class pathingCloseTestBlue extends OpMode {
    
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    
    // Waypoints from trajectory file (trajectory 5)
    private final Pose startpoint = new Pose(20.7, 123, Math.toRadians(145)); // Start at 145°
    private final Pose waypoint1 = new Pose(46.5, 93, Math.toRadians(200)); // Path 1 end at 200° (shooting position)
    private final Pose waypoint2 = new Pose(27, 93, Math.toRadians(200)); // Path 2 end at 200° (intake on during this path)
    
    // Path chain
    private PathChain path1, path2, path3;
    
    // Shooting hardware
    private dumbMapLime robot; // For spinner access
    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    // GoBILDA 6000 RPM motor: 28 PPR (Pulses Per Revolution)
    private static final int TICKS_PER_REVOLUTION = 28;
    
    // Spinner position
    private static final int SPINNER_TARGET_POSITION = 130;
    
    // Hood servo position mapping (old range to new range)
    // Old range: 0.217 (min) to 0.251 (max) - effective old range
    // New range: 0.677 (min) to 0.717 (max) - actual servo limits
    private static final double HOOD_OLD_MIN = 0.217;
    private static final double HOOD_OLD_MAX = 0.251;
    private static final double HOOD_NEW_MIN = 0.677;
    private static final double HOOD_NEW_MAX = 0.717;
    
    // Shooting constants
    private static final double SHOOTER_TARGET_RPM = -2400.0; // Target RPM for first shooting
    private static final double SHOOTER_TARGET_RPM_SECOND = -2250.0; // Target RPM for second shooting
    private static final double HOOD_OLD_POSITION = 0.2255; // Old hood position to map
    
    public void buildPaths() {
        // Path 1: startpoint (20.7, 123, 145°) → waypoint1 (45, 93, 200°)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startpoint, waypoint1))
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(200))
                .build();
        
        // Path 2: waypoint1 (46, 93, 200°) → waypoint2 (27, 93, 200°)
        // Intake will be on during this path - path constraints will be set on follower before following
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint1, waypoint2))
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                .build();
        
        // Path 3: waypoint2 (20, 93, 200°) → waypoint1 (45, 93, 200°)
        // Return to waypoint1
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint2, waypoint1))
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                .build();
    }
    
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        
        // Create follower using Constants helper method
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startpoint);
        
        buildPaths();
        
        // Initialize shooting hardware
        initializeShootingHardware();
    }
    
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        pathState = 0;
    }
    
    @Override
    public void loop() {
        // Update follower - this is REQUIRED for the path following to work!
        follower.update();
        
        autonomousPathUpdate();
        
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Time", opmodeTimer.getElapsedTime());
        
        // Show current pose for debugging
        if (follower != null) {
            Pose currentPose = follower.getPose();
            telemetry.addData("Current X", "%.1f", currentPose.getX());
            telemetry.addData("Current Y", "%.1f", currentPose.getY());
            telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        }
    }
    
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start Path 1 - call followPath() and immediately transition to waiting state
                follower.followPath(path1, true);
                telemetry.addLine("🚀 Starting Path 1...");
                setPathState(1);
                break;
                
            case 1:
                // Wait for Path 1 to complete
                if (!follower.isBusy()) {
                    // Path 1 complete - set up shooting at waypoint1
                    telemetry.addLine("✅ Path 1 complete! Setting up shooting...");
                    setupShooting();
                    setPathState(2);
                } else {
                    // Show path progress
                    Pose currentPose = follower.getPose();
                    telemetry.addLine("📍 Following path 1...");
                    telemetry.addData("  Current X", "%.1f", currentPose.getX());
                    telemetry.addData("  Current Y", "%.1f", currentPose.getY());
                    telemetry.addData("  Current Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
                }
                break;
                
            case 2:
                // Wait for shooter to reach target RPM
                // Continuously update spinner and shooter to ensure they keep running
                if (robot != null && robot.spinner != null) {
                    if (robot.spinner.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        robot.spinner.setTargetPosition(SPINNER_TARGET_POSITION);
                        robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    robot.spinner.setPower(0.8);
                }
                
                if (shooterMotor != null) {
                    // Continuously re-apply velocity command
                    double velocityTicksPerSec = (SHOOTER_TARGET_RPM / 60.0) * TICKS_PER_REVOLUTION;
                    int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                    shooterMotor.setVelocity(velocityTicksPerSecInt);
                }
                
                // Always show current RPM
                if (shooterMotor != null) {
                    try {
                        double currentVelocity = shooterMotor.getVelocity();
                        double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                        telemetry.addLine(String.format("📊 Shooter RPM: %.0f / %.0f", currentRPM, SHOOTER_TARGET_RPM));
                    } catch (Exception e) {
                        telemetry.addLine("📊 Shooter RPM: Unable to read");
                    }
                }
                
                if (isShooterAtRPM(SHOOTER_TARGET_RPM)) {
                    // Shooter at RPM - turn on intake and transfer, then proceed to shooting
                    if (intakeMotor != null) {
                        intakeMotor.setPower(1.0);
                    }
                    if (transferMotor != null) {
                        transferMotor.setPower(1.0);
                    }
                    telemetry.addLine("✅ Shooter at RPM - Intake & Transfer ON - Shooting!");
                    setPathState(3);
                } else {
                    // Still waiting for RPM
                    telemetry.addLine("⏳ Waiting for shooter to reach target RPM...");
                }
                break;
                
            case 3:
                // Shooting at waypoint1 - 5 seconds with transfer, intake, and shooter on
                // Keep all motors running
                if (robot != null && robot.spinner != null) {
                    if (robot.spinner.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        robot.spinner.setTargetPosition(SPINNER_TARGET_POSITION);
                        robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    robot.spinner.setPower(0.8);
                }
                if (intakeMotor != null) {
                    intakeMotor.setPower(1.0);
                }
                if (transferMotor != null) {
                    transferMotor.setPower(1.0);
                }
                if (shooterMotor != null) {
                    // Continuously re-apply velocity command
                    double velocityTicksPerSec = (SHOOTER_TARGET_RPM / 60.0) * TICKS_PER_REVOLUTION;
                    int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                    shooterMotor.setVelocity(velocityTicksPerSecInt);
                }
                
                // Show current RPM while shooting
                if (shooterMotor != null) {
                    try {
                        double currentVelocity = shooterMotor.getVelocity();
                        double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                        telemetry.addLine(String.format("📊 Shooting - Shooter RPM: %.0f", currentRPM));
                    } catch (Exception e) {
                        telemetry.addLine("📊 Shooting - Shooter RPM: Unable to read");
                    }
                }
                
                if (pathTimer.getElapsedTime() > 2000) { // 2.0 second wait for first shooting (only 2 balls)
                    // Shooting complete - turn off shooter, transfer, and spinner
                    // Keep intake ON for Path 2 (explicitly set to ensure continuity)
                    if (intakeMotor != null) {
                        intakeMotor.setPower(1.0); // Keep intake on
                    }
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(0);
                    }
                    if (transferMotor != null) {
                        transferMotor.setPower(0.0);
                    }
                    if (robot != null && robot.spinner != null) {
                        robot.spinner.setPower(0.0);
                    }
                    telemetry.addLine("✅ Shooting complete! Starting Path 2 (intake ON)...");
                    follower.followPath(path2, true);
                    setPathState(4);
                } else {
                    long remaining = (2000 - pathTimer.getElapsedTime()) / 1000;
                    telemetry.addLine(String.format("⏳ Shooting... %d seconds remaining", remaining));
                }
                break;
                
            case 4:
                // Wait for Path 2 to complete (intake on during this path to collect balls)
                // Intake is already on from shooting phase
                if (intakeMotor != null) {
                    intakeMotor.setPower(1.0);
                }
                
                if (!follower.isBusy()) {
                    // Path 2 complete - turn off intake, return to waypoint1
                    if (intakeMotor != null) {
                        intakeMotor.setPower(0.0);
                    }
                    telemetry.addLine("✅ Path 2 complete! Returning to waypoint1...");
                    follower.followPath(path3, true);
                    setPathState(5);
                } else {
                    // Show path progress
                    Pose currentPose = follower.getPose();
                    telemetry.addLine("📍 Following path 2 (intake ON - collecting balls)...");
                    telemetry.addData("  Current X", "%.1f", currentPose.getX());
                    telemetry.addData("  Current Y", "%.1f", currentPose.getY());
                    telemetry.addData("  Current Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
                }
                break;
                
            case 5:
                // Wait for Path 3 to complete (returning to waypoint1)
                // Only intake is on during Path 3 (collecting balls)
                if (intakeMotor != null) {
                    intakeMotor.setPower(1.0);
                }
                // Transfer is OFF during Path 3
                if (transferMotor != null) {
                    transferMotor.setPower(0.0);
                }
                
                if (!follower.isBusy()) {
                    // Path 3 complete - turn OFF intake before RPM buildup
                    if (intakeMotor != null) {
                        intakeMotor.setPower(0.0);
                    }
                    // Returned to waypoint1 - set up shooting again (without moving spinner/hood)
                    telemetry.addLine("✅ Returned to waypoint1! Setting up shooting again...");
                    setupShootingWithoutSpinner();
                    setPathState(6);
                } else {
                    // Show path progress
                    Pose currentPose = follower.getPose();
                    telemetry.addLine("📍 Returning to waypoint1 (intake ON - collecting balls)...");
                    telemetry.addData("  Current X", "%.1f", currentPose.getX());
                    telemetry.addData("  Current Y", "%.1f", currentPose.getY());
                    telemetry.addData("  Current Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
                }
                break;
                
            case 6:
                // Wait for shooter to reach target RPM (second shooting at waypoint1)
                // Don't move spinner - keep it at current position
                // Keep intake and transfer OFF during RPM buildup
                if (intakeMotor != null) {
                    intakeMotor.setPower(0.0);
                }
                if (transferMotor != null) {
                    transferMotor.setPower(0.0);
                }
                
                // Re-initialize shooter motor if it's null (was turned off earlier)
                if (shooterMotor == null) {
                    shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
                    if (shooterMotor == null) {
                        shooterMotor = hardwareMap.get(DcMotorEx.class, "outtake");
                    }
                    if (shooterMotor != null) {
                        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                }
                
                if (shooterMotor != null) {
                    // Continuously re-apply velocity command (using second shooting RPM: -2250)
                    double velocityTicksPerSec = (SHOOTER_TARGET_RPM_SECOND / 60.0) * TICKS_PER_REVOLUTION;
                    int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                    shooterMotor.setVelocity(velocityTicksPerSecInt);
                }
                
                // Always show current RPM
                if (shooterMotor != null) {
                    try {
                        double currentVelocity = shooterMotor.getVelocity();
                        double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                        telemetry.addLine(String.format("📊 Shooter RPM: %.0f / %.0f", currentRPM, SHOOTER_TARGET_RPM_SECOND));
                    } catch (Exception e) {
                        telemetry.addLine("📊 Shooter RPM: Unable to read");
                    }
                }
                
                if (isShooterAtRPM(SHOOTER_TARGET_RPM_SECOND)) {
                    // Shooter at RPM - turn on intake and transfer, then proceed to shooting
                    if (intakeMotor != null) {
                        intakeMotor.setPower(1.0);
                    }
                    if (transferMotor != null) {
                        transferMotor.setPower(1.0);
                    }
                    telemetry.addLine("✅ Shooter at RPM - Intake & Transfer ON - Shooting!");
                    setPathState(7);
                } else {
                    // Still waiting for RPM
                    telemetry.addLine("⏳ Waiting for shooter to reach target RPM...");
                }
                break;
                
            case 7:
                // Second shooting at waypoint1 - 5 seconds with transfer, intake, and shooter on
                // Keep all motors running (spinner stays at current position)
                if (intakeMotor != null) {
                    intakeMotor.setPower(1.0);
                }
                if (transferMotor != null) {
                    transferMotor.setPower(1.0);
                }
                if (shooterMotor != null) {
                    // Continuously re-apply velocity command (using second shooting RPM: -2250)
                    double velocityTicksPerSec = (SHOOTER_TARGET_RPM_SECOND / 60.0) * TICKS_PER_REVOLUTION;
                    int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                    shooterMotor.setVelocity(velocityTicksPerSecInt);
                }
                
                // Show current RPM while shooting
                if (shooterMotor != null) {
                    try {
                        double currentVelocity = shooterMotor.getVelocity();
                        double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                        telemetry.addLine(String.format("📊 Shooting - Shooter RPM: %.0f", currentRPM));
                    } catch (Exception e) {
                        telemetry.addLine("📊 Shooting - Shooter RPM: Unable to read");
                    }
                }
                
                if (pathTimer.getElapsedTime() > 5000) { // 5 second wait for second shooting
                    // Shooting complete - turn off all motors
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(0);
                    }
                    if (transferMotor != null) {
                        transferMotor.setPower(0.0);
                    }
                    if (intakeMotor != null) {
                        intakeMotor.setPower(0.0);
                    }
                    if (robot != null && robot.spinner != null) {
                        robot.spinner.setPower(0.0);
                    }
                    telemetry.addLine("✅ Second shooting complete! Autonomous done.");
                    setPathState(8);
                } else {
                    long remaining = (5000 - pathTimer.getElapsedTime()) / 1000;
                    telemetry.addLine(String.format("⏳ Shooting... %d seconds remaining", remaining));
                }
                break;

            case 8:
                // All done
                telemetry.addLine("✅ Autonomous complete");
                break;
        }
    }
    
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    
    /**
     * Map old hood position value to new hood position range
     * Converts from old range (0.217-0.251) to new range (0.677-0.717)
     * 
     * @param oldHoodPosition Hood position in old range (0.217-0.251, values > 0.251 will be clamped)
     * @return Hood position in new range (0.677-0.717)
     */
    private double mapHoodPosition(double oldHoodPosition) {
        // Clamp to old range first (0.217-0.251)
        double clampedOld = Math.max(HOOD_OLD_MIN, Math.min(HOOD_OLD_MAX, oldHoodPosition));
        
        // Linear mapping: newValue = newMin + (oldValue - oldMin) * (newRange / oldRange)
        double oldRange = HOOD_OLD_MAX - HOOD_OLD_MIN;
        double newRange = HOOD_NEW_MAX - HOOD_NEW_MIN;
        double mappedValue = HOOD_NEW_MIN + (clampedOld - HOOD_OLD_MIN) * (newRange / oldRange);
        
        return mappedValue;
    }
    
    /**
     * Initialize shooting hardware (spinner, shooter motor, hood servo, intake motor)
     */
    private void initializeShootingHardware() {
        try {
            // Initialize robot (for spinner access)
            robot = new dumbMapLime(this);
            robot.initMotors();
            
            // Initialize spinner motor
            if (robot.spinner != null) {
                robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.spinner.setPower(0.0);
            }
            
            // Initialize shooter motor (will use VELOCITY control - RPM-based)
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            if (shooterMotor == null) {
                shooterMotor = hardwareMap.get(DcMotorEx.class, "outtake");
            }
            if (shooterMotor != null) {
                shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // Velocity control mode
                shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                shooterMotor.setVelocity(0); // Start with velocity off
            }
            
            // Initialize hood servo
            hoodServo = hardwareMap.get(Servo.class, "hood");
            if (hoodServo != null) {
                // Set to default position (will be updated at end point)
                double defaultNewValue = mapHoodPosition(HOOD_OLD_POSITION);
                hoodServo.setPosition(defaultNewValue);
            }
            
            // Initialize intake motor
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            if (intakeMotor != null) {
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intakeMotor.setPower(0.0);
            }
            
            // Initialize transfer motor
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            if (transferMotor == null) {
                transferMotor = hardwareMap.get(DcMotor.class, "flicker");
            }
            if (transferMotor != null) {
                transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                transferMotor.setPower(0.0);
            }
            
            // Report initialization status
            telemetry.addLine("\n=== Shooting Hardware Initialization ===");
            telemetry.addData("Robot/Spinner", (robot != null && robot.spinner != null) ? "✅ Found" : "❌ NOT FOUND");
            telemetry.addData("Shooter Motor", shooterMotor != null ? "✅ Found" : "❌ NOT FOUND");
            telemetry.addData("Hood Servo", hoodServo != null ? "✅ Found" : "❌ NOT FOUND");
            telemetry.addData("Intake Motor", intakeMotor != null ? "✅ Found" : "❌ NOT FOUND");
            telemetry.addData("Transfer Motor", transferMotor != null ? "✅ Found" : "❌ NOT FOUND");
        } catch (Exception e) {
            telemetry.addData("Shooting Hardware Init Error", e.getMessage());
        }
    }
    
    /**
     * Set up shooting at end point
     * - Shooter RPM: -2600
     * - Hood position: 0.232 (old value, mapped to new range)
     */
    private void setupShooting() {
        try {
            telemetry.addLine("🔧 Setting up shooting at end point...");
            
            // Set spinner to encoder position 50
            if (robot != null && robot.spinner != null) {
                robot.spinner.setTargetPosition(SPINNER_TARGET_POSITION);
                robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.spinner.setPower(0.8);
                telemetry.addLine(String.format("✅ Spinner: Moving to position %d", SPINNER_TARGET_POSITION));
            } else {
                telemetry.addLine("❌ Spinner: NOT FOUND");
            }
            
            // Set shooter to VELOCITY control (RPM-based) - maintains target RPM regardless of voltage
            if (shooterMotor != null) {
                // Set mode to RUN_USING_ENCODER for velocity control
                shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                
                // Convert target RPM (-2600) to ticks per second for velocity control
                // Formula: velocity (ticks/sec) = (RPM / 60) * TICKS_PER_REVOLUTION
                double velocityTicksPerSec = (SHOOTER_TARGET_RPM / 60.0) * TICKS_PER_REVOLUTION;
                int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                shooterMotor.setVelocity(velocityTicksPerSecInt);
                
                telemetry.addLine(String.format("✅ Shooter: Set to %.0f RPM = %d ticks/sec", 
                    SHOOTER_TARGET_RPM, velocityTicksPerSecInt));
            } else {
                telemetry.addLine("❌ Shooter: NOT FOUND");
            }
            
            // Set hood position to 0.232 (old value, mapped to new range)
            if (hoodServo != null) {
                double newHoodValue = mapHoodPosition(HOOD_OLD_POSITION);
                hoodServo.setPosition(newHoodValue);
                telemetry.addLine(String.format("✅ Hood: %.3f (old) → %.3f (new)", HOOD_OLD_POSITION, newHoodValue));
            } else {
                telemetry.addLine("❌ Hood: NOT FOUND");
            }
            
            // Show current shooter status
            if (shooterMotor != null) {
                try {
                    double currentVelocity = shooterMotor.getVelocity();
                    double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                    telemetry.addLine(String.format("📊 Current Shooter RPM: %.0f", currentRPM));
                    
                    // Battery voltage monitoring
                    try {
                        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
                        double batteryVoltage = voltageSensor.getVoltage();
                        telemetry.addData("  Battery Voltage", "%.2f V", batteryVoltage);
                    } catch (Exception e) {
                        // Ignore voltage sensor errors
                    }
                } catch (Exception e) {
                    telemetry.addLine("❌ Error reading shooter status");
                }
            }
            
            telemetry.addLine("✅ Shooting setup complete");
        } catch (Exception e) {
            telemetry.addData("❌ Shooting Setup Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Set up shooting without moving spinner/hood (for second shooting at waypoint1)
     * Only sets shooter RPM - keeps spinner and hood at current position
     */
    private void setupShootingWithoutSpinner() {
        try {
            telemetry.addLine("🔧 Setting up shooting (spinner/hood stay in place)...");
            
            // Set shooter to VELOCITY control (RPM-based) - maintains target RPM regardless of voltage
            if (shooterMotor != null) {
                // Set mode to RUN_USING_ENCODER for velocity control
                shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                
                // Convert target RPM to ticks per second for velocity control (using second shooting RPM: -2250)
                double velocityTicksPerSec = (SHOOTER_TARGET_RPM_SECOND / 60.0) * TICKS_PER_REVOLUTION;
                int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                shooterMotor.setVelocity(velocityTicksPerSecInt);
                
                telemetry.addLine(String.format("✅ Shooter: Set to %.0f RPM = %d ticks/sec", 
                    SHOOTER_TARGET_RPM_SECOND, velocityTicksPerSecInt));
                telemetry.addLine("✅ Spinner/Hood: Keeping current position");
            } else {
                telemetry.addLine("❌ Shooter: NOT FOUND");
            }
            
            telemetry.addLine("✅ Shooting setup complete (spinner/hood unchanged)");
        } catch (Exception e) {
            telemetry.addData("❌ Shooting Setup Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Check if shooter has reached target RPM
     * Returns true if within 100 RPM of target
     * @param targetRPM The target RPM to check against
     */
    private boolean isShooterAtRPM(double targetRPM) {
        if (shooterMotor == null) {
            return false;
        }
        
        try {
            double currentVelocity = shooterMotor.getVelocity();
            double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
            double rpmDifference = Math.abs(currentRPM - targetRPM);
            
            // Consider "at RPM" if within 100 RPM of target
            return rpmDifference <= 100.0;
        } catch (Exception e) {
            return false;
        }
    }
    
    @Override
    public void stop() {
        // Stop all motors
        try {
            if (robot != null && robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            if (shooterMotor != null) {
                shooterMotor.setVelocity(0); // Using velocity control
            }
            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }
            if (transferMotor != null) {
                transferMotor.setPower(0.0);
            }
        } catch (Exception e) {
            // Ignore
        }
    }
}

