package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Limelight and alignment imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import org.firstinspires.ftc.teamcode.LimeLight.dumbMapLime;
import java.util.List;

@Autonomous(name = "pathingFarTestBlue")
public class pathingFarTestBlue extends OpMode {
    
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    
    // Waypoints from trajectory file (trajectory 4) - organized by sequence
    private final Pose startpoint = new Pose(56.5, 8, Math.toRadians(90)); // Start at 90°, ends at 180°
    private final Pose shootingPoint = new Pose(56.5, 15, Math.toRadians(90)); // Shooting position (x=56.5, y=15)
    private final Pose waypoint1 = new Pose(45, 33, Math.toRadians(180)); // Path 1 end
    private final Pose waypoint2 = new Pose(20, 33, Math.toRadians(180)); // Path 2 end
    private final Pose waypoint3 = new Pose(50.5, 15, Math.toRadians(90)); // Path 3 end (back to start)
    private final Pose waypoint4 = new Pose(25, 15, Math.toRadians(180)); // Path 4 end
    private final Pose waypoint5 = new Pose(15, 15, Math.toRadians(180)); // Path 5 end
    private final Pose waypoint6 = new Pose(50.5, 15, Math.toRadians(180)); // Path 6 end (park)
    
    // Path chains - 7 paths (new shooting path + 6 original paths)
    private PathChain path0, path1, path2, path3, path4, path5, path6;
    
    // Shooting hardware
    private DcMotor transferMotor;
    private DcMotor intakeMotor;
    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    // GoBILDA 6000 RPM motor: 28 PPR (Pulses Per Revolution)
    // Using 28 (not 112) - the motor controller interprets PPR directly for velocity control
    private static final int TICKS_PER_REVOLUTION = 28;
    
    // Hood servo position mapping (old range to new range)
    // Old range: 0.217 (min) to 0.251 (max) - effective old range
    // New range: 0.677 (min) to 0.717 (max) - actual servo limits
    // Note: Old max 0.282 is no longer reachable. New max 0.717 corresponds to old 0.251, not 0.282
    private static final double HOOD_OLD_MIN = 0.217;
    private static final double HOOD_OLD_MAX = 0.251; // Changed from 0.282 - 0.717 maps to 0.251, not 0.282
    private static final double HOOD_NEW_MIN = 0.677; // New min position
    private static final double HOOD_NEW_MAX = 0.717; // New max position (corresponds to old 0.251)
    
    // Limelight and alignment hardware
    private dumbMapLime robot;
    private Limelight3A limelight;
    private AprilTagDetector aprilTagDetector;
    
    // Alignment constants (from AutoAimShooter)
    private static final int TARGET_TAG_ID = 20; // AprilTag ID for blue alliance (20 for blue, 24 for red)
    private static final double CAMERA_HEIGHT = 13.0; // Inches
    private static final double CAMERA_ANGLE = 0.0; // Degrees
    private static final double MAX_DISTANCE = 144.0; // Maximum distance for tag detection
    private static final int SPINNER_MIN = -500; // Encoder limit (min)
    private static final int SPINNER_MAX = 500; // Encoder limit (max)
    private static final double TURRET_KP = 0.03;
    private static final double TURRET_MIN_POWER = 0.15;
    private static final double TURRET_MAX_POWER = 0.35;
    private static final double TURRET_DEADBAND = 0.5; // Degrees - lock when within 0.5° (as requested)
    private static final double TURRET_SLOW_ZONE = 5.0; // Degrees
    private static final double TURRET_SLOW_POWER = 0.2;
    private static final double HORIZONTAL_OFFSET_DEG = 2.0;
    
    // Alignment state variables
    private AprilTagDetector.AprilTagResult cachedTagResult = null;
    private int limelightLoopCounter = 0;
    private static final int LIMELIGHT_CALL_INTERVAL = 1;
    private int lostDetectionCount = 0;
    private static final int MAX_LOST_DETECTIONS = 2;
    private boolean tagDetected = false;
    private boolean isAligned = false; // Whether turret is aligned (within 0.5° deadband)
    
    // Shooter diagnostic tracking
    private int previousShooterPosition = 0;
    private long previousShooterCheckTime = 0;
    
    public void buildPaths() {
        // Path 0: startpoint (56.5, 8, 90°) → shootingPoint (56.5, 15, 90°)
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(startpoint, shootingPoint))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
        
        // Path 1: shootingPoint (56.5, 15, 90°) → waypoint1 (45, 33, 180°)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(shootingPoint, waypoint1))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        
        // Path 2: waypoint1 (40.5, 36, 180°) → waypoint2 (20, 36, 180°)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint1, waypoint2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 3: waypoint2 (20, 36, 180°) → waypoint3 (56.5, 8, 90°)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint2, waypoint3))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
        
        // Path 4: waypoint3 (56.5, 8, 90°) → waypoint4 (15, 10, 180°) with control point (41.93, 30.76)
        // Use BezierCurve for paths with control points (quadratic Bezier: start, control, end)
        Pose controlPoint4 = new Pose(41.9265306122449, 30.759183673469384, 0);
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(waypoint3, controlPoint4, waypoint4))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        
        // Path 5: waypoint4 (15, 10, 180°) → waypoint5 (8.5, 10, 180°)
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint4, waypoint5))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 6: waypoint5 (8.5, 10, 180°) → waypoint6 (56.5, 10, 180°) - Park
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint5, waypoint6))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    
    public void autonomousPathUpdate() {
        switch (pathState) {
            // ALIGNMENT COMMENTED OUT FOR PATHING TEST
            // Alignment at start position (x=56.5, 8)
            /*
            case 0:
                // Detect and align to AprilTag
                boolean aligned = detectAndAlign();
                
                if (aligned) {
                    // Aligned within 0.5° - proceed to next path
                    if (robot.spinner != null) {
                        robot.spinner.setPower(0.0); // Stop turret
                    }
                    telemetry.addLine("✅ Aligned! Proceeding to path...");
                    setPathState(1);
                } else {
                    // Not aligned yet - keep trying to align
                    telemetry.addLine("🔍 Aligning to AprilTag...");
                }
                break;
            */
            
            case 0:
                // Path 0: startpoint → shootingPoint (56.5, 15)
                follower.followPath(path0, true);
                setPathState(1);
                break;
                
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2); // Go to shooting setup state
                }
                break;
                
            case 2:
                // Set up shooting at (56.5, 15) - spinner, shooter RPM, hood (NOT transfer yet)
                // Check if hardware is initialized first
                if (robot == null || shooterMotor == null || hoodServo == null) {
                    telemetry.addLine("❌ ERROR: Hardware not initialized!");
                    telemetry.addData("  Robot", robot != null ? "OK" : "NULL");
                    telemetry.addData("  Shooter", shooterMotor != null ? "OK" : "NULL");
                    telemetry.addData("  Hood", hoodServo != null ? "OK" : "NULL");
                    // Don't advance - stay in state 2 to show error
                } else {
                    setupShooting();
                    setPathState(3);
                }
                break;
                
            case 3:
                // Wait for shooter to reach target RPM, then turn on transfer
                // Timeout: If 5 seconds pass, proceed with whatever RPM we have
                // Turn on intake while waiting for RPM/shooting
                if (intakeMotor != null) {
                    intakeMotor.setPower(1.0);
                }
                
                long timeInState3 = pathTimer.getElapsedTime();
                boolean timeoutReached = timeInState3 > 5000; // 5 second timeout
                
                if (isShooterAtRPM() || timeoutReached) {
                    // Shooter is at RPM OR timeout reached - turn on transfer and intake
                    if (transferMotor != null) {
                        transferMotor.setPower(1.0);
                    }
                    if (intakeMotor != null) {
                        intakeMotor.setPower(1.0); // Turn on intake when shooting
                    }
                    
                    if (timeoutReached) {
                        // Timeout reached - proceed with current RPM
                        if (shooterMotor != null) {
                            try {
                                double currentVelocity = shooterMotor.getVelocity();
                                double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                                telemetry.addLine(String.format("⏰ Timeout reached (5s) - Using current RPM: %.0f", currentRPM));
                            } catch (Exception e) {
                                telemetry.addLine("⏰ Timeout reached (5s) - Proceeding anyway");
                            }
                        } else {
                            telemetry.addLine("⏰ Timeout reached (5s) - Proceeding anyway");
                        }
                    } else {
                        telemetry.addLine("✅ Shooter at RPM - Transfer & Intake ON");
                    }
                    setPathState(4); // Go to 5-second wait
                } else {
                    // Still waiting for RPM (and no timeout yet)
                    // Continuously update motors to ensure they keep running
                    
                    // Turn on intake while waiting for RPM/shooting
                    if (intakeMotor != null) {
                        intakeMotor.setPower(1.0);
                    }
                    
                    // Keep spinner moving to position
                    if (robot != null && robot.spinner != null) {
                        if (robot.spinner.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                            robot.spinner.setTargetPosition(-50);
                            robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                        robot.spinner.setPower(0.8); // Re-apply power
                        telemetry.addData("  Spinner Pos", "%d / -50", robot.spinner.getCurrentPosition());
                        telemetry.addData("  Spinner Power", "%.2f", robot.spinner.getPower());
                    }
                    
                    // Use VELOCITY control for shooter (RPM-based) - maintains target RPM regardless of voltage
                    if (shooterMotor != null) {
                        try {
                            // Ensure mode is correct for velocity control
                            if (shooterMotor.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
                                shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                            }
                            
                            // Continuously re-apply velocity command to maintain target RPM
                            double targetRPM = -3600.0;
                            double velocityTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;
                            int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                            shooterMotor.setVelocity(velocityTicksPerSecInt);
                            
                            // Read current velocity to calculate actual RPM
                            double currentVelocity = shooterMotor.getVelocity();
                            double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
                            double rpmError = Math.abs(currentRPM - targetRPM);
                            
                            // Track encoder position changes to verify motor is actually spinning
                            int currentPosition = shooterMotor.getCurrentPosition();
                            long currentTime = System.currentTimeMillis();
                            
                            // Calculate position change over time
                            int positionChange = 0;
                            double positionChangeRate = 0.0;
                            if (previousShooterCheckTime > 0) {
                                positionChange = currentPosition - previousShooterPosition;
                                long timeDelta = currentTime - previousShooterCheckTime;
                                if (timeDelta > 0) {
                                    positionChangeRate = (positionChange * 1000.0) / timeDelta; // ticks/sec
                                }
                            }
                            previousShooterPosition = currentPosition;
                            previousShooterCheckTime = currentTime;
                            
                            long remaining = (5000 - timeInState3) / 1000;
                            telemetry.addLine(String.format("⏳ Shooter: Target=%.0f RPM, Current=%.0f RPM (Timeout in %d sec)", 
                                targetRPM, currentRPM, remaining));
                            telemetry.addData("  Target RPM", "%.0f", targetRPM);
                            telemetry.addData("  Target Velocity", "%d ticks/sec", velocityTicksPerSecInt);
                            telemetry.addData("  Current RPM", "%.0f", currentRPM);
                            telemetry.addData("  Current Velocity", "%.1f ticks/sec", currentVelocity);
                            telemetry.addData("  RPM Error", "%.0f", rpmError);
                            telemetry.addData("  Shooter Encoder", "%d ticks", currentPosition);
                            telemetry.addData("  Encoder Change", "%d ticks", positionChange);
                            telemetry.addData("  Encoder Rate", "%.1f ticks/sec", positionChangeRate);
                            
                            // Calculate RPM from position change (alternative method)
                            double rpmFromPosition = (positionChangeRate / TICKS_PER_REVOLUTION) * 60.0;
                            telemetry.addData("  RPM (from pos)", "%.0f", rpmFromPosition);
                            
                            telemetry.addData("  Shooter Mode", shooterMotor.getMode().toString());
                            telemetry.addData("  TPR Used", "%d", TICKS_PER_REVOLUTION);
                            
                            // Battery voltage monitoring (for diagnostics - velocity control compensates automatically)
                            double batteryVoltage = 0.0;
                            try {
                                VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
                                batteryVoltage = voltageSensor.getVoltage();
                                telemetry.addData("  Battery Voltage", "%.2f V", batteryVoltage);
                                
                            } catch (Exception e) {
                                telemetry.addLine("  Battery Voltage: Unable to read");
                            }
                            
                            // Warnings
                            if (batteryVoltage > 0 && batteryVoltage < 11.5) {
                                telemetry.addLine("⚠️ WARNING: Battery voltage is LOW!");
                                telemetry.addLine(String.format("  Voltage: %.2fV - Velocity control will compensate, but may struggle", batteryVoltage));
                            }
                            
                            if (Math.abs(currentVelocity) < 10 && Math.abs(positionChangeRate) < 10) {
                                telemetry.addLine("⚠️ WARNING: Motor appears to NOT be spinning!");
                                telemetry.addLine("  Possible causes: Battery too low, current limit, mechanical binding");
                            }
                            
                            if (rpmError > 200) {
                                telemetry.addLine(String.format("⚠️ WARNING: RPM error is large (%.0f RPM)", rpmError));
                                telemetry.addLine("  Motor may be struggling to reach target - check battery/load");
                            }
                            
                        } catch (Exception e) {
                            telemetry.addLine("❌ Error with shooter: " + e.getMessage());
                            e.printStackTrace();
                        }
                    } else {
                        telemetry.addLine("❌ Shooter motor is NULL - cannot check RPM");
                    }
                }
                break;
                
            case 4:
                // Wait 5 seconds before moving to path1
                if (pathTimer.getElapsedTime() > 5000) {
                    telemetry.addLine("✅ 5-second wait complete - starting path1");
                    setPathState(5);
                } else {
                    long remaining = (5000 - pathTimer.getElapsedTime()) / 1000;
                    telemetry.addLine(String.format("⏳ Waiting: %d seconds remaining", remaining));
                }
                break;
                
            case 5:
                // Path 1: shootingPoint → waypoint1
                follower.followPath(path1, true);
                setPathState(6);
                break;
                
            case 6:
                if (!follower.isBusy()) {
                    setPathState(7); // Go to wait state
                }
                break;
                
            case 7:
                // Wait 3 seconds before next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // Path 2: waypoint1 → waypoint2
                    follower.followPath(path2, true);
                    setPathState(8);
                }
                break;
                
            case 8:
                if (!follower.isBusy()) {
                    setPathState(9); // Go to wait state
                }
                break;
                
            case 9:
                // Wait 3 seconds before next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // Path 3: waypoint2 → waypoint3 (returns to x=56.5, 8)
                    follower.followPath(path3, true);
                    setPathState(10);
                }
                break;
                
            case 10:
                if (!follower.isBusy()) {
                    setPathState(11); // Go to wait state
                }
                break;
                
            case 11:
                // Wait 3 seconds before next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // Path 4: waypoint3 → waypoint4 (with control point)
                    follower.followPath(path4, true);
                    setPathState(12);
                }
                break;
                
            case 12:
                if (!follower.isBusy()) {
                    setPathState(13); // Go to wait state
                }
                break;
                
            case 13:
                // Wait 3 seconds before next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // Path 5: waypoint4 → waypoint5
                    follower.followPath(path5, true);
                    setPathState(14);
                }
                break;
                
            case 14:
                if (!follower.isBusy()) {
                    setPathState(15); // Go to wait state
                }
                break;
                
            case 15:
                // Wait 3 seconds before next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // Path 6: waypoint5 → waypoint6 (Park)
                    follower.followPath(path6, true);
                    setPathState(16);
                }
                break;
                
            case 16:
                if (!follower.isBusy()) {
                    setPathState(17);
                }
                break;
                
            case 17:
                // All paths complete
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
     * Note: Old values > 0.251 (like 0.282) will be clamped to 0.251 since they're no longer reachable
     * 
     * @param oldHoodPosition Hood position in old range (0.217-0.251, values > 0.251 will be clamped)
     * @return Hood position in new range (0.677-0.717)
     */
    private double mapHoodPosition(double oldHoodPosition) {
        // Clamp to old range first (0.217-0.251, not 0.217-0.282)
        double clampedOld = Math.max(HOOD_OLD_MIN, Math.min(HOOD_OLD_MAX, oldHoodPosition));
        
        // Linear mapping: newValue = newMin + (oldValue - oldMin) * (newRange / oldRange)
        double oldRange = HOOD_OLD_MAX - HOOD_OLD_MIN;
        double newRange = HOOD_NEW_MAX - HOOD_NEW_MIN;
        double mappedValue = HOOD_NEW_MIN + (clampedOld - HOOD_OLD_MIN) * (newRange / oldRange);
        
        return mappedValue;
    }
    
    /**
     * Set up shooting at (56.5, 15) position
     * - Spinner encoder position: -50
     * - Shooter RPM: -3600 (transfer will be turned on after RPM is reached)
     * - Hood position: 0.24 (old value, will be mapped to new range)
     */
    private void setupShooting() {
        try {
            telemetry.addLine("🔧 Setting up shooting hardware...");
            
            // Initialize shooter diagnostic tracking
            if (shooterMotor != null) {
                previousShooterPosition = shooterMotor.getCurrentPosition();
                previousShooterCheckTime = System.currentTimeMillis();
            }
            
            // Set spinner to encoder position -50
            if (robot != null && robot.spinner != null) {
                // Set target position and mode for position control
                robot.spinner.setTargetPosition(-50);
                robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.spinner.setPower(0.8); // Increased power to ensure movement
                telemetry.addLine("✅ Spinner: Moving to position -50 (power=0.8)");
                telemetry.addData("  Current Pos", robot.spinner.getCurrentPosition());
                telemetry.addData("  Target Pos", -50);
                telemetry.addData("  Mode", robot.spinner.getMode().toString());
            } else {
                telemetry.addLine("❌ Spinner: NOT FOUND");
            }
            
            // Set shooter to VELOCITY control (RPM-based) - maintains target RPM regardless of voltage
            if (shooterMotor != null) {
                // Set mode to RUN_USING_ENCODER for velocity control
                shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                
                // Convert target RPM (-3600) to ticks per second for velocity control
                // Formula: velocity (ticks/sec) = (RPM / 60) * TICKS_PER_REVOLUTION
                double targetRPM = -3600.0;
                double velocityTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;
                int velocityTicksPerSecInt = (int)Math.round(velocityTicksPerSec);
                shooterMotor.setVelocity(velocityTicksPerSecInt);
                
                telemetry.addLine(String.format("✅ Shooter: Using VELOCITY control (target: %.0f RPM = %d ticks/sec)", 
                    targetRPM, velocityTicksPerSecInt));
                telemetry.addData("  Target RPM", "%.0f", targetRPM);
                telemetry.addData("  Target Velocity", "%d ticks/sec", velocityTicksPerSecInt);
                telemetry.addData("  TPR", "%d", TICKS_PER_REVOLUTION);
                telemetry.addData("  Shooter Mode", shooterMotor.getMode().toString());
            } else {
                telemetry.addLine("❌ Shooter: NOT FOUND");
            }
            
            // Set hood position to 0.24 (old value, mapped to new range)
            if (hoodServo != null) {
                double oldHoodValue = 0.24;
                double newHoodValue = mapHoodPosition(oldHoodValue);
                hoodServo.setPosition(newHoodValue);
                telemetry.addLine(String.format("✅ Hood: %.3f (old) → %.3f (new)", oldHoodValue, newHoodValue));
            } else {
                telemetry.addLine("❌ Hood: NOT FOUND");
            }
            
            telemetry.addLine("✅ Shooting setup complete - waiting for RPM...");
        } catch (Exception e) {
            telemetry.addData("❌ Shooting Setup Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Check if shooter has reached target RPM (-3600)
     * Note: Using VELOCITY control (RPM-based) - maintains target RPM regardless of voltage
     * Returns true if within 100 RPM of target
     */
    private boolean isShooterAtRPM() {
        if (shooterMotor == null) {
            return false;
        }
        
        try {
            double currentVelocity = shooterMotor.getVelocity();
            double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
            double targetRPM = -3600.0;
            double rpmDifference = Math.abs(currentRPM - targetRPM);
            
            // Additional diagnostics
            int currentPosition = shooterMotor.getCurrentPosition();
            double actualPower = shooterMotor.getPower();
            DcMotorEx.RunMode currentMode = shooterMotor.getMode();
            
            // Diagnostic telemetry
            double targetVelocity = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;
            telemetry.addLine(String.format("📊 Shooter Status (Velocity Control):"));
            telemetry.addData("  Target RPM", "%.0f", targetRPM);
            telemetry.addData("  Target Velocity", "%.0f ticks/sec", targetVelocity);
            telemetry.addData("  Current RPM", "%.0f", currentRPM);
            telemetry.addData("  Current Velocity", "%.1f ticks/sec", currentVelocity);
            telemetry.addData("  RPM Error", "%.0f", rpmDifference);
            telemetry.addData("  Actual Power", "%.2f (auto-adjusted)", actualPower);
            telemetry.addData("  Encoder Position", "%d ticks", currentPosition);
            telemetry.addData("  TPR Used", "%d", TICKS_PER_REVOLUTION);
            telemetry.addData("  Mode", currentMode.toString());
            
            // Battery voltage monitoring (for diagnostics - velocity control compensates automatically)
            try {
                VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
                double batteryVoltage = voltageSensor.getVoltage();
                telemetry.addData("  Battery Voltage", "%.2f V", batteryVoltage);
                
                if (batteryVoltage < 11.5) {
                    telemetry.addLine("⚠️ WARNING: Battery voltage is LOW!");
                    telemetry.addLine(String.format("  Voltage: %.2fV - Velocity control will compensate, but may struggle", batteryVoltage));
                } else if (batteryVoltage < 12.0) {
                    telemetry.addLine("⚠️ WARNING: Battery voltage is dropping!");
                }
            } catch (Exception e) {
                // Ignore voltage sensor errors
            }
            
            // Check if motor is actually spinning (encoder position should change)
            if (Math.abs(currentVelocity) < 10) {
                telemetry.addLine("⚠️ WARNING: Velocity is very low - motor may not be spinning!");
            }
            
            if (rpmDifference > 200) {
                telemetry.addLine(String.format("⚠️ WARNING: RPM error is large (%.0f RPM)", rpmDifference));
                telemetry.addLine("  Motor may be struggling to reach target - check battery/load");
            }
            
            // Consider "at RPM" if within 100 RPM of target (velocity control should achieve this)
            return rpmDifference <= 100.0;
        } catch (Exception e) {
            telemetry.addData("RPM Check Error", e.getMessage());
            e.printStackTrace();
            return false;
        }
    }
    
    /**
     * Detect AprilTag and align turret (from AutoAimShooter)
     * Returns true if tag is detected and aligned
     */
    private boolean detectAndAlign() {
        if (limelight == null || aprilTagDetector == null || robot == null || robot.spinner == null) {
            return false;
        }
        
        try {
            limelightLoopCounter++;
            boolean shouldCallLimelight = (limelightLoopCounter % LIMELIGHT_CALL_INTERVAL == 0);
            
            boolean freshTagDetected = false;
            double currentTx = 0.0;
            
            if (shouldCallLimelight) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();
                    
                    double MIN_TAG_AREA = 0.003;
                    List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    boolean hasTargetTag = false;
                    
                    if (fiducials != null && !fiducials.isEmpty()) {
                        for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid : fiducials) {
                            if (fid.getFiducialId() == TARGET_TAG_ID) {
                                hasTargetTag = true;
                                break;
                            }
                        }
                    }
                    
                    if (ta > MIN_TAG_AREA && hasTargetTag) {
                        try {
                            cachedTagResult = aprilTagDetector.getTagById(TARGET_TAG_ID);
                            if (cachedTagResult != null && cachedTagResult.isValid && cachedTagResult.distance > 0) {
                                freshTagDetected = true;
                                lostDetectionCount = 0;
                                tagDetected = true;
                                currentTx = tx;
                            }
                        } catch (Exception e) {
                            // Ignore
                        }
                    }
                }
                
                if (!freshTagDetected) {
                    lostDetectionCount++;
                    if (lostDetectionCount > MAX_LOST_DETECTIONS) {
                        tagDetected = false;
                        cachedTagResult = null;
                        if (robot.spinner != null) {
                            robot.spinner.setPower(0.0);
                        }
                    }
                }
            }
            
            // Update turret alignment if tag detected
            if (freshTagDetected && tagDetected && cachedTagResult != null && cachedTagResult.isValid) {
                updateTurret(currentTx);
                return isAligned; // Return true if aligned (within 0.5° deadband)
            } else {
                if (robot.spinner != null) {
                    robot.spinner.setPower(0.0);
                }
                return false;
            }
        } catch (Exception e) {
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            return false;
        }
    }
    
    /**
     * Update turret alignment (from AutoAimShooter)
     */
    private void updateTurret(double tx) {
        if (robot.spinner == null) {
            return;
        }
        
        try {
            double adjustedTx = tx + HORIZONTAL_OFFSET_DEG;
            double absTx = Math.abs(adjustedTx);
            
            // Check if within deadband - aligned!
            if (absTx <= TURRET_DEADBAND) {
                robot.spinner.setPower(0.0);
                isAligned = true;
                return;
            }
            
            isAligned = false; // Not aligned yet
            
            // Check encoder limits
            int currentPos;
            try {
                currentPos = robot.spinner.getCurrentPosition();
            } catch (Exception e) {
                robot.spinner.setPower(0.0);
                return;
            }
            
            if (currentPos >= SPINNER_MAX || currentPos <= SPINNER_MIN) {
                robot.spinner.setPower(0.0);
                return;
            }
            
            // Progressive power control
            double cmd;
            double sign = Math.signum(adjustedTx);
            
            if (absTx <= TURRET_SLOW_ZONE) {
                cmd = TURRET_SLOW_POWER * sign;
            } else {
                cmd = adjustedTx * TURRET_KP;
                double absCmd = Math.abs(cmd);
                if (absCmd > TURRET_MAX_POWER) {
                    cmd = TURRET_MAX_POWER * sign;
                } else if (absCmd < TURRET_MIN_POWER) {
                    cmd = TURRET_MIN_POWER * sign;
                }
            }
            
            // Check approaching limits
            if (cmd > 0 && currentPos >= SPINNER_MAX - 50) {
                cmd = 0.0;
            } else if (cmd < 0 && currentPos <= SPINNER_MIN + 50) {
                cmd = 0.0;
            }
            
            robot.spinner.setPower(cmd);
        } catch (Exception e) {
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
        }
    }
    
    
    @Override
    public void loop() {
        follower.update();
        
        // ALIGNMENT CODE COMMENTED OUT FOR PATHING TEST
        // If in alignment state (0), detect and align every loop
        /*
        if (pathState == 0) {
            detectAndAlign();
        } else {
            // Not in alignment state - stop turret
            if (robot != null && robot.spinner != null) {
                try {
                    robot.spinner.setPower(0.0);
                } catch (Exception e) {
                    // Ignore
                }
            }
        }
        */
        
        autonomousPathUpdate();
        
        // Telemetry
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("busy? ", follower.isBusy());
        
        // ALIGNMENT TELEMETRY COMMENTED OUT FOR PATHING TEST
        /*
        // Alignment telemetry
        if (pathState == 0) {
            telemetry.addLine("\n--- Alignment Status ---");
            telemetry.addData("Tag Detected", tagDetected ? "YES" : "NO");
            telemetry.addData("Aligned", isAligned ? "YES (within 0.5°)" : "NO");
            if (cachedTagResult != null && cachedTagResult.isValid && cachedTagResult.distance > 0) {
                telemetry.addData("Distance", "%.1f inches", cachedTagResult.distance);
            }
        }
        */
        
        telemetry.update();
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
        
        // ALIGNMENT INITIALIZATION COMMENTED OUT FOR PATHING TEST
        // Initialize alignment hardware
        // initializeAlignmentHardware();
    }
    
    /**
     * Initialize transfer motor, shooter motor, hood servo, and robot (for spinner)
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
            
            // Initialize intake motor
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            if (intakeMotor != null) {
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intakeMotor.setPower(0.0);
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
                double defaultOldValue = 0.24; // Old default value
                double defaultNewValue = mapHoodPosition(defaultOldValue);
                hoodServo.setPosition(defaultNewValue); // Set to default position (mapped to new range)
            }
            
            // Report initialization status
            telemetry.addLine("\n=== Shooting Hardware Initialization ===");
            telemetry.addData("Robot/Spinner", (robot != null && robot.spinner != null) ? "✅ Found" : "❌ NOT FOUND");
            telemetry.addData("Shooter Motor", shooterMotor != null ? "✅ Found" : "❌ NOT FOUND");
            telemetry.addData("Transfer Motor", transferMotor != null ? "✅ Found" : "❌ NOT FOUND");
            telemetry.addData("Intake Motor", intakeMotor != null ? "✅ Found" : "❌ NOT FOUND");
            telemetry.addData("Hood Servo", hoodServo != null ? "✅ Found" : "❌ NOT FOUND");
        } catch (Exception e) {
            telemetry.addData("❌ Shooting Hardware Init Error", e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Initialize Limelight and turret hardware
     */
    private void initializeAlignmentHardware() {
        try {
            // Initialize robot (for spinner/turret access)
            robot = new dumbMapLime(this);
            robot.initMotors();
            
            // Initialize spinner/turret motor
            // IMPORTANT: Encoder resets to 0 here (in init() phase).
            // Before pressing INIT, position the turret in a known position (e.g., straight ahead).
            // After INIT, the encoder will track movement from that zero position.
            // The alignment code will automatically move the turret to align with the AprilTag.
            if (robot.spinner != null) {
                robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.spinner.setPower(0.0);
            }
            
            // Initialize Limelight
            String[] possibleNames = {
                "limelight", "limelight3a", "limelight3A", 
                "limelight_3a", "limelight_3A", "limelight3a_1",
                "limelight3a_2", "limelight3a_3", "limelight3a_4"
            };
            
            for (String name : possibleNames) {
                try {
                    limelight = hardwareMap.get(Limelight3A.class, name);
                    if (limelight != null) {
                        break;
                    }
                } catch (Exception e) {
                    // Try next name
                }
            }
            
            if (limelight == null) {
                try {
                    for (Limelight3A device : hardwareMap.getAll(Limelight3A.class)) {
                        limelight = device;
                        break;
                    }
                } catch (Exception e) {
                    // Ignore
                }
            }
            
            if (limelight != null) {
                limelight.start();
                limelight.pipelineSwitch(3); // Pipeline 3 for blue AprilTag (20)
                aprilTagDetector = new AprilTagDetector(limelight, CAMERA_HEIGHT, CAMERA_ANGLE, MAX_DISTANCE);
                telemetry.addLine("✅ Limelight initialized (Blue - Tag 20)");
            } else {
                telemetry.addLine("⚠️ Limelight not found");
            }
        } catch (Exception e) {
            telemetry.addData("Alignment Hardware Init Error", e.getMessage());
        }
    }
    
    @Override
    public void init_loop() {}
    
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    
    @Override
    public void stop() {
        // Stop all motors
        try {
            if (robot != null && robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            if (transferMotor != null) {
                transferMotor.setPower(0.0);
            }
            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }
            if (shooterMotor != null) {
                shooterMotor.setVelocity(0); // Using velocity control
            }
        } catch (Exception e) {
            // Ignore
        }
    }
}

