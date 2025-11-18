package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import org.firstinspires.ftc.teamcode.dumbMap;

/**
 * Test class to isolate and test the shooter turn (horizontal turret alignment) functionality.
 * Tests horizontal alignment using Limelight tx values only.
 */
@TeleOp(name = "Shooter Turn Test", group = "Test")
public class ShooterTurnTest extends LinearOpMode {
    private dumbMap robot;
    
    // Limelight + AprilTag detection
    private AprilTagDetector tagDetector;
    private Limelight3A limelight;
    
    // Shooter turn motor (horizontal turret)
    private DcMotor shooterTurn;
    
    // --- TUNING: Camera mounting (same as main teleop) ---
    private static final double CAMERA_HEIGHT_IN = 12.0; // Inches. Measure lens height from floor
    private static final double CAMERA_ANGLE_DEG = 15.0; // Degrees down from horizontal. Measure on robot
    private static final double MAX_SHOT_DISTANCE_IN = 144.0; // Maximum distance for tag detection
    
    // --- TUNING: Horizontal turret control (uses shooterTurn motor) ---
    // This aligns the shooter horizontally so Limelight tx -> 0°
    private static final double TURRET_KP = 0.02;      // deg -> power (proportional gain)
    private static final double TURRET_MIN = 0.12;     // minimum power to move
    private static final double TURRET_MAX = 0.5;      // power cap
    private static final double TURRET_DEADZONE_DEG = 0.5;  // deadzone - no movement within this range
    private static final double TURRET_TOL_DEG = 3.0;  // within this, treat as aligned (stops overcorrecting)
    
    // Manual override controls
    private boolean manualMode = false;
    private double manualPower = 0.0;
    private boolean lastX = false;
    
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
            telemetry.addData("Limelight", "Initialized");
        } else {
            telemetry.addData("Limelight", "NOT FOUND");
        }
        
        // Map shooter turn motor (horizontal turret)
        try {
            shooterTurn = hardwareMap.get(DcMotor.class, "spinner");
            shooterTurn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterTurn.setPower(0.0);
            telemetry.addData("Shooter Turn", "Motor mapped");
            telemetry.addData("Motor Name", "spinner");
        } catch (Exception e) {
            shooterTurn = null;
            telemetry.addData("Shooter Turn", "Motor NOT found");
            telemetry.addData("Error", e.getMessage());
        }
        
        telemetry.addData("Status", "Initialized. Press START to begin.");
        telemetry.addData("Controls", "X = Toggle auto/manual mode");
        telemetry.addData("Controls", "Left Stick X = Manual turret control (in manual mode)");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Toggle between auto and manual mode with X button (edge detection)
            boolean xPressed = gamepad1.x;
            if (xPressed && !lastX) {
                manualMode = !manualMode;
                if (manualMode && shooterTurn != null) {
                    shooterTurn.setPower(0.0); // Stop when switching to manual
                }
            }
            lastX = xPressed;
            
            if (manualMode) {
                // Manual control: use left stick X axis for turret
                manualPower = -gamepad1.left_stick_x * 0.5; // Scale to 50% max power
                if (shooterTurn != null) {
                    shooterTurn.setPower(manualPower);
                }
                
                telemetry.addData("Mode", "MANUAL");
                telemetry.addData("Manual Turret Power", "%.2f", manualPower);
            } else {
                // Auto mode: use Limelight to align horizontally
                if (tagDetector != null) {
                    AprilTagDetector.AprilTagResult tag = tagDetector.getClosestTag();
                    
                    if (tag.isValid) {
                        double tx = tag.xDegrees; // horizontal error in degrees
                        
                        // Horizontal alignment: drive shooterTurn so tx -> 0°
                        if (shooterTurn != null) {
                            double cmd;
                            double absTx = Math.abs(tx);
                            
                            // Check if within tolerance (stop correcting - prevents overcorrection)
                            if (absTx <= TURRET_TOL_DEG) {
                                cmd = 0.0;
                            } 
                            // Check deadzone: if error is very small, don't move (prevents jitter)
                            else if (absTx <= TURRET_DEADZONE_DEG) {
                                cmd = 0.0;
                            }
                            // Outside both deadzone and tolerance - calculate correction
                            else {
                                cmd = tx * TURRET_KP;
                                double sign = Math.signum(cmd);
                                cmd = Math.min(TURRET_MAX, Math.max(TURRET_MIN, Math.abs(cmd))) * sign;
                            }
                            
                            // Apply command to motor (flip sign here if turret runs opposite direction)
                            shooterTurn.setPower(cmd);
                            
                            telemetry.addData("tx (raw)", "%.3f", tx);
                            telemetry.addData("cmd (before clamp)", "%.3f", tx * TURRET_KP);
                            telemetry.addData("Turret Cmd", "%.3f", cmd);
                            telemetry.addData("Motor Power (read)", "%.3f", shooterTurn.getPower());
                            telemetry.addData("In Deadzone", absTx <= TURRET_DEADZONE_DEG ? "YES" : "NO");
                            telemetry.addData("Aligned", absTx <= TURRET_TOL_DEG ? "YES" : "NO");
                        } else {
                            telemetry.addData("ERROR", "spinner motor is NULL!");
                        }
                        
                        // Telemetry
                        telemetry.addData("Mode", "AUTO");
                        telemetry.addData("Tag Found", "YES");
                        telemetry.addData("Tag ID", tag.tagId);
                        telemetry.addData("tx (deg)", "%.2f", tx);
                        telemetry.addData("Distance (in)", "%.1f", tag.distance);
                        telemetry.addData("Angle (deg)", "%.1f", tag.angle);
                    } else {
                        // No tag detected, stop turret
                        if (shooterTurn != null) shooterTurn.setPower(0.0);
                        
                        telemetry.addData("Mode", "AUTO");
                        telemetry.addData("Tag Found", "NO");
                        telemetry.addData("tx (deg)", "N/A");
                        telemetry.addData("Turret Cmd", "0.000");
                    }
                } else {
                    // Limelight not found
                    if (shooterTurn != null) shooterTurn.setPower(0.0);
                    telemetry.addData("Mode", "AUTO - Limelight not found");
                }
            }
            
            // Additional telemetry
            telemetry.addData("---", "---");
            telemetry.addData("Press X", "Toggle auto/manual");
            if (manualMode) {
                telemetry.addData("Manual", "Use Left Stick X for turret");
            }
            telemetry.update();
        }
        
        // Stop motor when opmode ends
        if (shooterTurn != null) {
            shooterTurn.setPower(0.0);
        }
    }
}

