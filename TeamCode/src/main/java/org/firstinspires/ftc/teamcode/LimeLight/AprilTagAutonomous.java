package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.dumbMap;

/**
 * Example Autonomous program using April tag detection
 * 
 * This program demonstrates how to use the AprilTagDetector helper class
 * to navigate to April tags in the FTC Decode game.
 */
@Autonomous(name = "April Tag Autonomous", group = "LimeLight")
public class AprilTagAutonomous extends LinearOpMode {

    // April tag detector
    private AprilTagDetector aprilTagDetector;
    
    // Drive motors (adjust names to match your robot configuration)
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    
    // Drive parameters
    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.3;
    private static final double TARGET_DISTANCE = 12.0; // inches - how close to get to tag
    private static final double ANGLE_TOLERANCE = 2.0;  // degrees - acceptable angle error
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        // Initialize hardware
        initializeHardware();
        
        // Initialize dumbMap and get LimeLight instance
        dumbMap robot = new dumbMap(this);
        robot.init2();  // This initializes the LimeLight and other hardware
        
        if (robot.getLimeLight() == null) {
            telemetry.addData("Error", "LimeLight not found in hardware map!");
            telemetry.update();
            return;
        }
        
        // Initialize April tag detector with the LimeLight instance
        aprilTagDetector = new AprilTagDetector(robot.getLimeLight());
        
        // Switch to red pipeline for red section tags
        aprilTagDetector.switchToRedPipeline();
        
        telemetry.addData("Status", "April Tag Autonomous Ready");
        telemetry.addData("Instructions", "Press START to begin");
        telemetry.update();
        
        waitForStart();
        
        // Main autonomous routine
        runAutonomousRoutine();
        
        // Cleanup
        aprilTagDetector.stop();
    }
    
    /**
     * Initialize robot hardware
     */
    private void initializeHardware() {
        // Initialize drive motors (adjust these names to match your robot)
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        
        // Set motor directions (adjust as needed for your robot)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Set motor modes
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetry.addData("Hardware", "Initialized successfully");
    }
    
    /**
     * Main autonomous routine
     */
    private void runAutonomousRoutine() throws InterruptedException {
        
        telemetry.addData("Phase", "Searching for April tags...");
        telemetry.update();
        
        // Wait for April tag detection
        int timeout = 0;
        while (!aprilTagDetector.hasDetectedTags() && opModeIsActive() && timeout < 100) {
            sleep(100);
            timeout++;
        }
        
        if (!aprilTagDetector.hasDetectedTags()) {
            telemetry.addData("Error", "No April tags detected within timeout");
            telemetry.update();
            return;
        }
        
        // Get the closest red section tag
        AprilTagDetector.AprilTagResult targetTag = aprilTagDetector.getClosestTagFromSection("RED");
        
        if (!targetTag.isValid) {
            telemetry.addData("Error", "No valid red section tags found");
            telemetry.update();
            return;
        }
        
        telemetry.addData("Target", "Tag ID: %d, Distance: %.1f\", Angle: %.1f°", 
                         targetTag.tagId, targetTag.distance, targetTag.angle);
        telemetry.update();
        
        // Navigate to the target tag
        navigateToTag(targetTag);
        
        telemetry.addData("Status", "Autonomous routine completed");
        telemetry.update();
    }
    
    /**
     * Navigate to a specific April tag
     */
    private void navigateToTag(AprilTagDetector.AprilTagResult tag) throws InterruptedException {
        
        // Phase 1: Turn to face the tag
        if (Math.abs(tag.angle) > ANGLE_TOLERANCE) {
            telemetry.addData("Phase", "Turning to face tag...");
            telemetry.update();
            
            turnToAngle(tag.angle);
        }
        
        // Phase 2: Drive to the tag
        if (tag.distance > TARGET_DISTANCE) {
            telemetry.addData("Phase", "Driving to tag...");
            telemetry.update();
            
            driveDistance(tag.distance - TARGET_DISTANCE);
        }
        
        // Phase 3: Fine-tune position
        telemetry.addData("Phase", "Fine-tuning position...");
        telemetry.update();
        
        fineTunePosition();
    }
    
    /**
     * Turn robot to face a specific angle
     */
    private void turnToAngle(double targetAngle) throws InterruptedException {
        
        double turnPower = targetAngle > 0 ? TURN_SPEED : -TURN_SPEED;
        
        // Start turning
        setMotorPowers(-turnPower, turnPower, -turnPower, turnPower);
        
        // Monitor angle and stop when close enough
        while (opModeIsActive() && Math.abs(targetAngle) > ANGLE_TOLERANCE) {
            
            // Get updated tag information
            AprilTagDetector.AprilTagResult currentTag = aprilTagDetector.getClosestTagFromSection("RED");
            if (currentTag.isValid) {
                targetAngle = currentTag.angle;
                
                // Adjust turn direction if needed
                if (Math.abs(targetAngle) <= ANGLE_TOLERANCE) {
                    break;
                }
                
                turnPower = targetAngle > 0 ? TURN_SPEED : -TURN_SPEED;
                setMotorPowers(-turnPower, turnPower, -turnPower, turnPower);
            }
            
            sleep(50);
        }
        
        // Stop motors
        setMotorPowers(0, 0, 0, 0);
    }
    
    /**
     * Drive forward a specific distance
     */
    private void driveDistance(double distance) throws InterruptedException {
        
        if (distance <= 0) return;
        
        // Simple time-based driving (you can improve this with encoders)
        double driveTime = distance / (DRIVE_SPEED * 12.0); // Rough estimate: 12 inches per second at 0.5 power
        
        setMotorPowers(DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED);
        sleep((long)(driveTime * 1000));
        setMotorPowers(0, 0, 0, 0);
    }
    
    /**
     * Fine-tune position near the target
     */
    private void fineTunePosition() throws InterruptedException {
        
        // Get current tag information
        AprilTagDetector.AprilTagResult currentTag = aprilTagDetector.getClosestTagFromSection("RED");
        
        if (!currentTag.isValid) return;
        
        // Small adjustments based on current position
        if (Math.abs(currentTag.angle) > 1.0) {
            double fineTurnPower = currentTag.angle > 0 ? 0.1 : -0.1;
            setMotorPowers(-fineTurnPower, fineTurnPower, -fineTurnPower, fineTurnPower);
            sleep(200);
            setMotorPowers(0, 0, 0, 0);
        }
        
        if (currentTag.distance > TARGET_DISTANCE + 2.0) {
            setMotorPowers(0.2, 0.2, 0.2, 0.2);
            sleep(500);
            setMotorPowers(0, 0, 0, 0);
        }
    }
    
    /**
     * Set motor powers for all drive motors
     */
    private void setMotorPowers(double leftFrontPower, double leftRearPower, 
                               double rightFrontPower, double rightRearPower) {
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }
}
