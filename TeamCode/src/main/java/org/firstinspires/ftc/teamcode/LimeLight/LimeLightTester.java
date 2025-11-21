package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.acmerobotics.dashboard.config.Config;



import org.firstinspires.ftc.teamcode.dumbMap;
//test
/**
 * LimeLight April Tag Detection for FTC
 * 
 * This OpMode demonstrates how to use LimeLight to detect April Tag 24 and calculate
 * distance and angle information.
 * 
 * Features:
 * - April Tag 24 detection
 * - Distance calculation to the tag
 * - Angle/heading calculation
 * - Telemetry output for debugging
 */
@Config
@TeleOp(name = "LimeLight April Tag 24 Detector", group = "LimeLight")
public class LimeLightTester extends LinearOpMode {

    // April Tag ID to detect
    private static final int TARGET_TAG_ID = 24;
    
    // Camera parameters for distance calculation
    // These values should be calibrated for your specific camera setup
    private static final double CAMERA_HEIGHT = 9.5; // inches - height of camera above ground
    private static final double CAMERA_ANGLE = 0.0;  // degrees - downward angle of camera
    private static final double MAX_DISTANCE = 150.0; // Maximum valid distance in inches
    
    // AprilTag detector and robot instance
    private AprilTagDetector aprilTagDetector;
    private dumbMapLime robot;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot hardware
        robot = new dumbMapLime(this);
        robot.init2();  // This initializes the LimeLight and other hardware
        
        if (robot.getLimeLight() == null) {
            telemetry.addData("Error", "LimeLight not found in hardware map!");
            telemetry.update();
            while (opModeIsActive()) {
                // Do nothing
            }
            return;
        }

        // Initialize AprilTag detector with the LimeLight from dumbMap
        aprilTagDetector = new AprilTagDetector(robot.getLimeLight(), CAMERA_HEIGHT, CAMERA_ANGLE, MAX_DISTANCE);
        
        // Explicitly switch to pipeline 0 (RedPipeline.vpr) to ensure AprilTag detection is active
        if (robot.getLimeLight() != null) {
            robot.getLimeLight().pipelineSwitch(0);
            sleep(100); // Give the pipeline time to switch
        }
        
        // Set up telemetry
        telemetry.setMsTransmissionInterval(50); // 20Hz update rate
        
        telemetry.addData("Status", "April Tag 24 Detector Ready");
        telemetry.addData("Instructions", "Press START to begin detection");
        telemetry.addData("Target Tag", "ID: %d", TARGET_TAG_ID);
        
        // Display LimeLight status
        if (robot.getLimeLight() != null) {
            Limelight3A limelight = robot.getLimeLight();
            telemetry.addData("LimeLight", "Initialized successfully");
            telemetry.addData("LimeLight Status", limelight.getStatus().toString());
        }
        
        telemetry.update();
        
        waitForStart();
        
        // Main detection loop
        while (opModeIsActive()) {
            // Look specifically for tag 24
            AprilTagDetector.AprilTagResult tagResult = aprilTagDetector.getTagById(TARGET_TAG_ID);
            
            // Debug: Show what the Limelight is actually returning
            if (robot.getLimeLight() != null) {
                LLResult result = robot.getLimeLight().getLatestResult();
                
                if (result != null) {
                    telemetry.addData("Limelight Result Valid", result.isValid());
                    telemetry.addData("Fiducial Count", result.getFiducialResults().size());
                    if (!result.getFiducialResults().isEmpty()) {
                        telemetry.addData("First Fiducial ID", result.getFiducialResults().get(0).getFiducialId());
                    }
                }
            }
            
            if (tagResult.isValid) {
                // Display tag information
                telemetry.addLine("--- April Tag Detected ---");
                telemetry.addData("Tag ID", tagResult.tagId);
                telemetry.addData("Distance", "%.1f inches", tagResult.distance);
                telemetry.addData("Angle", "%.1f degrees", tagResult.angle);
                telemetry.addData("X Degrees", "%.1f°", tagResult.xDegrees);
                telemetry.addData("Y Degrees", "%.1f°", tagResult.yDegrees);
                telemetry.addData("Family", tagResult.family);
                telemetry.addLine("------------------------");
                
                // Provide navigation guidance
                if (Math.abs(tagResult.angle) > 5.0) {
                    telemetry.addData("Action", "Turn %s %.1f°", 
                        tagResult.angle > 0 ? "right" : "left", 
                        Math.abs(tagResult.angle));
                } else {
                    telemetry.addData("Action", "Drive forward %.1f\"", tagResult.distance);
                }
            } else {
                telemetry.addLine("April Tag 24 not detected");
                telemetry.addLine("Make sure the tag is in view");
                
                // Show LimeLight status for debugging
                if (robot.getLimeLight() != null) {
                    telemetry.addData("LimeLight Status", 
                        robot.getLimeLight().getStatus().toString());
                }
            }
            
            // Update telemetry
            telemetry.update();
            
            // Small delay to prevent overwhelming the system
            sleep(50);
        }
        
        // Cleanup
        aprilTagDetector.stop();
    }
}
