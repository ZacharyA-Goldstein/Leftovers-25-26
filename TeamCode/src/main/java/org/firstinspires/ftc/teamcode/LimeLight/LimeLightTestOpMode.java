package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.dumbMap;

import java.util.List;

/**
 * Test OpMode for LimeLight April Tag detection
 * 
 * This OpMode demonstrates how to use the LimeLight camera through the dumbMap class
 * to detect April Tags and display their information.
 */
@TeleOp(name = "LimeLight Test", group = "Test") //test 
public class LimeLightTestOpMode extends LinearOpMode {
    private dumbMap robot;
    private int currentPipeline = 0;
    
    @Override
    public void runOpMode() {
        // Initialize the robot hardware
        robot = new dumbMap(this);
        robot.init2();  // This initializes the LimeLight and other hardware
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press START to begin detection");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (robot.limelight != null) {
                // Get the status and latest result
                LLStatus status = robot.limelight.getStatus();
                LLResult result = robot.limelight.getLatestResult();
                
                // Display LimeLight status
                telemetry.addLine("LimeLight Status:");
                telemetry.addData("Name", status.getName());
                telemetry.addData("Pipeline", "%d (%s)", 
                    status.getPipelineIndex(), 
                    status.getPipelineType());
                
                // Display target information if available
                if (result != null && result.isValid()) {
                    // Get basic target information
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = 0; // Area not directly available, we'll calculate from fiducials
                    
                    // Get fiducial (April Tag) results
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (!fiducials.isEmpty()) {
                        telemetry.addLine("\n--- TARGETS DETECTED ---");
                        for (LLResultTypes.FiducialResult fr : fiducials) {
                            telemetry.addData("Tag", "ID: %d, Family: %s", 
                                fr.getFiducialId(), fr.getFamily());
                            telemetry.addData("Position", "X: %.1f°, Y: %.1f°", 
                                fr.getTargetXDegrees(), fr.getTargetYDegrees());
                            ta = Math.max(ta, fr.getTargetArea());
                        }
                        telemetry.addLine("------------------------");
                    } else {
                        telemetry.addLine("\nNo targets detected");
                    }
                    
                    // Display basic targeting info
                    telemetry.addData("Target X", "%.1f°", tx);
                    telemetry.addData("Target Y", "%.1f°", ty);
                    telemetry.addData("Target Area", "%.1f%%", ta * 100);
                    
                    // Display latency information
                    telemetry.addData("Latency", "%.1fms", 
                        result.getCaptureLatency() + result.getTargetingLatency());
                } else {
                    telemetry.addLine("\nNo valid result from LimeLight");
                }
            } else {
                telemetry.addLine("LimeLight not initialized!");
            }
            
            // Add controls information
            telemetry.addLine("\nControls:");
            telemetry.addLine("X - Toggle LED");
            telemetry.addLine("A/B - Change pipeline");
            
            // Handle controller inputs
            if (gamepad1.x) {
                toggleLED();
                sleep(300); // Debounce
            }
            
            if (gamepad1.a) {
                changePipeline(1);
                sleep(300); // Debounce
            } else if (gamepad1.b) {
                changePipeline(-1);
                sleep(300); // Debounce
            }
            
            telemetry.update();
        }
        
        // Clean up
        if (robot.limelight != null) {
            robot.limelight.stop();
        }
    }
    
    /**
     * Toggle the LimeLight LED on/off
     */
    private void toggleLED() {
        // In the current API, there's no direct LED control method
        // We'll use the pipeline switch as a workaround
        // Pipeline 0: LED off (or default)
        // Pipeline 1: LED on
        if (robot.limelight != null) {
            int current = currentPipeline;
            int newPipeline = (current == 1) ? 0 : 1;
            robot.limelight.pipelineSwitch(newPipeline);
            currentPipeline = newPipeline;
            telemetry.addData("LED", newPipeline == 1 ? "ON" : "OFF");
        }
    }
    
    /**
     * Change the current pipeline (for different April Tag configurations)
     * @param delta The amount to change the pipeline by (can be negative)
     */
    private void changePipeline(int delta) {
        if (robot.limelight != null) {
            currentPipeline = Math.max(0, Math.min(currentPipeline + delta, 9)); // 0-9 pipelines
            robot.limelight.pipelineSwitch(currentPipeline);
            telemetry.addData("Pipeline", "Changed to %d", currentPipeline);
        }
    }
}
