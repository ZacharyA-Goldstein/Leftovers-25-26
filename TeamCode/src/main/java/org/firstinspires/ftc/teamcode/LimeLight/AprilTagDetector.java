package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Helper class for April tag detection using LimeLight
 * 
 * This class provides a simple interface for detecting April tags and getting
 * distance/angle information for autonomous navigation.
 */
public class AprilTagDetector { //test
    
    // LimeLight hardware
    private Limelight3A limelight;
    
    // Camera parameters for distance calculation
    private double cameraHeight;
    private double cameraAngle;
    private double maxDistance;
    
    // April tag IDs for FTC Decode game
    private static final int[] RED_TAG_IDS = {1, 2, 3, 4, 5, 6};
    private static final int[] BLUE_TAG_IDS = {7, 8, 9, 10, 11, 12};
    
    // Detection result class
    public static class AprilTagResult {
        public int tagId;
        public String family;
        public double xDegrees;
        public double yDegrees;
        public double distance;
        public double angle;
        public String section;
        public boolean isValid;
        
        public AprilTagResult(int tagId, String family, double xDegrees, double yDegrees, 
                            double distance, double angle, String section) {
            this.tagId = tagId;
            this.family = family;
            this.xDegrees = xDegrees;
            this.yDegrees = yDegrees;
            this.distance = distance;
            this.angle = angle;
            this.section = section;
            this.isValid = true;
        }
        
        public AprilTagResult() {
            this.isValid = false;
        }
    }
    
    /**
     * Constructor with default camera parameters
     */
    public AprilTagDetector(Limelight3A limelight) {
        this(limelight, 12.0, 15.0, 120.0); // Default: 12" height, 15° angle, 120" max distance
    }
    
    /**
     * Constructor with custom camera parameters
     */
    public AprilTagDetector(Limelight3A limelight, double cameraHeight, double cameraAngle, double maxDistance) {
        if (limelight == null) {
            throw new IllegalArgumentException("LimeLight instance cannot be null");
        }
        
        this.limelight = limelight;
        this.cameraHeight = cameraHeight;
        this.cameraAngle = cameraAngle;
        this.maxDistance = maxDistance;
        
        // Start the LimeLight if it's not already started
        try {
            limelight.start();
        } catch (Exception e) {
            // Silently continue - the device might already be started
            // Error will be visible in the LimeLight status if there's an issue
        }
    }
    
    /**
     * Switch to red section pipeline (pipeline 0)
     */
    public void switchToRedPipeline() {
        if (limelight != null) {
            limelight.pipelineSwitch(0);
        }
    }
    
    /**
     * Switch to blue section pipeline (pipeline 1)
     */
    public void switchToBluePipeline() {
        if (limelight != null) {
            limelight.pipelineSwitch(1);
        }
    }
    
    /**
     * Get all detected April tags
     */
    public List<AprilTagResult> getAllDetectedTags() {
        List<AprilTagResult> results = new ArrayList<>();
        
        if (limelight == null) return results;
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return results;
        
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        
        for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
            int tagId = fiducial.getFiducialId();
            String family = fiducial.getFamily();
            double xDegrees = fiducial.getTargetXDegrees();
            double yDegrees = fiducial.getTargetYDegrees();
            
            double distance = calculateDistance(yDegrees);
            double angle = xDegrees;
            String section = getTagSection(tagId);
            
            results.add(new AprilTagResult(tagId, family, xDegrees, yDegrees, distance, angle, section));
        }
        
        return results;
    }
    
    /**
     * Get the closest April tag
     */
    public AprilTagResult getClosestTag() {
        List<AprilTagResult> tags = getAllDetectedTags();
        if (tags.isEmpty()) return new AprilTagResult();
        
        AprilTagResult closest = tags.get(0);
        for (AprilTagResult tag : tags) {
            if (tag.distance < closest.distance) {
                closest = tag;
            }
        }
        
        return closest;
    }
    
    /**
     * Get the closest tag from a specific section (RED or BLUE)
     */
    public AprilTagResult getClosestTagFromSection(String targetSection) {
        List<AprilTagResult> tags = getAllDetectedTags();
        AprilTagResult closest = new AprilTagResult();
        double minDistance = Double.MAX_VALUE;
        
        for (AprilTagResult tag : tags) {
            if (tag.section.equals(targetSection) && tag.distance < minDistance) {
                closest = tag;
                minDistance = tag.distance;
            }
        }
        
        return closest;
    }
    
    /**
     * Get a specific tag by ID
     */
    public AprilTagResult getTagById(int targetId) {
        List<AprilTagResult> tags = getAllDetectedTags();
        
        for (AprilTagResult tag : tags) {
            if (tag.tagId == targetId) {
                return tag;
            }
        }
        
        return new AprilTagResult(); // Not found
    }
    
    /**
     * Check if any April tags are currently detected
     */
    public boolean hasDetectedTags() {
        return !getAllDetectedTags().isEmpty();
    }
    
    /**
     * Get LimeLight status information
     */
    public LLStatus getStatus() {
        if (limelight == null) return null;
        return limelight.getStatus();
    }
    
    /**
     * Stop the LimeLight (call this when done)
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
    
    /**
     * Calculate distance to April tag based on vertical angle
     * 
     * @param yDegrees The vertical angle to the target in degrees (positive = above center, negative = below center)
     * @return Distance to the target in inches
     */
    private double calculateDistance(double yDegrees) {
        // Get the latest result which contains the fiducial data
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            System.out.println("DEBUG - No valid result from Limelight");
            return 120.0;
        }
        
        // Get the first detected fiducial (AprilTag)
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            System.out.println("DEBUG - No fiducials detected");
            return 120.0;
        }
        
        LLResultTypes.FiducialResult fiducial = fiducials.get(0);
        
        // Get both X and Y angles in degrees
        double xDegrees = fiducial.getTargetXDegrees();
        yDegrees = fiducial.getTargetYDegrees();
        
        // Camera parameters
        double cameraHeight = this.cameraHeight; // Height of camera from ground
        double tagHeight = 18.0; // Height of AprilTag center from ground
        double heightDifference = Math.abs(tagHeight - cameraHeight);
        
        // Based on test data, we'll use a combination of Y angle and X angle correction
        // The Y angle is the primary factor for distance, while X angle provides a small correction
        
        // Calculate base distance using Y angle (main factor)
        // Using a linear approximation based on test data
        double baseDistance;
        if (yDegrees > 5.0) {
            // For closer distances (larger Y angles)
            baseDistance = 100.0 - (yDegrees * 8.0);  // Adjusted for better accuracy
        } else {
            // For farther distances (smaller Y angles)
            baseDistance = 130.0 - (yDegrees * 14.0);  // Adjusted for better accuracy
        }
        
        // Apply X angle correction (smaller effect)
        // The more we're looking to the side, the larger the distance correction should be
        double xCorrection = Math.abs(xDegrees) * 0.15;  // Reduced from 0.2 to 0.15 inches per degree
        
        // Apply the correction (add more distance when looking to the side)
        double correctedDistance = baseDistance + xCorrection;
        
        // Apply a scaling factor based on test data
        double finalDistance = correctedDistance * 0.85;  // Reduced from 0.75 to 0.85 for better accuracy
        
        // Debug output
        System.out.println("DEBUG - Distance Calculation:");
        System.out.println(String.format("  X Angle: %.1f°, Y Angle: %.1f°", xDegrees, yDegrees));
        System.out.println(String.format("  Base Distance: %.1f", baseDistance));
        System.out.println(String.format("  X Correction: %.1f", xCorrection));
        System.out.println(String.format("  Corrected Distance: %.1f", correctedDistance));
        System.out.println(String.format("  Final Distance: %.1f", finalDistance));
        
        // Ensure the distance is within reasonable bounds
        return Math.max(12.0, Math.min(finalDistance, 120.0));
    }
    
    /**
     * Determine if a tag belongs to red or blue section
     */
    private String getTagSection(int tagId) {
        for (int id : RED_TAG_IDS) {
            if (tagId == id) return "RED";
        }
        for (int id : BLUE_TAG_IDS) {
            if (tagId == id) return "BLUE";
        }
        return "UNKNOWN";
    }
    
    /**
     * Update camera parameters (useful for calibration)
     */
    public void updateCameraParameters(double cameraHeight, double cameraAngle, double maxDistance) {
        this.cameraHeight = cameraHeight;
        this.cameraAngle = cameraAngle;
        this.maxDistance = maxDistance;
    }
}
