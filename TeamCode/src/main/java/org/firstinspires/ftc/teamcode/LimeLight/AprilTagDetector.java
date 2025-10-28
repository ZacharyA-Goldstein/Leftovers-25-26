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
            return 60.0;  // Default to 60" when no valid result
        }
        
        // Get the first detected fiducial (AprilTag)
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            System.out.println("DEBUG - No fiducials detected");
            return 60.0;  // Default to 60" when no fiducials detected
        }
        
        LLResultTypes.FiducialResult fiducial = fiducials.get(0);
        
        // Get both X and Y angles in degrees
        double xDegrees = fiducial.getTargetXDegrees();
        yDegrees = fiducial.getTargetYDegrees();
        
        // Camera parameters (in inches)
        double cameraHeight = this.cameraHeight; // Height of camera from ground (9.5")
        double tagHeight = 30.0; // Height of AprilTag center from ground (user's setup)
        double heightDifference = tagHeight - cameraHeight; // 20.5" difference
        
        // Convert angles to radians for calculation
        double yRadians = Math.toRadians(yDegrees);
        double xRadians = Math.toRadians(xDegrees);
        
        // NEW CALIBRATION based on actual test data with 30" tag height:
        // Test 1: Y=15.1°, Actual=78.9"
        // Test 2: Y=16.0°, Actual=81.0"
        // Test 3: Y=15.7°, Actual=81.0"
        // Average: Y≈15.6°, Actual≈80.3"
        
        // Using trigonometry: distance = heightDifference / tan(cameraAngle + yAngle)
        // With camera angle = 0°, this simplifies to: distance = heightDifference / tan(yAngle)
        
        // Basic trigonometric calculation
        double trigDistance = heightDifference / Math.tan(yRadians);
        
        // Apply empirical correction factor based on test data
        // For Y=15.1°: trigDistance=75.9", actual=80" → factor = 80/75.9 = 1.05
        // This small correction accounts for measurement uncertainties and camera positioning
        double correctionFactor = 1.05;
        double baseDistance = trigDistance * correctionFactor;
        
        // Apply X angle correction for off-center viewing
        // When viewing from the side, actual distance is longer
        double xCorrection = 1.0 / Math.cos(xRadians);
        
        // Calculate final distance with corrections
        double calibratedDistance = baseDistance * xCorrection;
        
        // Ensure distance is within reasonable bounds
        calibratedDistance = Math.max(12.0, Math.min(calibratedDistance, this.maxDistance));
        
        // Debug output
        System.out.println("DEBUG - Distance Calculation:");
        System.out.println(String.format("  X Angle: %.1f°, Y Angle: %.1f°", xDegrees, yDegrees));
        System.out.println(String.format("  Base Distance: %.1f", baseDistance));
        System.out.println(String.format("  X Correction Factor: %.2f", xCorrection));
        System.out.println(String.format("  Calibrated Distance: %.1f", calibratedDistance));
        
        // Ensure the distance is within reasonable bounds
        return calibratedDistance;
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
