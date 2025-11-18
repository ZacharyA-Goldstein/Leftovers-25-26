# LimeLight April Tag Detection Setup Guide

This guide will help you set up LimeLight for April tag detection in the FTC Decode game.

## Hardware Setup

### 1. LimeLight Hardware Configuration
- Connect your LimeLight to a USB port on the Control Hub
- The LimeLight will appear in the Robot Configuration as an ethernet interface
- Note the IP address assigned to the Control Hub (displayed as "serial number")

### 2. Robot Configuration
1. Open Robot Configuration in the Driver Station app
2. Find your LimeLight device (it will show as an ethernet interface)
3. Tap on the LimeLight device
4. Rename it to "limelight" (this is the name used in the code)
5. Configure the LimeLight's IP address if needed

### 3. LimeLight Web Interface Setup
1. Connect to the LimeLight's WiFi network or access via ethernet
2. Open a web browser and go to the LimeLight's IP address (usually 10.0.0.1)
3. Upload your RedPipeline.vpr file to pipeline 0
4. Configure camera settings:
   - Resolution: 320x240 or 640x480
   - FPS: 30
   - Exposure: Auto or manual adjustment
   - Brightness: Adjust for your lighting conditions

## Pipeline Configuration

### Red Section Pipeline (Pipeline 0)
- Upload your `RedPipeline.vpr` file to pipeline 0
- This pipeline should be optimized for detecting red section April tags (IDs 1-6)
- Configure the pipeline to output fiducial (April tag) results

### Blue Section Pipeline (Pipeline 1)
- Create or upload a blue section pipeline to pipeline 1
- This pipeline should be optimized for detecting blue section April tags (IDs 7-12)

## Camera Calibration

### Important Parameters to Adjust
The following parameters in the code should be calibrated for your specific robot setup:

```java
// In LimeLightTester.java and AprilTagDetector.java
private static final double CAMERA_HEIGHT = 12.0; // inches - height of camera above ground
private static final double CAMERA_ANGLE = 15.0;  // degrees - downward angle of camera
```

### How to Calibrate
1. **Camera Height**: Measure the distance from the ground to your camera lens
2. **Camera Angle**: Measure the downward angle of your camera (0° = horizontal, 90° = straight down)
3. **Distance Testing**: 
   - Place April tags at known distances (12", 24", 36", etc.)
   - Run the LimeLightTester and check if calculated distances match actual distances
   - Adjust camera parameters if needed

## Usage Examples

### Basic Detection (LimeLightTester)
```java
// Run the LimeLightTester OpMode to test detection
// This will show all detected April tags with distance and angle information
```

### Integration in Autonomous (AprilTagAutonomous)
```java
// Use the AprilTagDetector helper class in your autonomous programs
AprilTagDetector detector = new AprilTagDetector(hardwareMap);
detector.switchToRedPipeline();

// Get the closest red section tag
AprilTagDetector.AprilTagResult target = detector.getClosestTagFromSection("RED");
if (target.isValid) {
    // Navigate to the tag using target.distance and target.angle
}
```

## Troubleshooting

### Common Issues

1. **"Failed to initialize LimeLight"**
   - Check that LimeLight is properly connected to Control Hub
   - Verify the device name is "limelight" in Robot Configuration
   - Ensure LimeLight is powered on and functioning

2. **"No April tags found"**
   - Check that your pipeline is correctly configured for April tag detection
   - Verify April tags are within the camera's field of view
   - Ensure proper lighting conditions
   - Check that the pipeline is set to output fiducial results

3. **Inaccurate distance calculations**
   - Recalibrate camera height and angle parameters
   - Test with April tags at known distances
   - Ensure camera is securely mounted and not moving

4. **Poor detection performance**
   - Adjust camera exposure and brightness settings
   - Ensure April tags are not damaged or dirty
   - Check for glare or reflections on April tags
   - Verify pipeline is optimized for your lighting conditions

### Performance Tips

1. **Lighting**: Ensure consistent, even lighting on April tags
2. **Mounting**: Securely mount the camera to prevent vibration
3. **Pipeline Optimization**: Fine-tune your pipeline for your specific environment
4. **Distance**: April tags work best within 3-10 feet
5. **Angle**: Keep April tags roughly perpendicular to the camera view

## April Tag IDs for FTC Decode

### Red Section (Pipeline 0)
- Tag IDs: 1, 2, 3, 4, 5, 6
- Use these for red alliance autonomous routines

### Blue Section (Pipeline 1)  
- Tag IDs: 7, 8, 9, 10, 11, 12
- Use these for blue alliance autonomous routines

## Code Files

- `LimeLightTester.java` - Test OpMode for April tag detection
- `AprilTagDetector.java` - Helper class for easy integration
- `AprilTagAutonomous.java` - Example autonomous program
- `README_LimeLight_Setup.md` - This setup guide

## Next Steps

1. Set up your LimeLight hardware and configuration
2. Upload your RedPipeline.vpr file
3. Calibrate camera parameters for your robot
4. Test detection using LimeLightTester
5. Integrate AprilTagDetector into your autonomous programs
6. Fine-tune detection and navigation parameters

For more information, refer to the LimeLight documentation and FTC April tag specifications.
