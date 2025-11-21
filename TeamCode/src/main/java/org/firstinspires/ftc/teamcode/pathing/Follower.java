package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A simple path follower implementation for differential drive robots
 */
public class Follower {
    // Motor objects for drive control
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private boolean teleopMode = false;
    private Pose currentPose;
    private Pose targetPose;
    
    // PID constants for position control
    private static final double kP_POSITION = 0.05;
    private static final double kP_HEADING = 0.5;
    
    // Maximum speeds
    private static final double MAX_SPEED = 0.8;
    private static final double MIN_SPEED = 0.1;
    
    // Tolerances (in inches and radians)
    private static final double POSITION_TOLERANCE = 1.0;  // 1 inch
    private static final double HEADING_TOLERANCE = 0.1;   // ~5.7 degrees

    public Follower() {
        // Default constructor for teleop mode
    }
    
    public Follower(HardwareMap hardwareMap) {
        // Initialize motors from hardware map
        try {
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            leftRear = hardwareMap.get(DcMotor.class, "leftBack");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightRear = hardwareMap.get(DcMotor.class, "rightBack");
            
            // Set motor directions
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            
            // Set zero power behavior
            DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
            leftFront.setZeroPowerBehavior(zeroPowerBehavior);
            leftRear.setZeroPowerBehavior(zeroPowerBehavior);
            rightFront.setZeroPowerBehavior(zeroPowerBehavior);
            rightRear.setZeroPowerBehavior(zeroPowerBehavior);
            
        } catch (Exception e) {
            // Handle any motor initialization errors
            throw new RuntimeException("Failed to initialize motors", e);
        }
        
        this.currentPose = new Pose(0, 0, 0);
        this.targetPose = null;
    }
    
    /**
     * Set the target pose to follow to
     * @param target The target pose
     */
    public void setTargetPose(Pose target) {
        this.targetPose = new Pose(target.x, target.y, target.heading);
    }
    
    /**
     * Update the current pose of the robot
     * @param currentPose The current pose of the robot
     */
    public void updatePose(Pose currentPose) {
        this.currentPose = new Pose(currentPose.x, currentPose.y, currentPose.heading);
    }
    
    /**
     * Calculate motor powers to reach the target pose
     * @return double array containing [leftPower, rightPower]
     */
    public double[] calculate() {
        if (targetPose == null) {
            return new double[]{0.0, 0.0};  // No target set
        }
        
        // Calculate position error
        double dx = targetPose.x - currentPose.x;
        double dy = targetPose.y - currentPose.y;
        double distance = Math.hypot(dx, dy);
        
        // Calculate target heading
        double targetHeading = Math.atan2(dy, dx);
        double headingError = normalizeAngle(targetHeading - currentPose.heading);
        
        // Calculate speed based on distance (with minimum speed)
        double speed = Math.max(Math.min(distance * kP_POSITION, MAX_SPEED), MIN_SPEED);
        
        // Calculate turn correction
        double turn = headingError * kP_HEADING;
        
        // Calculate left and right motor powers
        double leftPower = speed - turn;
        double rightPower = speed + turn;
        
        // Normalize powers to maintain differential drive constraints
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }
        
        return new double[]{leftPower, rightPower};
    }
    
    /**
     * Check if the robot has reached the target pose within tolerance
     * @return true if the target is reached, false otherwise
     */
    public boolean isTargetReached() {
        if (targetPose == null) return true;
        
        double dx = targetPose.x - currentPose.x;
        double dy = targetPose.y - currentPose.y;
        double distance = Math.hypot(dx, dy);
        double headingError = Math.abs(normalizeAngle(targetPose.heading - currentPose.heading));
        
        return (distance < POSITION_TOLERANCE) && (headingError < HEADING_TOLERANCE);
    }
    
    /**
     * Normalize an angle to the range [-π, π]
     */
    private double normalizeAngle(double angle) {
        return angle - 2 * Math.PI * Math.floor((angle + Math.PI) / (2 * Math.PI));
    }
    
    /**
     * Start teleoperated drive mode
     */
    public void startTeleopDrive() {
        this.teleopMode = true;
    }
    
    /**
     * Set movement vectors for teleoperated drive
     * @param forward Back/forward power (-1.0 to 1.0)
     * @param strafe Left/right power (-1.0 to 1.0)
     * @param rotate Rotational power (-1.0 to 1.0)
     * @param normalize Whether to normalize the power values
     */
    public void setTeleOpMovementVectors(double forward, double strafe, double rotate, boolean normalize) {
        if (!teleopMode) return;
        
        // Calculate mecanum wheel powers
        double leftFrontPower = forward + strafe + rotate;
        double leftRearPower = forward - strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double rightRearPower = forward + strafe - rotate;
        
        // Normalize if any power exceeds 1.0
        if (normalize) {
            double maxPower = Math.max(Math.max(
                Math.abs(leftFrontPower), 
                Math.abs(leftRearPower)), 
                Math.max(
                    Math.abs(rightFrontPower), 
                    Math.abs(rightRearPower)
                )
            );
            
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                leftRearPower /= maxPower;
                rightFrontPower /= maxPower;
                rightRearPower /= maxPower;
            }
        }
        
        // Set motor powers
        if (leftFront != null) leftFront.setPower(leftFrontPower);
        if (leftRear != null) leftRear.setPower(leftRearPower);
        if (rightFront != null) rightFront.setPower(rightFrontPower);
        if (rightRear != null) rightRear.setPower(rightRearPower);
    }
    
    /**
     * Get the average forward power from all drive motors
     * @return Average forward power (-1.0 to 1.0)
     */
    public double getAverageForwardPower() {
        if (leftFront == null || leftRear == null || rightFront == null || rightRear == null) {
            return 0.0;
        }
        double leftPower = (leftFront.getPower() + leftRear.getPower()) / 2.0;
        double rightPower = (rightFront.getPower() + rightRear.getPower()) / 2.0;
        return (leftPower + rightPower) / 2.0;
    }
    
    /**
     * Get the current pose of the robot
     * @return Current pose (x, y, heading)
     */
    public Pose getPose() {
        return currentPose;
    }
    
    /**
     * Update method to be called in the main loop
     */
    public void update() {
        // In teleop mode, this is handled by setTeleOpMovementVectors
        if (!teleopMode && targetPose != null) {
            // Autonomous path following logic would go here
            double[] powers = calculate();
            // Apply powers to motors if in autonomous mode
        }
    }
}
