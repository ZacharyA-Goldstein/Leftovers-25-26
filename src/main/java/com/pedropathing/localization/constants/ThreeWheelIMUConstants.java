package com.pedropathing.localization.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Encoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * This is the ThreeWheelIMUConstants class. It holds many constants and parameters for the Three Wheel + IMU Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */


public class ThreeWheelIMUConstants {

    /** The number of inches per tick of the encoder for forward movement
     * Default Value: .001989436789 */
    public double forwardTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for lateral movement (strafing)
     * Default Value: .001989436789 */
    public double strafeTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for turning
     * Default Value: .001989436789 */
    public double turnTicksToInches = .001989436789;

    /** The Y Offset of the Left Encoder (Deadwheel) from the center of the robot
     * Default Value: 1 */
    public double leftY = 1;

    /** The Y Offset of the Right Encoder (Deadwheel) from the center of the robot
     * Default Value: -1 */
    public double rightY = -1;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot
     * Default Value: -2.5 */
    public double strafeX = -2.5;

    /** The Hardware Map Name of the IMU (built-in IMU will be Port 0, "imu")
     * Default Value: "imu" */
    public String IMU_HardwareMapName = "imu";

    /** The name of the Left Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "leftFront" */
    public String leftEncoder_HardwareMapName = "leftFront";

    /** The name of the Right Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "rightRear" */
    public String rightEncoder_HardwareMapName = "rightRear";

    /** The name of the Strafe Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value:Default Value: "rightFront" */
    public String strafeEncoder_HardwareMapName = "rightFront";

    /** The Orientation of the Control Hub (for IMU) on the Robot
     * Default Value: new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT) */
    public RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    /** The direction of the Left Encoder
     * Default Value: Encoder.REVERSE */
    public double leftEncoderDirection = Encoder.REVERSE;

    /** The direction of the Right Encoder
     * Default Value: Encoder.FORWARD */
    public double rightEncoderDirection = Encoder.REVERSE;

    /** The direction of the Strafe Encoder
     * Default Value: Encoder.FORWARD */
    public double strafeEncoderDirection = Encoder.FORWARD;

    /**
     * This creates a new ThreeWheelIMUConstants with default values.
     */
    public ThreeWheelIMUConstants() {
        defaults();
    }

    public ThreeWheelIMUConstants forwardTicksToInches(double forwardTicksToInches) {
        this.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    public ThreeWheelIMUConstants strafeTicksToInches(double strafeTicksToInches) {
        this.strafeTicksToInches = strafeTicksToInches;
        return this;
    }

    public ThreeWheelIMUConstants turnTicksToInches(double turnTicksToInches) {
        this.turnTicksToInches = turnTicksToInches;
        return this;
    }

    public ThreeWheelIMUConstants leftY(double leftY) {
        this.leftY = leftY;
        return this;
    }

    public ThreeWheelIMUConstants rightY(double rightY) {
        this.rightY = rightY;
        return this;
    }

    public ThreeWheelIMUConstants strafeX(double strafeX) {
        this.strafeX = strafeX;
        return this;
    }

    public ThreeWheelIMUConstants IMU_HardwareMapName(String IMU_HardwareMapName) {
        this.IMU_HardwareMapName = IMU_HardwareMapName;
        return this;
    }

    public ThreeWheelIMUConstants leftEncoder_HardwareMapName(String leftEncoder_HardwareMapName) {
        this.leftEncoder_HardwareMapName = leftEncoder_HardwareMapName;
        return this;
    }

    public ThreeWheelIMUConstants rightEncoder_HardwareMapName(String rightEncoder_HardwareMapName) {
        this.rightEncoder_HardwareMapName = rightEncoder_HardwareMapName;
        return this;
    }

    public ThreeWheelIMUConstants strafeEncoder_HardwareMapName(String strafeEncoder_HardwareMapName) {
        this.strafeEncoder_HardwareMapName = strafeEncoder_HardwareMapName;
        return this;
    }

    public ThreeWheelIMUConstants IMU_Orientation(RevHubOrientationOnRobot IMU_Orientation) {
        this.IMU_Orientation = IMU_Orientation;
        return this;
    }

    public ThreeWheelIMUConstants leftEncoderDirection(double leftEncoderDirection) {
        this.leftEncoderDirection = leftEncoderDirection;
        return this;
    }

    public ThreeWheelIMUConstants rightEncoderDirection(double rightEncoderDirection) {
        this.rightEncoderDirection = rightEncoderDirection;
        return this;
    }

    public ThreeWheelIMUConstants strafeEncoderDirection(double strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    public void defaults() {
        forwardTicksToInches = .001989436789;
        strafeTicksToInches = .001989436789;
        turnTicksToInches = .001989436789;
        leftY = 1;
        rightY = -1;
        strafeX = -2.5;
        IMU_HardwareMapName = "imu";
        leftEncoder_HardwareMapName = "leftFront";
        rightEncoder_HardwareMapName = "rightRear";
        strafeEncoder_HardwareMapName = "rightFront";
        IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        leftEncoderDirection = Encoder.REVERSE;
        rightEncoderDirection = Encoder.REVERSE;
        strafeEncoderDirection = Encoder.FORWARD;
    }
}
