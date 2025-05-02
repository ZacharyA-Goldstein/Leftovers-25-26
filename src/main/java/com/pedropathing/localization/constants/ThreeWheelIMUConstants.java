package com.pedropathing.localization.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Encoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * This is the ThreeWheelIMUConstants class. It holds many constants and parameters for the Three Wheel + IMU Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

@Config
public class ThreeWheelIMUConstants {

    /** The number of inches per tick of the encoder for forward movement
     * Default Value: .001989436789 */
    public static double forwardTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for lateral movement (strafing)
     * Default Value: .001989436789 */
    public static double strafeTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for turning
     * Default Value: .001989436789 */
    public static double turnTicksToInches = .001989436789;

    /** The Y Offset of the Left Encoder (Deadwheel) from the center of the robot
     * Default Value: 1 */
    public static double leftY = 1;

    /** The Y Offset of the Right Encoder (Deadwheel) from the center of the robot
     * Default Value: -1 */
    public static double rightY = -1;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot
     * Default Value: -2.5 */
    public static double strafeX = -2.5;

    /** The Hardware Map Name of the IMU (built-in IMU will be Port 0, "imu")
     * Default Value: "imu" */
    public static String IMU_HardwareMapName = "imu";

    /** The name of the Left Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "leftFront" */
    public static String leftEncoder_HardwareMapName = "leftFront";

    /** The name of the Right Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "rightRear" */
    public static String rightEncoder_HardwareMapName = "rightRear";

    /** The name of the Strafe Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value:Default Value: "rightFront" */
    public static String strafeEncoder_HardwareMapName = "rightFront";

    /** The Orientation of the Control Hub (for IMU) on the Robot
     * Default Value: new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT) */
    public static RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    /** The direction of the Left Encoder
     * Default Value: Encoder.REVERSE */
    public static double leftEncoderDirection = Encoder.REVERSE;

    /** The direction of the Right Encoder
     * Default Value: Encoder.FORWARD */
    public static double rightEncoderDirection = Encoder.REVERSE;

    /** The direction of the Strafe Encoder
     * Default Value: Encoder.FORWARD */
    public static double strafeEncoderDirection = Encoder.FORWARD;

    /**
     * This creates a new ThreeWheelIMUConstants with default values.
     */
    public ThreeWheelIMUConstants() {
        defaults();
    }

    public ThreeWheelIMUConstants forwardTicksToInches(double forwardTicksToInches) {
        ThreeWheelIMUConstants.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    public ThreeWheelIMUConstants strafeTicksToInches(double strafeTicksToInches) {
        ThreeWheelIMUConstants.strafeTicksToInches = strafeTicksToInches;
        return this;
    }

    public ThreeWheelIMUConstants turnTicksToInches(double turnTicksToInches) {
        ThreeWheelIMUConstants.turnTicksToInches = turnTicksToInches;
        return this;
    }

    public ThreeWheelIMUConstants leftY(double leftY) {
        ThreeWheelIMUConstants.leftY = leftY;
        return this;
    }

    public ThreeWheelIMUConstants rightY(double rightY) {
        ThreeWheelIMUConstants.rightY = rightY;
        return this;
    }

    public ThreeWheelIMUConstants strafeX(double strafeX) {
        ThreeWheelIMUConstants.strafeX = strafeX;
        return this;
    }

    public ThreeWheelIMUConstants IMU_HardwareMapName(String IMU_HardwareMapName) {
        ThreeWheelIMUConstants.IMU_HardwareMapName = IMU_HardwareMapName;
        return this;
    }

    public ThreeWheelIMUConstants leftEncoder_HardwareMapName(String leftEncoder_HardwareMapName) {
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = leftEncoder_HardwareMapName;
        return this;
    }

    public ThreeWheelIMUConstants rightEncoder_HardwareMapName(String rightEncoder_HardwareMapName) {
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = rightEncoder_HardwareMapName;
        return this;
    }

    public ThreeWheelIMUConstants strafeEncoder_HardwareMapName(String strafeEncoder_HardwareMapName) {
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = strafeEncoder_HardwareMapName;
        return this;
    }

    public ThreeWheelIMUConstants IMU_Orientation(RevHubOrientationOnRobot IMU_Orientation) {
        ThreeWheelIMUConstants.IMU_Orientation = IMU_Orientation;
        return this;
    }

    public ThreeWheelIMUConstants leftEncoderDirection(double leftEncoderDirection) {
        ThreeWheelIMUConstants.leftEncoderDirection = leftEncoderDirection;
        return this;
    }

    public ThreeWheelIMUConstants rightEncoderDirection(double rightEncoderDirection) {
        ThreeWheelIMUConstants.rightEncoderDirection = rightEncoderDirection;
        return this;
    }

    public ThreeWheelIMUConstants strafeEncoderDirection(double strafeEncoderDirection) {
        ThreeWheelIMUConstants.strafeEncoderDirection = strafeEncoderDirection;
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
