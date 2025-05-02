package com.pedropathing.localization.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Encoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * This is the TwoWheelConstants class. It holds many constants and parameters for the Two Wheel Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

@Config
public class TwoWheelConstants {

    /** The number of inches per tick of the encoder for forward movement
     * Default Value: .001989436789 */
    public static double forwardTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for lateral movement (strafing)
     * Default Value: .001989436789 */
    public static double strafeTicksToInches = .001989436789;

    /** The y offset of the forward encoder (Deadwheel) from the center of the robot
     * Default Value: 1 */
    public static double forwardY = 1;

    /** The x offset of the strafe encoder (Deadwheel) from the center of the robot
     * Default Value: -2.5 */
    public static double strafeX = -2.5;

    /** The Hardware Map Name of the IMU (built-in IMU will be Port 0, "imu")
     * Default Value: "imu" */
    public static String IMU_HardwareMapName = "imu";

    /** The Hardware Map Name of the Forward Encoder (name of the motor port it is plugged into)
     * Default Value: "leftFront" */
    public static String forwardEncoder_HardwareMapName = "leftFront";

    /** The Hardware Map Name of the Strafe Encoder (name of the motor port it is plugged into)
     * Default Value: "rightRear" */
    public static String strafeEncoder_HardwareMapName = "rightRear";

    /** The Orientation of the IMU on the robot
     * Default Value: new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT) */
    public static RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    /** The direction of the forward encoder
     * Default Value: Encoder.REVERSE */
    public static double forwardEncoderDirection = Encoder.REVERSE;

    /** The direction of the strafe encoder
     * Default Value: Encoder.FORWARD */
    public static double strafeEncoderDirection = Encoder.FORWARD;

    /**
     * This creates a new TwoWheelConstants with default values.
     */
    public TwoWheelConstants() {
        defaults();
    }

    public TwoWheelConstants forwardTicksToInches(double forwardTicksToInches) {
        TwoWheelConstants.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    public TwoWheelConstants strafeTicksToInches(double strafeTicksToInches) {
        TwoWheelConstants.strafeTicksToInches = strafeTicksToInches;
        return this;
    }

    public TwoWheelConstants forwardY(double forwardY) {
        TwoWheelConstants.forwardY = forwardY;
        return this;
    }

    public TwoWheelConstants strafeX(double strafeX) {
        TwoWheelConstants.strafeX = strafeX;
        return this;
    }

    public TwoWheelConstants IMU_HardwareMapName(String IMU_HardwareMapName) {
        TwoWheelConstants.IMU_HardwareMapName = IMU_HardwareMapName;
        return this;
    }

    public TwoWheelConstants forwardEncoder_HardwareMapName(String forwardEncoder_HardwareMapName) {
        TwoWheelConstants.forwardEncoder_HardwareMapName = forwardEncoder_HardwareMapName;
        return this;
    }

    public TwoWheelConstants strafeEncoder_HardwareMapName(String strafeEncoder_HardwareMapName) {
        TwoWheelConstants.strafeEncoder_HardwareMapName = strafeEncoder_HardwareMapName;
        return this;
    }

    public TwoWheelConstants IMU_Orientation(RevHubOrientationOnRobot IMU_Orientation) {
        TwoWheelConstants.IMU_Orientation = IMU_Orientation;
        return this;
    }

    public TwoWheelConstants forwardEncoderDirection(double forwardEncoderDirection) {
        TwoWheelConstants.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public TwoWheelConstants strafeEncoderDirection(double strafeEncoderDirection) {
        TwoWheelConstants.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    /**
     * This sets the default values for the TwoWheelConstants.
     */
    public void defaults() {
        forwardTicksToInches = .001989436789;
        strafeTicksToInches = .001989436789;
        forwardY = 1;
        strafeX = -2.5;
        IMU_HardwareMapName = "imu";
        forwardEncoder_HardwareMapName = "leftFront";
        strafeEncoder_HardwareMapName = "rightRear";
        IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        forwardEncoderDirection = Encoder.REVERSE;
        strafeEncoderDirection = Encoder.FORWARD;
    }
}
