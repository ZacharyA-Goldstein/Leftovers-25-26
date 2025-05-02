package com.pedropathing.localization.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Encoder;

/**
 * This is the ThreeWheelConstants class. It holds many constants and parameters for the Three Wheel Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

@Config
public class ThreeWheelConstants {

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

    /** The name of the Left Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "leftFront" */
    public static String leftEncoder_HardwareMapName = "leftFront";

    /** The name of the Right Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "rightRear" */
    public static String rightEncoder_HardwareMapName = "rightRear";

    /** The name of the Strafe Encoder in the hardware map (name of the motor port it is plugged into)
     * Default Value: "rightFront" */
    public static String strafeEncoder_HardwareMapName = "rightFront";

    /** The direction of the Left Encoder
     * Default Value: Encoder.REVERSE */
    public static double leftEncoderDirection = Encoder.REVERSE;

    /** The direction of the Right Encoder
     * Default Value: Encoder.REVERSE */
    public static double rightEncoderDirection = Encoder.REVERSE;

    /** The direction of the Strafe Encoder
     * Default Value: Encoder.FORWARD */
    public static double strafeEncoderDirection = Encoder.FORWARD;

    /**
     * This creates a new ThreeWheelConstants with default values.
     */
    public ThreeWheelConstants() {
        defaults();
    }

    public ThreeWheelConstants forwardTicksToInches(double forwardTicksToInches) {
        ThreeWheelConstants.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    public ThreeWheelConstants strafeTicksToInches(double strafeTicksToInches) {
        ThreeWheelConstants.strafeTicksToInches = strafeTicksToInches;
        return this;
    }

    public ThreeWheelConstants turnTicksToInches(double turnTicksToInches) {
        ThreeWheelConstants.turnTicksToInches = turnTicksToInches;
        return this;
    }

    public ThreeWheelConstants leftY(double leftY) {
        ThreeWheelConstants.leftY = leftY;
        return this;
    }

    public ThreeWheelConstants rightY(double rightY) {
        ThreeWheelConstants.rightY = rightY;
        return this;
    }

    public ThreeWheelConstants strafeX(double strafeX) {
        ThreeWheelConstants.strafeX = strafeX;
        return this;
    }

    public ThreeWheelConstants leftEncoder_HardwareMapName(String leftEncoder_HardwareMapName) {
        ThreeWheelConstants.leftEncoder_HardwareMapName = leftEncoder_HardwareMapName;
        return this;
    }

    public ThreeWheelConstants rightEncoder_HardwareMapName(String rightEncoder_HardwareMapName) {
        ThreeWheelConstants.rightEncoder_HardwareMapName = rightEncoder_HardwareMapName;
        return this;
    }

    public ThreeWheelConstants strafeEncoder_HardwareMapName(String strafeEncoder_HardwareMapName) {
        ThreeWheelConstants.strafeEncoder_HardwareMapName = strafeEncoder_HardwareMapName;
        return this;
    }

    public ThreeWheelConstants leftEncoderDirection(double leftEncoderDirection) {
        ThreeWheelConstants.leftEncoderDirection = leftEncoderDirection;
        return this;
    }

    public ThreeWheelConstants rightEncoderDirection(double rightEncoderDirection) {
        ThreeWheelConstants.rightEncoderDirection = rightEncoderDirection;
        return this;
    }

    public ThreeWheelConstants strafeEncoderDirection(double strafeEncoderDirection) {
        ThreeWheelConstants.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    public void defaults() {
        forwardTicksToInches = .001989436789;
        strafeTicksToInches = .001989436789;
        turnTicksToInches = .001989436789;
        leftY = 1;
        rightY = -1;
        strafeX = -2.5;
        leftEncoder_HardwareMapName = "leftFront";
        rightEncoder_HardwareMapName = "rightRear";
        strafeEncoder_HardwareMapName = "rightFront";
        leftEncoderDirection = Encoder.REVERSE;
        rightEncoderDirection = Encoder.REVERSE;
        strafeEncoderDirection = Encoder.FORWARD;
    }
}
