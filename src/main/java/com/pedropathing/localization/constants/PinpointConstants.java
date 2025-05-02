package com.pedropathing.localization.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the PinpointConstants class. It holds many constants and parameters for the Pinpoint Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

@Config
public class PinpointConstants {

    /** The Y Offset of the Forward Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: 1 */
    public static double forwardY = 1;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: -2.5 */
    public static double strafeX = -2.5;

    /** The Unit of Distance that the Pinpoint uses to measure distance
     * Default Value: DistanceUnit.INCH */
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;

    /** The name of the Pinpoint in the hardware map (name of the I2C port it is plugged into)
     * Default Value: "pinpoint" */
    public static String hardwareMapName = "pinpoint";

    /** Use custom yaw scalar for the Pinpoint (overrides the calibration of the Pinpoint
     * Default Value: false */
    public static boolean useYawScalar = false;

    /** Custom Yaw Scalar for the Pinpoint (overrides the calibration of the Pinpoint
     * @see #useYawScalar
     * Default Value: 1.0 */
    public static double yawScalar = 1.0;

    /** Use custom encoder resolution for the Pinpoint
     * Default Value: false */
    public static boolean useCustomEncoderResolution = false;

    /** The Encoder Resolution for the Pinpoint. Used if useCustomEncoderResolution is false
     * @see #useCustomEncoderResolution
     * Default Value: GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD */
    public static GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /** The Encoder Resolution for the Pinpoint. Only used if useCustomEncoderResolution is true
     * @see #useCustomEncoderResolution
     * Default Value: 13.26291192 */
    public static double customEncoderResolution = 13.26291192;

    /** The Encoder Direction for the Forward Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.REVERSED */
    public static GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    /** The Encoder Direction for the Strafe Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.FORWARD */
    public static GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    /**
     * This creates a new PinpointConstants with default values.
     */
    public PinpointConstants() {
        defaults();
    }

    public PinpointConstants forwardY(double forwardY) {
        PinpointConstants.forwardY = forwardY;
        return this;
    }

    public PinpointConstants strafeX(double strafeX) {
        PinpointConstants.strafeX = strafeX;
        return this;
    }

    public PinpointConstants distanceUnit(DistanceUnit distanceUnit) {
        PinpointConstants.distanceUnit = distanceUnit;
        return this;
    }

    public PinpointConstants hardwareMapName(String hardwareMapName) {
        PinpointConstants.hardwareMapName = hardwareMapName;
        return this;
    }

    public PinpointConstants useYawScalar(boolean useYawScalar) {
        PinpointConstants.useYawScalar = useYawScalar;
        return this;
    }

    public PinpointConstants yawScalar(double yawScalar) {
        PinpointConstants.yawScalar = yawScalar;
        return this;
    }

    public PinpointConstants useCustomEncoderResolution(boolean useCustomEncoderResolution) {
        PinpointConstants.useCustomEncoderResolution = useCustomEncoderResolution;
        return this;
    }

    public PinpointConstants encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution) {
        PinpointConstants.encoderResolution = encoderResolution;
        return this;
    }

    public PinpointConstants customEncoderResolution(double customEncoderResolution) {
        PinpointConstants.customEncoderResolution = customEncoderResolution;
        return this;
    }

    public PinpointConstants forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection) {
        PinpointConstants.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public PinpointConstants strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection) {
        PinpointConstants.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    public void defaults() {
        forwardY = 1;
        strafeX = -2.5;
        distanceUnit = DistanceUnit.INCH;
        hardwareMapName = "pinpoint";
        useYawScalar = false;
        yawScalar = 1.0;
        useCustomEncoderResolution = false;
        encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        customEncoderResolution = 13.26291192;
        forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}
