package com.pedropathing.follower;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.drivetrain.Mecanum;
import com.pedropathing.drivetrain.MecanumConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.constants.DriveEncoderConstants;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.localization.constants.ThreeWheelConstants;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.pedropathing.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.localization.localizers.OTOSLocalizer;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.localization.localizers.ThreeWheelLocalizer;
import com.pedropathing.localization.localizers.TwoWheelLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** This is the FollowerBuilder.
 * It is used to create Followers with a specific drivetrain + localizer without having to use a full constructor
 *
 * @author Baron Henderson - 20077 The Indubitables
 */
public class FollowerBuilder {
    private final FollowerConstants constants;
    private PathConstraints constraints;
    private final HardwareMap hardwareMap;
    private Localizer localizer;
    private Drivetrain drivetrain;

    public FollowerBuilder(FollowerConstants constants, HardwareMap hardwareMap) {
        this.constants = constants;
        this.hardwareMap = hardwareMap;
        constraints = PathConstraints.defaultConstraints;
    }

    public FollowerBuilder setLocalizer(Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    public FollowerBuilder driveEncoderLocalizer(DriveEncoderConstants lConstants) {
        return setLocalizer(new DriveEncoderLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder OTOSLocalizer(OTOSConstants lConstants) {
        return setLocalizer(new OTOSLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder pinpointLocalizer(PinpointConstants lConstants) {
        return setLocalizer(new PinpointLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder threeWheelIMULocalizer(ThreeWheelIMUConstants lConstants) {
        return setLocalizer(new ThreeWheelIMULocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder threeWheelLocalizer(ThreeWheelConstants lConstants) {
        return setLocalizer(new ThreeWheelLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder twoWheelLocalizer(TwoWheelConstants lConstants) {
        return setLocalizer(new TwoWheelLocalizer(hardwareMap, lConstants));
    }

    public FollowerBuilder setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        return this;
    }

    public FollowerBuilder mecanumDrivetrain(MecanumConstants mecanumConstants) {
        return setDrivetrain(new Mecanum(hardwareMap, mecanumConstants));
    }

    public FollowerBuilder pathConstraints(PathConstraints pathConstraints) {
        this.constraints = pathConstraints;
        return this;
    }

    public Follower build() {
        return new Follower(constants, localizer, drivetrain, constraints);
    }
}
