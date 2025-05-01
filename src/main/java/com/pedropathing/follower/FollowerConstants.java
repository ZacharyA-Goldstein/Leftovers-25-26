package com.pedropathing.follower;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

public abstract class FollowerConstants {

    public abstract PIDFCoefficients translationalPIDFCoefficients();

    public abstract PIDFCoefficients translationalIntegral();

    public abstract double translationalPIDFFeedForward();

    public abstract PIDFCoefficients headingPIDFCoefficients();

    public abstract double headingPIDFFeedForward();

    public abstract FilteredPIDFCoefficients drivePIDFCoefficients();

    public abstract double drivePIDFFeedForward();

    public abstract PIDFCoefficients secondaryTranslationalPIDFCoefficients();

    public abstract PIDFCoefficients secondaryTranslationalIntegral();

    public abstract double secondaryTranslationalPIDFFeedForward();

    public abstract double headingPIDFSwitch();

    public abstract PIDFCoefficients secondaryHeadingPIDFCoefficients();

    public abstract double secondaryHeadingPIDFFeedForward();

    public abstract double drivePIDFSwitch();

    public abstract FilteredPIDFCoefficients secondaryDrivePIDFCoefficients();

    public abstract double secondaryDrivePIDFFeedForward();
    maxPowerScaling
    BEZIER_CURVE_SEARCH_LIMIT = com.pedropathing.follower.old.FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT;
    holdPointTranslationalScaling = com.pedropathing.follower.old.FollowerConstants.holdPointTranslationalScaling;
    holdPointHeadingScaling = com.pedropathing.follower.old.FollowerConstants.holdPointHeadingScaling;
    centripetalScaling = com.pedropathing.follower.old.FollowerConstants.centripetalScaling;
    turnHeadingErrorThreshold = com.pedropathing.follower.old.FollowerConstants.turnHeadingErrorThreshold;
}