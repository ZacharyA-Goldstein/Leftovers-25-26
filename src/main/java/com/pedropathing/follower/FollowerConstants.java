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
    public abstract double translationalPIDFSwitch();

    public abstract double secondaryTranslationalPIDFFeedForward();

    public abstract double headingPIDFSwitch();

    public abstract PIDFCoefficients secondaryHeadingPIDFCoefficients();

    public abstract double secondaryHeadingPIDFFeedForward();

    public abstract double drivePIDFSwitch();

    public abstract FilteredPIDFCoefficients secondaryDrivePIDFCoefficients();

    public abstract double secondaryDrivePIDFFeedForward();
    public abstract int BEZIER_CURVE_SEARCH_LIMIT();
    public abstract double holdPointTranslationalScaling();
    public abstract double holdPointHeadingScaling();
    public abstract double centripetalScaling();
    public abstract double turnHeadingErrorThreshold();

    public abstract boolean useSecondaryDrivePIDF();
    public abstract boolean useSecondaryTranslationalPIDF();
    public abstract boolean useSecondaryHeadingPIDF();

}