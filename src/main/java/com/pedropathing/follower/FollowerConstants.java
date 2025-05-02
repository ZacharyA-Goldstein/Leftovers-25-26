package com.pedropathing.follower;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

public class FollowerConstants {

    public PIDFCoefficients translationalPIDFCoefficients = new PIDFCoefficients(
            0.1,
            0,
            0,
            0);

    /**
     * Translational Integral
     * Default Value: new CustomPIDFCoefficients(0,0,0,0);
     */
    public PIDFCoefficients translationalIntegral = new PIDFCoefficients(
            0,
            0,
            0,
            0);

    /**
     * Feed forward constant added on to the translational PIDF
     * Default Value: 0.015
     */
    public double translationalPIDFFeedForward = 0.015;

    /**
     * Heading error PIDF coefficients
     * Default Value: new CustomPIDFCoefficients(1,0,0,0);
     */
    public PIDFCoefficients headingPIDFCoefficients = new PIDFCoefficients(
            1,
            0,
            0,
            0);

    /**
     * Feed forward constant added on to the heading PIDF
     * Default Value: 0.01
     */
    public double headingPIDFFeedForward = 0.01;


    /**
     * Drive PIDF coefficients
     * Default Value: new CustomFilteredPIDFCoefficients(0.025,0,0.00001,0.6,0);
     */
    public FilteredPIDFCoefficients drivePIDFCoefficients = new FilteredPIDFCoefficients(
            0.025,
            0,
            0.00001,
            0.6,
            0);

    /**
     * Feed forward constant added on to the drive PIDF
     * Default Value: 0.01
     */
    public double drivePIDFFeedForward = 0.01;

    /**
     * Secondary translational PIDF coefficients (don't use integral).
     * Default Value: new CustomPIDFCoefficients(0.3, 0, 0.01, 0)
     */
    public PIDFCoefficients secondaryTranslationalPIDFCoefficients = new PIDFCoefficients(
            0.3,
            0,
            0.01,
            0);

    /**
     * Secondary translational Integral value.
     * Default Value: new CustomPIDFCoefficients(0, 0, 0, 0)
     */
    public PIDFCoefficients secondaryTranslationalIntegral = new PIDFCoefficients(
            0,
            0,
            0,
            0);

    /**
     * Feed forward constant added on to the small translational PIDF.
     *
     * @see #secondaryTranslationalPIDFCoefficients
     * Default Value: 0.015
     */
    public double secondaryTranslationalPIDFFeedForward = 0.015;

    /**
     * The limit at which the heading PIDF switches between the main and secondary heading PIDFs.
     * Default Value: Math.PI / 20
     */
    public double headingPIDFSwitch = Math.PI / 20;

    /**
     * Secondary heading error PIDF coefficients.
     * Default Value: new CustomPIDFCoefficients(5, 0, 0.08, 0)
     */
    public PIDFCoefficients secondaryHeadingPIDFCoefficients = new PIDFCoefficients(
            5,
            0,
            0.08,
            0);

    /**
     * Feed forward constant added on to the secondary heading PIDF.
     *
     * @see #secondaryHeadingPIDFCoefficients
     * Default Value: 0.01
     */
    public double secondaryHeadingPIDFFeedForward = 0.01;

    /**
     * The limit at which the heading PIDF switches between the main and secondary drive PIDFs.
     * Default Value: 20
     */
    public double drivePIDFSwitch = 20;

    /**
     * Secondary drive PIDF coefficients.
     * Default Value: new CustomFilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0)
     */
    public FilteredPIDFCoefficients secondaryDrivePIDFCoefficients = new FilteredPIDFCoefficients(
            0.02,
            0,
            0.000005,
            0.6,
            0);

    /**
     * Feed forward constant added on to the secondary drive PIDF.
     * Default Value: 0.01
     */
    public double secondaryDrivePIDFFeedForward = 0.01;

    /** This scales the translational error correction power when the Follower is holding a Point.
     *  Default Value: 0.45 */
    public double holdPointTranslationalScaling = 0.45;

    /** This scales the heading error correction power when the Follower is holding a Point.
     *  Default Value: 0.35 */
    public double holdPointHeadingScaling = 0.35;

    /** This is the number of steps the search for the closest point uses. More steps lead to bigger
     * accuracy. However, more steps also take more time.
     *  Default Value: 10 */
    public int BEZIER_CURVE_SEARCH_LIMIT = 10;

    /** This activates/deactivates the secondary translational PIDF. It takes over at a certain translational error
     * @see #translationalPIDFSwitch
     *  Default Value: false */
    public boolean useSecondaryTranslationalPIDF = false;

    /** Use the secondary heading PIDF. It takes over at a certain heading error
     * @see #headingPIDFSwitch
     *  Default Value: false */
    public boolean useSecondaryHeadingPIDF = false;

    /** Use the secondary drive PIDF. It takes over at a certain drive error
     * @see #drivePIDFSwitch
     *  Default Value: false */
    public boolean useSecondaryDrivePIDF = false;

    /** The limit at which the translational PIDF switches between the main and secondary translational PIDFs,
     * if the secondary PID is active.
     *  Default Value: 3 */
    public double translationalPIDFSwitch = 3;

    /** Threshold that the turn and turnTo methods will be considered to be finished
     *  In Radians
     *  Default Value: 0.01 */
    public double turnHeadingErrorThreshold = 0.01;

    /** Centripetal force to power scaling
     *  Default Value: 0.0005 */
    public double centripetalScaling = 0.0005;

    public PIDFCoefficients translationalPIDFCoefficients() { return translationalPIDFCoefficients; }
    public PIDFCoefficients translationalIntegral() { return translationalIntegral; }
    public double translationalPIDFFeedForward() { return translationalPIDFFeedForward; }
    public PIDFCoefficients headingPIDFCoefficients() { return headingPIDFCoefficients; }
    public double headingPIDFFeedForward() { return headingPIDFFeedForward; }
    public FilteredPIDFCoefficients drivePIDFCoefficients() { return drivePIDFCoefficients; }
    public double drivePIDFFeedForward() { return drivePIDFFeedForward; }
    public PIDFCoefficients secondaryTranslationalPIDFCoefficients() { return secondaryTranslationalPIDFCoefficients; }
    public PIDFCoefficients secondaryTranslationalIntegral() { return secondaryTranslationalIntegral; }
    public double translationalPIDFSwitch() { return translationalPIDFSwitch; }
    public double secondaryTranslationalPIDFFeedForward() { return secondaryTranslationalPIDFFeedForward; }
    public double headingPIDFSwitch() { return headingPIDFSwitch; }
    public PIDFCoefficients secondaryHeadingPIDFCoefficients() { return secondaryHeadingPIDFCoefficients; }
    public double secondaryHeadingPIDFFeedForward() { return secondaryHeadingPIDFFeedForward; }
    public double drivePIDFSwitch() { return drivePIDFSwitch; }
    public FilteredPIDFCoefficients secondaryDrivePIDFCoefficients() { return secondaryDrivePIDFCoefficients; }
    public double secondaryDrivePIDFFeedForward() { return secondaryDrivePIDFFeedForward; }
    public int BEZIER_CURVE_SEARCH_LIMIT() { return BEZIER_CURVE_SEARCH_LIMIT; }
    public double holdPointTranslationalScaling() { return holdPointTranslationalScaling; }
    public double holdPointHeadingScaling() { return holdPointHeadingScaling; }
    public double centripetalScaling() { return centripetalScaling; }
    public double turnHeadingErrorThreshold() { return turnHeadingErrorThreshold; }
    public boolean useSecondaryDrivePIDF() { return useSecondaryDrivePIDF; }
    public boolean useSecondaryTranslationalPIDF() { return useSecondaryTranslationalPIDF; }
    public boolean useSecondaryHeadingPIDF() { return useSecondaryHeadingPIDF; }

    public FollowerConstants() {}

    public static FollowerConstants defaultInstance() {
        return new FollowerConstants();
    }

    public FollowerConstants translationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) {
        this.translationalPIDFCoefficients = translationalPIDFCoefficients;
        return this;
    }

    public FollowerConstants translationalIntegral(PIDFCoefficients translationalIntegral) {
        this.translationalIntegral = translationalIntegral;
        return this;
    }

    public FollowerConstants translationalPIDFFeedForward(double translationalPIDFFeedForward) {
        this.translationalPIDFFeedForward = translationalPIDFFeedForward;
        return this;
    }

    public FollowerConstants headingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) {
        this.headingPIDFCoefficients = headingPIDFCoefficients;
        return this;
    }

    public FollowerConstants headingPIDFFeedForward(double headingPIDFFeedForward) {
        this.headingPIDFFeedForward = headingPIDFFeedForward;
        return this;
    }

    public FollowerConstants drivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) {
        this.drivePIDFCoefficients = drivePIDFCoefficients;
        return this;
    }

    public FollowerConstants drivePIDFFeedForward(double drivePIDFFeedForward) {
        this.drivePIDFFeedForward = drivePIDFFeedForward;
        return this;
    }

    public FollowerConstants secondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) {
        this.secondaryTranslationalPIDFCoefficients = secondaryTranslationalPIDFCoefficients;
        return this;
    }

    public FollowerConstants secondaryTranslationalIntegral(PIDFCoefficients secondaryTranslationalIntegral) {
        this.secondaryTranslationalIntegral = secondaryTranslationalIntegral;
        return this;
    }

    public FollowerConstants secondaryTranslationalPIDFFeedForward(double secondaryTranslationalPIDFFeedForward) {
        this.secondaryTranslationalPIDFFeedForward = secondaryTranslationalPIDFFeedForward;
        return this;
    }

    public FollowerConstants headingPIDFSwitch(double headingPIDFSwitch) {
        this.headingPIDFSwitch = headingPIDFSwitch;
        return this;
    }

    public FollowerConstants secondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) {
        this.secondaryHeadingPIDFCoefficients = secondaryHeadingPIDFCoefficients;
        return this;
    }

    public FollowerConstants secondaryHeadingPIDFFeedForward(double secondaryHeadingPIDFFeedForward) {
        this.secondaryHeadingPIDFFeedForward = secondaryHeadingPIDFFeedForward;
        return this;
    }

    public FollowerConstants drivePIDFSwitch(double drivePIDFSwitch) {
        this.drivePIDFSwitch = drivePIDFSwitch;
        return this;
    }

    public FollowerConstants secondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) {
        this.secondaryDrivePIDFCoefficients = secondaryDrivePIDFCoefficients;
        return this;
    }

    public FollowerConstants secondaryDrivePIDFFeedForward(double secondaryDrivePIDFFeedForward) {
        this.secondaryDrivePIDFFeedForward = secondaryDrivePIDFFeedForward;
        return this;
    }

    public FollowerConstants holdPointTranslationalScaling(double holdPointTranslationalScaling) {
        this.holdPointTranslationalScaling = holdPointTranslationalScaling;
        return this;
    }

    public FollowerConstants holdPointHeadingScaling(double holdPointHeadingScaling) {
        this.holdPointHeadingScaling = holdPointHeadingScaling;
        return this;
    }

    public FollowerConstants BEZIER_CURVE_SEARCH_LIMIT(int BEZIER_CURVE_SEARCH_LIMIT) {
        this.BEZIER_CURVE_SEARCH_LIMIT = BEZIER_CURVE_SEARCH_LIMIT;
        return this;
    }

    public FollowerConstants useSecondaryTranslationalPIDF(boolean useSecondaryTranslationalPIDF) {
        this.useSecondaryTranslationalPIDF = useSecondaryTranslationalPIDF;
        return this;
    }

    public FollowerConstants useSecondaryHeadingPIDF(boolean useSecondaryHeadingPIDF) {
        this.useSecondaryHeadingPIDF = useSecondaryHeadingPIDF;
        return this;
    }

    public FollowerConstants useSecondaryDrivePIDF(boolean useSecondaryDrivePIDF) {
        this.useSecondaryDrivePIDF = useSecondaryDrivePIDF;
        return this;
    }

    public FollowerConstants translationalPIDFSwitch(double translationalPIDFSwitch) {
        this.translationalPIDFSwitch = translationalPIDFSwitch;
        return this;
    }

    public FollowerConstants turnHeadingErrorThreshold(double turnHeadingErrorThreshold) {
        this.turnHeadingErrorThreshold = turnHeadingErrorThreshold;
        return this;
    }

    public FollowerConstants centripetalScaling(double centripetalScaling) {
        this.centripetalScaling = centripetalScaling;
        return this;
    }

    public FollowerConstants build() {
        return this;
    }
}