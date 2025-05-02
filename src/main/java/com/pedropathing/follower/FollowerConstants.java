package com.pedropathing.follower;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

/**
 * This is the FollowerConstants class. It holds many constants and parameters for various parts of
 * the Follower. This is here to allow for easier tuning of Pedro Pathing, as well as concentrate
 * everything tunable for the Paths themselves in one place.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 5/1/2025
 */
public class FollowerConstants {

    public static PIDFCoefficients translationalPIDFCoefficients = new PIDFCoefficients(
            0.1,
            0,
            0,
            0);

    /**
     * Translational Integral
     * Default Value: new CustomPIDFCoefficients(0,0,0,0);
     */
    public static PIDFCoefficients translationalIntegral = new PIDFCoefficients(
            0,
            0,
            0,
            0);

    /**
     * Feed forward constant added on to the translational PIDF
     * Default Value: 0.015
     */
    public static double translationalPIDFFeedForward = 0.015;

    /**
     * Heading error PIDF coefficients
     * Default Value: new CustomPIDFCoefficients(1,0,0,0);
     */
    public static PIDFCoefficients headingPIDFCoefficients = new PIDFCoefficients(
            1,
            0,
            0,
            0);

    /**
     * Feed forward constant added on to the heading PIDF
     * Default Value: 0.01
     */
    public static double headingPIDFFeedForward = 0.01;


    /**
     * Drive PIDF coefficients
     * Default Value: new CustomFilteredPIDFCoefficients(0.025,0,0.00001,0.6,0);
     */
    public static FilteredPIDFCoefficients drivePIDFCoefficients = new FilteredPIDFCoefficients(
            0.025,
            0,
            0.00001,
            0.6,
            0);

    /**
     * Feed forward constant added on to the drive PIDF
     * Default Value: 0.01
     */
    public static double drivePIDFFeedForward = 0.01;

    /**
     * Secondary translational PIDF coefficients (don't use integral).
     * Default Value: new CustomPIDFCoefficients(0.3, 0, 0.01, 0)
     */
    public static PIDFCoefficients secondaryTranslationalPIDFCoefficients = new PIDFCoefficients(
            0.3,
            0,
            0.01,
            0);

    /**
     * Secondary translational Integral value.
     * Default Value: new CustomPIDFCoefficients(0, 0, 0, 0)
     */
    public static PIDFCoefficients secondaryTranslationalIntegral = new PIDFCoefficients(
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
    public static double secondaryTranslationalPIDFFeedForward = 0.015;

    /**
     * The limit at which the heading PIDF switches between the main and secondary heading PIDFs.
     * Default Value: Math.PI / 20
     */
    public static double headingPIDFSwitch = Math.PI / 20;

    /**
     * Secondary heading error PIDF coefficients.
     * Default Value: new CustomPIDFCoefficients(5, 0, 0.08, 0)
     */
    public static PIDFCoefficients secondaryHeadingPIDFCoefficients = new PIDFCoefficients(
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
    public static double secondaryHeadingPIDFFeedForward = 0.01;

    /**
     * The limit at which the heading PIDF switches between the main and secondary drive PIDFs.
     * Default Value: 20
     */
    public static double drivePIDFSwitch = 20;

    /**
     * Secondary drive PIDF coefficients.
     * Default Value: new CustomFilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0)
     */
    public static FilteredPIDFCoefficients secondaryDrivePIDFCoefficients = new FilteredPIDFCoefficients(
            0.02,
            0,
            0.000005,
            0.6,
            0);

    /**
     * Feed forward constant added on to the secondary drive PIDF.
     * Default Value: 0.01
     */
    public static double secondaryDrivePIDFFeedForward = 0.01;

    /**
     * This scales the translational error correction power when the Follower is holding a Point.
     * Default Value: 0.45
     */
    public static double holdPointTranslationalScaling = 0.45;

    /**
     * This scales the heading error correction power when the Follower is holding a Point.
     * Default Value: 0.35
     */
    public static double holdPointHeadingScaling = 0.35;

    /**
     * This is the number of steps the search for the closest point uses. More steps lead to bigger
     * accuracy. However, more steps also take more time.
     * Default Value: 10
     */
    public static int BEZIER_CURVE_SEARCH_LIMIT = 10;

    /**
     * This activates/deactivates the secondary translational PIDF. It takes over at a certain translational error
     *
     * @see #translationalPIDFSwitch
     * Default Value: false
     */
    public static boolean useSecondaryTranslationalPIDF = false;

    /**
     * Use the secondary heading PIDF. It takes over at a certain heading error
     *
     * @see #headingPIDFSwitch
     * Default Value: false
     */
    public static boolean useSecondaryHeadingPIDF = false;

    /**
     * Use the secondary drive PIDF. It takes over at a certain drive error
     *
     * @see #drivePIDFSwitch
     * Default Value: false
     */
    public static boolean useSecondaryDrivePIDF = false;

    /**
     * The limit at which the translational PIDF switches between the main and secondary translational PIDFs,
     * if the secondary PID is active.
     * Default Value: 3
     */
    public static double translationalPIDFSwitch = 3;

    /**
     * Threshold that the turn and turnTo methods will be considered to be finished
     * In Radians
     * Default Value: 0.01
     */
    public static double turnHeadingErrorThreshold = 0.01;

    /**
     * Centripetal force to power scaling
     * Default Value: 0.0005
     */
    public static double centripetalScaling = 0.0005;

    public static PIDFCoefficients translationalPIDFCoefficients() {
        return translationalPIDFCoefficients;
    }

    public static PIDFCoefficients translationalIntegral() {
        return translationalIntegral;
    }

    public static double translationalPIDFFeedForward() {
        return translationalPIDFFeedForward;
    }

    public static PIDFCoefficients headingPIDFCoefficients() {
        return headingPIDFCoefficients;
    }

    public static double headingPIDFFeedForward() {
        return headingPIDFFeedForward;
    }

    public static FilteredPIDFCoefficients drivePIDFCoefficients() {
        return drivePIDFCoefficients;
    }

    public static double drivePIDFFeedForward() {
        return drivePIDFFeedForward;
    }

    public static PIDFCoefficients secondaryTranslationalPIDFCoefficients() {
        return secondaryTranslationalPIDFCoefficients;
    }

    public static PIDFCoefficients secondaryTranslationalIntegral() {
        return secondaryTranslationalIntegral;
    }

    public static double translationalPIDFSwitch() {
        return translationalPIDFSwitch;
    }

    public static double secondaryTranslationalPIDFFeedForward() {
        return secondaryTranslationalPIDFFeedForward;
    }

    public static double headingPIDFSwitch() {
        return headingPIDFSwitch;
    }

    public static PIDFCoefficients secondaryHeadingPIDFCoefficients() {
        return secondaryHeadingPIDFCoefficients;
    }

    public static double secondaryHeadingPIDFFeedForward() {
        return secondaryHeadingPIDFFeedForward;
    }

    public static double drivePIDFSwitch() {
        return drivePIDFSwitch;
    }

    public static FilteredPIDFCoefficients secondaryDrivePIDFCoefficients() {
        return secondaryDrivePIDFCoefficients;
    }

    public static double secondaryDrivePIDFFeedForward() {
        return secondaryDrivePIDFFeedForward;
    }

    public static int BEZIER_CURVE_SEARCH_LIMIT() {
        return BEZIER_CURVE_SEARCH_LIMIT;
    }

    public static double holdPointTranslationalScaling() {
        return holdPointTranslationalScaling;
    }

    public static double holdPointHeadingScaling() {
        return holdPointHeadingScaling;
    }

    public static double centripetalScaling() {
        return centripetalScaling;
    }

    public static double turnHeadingErrorThreshold() {
        return turnHeadingErrorThreshold;
    }

    public static boolean useSecondaryDrivePIDF() {
        return useSecondaryDrivePIDF;
    }

    public static boolean useSecondaryTranslationalPIDF() {
        return useSecondaryTranslationalPIDF;
    }

    public static boolean useSecondaryHeadingPIDF() {
        return useSecondaryHeadingPIDF;
    }

    /**
     * This constructor is empty but on creation it will set the default values for the constants.
     * You can use .translationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0)) and other methods to set the values.
     */
    public FollowerConstants() {
        defaults();
    }

    public FollowerConstants translationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) {
        FollowerConstants.translationalPIDFCoefficients = translationalPIDFCoefficients;
        return this;
    }

    public FollowerConstants translationalIntegral(PIDFCoefficients translationalIntegral) {
        FollowerConstants.translationalIntegral = translationalIntegral;
        return this;
    }

    public FollowerConstants translationalPIDFFeedForward(double translationalPIDFFeedForward) {
        FollowerConstants.translationalPIDFFeedForward = translationalPIDFFeedForward;
        return this;
    }

    public FollowerConstants headingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) {
        FollowerConstants.headingPIDFCoefficients = headingPIDFCoefficients;
        return this;
    }

    public FollowerConstants headingPIDFFeedForward(double headingPIDFFeedForward) {
        FollowerConstants.headingPIDFFeedForward = headingPIDFFeedForward;
        return this;
    }

    public FollowerConstants drivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) {
        FollowerConstants.drivePIDFCoefficients = drivePIDFCoefficients;
        return this;
    }

    public FollowerConstants drivePIDFFeedForward(double drivePIDFFeedForward) {
        FollowerConstants.drivePIDFFeedForward = drivePIDFFeedForward;
        return this;
    }

    public FollowerConstants secondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) {
        FollowerConstants.secondaryTranslationalPIDFCoefficients = secondaryTranslationalPIDFCoefficients;
        return this;
    }

    public FollowerConstants secondaryTranslationalIntegral(PIDFCoefficients secondaryTranslationalIntegral) {
        FollowerConstants.secondaryTranslationalIntegral = secondaryTranslationalIntegral;
        return this;
    }

    public FollowerConstants secondaryTranslationalPIDFFeedForward(double secondaryTranslationalPIDFFeedForward) {
        FollowerConstants.secondaryTranslationalPIDFFeedForward = secondaryTranslationalPIDFFeedForward;
        return this;
    }

    public FollowerConstants headingPIDFSwitch(double headingPIDFSwitch) {
        FollowerConstants.headingPIDFSwitch = headingPIDFSwitch;
        return this;
    }

    public FollowerConstants secondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) {
        FollowerConstants.secondaryHeadingPIDFCoefficients = secondaryHeadingPIDFCoefficients;
        return this;
    }

    public FollowerConstants secondaryHeadingPIDFFeedForward(double secondaryHeadingPIDFFeedForward) {
        FollowerConstants.secondaryHeadingPIDFFeedForward = secondaryHeadingPIDFFeedForward;
        return this;
    }

    public FollowerConstants drivePIDFSwitch(double drivePIDFSwitch) {
        FollowerConstants.drivePIDFSwitch = drivePIDFSwitch;
        return this;
    }

    public FollowerConstants secondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) {
        FollowerConstants.secondaryDrivePIDFCoefficients = secondaryDrivePIDFCoefficients;
        return this;
    }

    public FollowerConstants secondaryDrivePIDFFeedForward(double secondaryDrivePIDFFeedForward) {
        FollowerConstants.secondaryDrivePIDFFeedForward = secondaryDrivePIDFFeedForward;
        return this;
    }

    public FollowerConstants holdPointTranslationalScaling(double holdPointTranslationalScaling) {
        FollowerConstants.holdPointTranslationalScaling = holdPointTranslationalScaling;
        return this;
    }

    public FollowerConstants holdPointHeadingScaling(double holdPointHeadingScaling) {
        FollowerConstants.holdPointHeadingScaling = holdPointHeadingScaling;
        return this;
    }

    public FollowerConstants BEZIER_CURVE_SEARCH_LIMIT(int BEZIER_CURVE_SEARCH_LIMIT) {
        FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT = BEZIER_CURVE_SEARCH_LIMIT;
        return this;
    }

    public FollowerConstants useSecondaryTranslationalPIDF(boolean useSecondaryTranslationalPIDF) {
        FollowerConstants.useSecondaryTranslationalPIDF = useSecondaryTranslationalPIDF;
        return this;
    }

    public FollowerConstants useSecondaryHeadingPIDF(boolean useSecondaryHeadingPIDF) {
        FollowerConstants.useSecondaryHeadingPIDF = useSecondaryHeadingPIDF;
        return this;
    }

    public FollowerConstants useSecondaryDrivePIDF(boolean useSecondaryDrivePIDF) {
        FollowerConstants.useSecondaryDrivePIDF = useSecondaryDrivePIDF;
        return this;
    }

    public FollowerConstants translationalPIDFSwitch(double translationalPIDFSwitch) {
        FollowerConstants.translationalPIDFSwitch = translationalPIDFSwitch;
        return this;
    }

    public FollowerConstants turnHeadingErrorThreshold(double turnHeadingErrorThreshold) {
        FollowerConstants.turnHeadingErrorThreshold = turnHeadingErrorThreshold;
        return this;
    }

    public FollowerConstants centripetalScaling(double centripetalScaling) {
        FollowerConstants.centripetalScaling = centripetalScaling;
        return this;
    }

    public void defaults() {
        translationalPIDFCoefficients.setCoefficients(0.1, 0, 0, 0);
        translationalIntegral.setCoefficients(0, 0, 0, 0);
        translationalPIDFFeedForward = 0.015;

        headingPIDFCoefficients.setCoefficients(1, 0, 0, 0);
        headingPIDFFeedForward = 0.01;

        drivePIDFCoefficients.setCoefficients(0.025, 0, 0.00001, 0.6, 0);
        drivePIDFFeedForward = 0.01;

        secondaryTranslationalPIDFCoefficients.setCoefficients(0.3, 0, 0.01, 0);
        secondaryTranslationalIntegral.setCoefficients(0, 0, 0, 0);
        secondaryTranslationalPIDFFeedForward = 0.015;

        headingPIDFSwitch = Math.PI / 20;
        secondaryHeadingPIDFCoefficients.setCoefficients(5, 0, 0.08, 0);
        secondaryHeadingPIDFFeedForward = 0.01;

        drivePIDFSwitch = 20;
        secondaryDrivePIDFCoefficients.setCoefficients(0.02, 0, 0.000005, 0.6, 0);
        secondaryDrivePIDFFeedForward = 0.01;

        holdPointTranslationalScaling = 0.45;
        holdPointHeadingScaling = 0.35;

        BEZIER_CURVE_SEARCH_LIMIT = 10;

        useSecondaryTranslationalPIDF = false;
        useSecondaryHeadingPIDF = false;
        useSecondaryDrivePIDF = false;

        translationalPIDFSwitch = 3;
        turnHeadingErrorThreshold = 0.01;
        centripetalScaling = 0.0005;
    }
}