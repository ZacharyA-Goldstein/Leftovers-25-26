package com.pedropathing.follower;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class FollowerConstants {

    /**
     * Translational PIDF coefficients
     * Default Value: new CustomPIDFCoefficients(0.1,0,0,0);
     */
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

    /**
     * This is the default value for the automatic hold end. If this is set to true, the Follower will
     * automatically hold the end when it reaches the end of the Path.
     * Default Value: true
     */
    public static boolean automaticHoldEnd = true;


    /**
     * This is the mass of the robot. This is used to calculate the centripetal force.
     * Default Value: 10.65
     */
    public static double mass = 10.65;

    /** Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
     * if not negative, then the robot thinks that its going to go faster under 0 power
     *  Default Value: -34.62719
     * @implNote This value is found via 'ForwardZeroPowerAccelerationTuner'*/
    public static double forwardZeroPowerAcceleration = -34.62719;

    /** Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
     * if not negative, then the robot thinks that its going to go faster under 0 power
     *  Default Value: -78.15554
     * @implNote This value is found via 'LateralZeroPowerAccelerationTuner'*/
    public static double lateralZeroPowerAcceleration = -78.15554;

    /** A multiplier for the zero power acceleration to change the speed the robot decelerates at
     * the end of paths.
     * Increasing this will cause the robot to try to decelerate faster, at the risk of overshoots
     * or localization slippage.
     * Decreasing this will cause the deceleration at the end of the Path to be slower, making the
     * robot slower but reducing risk of end-of-path overshoots or localization slippage.
     * This can be set individually for each Path, but this is the default.
     *  Default Value: 4
     */
    public static double zeroPowerAccelerationMultiplier = 4;


    /** When the robot is at the end of its current Path or PathChain and the velocity goes below
     * this value, then end the Path. This is in inches/second.
     * This can be custom set for each Path.
     *  Default Value: 0.1 */
    public static double pathEndVelocityConstraint = 0.1;

    /** When the robot is at the end of its current Path or PathChain and the translational error
     * goes below this value, then end the Path. This is in inches.
     * This can be custom set for each Path.
     *  Default Value: 0.1 */
    public static double pathEndTranslationalConstraint = 0.1;

    /** When the robot is at the end of its current Path or PathChain and the heading error goes
     * below this value, then end the Path. This is in radians.
     * This can be custom set for each Path.
     *  Default Value: 0.007 */
    public static double pathEndHeadingConstraint = 0.007;

    /** When the t-value of the closest point to the robot on the Path is greater than this value,
     * then the Path is considered at its end.
     * This can be custom set for each Path.
     *  Default Value: 0.995 */
    public static double pathEndTValueConstraint = 0.995;

    /** When the Path is considered at its end parametrically, then the Follower has this many
     * milliseconds to further correct by default.
     * This can be custom set for each Path.
     *  Default Value: 500 */
    public static double pathEndTimeoutConstraint = 500;

    /** This is how many steps the BezierCurve class uses to approximate the length of a BezierCurve.
     * @see #BEZIER_CURVE_SEARCH_LIMIT
     *  Default Value: 1000 */
    public static int APPROXIMATION_STEPS = 1000;

    /**
     * Multiplier for when the path should start its deceleration
     */
    public static double decelerationStartMultiplier = 1;

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

    public FollowerConstants automaticHoldEnd(boolean automaticHoldEnd) {
        FollowerConstants.automaticHoldEnd = automaticHoldEnd;
        return this;
    }

    public FollowerConstants mass(double mass) {
        FollowerConstants.mass = mass;
        return this;
    }

    public FollowerConstants forwardZeroPowerAcceleration(double forwardZeroPowerAcceleration) {
        FollowerConstants.forwardZeroPowerAcceleration = forwardZeroPowerAcceleration;
        return this;
    }

    public FollowerConstants lateralZeroPowerAcceleration(double lateralZeroPowerAcceleration) {
        FollowerConstants.lateralZeroPowerAcceleration = lateralZeroPowerAcceleration;
        return this;
    }

    public FollowerConstants zeroPowerAccelerationMultiplier(double zeroPowerAccelerationMultiplier) {
        FollowerConstants.zeroPowerAccelerationMultiplier = zeroPowerAccelerationMultiplier;
        return this;
    }

    public FollowerConstants pathEndVelocityConstraint(double pathEndVelocityConstraint) {
        FollowerConstants.pathEndVelocityConstraint = pathEndVelocityConstraint;
        return this;
    }

    public FollowerConstants pathEndTranslationalConstraint(double pathEndTranslationalConstraint) {
        FollowerConstants.pathEndTranslationalConstraint = pathEndTranslationalConstraint;
        return this;
    }

    public FollowerConstants pathEndHeadingConstraint(double pathEndHeadingConstraint) {
        FollowerConstants.pathEndHeadingConstraint = pathEndHeadingConstraint;
        return this;
    }

    public FollowerConstants pathEndTValueConstraint(double pathEndTValueConstraint) {
        FollowerConstants.pathEndTValueConstraint = pathEndTValueConstraint;
        return this;
    }

    public FollowerConstants pathEndTimeoutConstraint(double pathEndTimeoutConstraint) {
        FollowerConstants.pathEndTimeoutConstraint = pathEndTimeoutConstraint;
        return this;
    }

    public FollowerConstants APPROXIMATION_STEPS(int APPROXIMATION_STEPS) {
        FollowerConstants.APPROXIMATION_STEPS = APPROXIMATION_STEPS;
        return this;
    }

    public FollowerConstants decelerationStartMultiplier(double decelerationStartMultiplier) {
        FollowerConstants.decelerationStartMultiplier = decelerationStartMultiplier;
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

        automaticHoldEnd = true;
        mass = 10.65;

        forwardZeroPowerAcceleration = -41.278;
        lateralZeroPowerAcceleration = -59.7819;
    }
}