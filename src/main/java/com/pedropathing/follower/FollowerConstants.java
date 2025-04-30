package com.pedropathing.follower;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

public abstract class FollowerConstants {
    /** Translational PIDF coefficients (don't use integral)
     *  Default Value: new CustomPIDFCoefficients(0.1,0,0,0); */
    public static PIDFCoefficients translationalPIDFCoefficients = new PIDFCoefficients(
            0.1,
            0,
            0,
            0);

    /** Translational Integral
     *  Default Value: new CustomPIDFCoefficients(0,0,0,0); */
    public static PIDFCoefficients translationalIntegral = new PIDFCoefficients(
            0,
            0,
            0,
            0);

    /** Feed forward constant added on to the translational PIDF
     *  Default Value: 0.015 */
    public static double translationalPIDFFeedForward = 0.015;

    /** Heading error PIDF coefficients
     *  Default Value: new CustomPIDFCoefficients(1,0,0,0); */
    public static PIDFCoefficients headingPIDFCoefficients = new PIDFCoefficients(
            1,
            0,
            0,
            0);

    /** Feed forward constant added on to the heading PIDF
     *  Default Value: 0.01 */
    public static double headingPIDFFeedForward = 0.01;


    /** Drive PIDF coefficients
     *  Default Value: new CustomFilteredPIDFCoefficients(0.025,0,0.00001,0.6,0); */
    public static FilteredPIDFCoefficients drivePIDFCoefficients = new FilteredPIDFCoefficients(
            0.025,
            0,
            0.00001,
            0.6,
            0);

    /** Feed forward constant added on to the drive PIDF
     *  Default Value: 0.01 */
    public static double drivePIDFFeedForward = 0.01;

    /** Secondary translational PIDF coefficients (don't use integral).
     *  Default Value: new CustomPIDFCoefficients(0.3, 0, 0.01, 0) */
    public static PIDFCoefficients secondaryTranslationalPIDFCoefficients = new PIDFCoefficients(
            0.3,
            0,
            0.01,
            0);

    /** Secondary translational Integral value.
     *  Default Value: new CustomPIDFCoefficients(0, 0, 0, 0) */
    public static PIDFCoefficients secondaryTranslationalIntegral = new PIDFCoefficients(
            0,
            0,
            0,
            0);

    /** Feed forward constant added on to the small translational PIDF.
     * @see #secondaryTranslationalPIDFCoefficients
     *  Default Value: 0.015 */
    public static double secondaryTranslationalPIDFFeedForward = 0.015;

    /** The limit at which the heading PIDF switches between the main and secondary heading PIDFs.
     *  Default Value: Math.PI / 20 */
    public static double headingPIDFSwitch = Math.PI / 20;

    /** Secondary heading error PIDF coefficients.
     *  Default Value: new CustomPIDFCoefficients(5, 0, 0.08, 0) */
    public static PIDFCoefficients secondaryHeadingPIDFCoefficients = new PIDFCoefficients(
            5,
            0,
            0.08,
            0);

    /** Feed forward constant added on to the secondary heading PIDF.
     * @see #secondaryHeadingPIDFCoefficients
     *  Default Value: 0.01 */
    public static double secondaryHeadingPIDFFeedForward = 0.01;

    /** The limit at which the heading PIDF switches between the main and secondary drive PIDFs.
     *  Default Value: 20 */
    public static double drivePIDFSwitch = 20;

    /** Secondary drive PIDF coefficients.
     *  Default Value: new CustomFilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0) */
    public static FilteredPIDFCoefficients secondaryDrivePIDFCoefficients = new FilteredPIDFCoefficients(
            0.02,
            0,
            0.000005,
            0.6,
            0);

    /** Feed forward constant added on to the secondary drive PIDF.
     *  Default Value: 0.01 */
    public static double secondaryDrivePIDFFeedForward = 0.01;
}
