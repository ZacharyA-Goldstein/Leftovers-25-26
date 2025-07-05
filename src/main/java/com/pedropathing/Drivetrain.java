package com.pedropathing;

import com.pedropathing.math.Vector;
import com.pedropathing.math.MathFunctions;

public abstract class Drivetrain {

    protected Vector[] vectors;
    protected double maxPowerScaling;

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array of four doubles, one for each wheel's motor power.
     *
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower this Vector points in the direction of the robot's current heading, and
     *                     the magnitude tells the robot how much it should turn and in which
     *                     direction.
     * @param pathingPower this Vector points in the direction the robot needs to go to continue along
     *                     the Path.
     * @param robotHeading this is the current heading of the robot, which is used to calculate how
     *                     much power to allocate to each wheel.
     * @return this returns an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public abstract double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading);

    public void setMaxPowerScaling(double maxPowerScaling) {
        this.maxPowerScaling = MathFunctions.clamp(maxPowerScaling, 0, 1);;
    }

    public double getMaxPowerScaling() {
        return maxPowerScaling;
    }
    public abstract void updateConstants();

    public abstract void breakFollowing();

    public abstract void runPowers(double[] drivePowers);

    public abstract void getAndRunDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading);

    public abstract void startTeleopDrive();
    public abstract void startTeleopDrive(boolean brakeMode);

    public abstract double xMovement();
    public abstract double yMovement();
    public abstract void setXMovement(double xMovement);
    public abstract void setYMovement(double yMovement);

    public abstract String debugString();

}
