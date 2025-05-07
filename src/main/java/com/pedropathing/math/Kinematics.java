package com.pedropathing.math;

/**
 * Implementation of kinematic equations that work with both positive and negative "distances."
 * Used for decelerating the robot.
 *
 * <pre>
 * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
 * v<sub>f</sub> = v<sub>i</sub> + a·△t
 * </pre>
 * Where:<br>
 * <ul>
 *   <li><b>v<sub>f</sub></b> = final velocity</li>
 *   <li><b>v<sub>i</sub></b> = initial velocity</li>
 *   <li><b>a</b> = acceleration</li> (or deceleration; always negative)
 *   <li><b>d</b> = distance (equations arranged in code for positive and negative)</li>
 * </ul>
 *
 * @author Jacob Ophoven - 18535 Frozen Code
 * @version 1.1.0, 5/6/2025
 */
public final class Kinematics {
    /**
     * Get the final velocity at a given distance with the deceleration.
     *
     * <pre>
     * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
     * Solve for vi
     * v<sub>i</sub> = sqrt(v<sub>f</sub><sup>2</sup> - 2·a·d)
     * Set v<sub>f</sub> to 0 to stop
     * v<sub>i</sub> = sqrt(0 - 2·a·d)
     * v<sub>i</sub> = sqrt(-2·a·d)
     * </pre>
     */
    public static double getVelocityToStopWithDeceleration(
        double directionalDistance,
        double deceleration
    ) {
        return Math.signum(directionalDistance)
            * Math.sqrt(Math.abs(-2 * deceleration * directionalDistance));
    }

    /**
     * Calculate the velocity the robot would be going after traveling the given distance
     * as it decelerates.
     *
     * <pre>
     * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
     * Solve for v<sub>f</sub>
     * v<sub>f</sub> = sqrt(v<sub>i</sub><sup>2</sup> + 2·a·d)
     * </pre>
     */
    public static double getFinalVelocityAtDistance(
        double currentVelocity,
        double deceleration,
        double directionalDistance
    ) {
        return Math.signum(directionalDistance) * Math.sqrt(currentVelocity * currentVelocity
            + 2 * deceleration * Math.abs(directionalDistance));
    }

    /**
     * Predict the next loop's velocity.
     *
     * <pre>
     * v<sub>next</sub> = 2·v<sub>current</sub> - v<sub>previous</sub>
     * </pre>
     */
    public static double predictNextLoopVelocity(double currentVelocity, double previousVelocity) {
        return 2 * currentVelocity - previousVelocity;
    }

    /**
     * Calculates the distance needed to reach a velocity with the deceleration.
     *
     * <pre>
     * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
     * Solve for d
     * d = -(v<sub>f</sub><sup>2</sup> - v<sub>i</sub><sup>2</sup>) / (2·a)
     * </pre>
     *
     * @param velocity Negative or positive.
     * @param deceleration Negative.
     */
    public static double getDistanceToVelocity(
        double velocity,
        double deceleration,
        double targetVelocity
    ) {
        return -(targetVelocity * targetVelocity - velocity * velocity) / (2 * deceleration);
    }

    /**
     * Calculates the distance needed to stop at a given velocity with the deceleration.
     *
     * <pre>
     * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
     * Solve for d
     * d = -(v<sub>f</sub><sup>2</sup> - v<sub>i</sub><sup>2</sup>) / (2·a)
     * distanceToTarget > d
     * </pre>
     *
     * @param velocity Negative or positive.
     * @param deceleration Negative.
     */
    public static double getStoppingDistance(
        double velocity,
        double deceleration
    ) {
        return getDistanceToVelocity(velocity, deceleration, 0);
    }
}
