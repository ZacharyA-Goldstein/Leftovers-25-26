package com.pedropathing.paths;


import com.pedropathing.math.MathFunctions;

/**
 * A heading interpolator is a function that takes a path and returns the heading goal the robot
 * should be at a specific point on the path.
 * Note: these methods all use degrees for input and radians internally.
 *
 * <p>
 * Example usage:
 * <pre><code>
 * // Default: tangent to the path
 * .setHeadingInterpolation(HeadingInterpolator.tangent);
 * // Offset the tangent by 90 degrees
 * .setHeadingInterpolation(HeadingInterpolator.tangent.offset(90));
 * // Follow the path with the robot facing backwards
 * .setHeadingInterpolation(HeadingInterpolator.tangent.reverse());
 * // Follow the path while the robot is facing a point
 * .setHeadingInterpolation(HeadingInterpolator.facingPoint(5, 5));
 * // Make the robot have a constant heading of 45 degrees
 * .setHeadingInterpolation(HeadingInterpolator.constant(45));
 * // Make the robot transition from 0 degrees to 90 degrees over the path
 * .setHeadingInterpolation(HeadingInterpolator.linear(0, 90));
 *
 * .offset and .reverse methods can be applied to any HeadingInterpolator.
 * </code></pre>
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 */
@FunctionalInterface
public interface HeadingInterpolator {
    double interpolate(PathPoint closestPoint);
    
    /**
     * Offsets the heading interpolator by a given amount.
     */
    default HeadingInterpolator offset(double offsetDeg) {
        double headingOffsetRadians = Math.toRadians(offsetDeg);
        return closestPoint -> this.interpolate(closestPoint) + headingOffsetRadians;
    }
    
    /**
     * Reverses the direction of a heading interpolator.
     * <p>
     * Note: For facing a point this will make the robot face the opposite direction of the point.
     */
    default HeadingInterpolator reverse() {
        return closestPoint ->
            MathFunctions.normalizeAngle(this.interpolate(closestPoint) + Math.PI);
    }
    
    /**
     * The robot will face the the direction of the path.
     */
    HeadingInterpolator tangent = closestPoint -> closestPoint.pose.getHeading();
    
    /**
     * A constant heading along a path.
     */
    static HeadingInterpolator constant(double headingDeg) {
        double headingRadians = MathFunctions.normalizeAngle(Math.toRadians(headingDeg));
        return path -> headingRadians;
    }
    
    /**
     * The robot will transition from the start heading to the end heading.
     */
    static HeadingInterpolator linear(double startHeadingDeg, double endHeadingDeg) {
        return linear(startHeadingDeg, endHeadingDeg, 1);
    }
    
    /**
     * The robot will transition from the start heading to the end heading by endT.
     */
    static HeadingInterpolator linear(double startHeadingDeg, double endHeadingDeg, double endT) {
        return closestPoint -> {
            double clampedEndT = MathFunctions.clamp(endT, 0.0001, 1);
            double t = Math.min(closestPoint.tValue / clampedEndT, 1.0);
            double startHeading = Math.toRadians(startHeadingDeg);
            double endHeading = Math.toRadians(endHeadingDeg);
            double deltaHeading = MathFunctions.normalizeAngle(endHeading - startHeading);
            return startHeading + deltaHeading * t;
        };
    }
    
    /**
     * The robot will always be facing the given point while following the path.
     */
    static HeadingInterpolator facingPoint(double x, double y) {
        return closestPoint -> Math.atan2(
            x - closestPoint.pose.getY(),
            y - closestPoint.pose.getX()
        );
    }
}