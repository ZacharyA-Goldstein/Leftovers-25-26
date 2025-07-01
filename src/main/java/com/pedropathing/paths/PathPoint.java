package com.pedropathing.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

/**
 * Represents a point on a path, including its parametric t-value, pose, and tangent vector.
 * Used to provide detailed information about a specific location along a path.
 */
public class PathPoint {
    /** The parametric t-value of the point on the path. */
    public final double tValue;
    /** The pose (position and orientation) at this point on the path. */
    public final Pose pose;
    /** The tangent vector at this point on the path. */
    public final Vector tangentVector;

    /**
     * Constructs a default PathPoint at t = 0 with default pose and tangent vector.
     */
    public PathPoint() {
        this(0, new Pose(), new Vector());
    }

    /**
     * Constructs a PathPoint with the specified t-value, pose, and tangent vector.
     *
     * @param tValue the parametric t-value of the point on the path
     * @param pose the pose (position and orientation) at this point
     * @param tangentVector the tangent vector at this point
     */
    public PathPoint(double tValue, Pose pose, Vector tangentVector) {
        this.tValue = tValue;
        this.pose = pose;
        this.tangentVector = tangentVector;
    }

    /**
     * Returns the t-value of this path point.
     *
     * @return the t-value
     */
    public double getTValue() {
        return tValue;
    }

    /**
     * Returns the pose at this path point.
     *
     * @return the pose
     */
    public Pose getPose() {
        return pose;
    }

    /**
     * Returns the tangent vector at this path point.
     *
     * @return the tangent vector
     */
    public Vector getTangentVector() {
        return tangentVector;
    }
}
