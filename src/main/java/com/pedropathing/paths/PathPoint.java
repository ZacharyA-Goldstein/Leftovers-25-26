package com.pedropathing.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

/**
 * This is the PathPoint class. This class represents a point in a path. It contains information about the t-value, pose, and tangent vector of the point.
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class PathPoint {
    /**
     * The t-value of the point
     */
    public final double tValue;

    /**
     * The pose of the point
     */
    public final Pose pose;

    /**
     * The tangent vector of the point
     */
    public final Vector tangentVector;

    /**
     * Default constructor for the PathPoint class. Creates a point with a t-value of 0, a pose of (0, 0, 0), and a tangent vector of (0, 0).
     */
    public PathPoint() {
        this(0, new Pose(), new Vector());
    }

    /**
     * Constructor for the PathPoint class.
     * @param tValue The t-value of the point
     * @param pose The pose of the point
     * @param tangentVector The tangent vector of path at the point
     */
    public PathPoint(double tValue, Pose pose, Vector tangentVector) {
        this.tValue = tValue;
        this.pose = pose;
        this.tangentVector = tangentVector;
    }

    /**
     * Getter for the t-value of the point.
     * @return The t-value of the point
     */
    public double getTValue() {
        return tValue;
    }

    /**
     * Getter for the pose of the point.
     * @return The pose of the point
     */
    public Pose getPose() {
        return pose;
    }

    /**
     * Getter for the tangent vector of the point.
     * @return The tangent vector of the point
     */
    public Vector getTangentVector() {
        return tangentVector;
    }
}
