package com.pedropathing.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class PathPoint {
    public final double tValue;
    public final Pose pose;
    public final Vector tangentVector;

    public PathPoint() {
        this(0, new Pose(), new Vector());
    }

    public PathPoint(double tValue, Pose pose, Vector tangentVector) {
        this.tValue = tValue;
        this.pose = pose;
        this.tangentVector = tangentVector;
    }

    public double getTValue() {
        return tValue;
    }

    public Pose getPose() {
        return pose;
    }

    public Vector getTangentVector() {
        return tangentVector;
    }
}
