package com.pedropathing.paths;

import com.pedropathing.geometry.Pose;

public class PathPoint {
    public final double tValue;
    public final Pose pose;
    
    public PathPoint(double tValue, Pose pose) {
        this.tValue = tValue;
        this.pose = pose;
    }
}
