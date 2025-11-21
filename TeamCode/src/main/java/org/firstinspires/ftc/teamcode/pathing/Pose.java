package org.firstinspires.ftc.teamcode.pathing;

/**
 * Represents a position and heading in 2D space
 */
public class Pose {
    public double x;  // in inches
    public double y;  // in inches
    public double heading;  // in radians

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
