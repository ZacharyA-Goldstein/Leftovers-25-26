package com.pedropathing.paths;

public final class PathConstraints {
    /**
     * When the robot is at the end of its current Path or PathChain and the velocity goes below
     * this value, then end the Path. This is in inches/second.
     * This can be custom set for each Path.
     * Default Value: 0.1
     */
    private double velocityConstraint;

    /**
     * When the robot is at the end of its current Path or PathChain and the translational error
     * goes below this value, then end the Path. This is in inches.
     * This can be custom set for each Path.
     * Default Value: 0.1
     */
    private double translationalConstraint;

    /**
     * When the robot is at the end of its current Path or PathChain and the heading error goes
     * below this value, then end the Path. This is in radians.
     * This can be custom set for each Path.
     * Default Value: 0.007
     */
    private double headingConstraint;

    /**
     * When the t-value of the closest point to the robot on the Path is greater than this value,
     * then the Path is considered at its end.
     * This can be custom set for each Path.
     * Default Value: 0.995
     */
    private double tValueConstraint;

    /**
     * When the Path is considered at its end parametrically, then the Follower has this many
     * milliseconds to further correct by default.
     * This can be custom set for each Path.
     * Default Value: 100
     */
    private double timeoutConstraint;

    /**
     * A multiplier for the zero power acceleration to change the speed the robot decelerates at
     * the end of paths.
     * Increasing this will cause the robot to try to decelerate faster, at the risk of overshoots
     * or localization slippage.
     * Decreasing this will cause the deceleration at the end of the Path to be slower, making the
     * robot slower but reducing risk of end-of-path overshoots or localization slippage.
     * This can be set individually for each Path, but this is the default.
     * Default Value: 1
     */
    private double decelerationStrength;

    /**
     * Multiplier for when the path should start its deceleration
     */
    private double decelerationStart;

    /**
     * The number of steps in searching for the closest point
     */
    private int BEZIER_CURVE_SEARCH_LIMIT;

    public PathConstraints(double tValueConstraint, double velocityConstraint, double translationalConstraint, double headingConstraint, double timeoutConstraint, double decelerationStrength, int BEZIER_CURVE_SEARCH_LIMIT, double decelerationStart) {
        this.tValueConstraint = tValueConstraint;
        this.velocityConstraint = velocityConstraint;
        this.translationalConstraint = translationalConstraint;
        this.headingConstraint = headingConstraint;
        this.timeoutConstraint = timeoutConstraint;
        this.decelerationStrength = decelerationStrength;
        this.decelerationStart = decelerationStart;
        this.BEZIER_CURVE_SEARCH_LIMIT = BEZIER_CURVE_SEARCH_LIMIT;
    }

    public PathConstraints(double tValueConstraint, double timeoutConstraint, double decelerationStrength, double decelerationStart) {
        this(tValueConstraint, 0.1, 0.1, 0.007, timeoutConstraint, decelerationStrength, 10, decelerationStart);
    }

    public PathConstraints(double tValueConstraint, double timeoutConstraint) {
        this(tValueConstraint, 0.1, 0.1, 0.007, timeoutConstraint, 1, 10, 1);
    }

    public static PathConstraints defaultConstraints = new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, 1, 10, 1);

    public double getVelocityConstraint() {
        return velocityConstraint;
    }

    public double getTranslationalConstraint() {
        return translationalConstraint;
    }

    public double getHeadingConstraint() {
        return headingConstraint;
    }

    public double getTValueConstraint() {
        return tValueConstraint;
    }

    public double getTimeoutConstraint() {
        return timeoutConstraint;
    }

    public double getDecelerationStrength() {
        return decelerationStrength;
    }

    public double getDecelerationStart() {
        return decelerationStart;
    }

    public int getBEZIER_CURVE_SEARCH_LIMIT() {
        return BEZIER_CURVE_SEARCH_LIMIT;
    }

    public static void setDefaultConstraints(PathConstraints defaultConstraints) {
        PathConstraints.defaultConstraints = defaultConstraints;
    }

    public void setBEZIER_CURVE_SEARCH_LIMIT(int BEZIER_CURVE_SEARCH_LIMIT) {
        this.BEZIER_CURVE_SEARCH_LIMIT = BEZIER_CURVE_SEARCH_LIMIT;
    }

    public void setDecelerationStart(double decelerationStart) {
        this.decelerationStart = decelerationStart;
    }

    public void setDecelerationStrength(double decelerationStrength) {
        this.decelerationStrength = decelerationStrength;
    }

    public void setHeadingConstraint(double headingConstraint) {
        this.headingConstraint = headingConstraint;
    }

    public void setTimeoutConstraint(double timeoutConstraint) {
        this.timeoutConstraint = timeoutConstraint;
    }

    public void setTranslationalConstraint(double translationalConstraint) {
        this.translationalConstraint = translationalConstraint;
    }

    public void setTValueConstraint(double tValueConstraint) {
        this.tValueConstraint = tValueConstraint;
    }

    public void setVelocityConstraint(double velocityConstraint) {
        this.velocityConstraint = velocityConstraint;
    }
}
