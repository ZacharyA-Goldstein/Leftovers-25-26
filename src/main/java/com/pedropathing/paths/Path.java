package com.pedropathing.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.util.ArrayList;

/**
 * This is the Path class. This class handles containing information on the actual path the Follower
 * will follow, not just the shape of the path that the BezierCurve class handles. This contains
 * information on the stop criteria for a Path, the heading interpolation, and deceleration.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/10/2024
 */
public class Path {
    private final BezierCurve curve;
    private PathConstraints constraints;
    private double closestPointCurvature;
    private double closestPointTValue = 0;
    private HeadingInterpolator headingInterpolator = HeadingInterpolator.tangent;
    private Vector closestPointTangentVector;
    private Vector closestPointNormalVector;
    private Pose closestPose;
    
    // A multiplier for the zero power acceleration to change the speed the robot decelerates at
    // the end of paths.
    // Increasing this will cause the robot to try to decelerate faster, at the risk of overshoots
    // or localization slippage.
    // Decreasing this will cause the deceleration at the end of the Path to be slower, making the
    // robot slower but reducing risk of end-of-path overshoots or localization slippage.
    // This can be set individually for each Path, but this is the default.
    private double zeroPowerAccelerationMultiplier;

    // When the robot is at the end of its current Path or PathChain and the velocity goes
    // this value, then end the Path. This is in inches/second.
    // This can be custom set for each Path.
    private double pathEndVelocityConstraint;

    // When the robot is at the end of its current Path or PathChain and the translational error
    // goes below this value, then end the Path. This is in inches.
    // This can be custom set for each Path.
    private double pathEndTranslationalConstraint;

    // When the robot is at the end of its current Path or PathChain and the heading error goes
    // below this value, then end the Path. This is in radians.
    // This can be custom set for each Path.
    private double pathEndHeadingConstraint;

    // When the t-value of the closest point to the robot on the Path is greater than this value,
    // then the Path is considered at its end.
    // This can be custom set for each Path.
    private double pathEndTValueConstraint;

    // When the Path is considered at its end parametrically, then the Follower has this many
    // milliseconds to further correct by default.
    // This can be custom set for each Path.
    private double pathEndTimeoutConstraint;

    private final int BEZIER_CURVE_SEARCH_LIMIT;

    /**
     * Creates a new Path from a BezierCurve. The default heading interpolation is tangential.
     *
     * @param curve the BezierCurve.
     * @param constraints the constraints.
     */
    public Path(BezierCurve curve, PathConstraints constraints) {
        zeroPowerAccelerationMultiplier = constraints.getZeroPowerAccelerationMultiplier();
        pathEndVelocityConstraint = constraints.getVelocityConstraint();
        pathEndTranslationalConstraint = constraints.getTranslationalConstraint();
        pathEndHeadingConstraint = constraints.getHeadingConstraint();
        pathEndTValueConstraint = constraints.getTValueConstraint();
        pathEndTimeoutConstraint = constraints.getTimeoutConstraint();
        BEZIER_CURVE_SEARCH_LIMIT = constraints.getBEZIER_CURVE_SEARCH_LIMIT();
        this.constraints = constraints;
        this.curve = curve;
    }

    /**
     * Creates a new Path from a BezierCurve. The default heading interpolation is tangential.
     *
     * @param curve the BezierCurve.
     */
    public Path(BezierCurve curve) {
        this(curve, PathConstraints.defaultConstraints);
    }

    /**
     * This sets the heading interpolation to linear with a specified start heading and end heading
     * for the Path. This will interpolate across the entire length of the Path, so there may be
     * some issues with end heading accuracy and precision if this is used. If a precise end heading
     * is necessary, then use the setLinearHeadingInterpolation(double startHeading,
     * double endHeading, double endTime) method.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading   The end of the linear heading interpolation.
     *                     This will be reached at the end of the Path if no end time is specified.
     */
    public void setLinearHeadingInterpolation(double startHeading, double endHeading) {
        this.headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading);
    }

    /**
     * This sets the heading interpolation to linear with a specified start heading and end heading
     * for the Path. This will interpolate from the start of the Path to the specified end time.
     * This ensures high accuracy and precision than interpolating across the entire Path. However,
     * interpolating too quickly can cause undesired oscillations and inaccuracies of its own, so
     * generally interpolating to something like 0.8 of your Path should work best.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading   The end of the linear heading interpolation.
     *                     This will be reached at the end of the Path if no end time is specified.
     * @param endTime      The end time on the Path that the linear heading interpolation will finish.
     *                     This value ranges from [0, 1] since Bezier curves are parametric functions.
     */
    public void setLinearHeadingInterpolation(double startHeading, double endHeading, double endTime) {
        this.headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading, endTime);
    }

    /**
     * This sets the heading interpolation to linear with a specified start heading and end heading
     * for the Path. This will interpolate from the start of the Path to the specified end time.
     * This ensures high accuracy and precision than interpolating across the entire Path. However,
     * interpolating too quickly can cause undesired oscillations and inaccuracies of its own, so
     * generally interpolating to something like 0.8 of your Path should work best.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading   The end of the linear heading interpolation.
     *                     This will be reached at the end of the Path if no end time is specified.
     * @param endTime      The end time on the Path that the linear heading interpolation will finish.
     *                     This value ranges from [0, 1] since Bezier curves are parametric functions.
     */
    public void setLinearHeadingInterpolation(double startHeading, double endHeading, double endTime, boolean reversed) {
        this.headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading, endTime);
        if (reversed) {
            reverseHeadingInterpolation();
        }
    }

    /**
     * This sets the heading interpolation to maintain a constant heading.
     *
     * @param setHeading the constant heading for the Path.
     */
    public void setConstantHeadingInterpolation(double setHeading) {
        this.headingInterpolator = HeadingInterpolator.constant(setHeading);
    }

    /**
     * This gets the closest Point from a specified pose to the BezierCurve with a Newton search
     * that is limited to some specified step limit.
     *
     * @param pose        the pose.
     * @param searchLimit the maximum number of iterations to run.
     * @param initialTValueGuess the initial guess for the t-value of the pose
     * @return returns the closest Point.
     */
    public PathPoint getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess) {
        initialTValueGuess = curve.getClosestPoint(pose, searchLimit, initialTValueGuess);
        return new PathPoint(initialTValueGuess, getPoint(initialTValueGuess), curve.getDerivative(initialTValueGuess));
    }

    public PathPoint getClosestPoint(Pose pose, int searchLimit) {
        return getClosestPoint(pose, searchLimit, closestPointTValue);
    }

    /**
     * This gets the point on the path closest to the specified pose.
     * @param pose the pose to find the closest point to
     * @return the closest point t-value
     */
    public PathPoint getClosestPoint(Pose pose) {
        return getClosestPoint(pose, BEZIER_CURVE_SEARCH_LIMIT);
    }

    public PathPoint getClosestPose() {
        return new PathPoint(closestPointTValue, closestPose, closestPointTangentVector);
    }

    public PathPoint updateClosestPose(Pose currentPose, int searchLimit) {
        PathPoint closestPoint = getClosestPoint(currentPose, searchLimit);
        closestPointTValue = closestPoint.getTValue();
        closestPose = closestPoint.getPose();
        closestPointTangentVector = curve.getDerivative(closestPointTValue);
        closestPointNormalVector = curve.getApproxSecondDerivative(closestPointTValue);
        closestPointCurvature = curve.getCurvature(closestPointTValue);
        return closestPoint;
    }

    public PathPoint updateClosestPose(Pose currentPose) {
        return updateClosestPose(currentPose, BEZIER_CURVE_SEARCH_LIMIT);
    }

    /**
     * Reverse the direction of the heading interpolation.
     */
    public void reverseHeadingInterpolation() {
        this.headingInterpolator = headingInterpolator.reverse();
    }

    /**
     * This sets the heading interpolation to tangential.
     */
    public void setTangentHeadingInterpolation() {
        this.headingInterpolator = HeadingInterpolator.tangent;
    }

    /**
     * This gets the tangent Vector at the specified t-value.
     * @param tvalue the t-value to get the tangent Vector at.
     */
    public Vector getTangentVector(double tvalue) {
        return curve.getDerivative(tvalue);
    }

    /**
     * This returns the unit tangent Vector at the end of the BezierCurve.
     *
     * @return returns the end tangent Vector.
     */
    public Vector getEndTangent() {
        return curve.getEndTangent();
    }

    /**
     * NOTE: THIS DOES NOT RETURN HEADING. This returns the point on the Bezier curve that is specified by the parametric t value. A
     * Bezier curve is a parametric function that returns points along it with t ranging from [0, 1],
     * with 0 being the beginning of the curve and 1 being at the end. The Follower will follow
     * BezierCurves from 0 to 1, in terms of t.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the point requested.
     */
    public Pose getPoint(double t) {
        return curve.getPose(t);
    }

    public Pose getPose(double t) {
        Pose position = curve.getPose(t);
        return new Pose(position.getX(), position.getY(), getHeadingGoal(t));
    }

    public PathPoint getPoseInformation(double t) {
        return new PathPoint(t, getPose(t), getTangentVector(t));
    }

    /**
     * This returns the t-value of the closest Point on the BezierCurve.
     *
     * @return returns the closest Point t-value.
     */
    public double getClosestPointTValue() {
        return closestPointTValue;
    }

    /**
     * This returns the approximated length of the BezierCurve.
     *
     * @return returns the length of the BezierCurve.
     */
    public double length() {
        return curve.length();
    }

    /**
     * This returns the curvature of the BezierCurve at a specified t-value.
     *
     * @param t the specified t-value.
     * @return returns the curvature of the BezierCurve at the specified t-value.
     */
    public double getCurvature(double t) {
        return curve.getCurvature(t);
    }

    /**
     * This returns the curvature of the BezierCurve at the closest Point.
     *
     * @return returns the curvature of the BezierCurve at the closest Point.
     */
    public double getClosestPointCurvature() {
        return closestPointCurvature;
    }

    /**
     * This returns the normal Vector at the closest Point.
     *
     * @return returns the normal Vector at the closest Point.
     */
    public Vector getClosestPointNormalVector() {
        return MathFunctions.copyVector(closestPointNormalVector);
    }

    /**
     * This returns the tangent Vector at the closest Point.
     *
     * @return returns the tangent Vector at the closest Point.
     */
    public Vector getClosestPointTangentVector() {
        return MathFunctions.copyVector(closestPointTangentVector);
    }

    /**
     * This returns the heading goal at the closest Point.
     *
     * @return returns the heading goal at the closest Point.
     */
    public double getClosestPointHeadingGoal() {
        return getHeadingGoal(new PathPoint(closestPointTValue, closestPose, closestPointTangentVector));
    }
    
    public double getHeadingGoal(PathPoint closestPoint) {
        return this.headingInterpolator.interpolate(closestPoint);
    }

    public double getHeadingGoal(double t) {
        return this.headingInterpolator.interpolate(new PathPoint(t, curve.getPose(t), getTangentVector(t)));
    }
    
    public void setHeadingInterpolation(HeadingInterpolator interpolator) {
        this.headingInterpolator = interpolator;
    }

    /**
     * This returns if the robot is at the end of the Path, according to the parametric t-value.
     *
     * @return returns if at end.
     */
    public boolean isAtParametricEnd() {
        return curve.atParametricEnd(closestPointTValue);
    }

    /**
     * This returns if the robot is at the beginning of the Path, according to the parametric t-value.
     *
     * @return returns if at start.
     */
    public boolean isAtParametricStart() {
        return closestPointTValue <= 1 - pathEndTValueConstraint;
    }

    /**
     * Returns the ArrayList of control points for this BezierCurve.
     *
     * @return This returns the control points.
     */
    public ArrayList<Pose> getControlPoints() {
        return curve.getControlPoints();
    }

    /**
     * Returns the first control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getFirstControlPoint() {
        return curve.getFirstControlPoint();
    }

    /**
     * Returns the second control point, or the one after the start, for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getSecondControlPoint() {
        return curve.getSecondControlPoint();
    }

    /**
     * Returns the second to last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getSecondToLastControlPoint() {
        return curve.getSecondToLastControlPoint();
    }

    /**
     * Returns the last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getLastControlPoint() {
        return curve.getLastControlPoint();
    }

    /**
     * This sets the path's deceleration factor in terms of the natural deceleration of the robot
     * when power is cut from the drivetrain.
     *
     * @param set This sets the multiplier.
     */
    public void setZeroPowerAccelerationMultiplier(double set) {
        zeroPowerAccelerationMultiplier = set;
    }

    /**
     * This sets the velocity stop criteria. When velocity is below this amount, then this is met.
     *
     * @param set This sets the velocity end constraint.
     */
    public void setPathEndVelocityConstraint(double set) {
        pathEndVelocityConstraint = set;
    }

    /**
     * This sets the translational stop criteria. When the translational error, or how far off the
     * end point the robot is, goes below this, then the translational end criteria is met.
     *
     * @param set This sets the translational end constraint.
     */
    public void setPathEndTranslationalConstraint(double set) {
        pathEndTranslationalConstraint = set;
    }

    /**
     * This sets the heading stop criteria. When the heading error is less than this amount, then
     * the heading end criteria is met.
     *
     * @param set This sets the heading end constraint.
     */
    public void setPathEndHeadingConstraint(double set) {
        pathEndHeadingConstraint = set;
    }

    /**
     * This sets the parametric end criteria. When the t-value of the closest Point on the Path is
     * greater than this amount, then the parametric end criteria is met.
     *
     * @param set This sets the t-value end constraint.
     */
    public void setPathEndTValueConstraint(double set) {
        pathEndTValueConstraint = set;
    }

    /**
     * This sets the Path end timeout. If the Path is at its end parametrically, then the Follower
     * has this many milliseconds to correct before the Path gets ended anyways.
     *
     * @param set This sets the Path end timeout.
     */
    public void setPathEndTimeoutConstraint(double set) {
        pathEndTimeoutConstraint = set;
    }

    /**
     * This gets the deceleration multiplier.
     *
     * @return This returns the deceleration multiplier.
     */
    public double getZeroPowerAccelerationMultiplier() {
        return zeroPowerAccelerationMultiplier;
    }

    /**
     * This gets the velocity stop criteria.
     *
     * @return This returns the velocity stop criteria.
     */
    public double getPathEndVelocityConstraint() {
        return pathEndVelocityConstraint;
    }

    /**
     * This gets the translational stop criteria.
     *
     * @return This returns the translational stop criteria.
     */
    public double getPathEndTranslationalConstraint() {
        return pathEndTranslationalConstraint;
    }

    /**
     * This gets the heading stop criteria.
     *
     * @return This returns the heading stop criteria.
     */
    public double getPathEndHeadingConstraint() {
        return pathEndHeadingConstraint;
    }

    /**
     * This gets the parametric end criteria.
     *
     * @return This returns the parametric end criteria.
     */
    public double getPathEndTValueConstraint() {
        return pathEndTValueConstraint;
    }

    /**
     * This gets the Path end correction time.
     *
     * @return This returns the Path end correction time.
     */
    public double getPathEndTimeoutConstraint() {
        return pathEndTimeoutConstraint;
    }

    /**
     * Returns the type of path. This is used in case we need to identify the type of BezierCurve
     * this is.
     *
     * @return returns the type of path.
     */
    public String pathType() {
        return curve.pathType();
    }

    /**
     * This returns a 2D Array of doubles containing the x and y positions of points to draw on FTC
     * Dashboard.
     *
     * @return returns the 2D Array to draw on FTC Dashboard
     */
    public double[][] getDashboardDrawingPoints() {
        return curve.getDashboardDrawingPoints();
    }

    /**
     * This gets the heading interpolation.
     * @return This returns the heading interpolation.
     */
    public HeadingInterpolator getHeadingInterpolator() {
        return headingInterpolator;
    }

    public Pose endPose() {
        Pose lastControlPoint = curve.getLastControlPoint();
        return new Pose(lastControlPoint.getX(), lastControlPoint.getY(),
            getHeadingGoal(new PathPoint(1, lastControlPoint, curve.getEndTangent())));
    }

    public Path getReversed() {
        return new Path(curve.getReversed());
    }

    public void setConstraints(PathConstraints constraints) {
        this.constraints = constraints;
        zeroPowerAccelerationMultiplier = constraints.getZeroPowerAccelerationMultiplier();
        pathEndVelocityConstraint = constraints.getVelocityConstraint();
        pathEndTranslationalConstraint = constraints.getTranslationalConstraint();
        pathEndHeadingConstraint = constraints.getHeadingConstraint();
        pathEndTValueConstraint = constraints.getTValueConstraint();
        pathEndTimeoutConstraint = constraints.getTimeoutConstraint();
    }
}