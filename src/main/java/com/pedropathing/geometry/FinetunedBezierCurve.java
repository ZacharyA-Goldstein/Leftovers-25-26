package com.pedropathing.geometry;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

public class FinetunedBezierCurve extends BezierCurve {
    private final Pose endPoint;
    private double crossingThreshold;
    private BezierCurve modifiedCurve;
    private final double pathEndTValueConstraint;

    public FinetunedBezierCurve(List<Pose> controlPoints, Pose endPoint, int searchLimit, double pathEndTValueConstraint) {
        super(controlPoints);
        this.endPoint = endPoint;
        this.pathEndTValueConstraint = pathEndTValueConstraint;
        crossingThreshold = getClosestPoint(endPoint, searchLimit, 1.0);
        if (crossingThreshold == 0) crossingThreshold += 0.001;

        if (crossingThreshold < pathEndTValueConstraint) {
            if (this.pathType() == "curve") {
                ArrayList<Pose> points = new ArrayList<>(controlPoints);
                points.set(points.size() - 1, endPoint);
                modifiedCurve = new BezierCurve(points);
            } else if (this.pathType() == "line") {
                modifiedCurve = new BezierLine(controlPoints.get(0), endPoint);
            } else {
                modifiedCurve = new BezierPoint(endPoint);
            }
        } else {
            modifiedCurve = new BezierLine(getLastControlPoint(), endPoint);
        }
    }

    public Pose getEndPoint() {
        return endPoint;
    }

    /**
     * Sets the first t-value where the endpoint starts taking effect instead of the last control point. This allows the user to control how local their finetuning changes are.
     * @param crossingThreshold the first t-value where the endpoint starts taking effect
     */
    public void setCrossingThreshold(double crossingThreshold) {
        this.crossingThreshold = crossingThreshold;
    }

    private double convertT(double t) {
        return t/crossingThreshold;
    }

    @Override
    public Pose getPose(double t) {
        if (t < crossingThreshold) {
            return getPose(t);
        } else if (crossingThreshold < pathEndTValueConstraint) {
            double scale = Range.scale(t, crossingThreshold, 1.0, 0.0, 1.0);
            return MathFunctions.linearCombination(super.getPose(t), modifiedCurve.getPose(t), scale, 1-scale);
        } else {
            double scale = Range.scale(t, crossingThreshold, 1.0, 0.0, 1.0);
            return modifiedCurve.getPose(scale);
        }
    }
}
