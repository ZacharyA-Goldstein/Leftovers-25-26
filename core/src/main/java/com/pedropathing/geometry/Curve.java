package com.pedropathing.geometry;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.ArrayList;

public interface Curve {
    Vector getNormalVector(double t);
    Curve getReversed();
    Vector getDerivative(double t);
    Pose getPose(double t);
    Vector getSecondDerivative(double t);
    boolean atParametricEnd(double t);
    ArrayList<Pose> getControlPoints();
    void setPathConstraints(PathConstraints constraints);
    PathConstraints getPathConstraints();
    double getPathCompletion(double t);
    double getT(double pathCompletion);

    default Pose getFirstControlPoint() {
        return getControlPoints().get(0);
    }

    default Pose getLastControlPoint() {
        ArrayList<Pose> controlPoints = getControlPoints();
        return controlPoints.get(controlPoints.size() - 1);
    }

    default Pose getSecondToLastControlPoint() {
        ArrayList<Pose> controlPoints = getControlPoints();
        return controlPoints.get(controlPoints.size() - 2);
    }

    default Pose getSecondControlPoint() {
        ArrayList<Pose> controlPoints = getControlPoints();
        return controlPoints.get(1);
    }

    default double getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess) {
        for (int i = 0; i < searchLimit; i++) {
            Pose lastPoint = getPose(initialTValueGuess);

            Vector differenceVector = new Vector(lastPoint.minus(pose));

            double firstDerivative = 2 * getDerivative(initialTValueGuess).dot(differenceVector);
            double secondDerivative = 2 * (Math.pow(getDerivative(initialTValueGuess).getMagnitude(), 2) +
                    differenceVector.dot(getSecondDerivative(initialTValueGuess)));

            initialTValueGuess = MathFunctions.clamp(initialTValueGuess - firstDerivative / (secondDerivative + 1e-9), 0, 1);
            if (getPose(initialTValueGuess).distanceFrom(lastPoint) < 0.1) break;
        }

        return initialTValueGuess;
    }

    default double getCurvature(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Vector derivative = getDerivative(t);
        Vector secondDerivative = getSecondDerivative(t);

        if (derivative.getMagnitude() == 0) return 0;
        return (derivative.cross(secondDerivative))/Math.pow(derivative.getMagnitude(),3);
    }

    default double length() {
        Pose previousPoint = getPose(0);
        Pose currentPoint;
        double approxLength = 0;
        for (int i = 1; i <= 1000; i++) {
            currentPoint = getPose(i/1000.0);
            approxLength += previousPoint.distanceFrom(currentPoint);
            previousPoint = currentPoint;
        }
        return approxLength;
    }

    default double[][] getPanelsDrawingPoints() {
        double[][] panelsDrawingPoints = new double[2][101];
        for (int i = 0; i <= 100; i++) {
            Pose currentPoint = this.getPose(i/(double) (100));
            panelsDrawingPoints[0][i] = currentPoint.getX();
            panelsDrawingPoints[1][i] = currentPoint.getY();
        }

        return panelsDrawingPoints;
    }

    default Vector getEndTangent() {
        return getDerivative(1.0);
    }

    default String pathType() {
        return "other";
    }
}
