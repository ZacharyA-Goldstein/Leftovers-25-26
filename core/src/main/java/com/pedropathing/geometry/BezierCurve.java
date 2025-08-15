package com.pedropathing.geometry;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import static com.pedropathing.math.AbstractBijectiveMap.NumericBijectiveMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Future;

/**
 * This is the BezierCurve class. This class handles the creation of Bezier curves, which are used
 * as the basis of the path for the Path class. Bezier curves are parametric curves defined by a set
 * of control points. So, they take in one input, t, that ranges from [0, 1] and that returns a point
 * on the curve. Essentially, Bezier curves are a way of defining a parametric line easily. You can
 * read more on Bezier curves here: <a href="https://en.wikipedia.org/wiki/Bézier_curve">...</a>
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
public class BezierCurve implements Curve {
    private ArrayList<Pose> controlPoints;

    protected ArrayList<FuturePose> futureControlPoints = new ArrayList<>();

    protected boolean initialized = false;

    private Vector endTangent = new Vector();

    protected final int APPROXIMATION_STEPS = 1000;

    private final int DASHBOARD_DRAWING_APPROXIMATION_STEPS = 100;

    private double[][] panelsDrawingPoints;

    private double length;

    private Matrix cachedMatrix = new Matrix();

    private int[][] diffPowers;
    private int[][] diffCoefficients;
    protected PathConstraints pathConstraints;

    protected NumericBijectiveMap completionMap = new NumericBijectiveMap();

    /**
     * This is the default constructor for the BezierCurve class. It initializes an empty BezierCurve.
     * This is not recommended to use, as it does not set any control points or path constraints.
     */
    public BezierCurve() {
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and path constraints.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     * @param constraints the path constraints for the BezierCurve.
     */
    public BezierCurve(List<Pose> controlPoints, PathConstraints constraints){
        this.pathConstraints = constraints;
        if (controlPoints.size()<3) {
            try {
                throw new Exception("Too few control points");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        this.controlPoints = new ArrayList<>(controlPoints);
        initialize();
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and path constraints.
     * @param constraints the path constraints for the BezierCurve.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     */
    protected BezierCurve(PathConstraints constraints, List<FuturePose> controlPoints) {
        this.pathConstraints = constraints;
        if (controlPoints.size()<3) {
            try {
                throw new Exception("Too few control points");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        boolean lazyInitialize = false;
        ArrayList<Pose> initializedControlPoints = new ArrayList<>();
        for (FuturePose pose : controlPoints) {
            if (!pose.initialized()) {
                lazyInitialize = true;
                break;
            }

            initializedControlPoints.add(pose.getPose());
        }

        if (lazyInitialize) {
            this.controlPoints = new ArrayList<>();
            this.futureControlPoints = new ArrayList<>(controlPoints);
        } else {
            this.controlPoints = initializedControlPoints;
            initialize();
        }
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and the default path constraints.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     */
    public BezierCurve(List<Pose> controlPoints) {
        this(controlPoints, PathConstraints.defaultConstraints);
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and path constraints.
     * @param constraints the path constraints for the BezierCurve.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     */
    public BezierCurve(PathConstraints constraints, FuturePose... controlPoints) {
        this(constraints, new ArrayList<>(Arrays.asList(controlPoints)));
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and the default path constraints.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     */
    public BezierCurve(FuturePose... controlPoints) {
        this(PathConstraints.defaultConstraints, controlPoints);
    }

    /**
     * This handles most of the initialization of the BezierCurve that is called from the constructor.
     */
    public void initialize() {
        if (initialized) return; // If already initialized, do nothing
        if (controlPoints.isEmpty() && !futureControlPoints.isEmpty()) {
            for (FuturePose pose : futureControlPoints) {
                controlPoints.add(pose.getPose());
            }
            futureControlPoints.clear();
        }
        initialized = true;
        generateBezierCurve();
        length = approximateLength();
        endTangent.setOrthogonalComponents(controlPoints.get(controlPoints.size()-1).getX()-controlPoints.get(controlPoints.size()-2).getX(),
                controlPoints.get(controlPoints.size()-1).getY()-controlPoints.get(controlPoints.size()-2).getY());
        endTangent = endTangent.normalize();
        initializePanelsDrawingPoints();
    }

    /**
     * This creates the Array that holds the Points to draw on Panels.
     */
    public void initializePanelsDrawingPoints() {
        this.panelsDrawingPoints = new double[2][this.DASHBOARD_DRAWING_APPROXIMATION_STEPS + 1];
        for (int i = 0; i <= this.DASHBOARD_DRAWING_APPROXIMATION_STEPS; i++) {
            Pose currentPoint = this.getPose(i/(double) (this.DASHBOARD_DRAWING_APPROXIMATION_STEPS));
            this.panelsDrawingPoints[0][i] = currentPoint.getX();
            this.panelsDrawingPoints[1][i] = currentPoint.getY();
        }
    }

    /**
     * This returns a 2D Array of doubles containing the x and y positions of points to draw on Panels.
     * @return returns the 2D Array to draw on Panels
     */
    public double[][] getPanelsDrawingPoints() {
        return this.panelsDrawingPoints;
    }

    /**
     * This generates the Bezier curve. It assumes that the ArrayList of control points has been set.
     * This caches the matrix generated by multiplying the characteristic matrix and the matrix where each control
     * point is a row vector.
     */
    public void generateBezierCurve() {
        Matrix controlPointMatrix = new Matrix(this.controlPoints.size(), 2);
        for (int i = 0; i < this.controlPoints.size(); i++) {
            Pose p = this.controlPoints.get(i);
            controlPointMatrix.set(i, new double[]{p.getX(), p.getY()});
        }
        this.cachedMatrix = CharacteristicMatrixSupplier.getBezierCharacteristicMatrix(this.controlPoints.size() - 1).multiply(controlPointMatrix);
        initializeDegreeArray();
        initializeCoefficientArray();
    }

    /**
     * This returns the unit tangent Vector at the end of the BezierCurve.
     *
     * @return returns the end tangent Vector.
     */
    public Vector getEndTangent() {
        return endTangent.copy();
    }

    /**
     * This approximates the length of the BezierCurve in APPROXIMATION_STEPS number of steps. It's
     * like a Riemann's sum, but for a parametric function's arc length.
     *
     * @return returns the approximated length of the BezierCurve.
     */
    public double approximateLength() {
        Pose previousPoint = getPose(0);
        Pose currentPoint;
        double approxLength = 0;
        for (int i = 1; i <= APPROXIMATION_STEPS; i++) {
            double t = i/(double)APPROXIMATION_STEPS;
            currentPoint = getPose(t);
            approxLength += previousPoint.distanceFrom(currentPoint);
            previousPoint = currentPoint;
            completionMap.put(t, approxLength);
        }
        return approxLength;
    }

    /**
     * Initializes the degree/power array (for later processing) and cache them
     */
    public void initializeDegreeArray(){
        int deg = this.controlPoints.size() - 1;
        // for now, cache position, velocity, and acceleration powers (thus 3) per bezier obj (change to global caching later)
        this.diffPowers = new int[3][this.controlPoints.size()];

        for (int i = 0; i < this.diffPowers.length; i++) {
            this.diffPowers[i] = BezierCurve.genDiff(deg, i);
        }
    }

    /**
     * Generate and return a polynomial's powers at the differentiation level
     * @param deg degree of poly
     * @param diffLevel number of differentiations
     * @return powers of each term in integers
     */
    private static int[] genDiff(int deg, int diffLevel){
        int[] output = new int[deg + 1];

        for (int i = diffLevel; i < output.length; i++) {
            output[i] = i - diffLevel;
        }

        return output;
    }

    /**
     * Initializes the coefficient array (for later processing) and cache them.
     * Each row is a different level of differentiation.
     */
    public void initializeCoefficientArray(){
        // for now, cache coefficients for the 0th, 1st, and 2nd derivatives (change to global caching later)
        this.diffCoefficients = new int[3][this.controlPoints.size()];

        Arrays.fill(this.diffCoefficients[0], 1);

        for (int row = 1; row < this.diffCoefficients.length; row++) {
            for (int col = 0; col < this.diffCoefficients[0].length; col++) {
                this.diffCoefficients[row][col] = this.diffCoefficients[row - 1][col] * this.diffPowers[row - 1][col];
            }
        }
    }

    /**
     * This method gets the t-vector at the specified differentiation level.
     * @param t t value of the parametric curve; [0, 1]
     * @param diffLevel specifies how many differentiations are done
     * @return t vector
     */
    public double[] getTVector(double t, int diffLevel){
        int[] degrees = this.diffPowers[diffLevel];
        double[] powers = new double[this.controlPoints.size()];

        powers[0] = 1;
        for (int i = 1; i < powers.length; i++) {
            powers[i] = t * powers[i - 1];
        }

        double[] output = new double[powers.length];

        for (int i = 0; i < degrees.length; i++) {
            output[i] = powers[degrees[i]] * this.diffCoefficients[diffLevel][i];
        }

        return output;
    }

    /**
     * This returns the point on the Bezier curve that is specified by the parametric t value. A
     * Bezier curve is a parametric function that returns points along it with t ranging from [0, 1],
     * with 0 being the beginning of the curve and 1 being at the end. The Follower will follow
     * BezierCurves from 0 to 1, in terms of t.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the point requested.
     */
    public Pose getPose(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Matrix outPos = (new Matrix(new double[][]{getTVector(t, 0)})).multiply(this.cachedMatrix);
        return new Pose(outPos.get(0, 0), outPos.get(0, 1));
    }

    /**
     * This returns the curvature of the Bezier curve at a specified t-value.
     *
     * @param t the parametric t input.
     * @return returns the curvature.
     */
    public double getCurvature(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Vector derivative = getDerivative(t);
        Vector secondDerivative = getSecondDerivative(t);

        if (derivative.getMagnitude() == 0) return 0;
        return (derivative.cross(secondDerivative))/Math.pow(derivative.getMagnitude(),3);
    }

    /**
     * This returns the derivative on the BezierCurve that is specified by the parametric t value.
     * This is returned as a Vector, and this Vector is the tangent to the BezierCurve.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the derivative requested.
     */
    public Vector getDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Matrix outVel = (new Matrix(new double[][]{getTVector(t, 1)})).multiply(this.cachedMatrix);
        Vector output = new Vector();
        output.setOrthogonalComponents(outVel.get(0, 0), outVel.get(0, 1));
        return output;
    }

    /**
     * This returns the second derivative on the BezierCurve that is specified by the parametric t value.
     * This is returned as a Vector, and this Vector is the acceleration on the BezierCurve.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the second derivative requested.
     */
    public Vector getSecondDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Matrix outAccel = (new Matrix(new double[][]{getTVector(t, 2)})).multiply(this.cachedMatrix);
        Vector output = new Vector();
        output.setOrthogonalComponents(outAccel.get(0, 0), outAccel.get(0, 1));
        return output;
    }

    /**
     * This method calculates the position, velocity, and acceleration and puts them into a matrix
     * as row vectors.
     * @param t t value of the parametric curve; [0, 1]
     * @return matrix with row vectors corresponding to position, velocity, and acceleration at the requested t value
     */
    public Matrix getPointCharacteristics(double t){
        t = MathFunctions.clamp(t, 0, 1);

        return new Matrix(new double[][]{
                getTVector(t, 0),
                getTVector(t, 1),
                getTVector(t, 2)
        }).multiply(this.cachedMatrix);
    }

    /**
     * This gets an approximate second derivative essentially using the limit method. This is used for heading only
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the approximated second derivative.
     */
    @Override
    public Vector getNormalVector(double t) {
        double current = getDerivative(t).getTheta();
        double deltaCurrent = getDerivative(t + 0.0001).getTheta();

        return new Vector(1, deltaCurrent - current);
    }

    /**
     * Returns the ArrayList of control points for this BezierCurve.
     *
     * @return This returns the control points.
     */
    public ArrayList<Pose> getControlPoints() {
        return controlPoints;
    }

    /**
     * Returns the first control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getFirstControlPoint() {
        return controlPoints.get(0);
    }

    /**
     * Returns the second control point, or the one after the start, for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getSecondControlPoint() {
        return controlPoints.get(1);
    }

    /**
     * Returns the second to last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getSecondToLastControlPoint() {
        return controlPoints.get(controlPoints.size()-2);
    }

    /**
     * Returns the last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getLastControlPoint() {
        return controlPoints.get(controlPoints.size()-1);
    }

    /**
     * Returns the approximate length of this BezierCurve.
     *
     * @return This returns the length.
     */
    public double length() {
        return length;
    }

    /**
     * Returns the type of path. This is used in case we need to identify the type of BezierCurve
     * this is.
     *
     * @return returns the type of path.
     */
    public String pathType() {
        return "curve";
    }

    /**
     * Returns a new BezierCurve with the control points reversed.
     *
     * @return a new BezierCurve with reversed control points.
     */
    public BezierCurve getReversed() {
        ArrayList<Pose> reversedControlPoints = new ArrayList<>(controlPoints);
        Collections.reverse(reversedControlPoints);
        BezierCurve reversedCurve = new BezierCurve(reversedControlPoints);
        reversedCurve.initialize();
        return reversedCurve;
    }

    /**
     * Returns the closest point t-value to the specified pose.
     * @param pose the pose to find the closest point to
     * @return the closest point t-value
     */
    public double getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess) {
        double[] searchEstimates = new double[]{0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, initialTValueGuess};
        double closestDist = 1e9;
        double bestGuess = 0;

        for (double t : searchEstimates) {
            double dist = getPose(t).distSquared(pose);

            if (dist < closestDist) {
                closestDist = dist;
                bestGuess = t;
            }
        }

        initialTValueGuess = bestGuess;

        for (int i = 0; i < searchLimit; i++) {
            Matrix pointAtT = this.getPointCharacteristics(initialTValueGuess);
            Pose lastPos = new Pose(pointAtT.get(0, 0), pointAtT.get(0, 1));
            /*
            Resultant matrix is:
            [...] // first row is ignored
            [(lastPos.x - pose.x) * x'(t) + (lastPos.y - pose.y) * y'(t)]
            [(lastPos.x - pose.x) * x''(t) + (lastPos.y - pose.y) * y''(t)]
             */
            Matrix resultant = pointAtT.multiply(new Matrix(new double[][]{
                    {(lastPos.getX() - pose.getX())},
                    {(lastPos.getY() - pose.getY())}
            }));

            double firstDerivative = 2 * resultant.get(1, 0);
            double secondDerivative = 2 * (resultant.get(2, 0) + (pointAtT.get(1, 0) * pointAtT.get(1, 0) + (pointAtT.get(1, 1) * pointAtT.get(1, 1))));

            initialTValueGuess = MathFunctions.clamp(initialTValueGuess - firstDerivative / (secondDerivative + 1e-9), 0, 1);
            if (getPose(initialTValueGuess).distanceFrom(lastPos) < 0.1) break;
        }

        return initialTValueGuess;
    }

    /**
     * Returns the closest point t-value to the specified pose.
     * @param pose the pose to find the closest point to
     * @return the closest point t-value
     */
    public double getClosestPoint(Pose pose, double initialTValueGuess) {
        return getClosestPoint(pose, getPathConstraints().getBEZIER_CURVE_SEARCH_LIMIT(), initialTValueGuess);
    }

    /**
     * Returns whether the t value is at the end of the parametric curve.
     * @param t the t value of the parametric curve; [0, 1]
     * @return true if at the end, false otherwise
     */
    public boolean atParametricEnd(double t) {
        return t >= pathConstraints.getTValueConstraint();
    }

    /**
     * Sets the control points for this BezierCurve.
     * @param controlPoints the new control points to set
     */
    public void setControlPoints(ArrayList<Pose> controlPoints) {
        this.controlPoints = controlPoints;
    }

    /**
     * Sets the path constraints for this BezierCurve.
     * @param pathConstraints the new path constraints to set
     */
    public void setPathConstraints(PathConstraints pathConstraints) {
        this.pathConstraints = pathConstraints;
    }

    @Override
    public PathConstraints getPathConstraints() {
        return pathConstraints;
    }

    /**
     * Returns the path completion at a given t value.
     * This is used to get the percentage of the path that has been completed.
     *
     * @param t the t value of the parametric curve; [0, 1]
     * @return returns the path completion as a decimal on the range [0,1].
     */
    public double getPathCompletion(double t) {
        if (length == 0) return 0.0;
        return completionMap.interpolateKey(t) / length;
    }

    /**
     * Returns the t value corresponding to a given path completion percentage.
     * @param pathCompletion the path completion percentage; [0, 1]
     * @return returns the t value corresponding to the path completion percentage.
     */
    public double getT(double pathCompletion) {
        return completionMap.interpolateValue(pathCompletion);
    }

    /**
     * Returns whether the BezierCurve has been initialized.
     * @return true if the BezierCurve has been initialized, false otherwise.
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Computes the control points for a cubic Bézier spline that passes through the specified points.
     * This method uses a cubic spline interpolation algorithm to compute the control points.
     *
     * @param throughPoints the points that the Bézier spline should pass through.
     * @return a list of control points for the Bézier spline.
     */
    public static List<Pose> computeControlPointsThrough(List<Pose> throughPoints) {
        int n = throughPoints.size() - 1;
        if (n < 1) throw new IllegalArgumentException("Need at least two points");

        List<Pose> c1 = new ArrayList<>();
        List<Pose> c2 = new ArrayList<>();

        if (n == 1) {
            // Straight line case
            Pose p0 = throughPoints.get(0);
            Pose p3 = throughPoints.get(1);
            Pose ctrl1 = new Pose(
                    p0.getX() + (p3.getX() - p0.getX()) / 3.0,
                    p0.getY() + (p3.getY() - p0.getY()) / 3.0
            );
            Pose ctrl2 = new Pose(
                    p0.getX() + 2 * (p3.getX() - p0.getX()) / 3.0,
                    p0.getY() + 2 * (p3.getY() - p0.getY()) / 3.0
            );
            return Arrays.asList(p0, ctrl1, ctrl2, p3);
        }

        double[] a = new double[n];
        double[] b = new double[n];
        double[] c = new double[n];
        double[] rhsX = new double[n];
        double[] rhsY = new double[n];

        a[0] = 0;
        b[0] = 2;
        c[0] = 1;
        rhsX[0] = throughPoints.get(0).getX() + 2 * throughPoints.get(1).getX();
        rhsY[0] = throughPoints.get(0).getY() + 2 * throughPoints.get(1).getY();

        for (int i = 1; i < n - 1; i++) {
            a[i] = 1;
            b[i] = 4;
            c[i] = 1;
            rhsX[i] = 4 * throughPoints.get(i).getX() + 2 * throughPoints.get(i + 1).getX();
            rhsY[i] = 4 * throughPoints.get(i).getY() + 2 * throughPoints.get(i + 1).getY();
        }

        a[n - 1] = 2;
        b[n - 1] = 7;
        c[n - 1] = 0;
        rhsX[n - 1] = 8 * throughPoints.get(n - 1).getX() + throughPoints.get(n).getX();
        rhsY[n - 1] = 8 * throughPoints.get(n - 1).getY() + throughPoints.get(n).getY();

        // Forward elimination
        for (int i = 1; i < n; i++) {
            double m = a[i] / b[i - 1];
            b[i] -= m * c[i - 1];
            rhsX[i] -= m * rhsX[i - 1];
            rhsY[i] -= m * rhsY[i - 1];
        }

        double[] xC1 = new double[n];
        double[] yC1 = new double[n];
        xC1[n - 1] = rhsX[n - 1] / b[n - 1];
        yC1[n - 1] = rhsY[n - 1] / b[n - 1];

        // Back substitution
        for (int i = n - 2; i >= 0; i--) {
            xC1[i] = (rhsX[i] - c[i] * xC1[i + 1]) / b[i];
            yC1[i] = (rhsY[i] - c[i] * yC1[i + 1]) / b[i];
        }

        for (int i = 0; i < n; i++) {
            Pose ctrl1 = new Pose(xC1[i], yC1[i]);
            Pose ctrl2;
            if (i < n - 1) {
                ctrl2 = new Pose(
                        2 * throughPoints.get(i + 1).getX() - xC1[i + 1],
                        2 * throughPoints.get(i + 1).getY() - yC1[i + 1]
                );
            } else {
                ctrl2 = new Pose(
                        (throughPoints.get(n).getX() + xC1[n - 1]) / 2,
                        (throughPoints.get(n).getY() + yC1[n - 1]) / 2
                );
            }
            c1.add(ctrl1);
            c2.add(ctrl2);
        }

        // This returns the full control point sequence for a multi-segment Bézier spline
        List<Pose> controlPoints = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            controlPoints.add(throughPoints.get(i));
            controlPoints.add(c1.get(i));
            controlPoints.add(c2.get(i));
        }
        controlPoints.add(throughPoints.get(n));

        return controlPoints;
    }

    public static BezierCurve through(Pose... throughPoints) {
        return new BezierCurve(computeControlPointsThrough(Arrays.asList(throughPoints)));
    }
}