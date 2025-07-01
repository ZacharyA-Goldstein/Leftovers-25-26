package com.pedropathing.math;

import androidx.annotation.NonNull;
import com.pedropathing.geometry.Pose;

    /**
     * Represents a 2D vector with both polar (magnitude, theta) and Cartesian (x, y) components.
     * Provides methods for setting and retrieving vector properties, as well as converting between
     * polar and Cartesian representations.
     * <p>
     * Vectors are useful for mathematical operations in robotics, such as calculating directions,
     * magnitudes, and performing vector math (dot/cross products, rotations, etc.).
     * </p>
     *
     * <p>
     * Authors:
     * <ul>
     *   <li>Anyi Lin - 10158 Scott's Bots</li>
     *   <li>Aaron Yang - 10158 Scott's Bots</li>
     *   <li>Harrison Womack - 10158 Scott's Bots</li>
     * </ul>
     * </p>
     *
     * @version 1.0, 3/11/2024
     */
    public class Vector {

        /** The magnitude (length) of the vector. */
        private double magnitude;
        /** The direction (angle in radians) of the vector. */
        private double theta;
        /** The x component of the vector in Cartesian coordinates. */
        private double xComponent;
        /** The y component of the vector in Cartesian coordinates. */
        private double yComponent;

        /**
         * Constructs a new Vector with zero magnitude and direction.
         */
        public Vector() {
            setComponents(0, 0);
        }

        /**
         * Constructs a new Vector from a given Pose's x and y coordinates.
         *
         * @param pose the Pose object to extract x and y from
         */
        public Vector(Pose pose) {
            setOrthogonalComponents(pose.getX(), pose.getY());
        }

        /**
         * Constructs a new Vector with a specified magnitude and direction.
         *
         * @param magnitude the magnitude (length) of the vector
         * @param theta the direction (angle in radians) of the vector
         */
        public Vector(double magnitude, double theta) {
            setComponents(magnitude, theta);
        }

        /**
         * Sets the vector's magnitude and direction (polar coordinates).
         * Updates the Cartesian components accordingly.
         *
         * @param magnitude the magnitude to set
         * @param theta the direction (angle in radians) to set
         */
        public void setComponents(double magnitude, double theta) {
            double[] orthogonalComponents;
            if (magnitude < 0) {
                this.magnitude = -magnitude;
                this.theta = MathFunctions.normalizeAngle(theta + Math.PI);
            } else {
                this.magnitude = magnitude;
                this.theta = MathFunctions.normalizeAngle(theta);
            }
            orthogonalComponents = Pose.polarToCartesian(magnitude, theta);
            xComponent = orthogonalComponents[0];
            yComponent = orthogonalComponents[1];
        }

        /**
         * Sets only the magnitude of the vector, keeping the direction unchanged.
         *
         * @param magnitude the new magnitude
         */
        public void setMagnitude(double magnitude) {
            setComponents(magnitude, theta);
        }

        /**
         * Sets only the direction (theta) of the vector, keeping the magnitude unchanged.
         *
         * @param theta the new direction (angle in radians)
         */
        public void setTheta(double theta) {
            setComponents(magnitude, theta);
        }

        /**
         * Rotates the vector by a given angle (in radians).
         *
         * @param theta2 the angle to add to the current direction
         */
        public void rotateVector(double theta2) {
            setTheta(theta + theta2);
        }

        /**
         * Sets the vector's Cartesian components (x, y).
         * Updates the polar representation accordingly.
         *
         * @param xComponent the x component to set
         * @param yComponent the y component to set
         */
        public void setOrthogonalComponents(double xComponent, double yComponent) {
            double[] polarComponents;
            this.xComponent = xComponent;
            this.yComponent = yComponent;
            polarComponents = Pose.cartesianToPolar(xComponent, yComponent);
            magnitude = polarComponents[0];
            theta = polarComponents[1];
        }

        /**
         * Returns the magnitude (length) of the vector.
         *
         * @return the magnitude
         */
        public double getMagnitude() {
            return magnitude;
        }

        /**
         * Returns the direction (angle in radians) of the vector.
         *
         * @return the theta value
         */
        public double getTheta() {
            return theta;
        }

        /**
         * Returns the x component of the vector.
         *
         * @return the x component
         */
        public double getXComponent() {
            return xComponent;
        }

        /**
         * Returns the y component of the vector.
         *
         * @return the y component
         */
        public double getYComponent() {
            return yComponent;
        }

        /**
         * Returns a copy of this vector.
         *
         * @return a new Vector with the same magnitude and direction
         */
        public Vector copy() {
            return new Vector(this.magnitude, this.theta);
        }

        /**
         * Returns a string representation of the vector, including magnitude, theta, x, and y components.
         *
         * @return a string describing the vector
         */
        @NonNull
        @Override
        public String toString() {
            return "Vector{" +
                    "magnitude=" + magnitude +
                    ", theta=" + theta +
                    ", xComponent=" + xComponent +
                    ", yComponent=" + yComponent +
                    '}';
        }
    }