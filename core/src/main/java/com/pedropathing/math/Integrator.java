package com.pedropathing.math;

import com.pedropathing.util.NanoTimer;

import java.util.concurrent.TimeUnit;

/**
 * This class handles approximations of integrations.
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class Integrator {
    public enum Differential {
        TIME,
        TVALUE,
        OTHER
    }
    private Differential differential;
    private final NanoTimer timer = new NanoTimer();
    private double sum = 0;
    private double prevDifferentialVal = 0;

    /**
     * Constructor for the Integrator class
     * @param differential the differential to use
     */
    public Integrator(Differential differential) {
        this.differential = differential;
    }

    /**
     * Constructor for the Integrator class
     */
    public Integrator() {
        this(Differential.TIME);
    }

    /**
     * Updates the integral by adding in new data
     * @param state the function's value for that particular loop
     */
    public void update(double state) {
        if (differential == Differential.TIME) {
            sum += state * timer.getElapsedTime(TimeUnit.SECONDS);
            timer.resetTimer();
        } else {
            sum += state * prevDifferentialVal;
        }
    }

    /**
     * Updates the integral by adding in new data
     * @param state the function's value for that particular loop
     * @param differentialVal the small change in the input value for that particular loop
     */
    public void update(double state, double differentialVal) {
        sum += state * differentialVal;
        prevDifferentialVal = differentialVal;
    }

    /**
     * Returns the integral at the given time
     * @return the integral
     */
    public double getIntegral() {
        return sum;
    }

    /**
     * Returns the differential to use
     * @return the differential
     */
    public Differential getDifferential() {
        return differential;
    }
}
