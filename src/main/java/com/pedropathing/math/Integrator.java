package com.pedropathing.math;

import com.pedropathing.util.NanoTimer;

import java.util.concurrent.TimeUnit;

/**
 * This class handles approximations of integrations.
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class Integrator {
    private final NanoTimer timer = new NanoTimer();
    private double sum = 0;

    /**
     * Updates the integral by adding in new data
     * @param state the function's value for that particular loop
     */
    public void update(double state) {
        sum += state * timer.getElapsedTime(TimeUnit.SECONDS);
        timer.resetTimer();
    }

    /**
     * Returns the integral at the given time
     * @return the integral
     */
    public double getIntegral() {
        return sum;
    }
}
