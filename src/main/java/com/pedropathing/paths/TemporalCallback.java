package com.pedropathing.paths;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is the TemporalCallback class. This class handles callbacks that occur a given amount of time after a Path in a PathChain has begun running.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class TemporalCallback implements PathCallback {
    private ElapsedTime elapsedTime;
    private double startCondition;
    private Runnable runnable;
    private double startTime = 0;
    private int index;

    public TemporalCallback(int index, double startCondition, Runnable runnable) {
        this.index = index;
        this.startCondition = startCondition;
        this.runnable = runnable;
    }

    @Override
    public boolean run() {
        runnable.run();
        return true;
    }

    @Override
    public boolean isReady() {
        return System.currentTimeMillis() - startTime >= startCondition;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public int getPathIndex() {
        return index;
    }
}
