package com.pedropathing.util;


import com.bylazar.ftcontrol.panels.json.Canvas;
import com.bylazar.ftcontrol.panels.json.Circle;
import com.bylazar.ftcontrol.panels.json.Line;
import com.bylazar.ftcontrol.panels.json.Look;
import com.bylazar.ftcontrol.panels.json.Point;
import com.bylazar.ftcontrol.panels.json.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @author Logan Nash
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/22/2024
 */
public class Drawing {
    public static final double ROBOT_RADIUS = 9;

    private static TelemetryPacket packet;

    private static Canvas canvas;

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), "#3F51B5");
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), "#3F51B5");
        }
        drawPoseHistory(follower.getDashboardPoseTracker(), "#4CAF50");
        drawRobot(follower.getPose(), "#4CAF50");

        sendPacket();
    }

    /**
     * This adds instructions to the current packet to draw a robot at a specified Pose with a specified
     * color. If no packet exists, then a new one is created.
     *
     * @param pose  the Pose to draw the robot at
     * @param color the color to draw the robot with
     */
    public static void drawRobot(Pose pose, String color) {
        if (canvas == null) canvas = new Canvas();
        canvas.add(
                new Circle(
                        new Point(pose.getX(), pose.getY()),
                        ROBOT_RADIUS
                ).withLook(
                        new Look(
                                "", color, 0.0, 1.0
                        )
                )
        );

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        canvas.add(
                new Line(
                        new Point(x1, y1),
                        new Point(x2, y2)
                ).withLook(
                        new Look(
                                "", color, 0.0, 1.0
                        )
                )
        );
    }

    /**
     * This adds instructions to the current packet to draw a Path with a specified color. If no
     * packet exists, then a new one is created.
     *
     * @param path  the Path to draw
     * @param color the color to draw the Path with
     */
    public static void drawPath(Path path, String color) {
        if (canvas == null) canvas = new Canvas();

        double[][] points = path.getDashboardDrawingPoints();

        canvas.add(new Line(
                        new Point(
                                points[0][0],
                                points[0][1]
                        ),
                        new Point(
                                points[1][0],
                                points[1][1]
                        )
                ).withLook(
                        new Look(
                                "", color, 0.0, 1.0
                        )
                )
        );
    }

    /**
     * This adds instructions to the current packet to draw all the Paths in a PathChain with a
     * specified color. If no packet exists, then a new one is created.
     *
     * @param pathChain the PathChain to draw
     * @param color     the color to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, String color) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), color);
        }
    }

    /**
     * This adds instructions to the current packet to draw the pose history of the robot. If no
     * packet exists, then a new one is created.
     *
     * @param poseTracker the DashboardPoseTracker to get the pose history from
     * @param color       the color to draw the pose history with
     */
    public static void drawPoseHistory(DashboardPoseTracker poseTracker, String color) {
        if (canvas == null) canvas = new Canvas();

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            canvas.add(
                    new Line(
                            new Point(
                                    poseTracker.getXPositionsArray()[i],
                                    poseTracker.getYPositionsArray()[i]
                            ),
                            new Point(
                                    poseTracker.getXPositionsArray()[i + 1],
                                    poseTracker.getYPositionsArray()[i + 1]
                            )
                    ).withLook(
                            new Look(
                                    "", color, 0.0, 1.0
                            )
                    )
            );
        }
    }

    /**
     * This tries to send the current packet to FTC Dashboard.
     *
     * @return returns if the operation was successful.
     */
    public static boolean sendPacket() {
        if (canvas != null) {
//            Panels.getTelemetry().debug(canvas);
//            TODO: draw
            canvas.clear();
            canvas = null;
            return true;
        }
        return false;
    }
}