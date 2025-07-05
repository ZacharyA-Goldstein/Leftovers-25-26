package com.pedropathing.ftc;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.json.Canvas;
import com.bylazar.ftcontrol.panels.json.CanvasRotation;
import com.bylazar.ftcontrol.panels.json.Circle;
import com.bylazar.ftcontrol.panels.json.Line;
import com.bylazar.ftcontrol.panels.json.Look;
import com.bylazar.ftcontrol.panels.json.Point;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
public class Drawing {
    public static final double ROBOT_RADIUS = 9;
    private static final Canvas canvas = new Canvas().withOffsets(-24 * 3, 24 * 3, CanvasRotation.DEG_270);

    private static final Look robotLook = new Look(
            "", "#3F51B5", 0.0, 1.0
    );
    private static final Look historyLook = new Look(
            "", "#4CAF50", 0.0, 1.0
    );

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     * @param look the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Look look) {
        canvas.add(
                new Circle(
                        new Point(pose.getX(), pose.getY()),
                        ROBOT_RADIUS
                ).withLook(look)
        );

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        canvas.add(
                new Line(
                        new Point(x1, y1),
                        new Point(x2, y2)
                ).withLook(look)
        );
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path the Path to draw
     * @param look the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Look look) {
        double[][] points = path.getPanelsDrawingPoints();

        canvas.add(new Line(
                        new Point(
                                points[0][0],
                                points[0][1]
                        ),
                        new Point(
                                points[1][0],
                                points[1][1]
                        )
                ).withLook(look)
        );
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param look      the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Look look) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), look);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param look        the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Look look) {
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
                    ).withLook(look)
            );
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     *
     * @return returns if the operation found data to send.
     */
    public static boolean sendPacket() {
        if (!canvas.isEmpty()) {
            Panels.getTelemetry().sendCanvas(canvas);
            canvas.clear();
            return true;
        }
        return false;
    }
}