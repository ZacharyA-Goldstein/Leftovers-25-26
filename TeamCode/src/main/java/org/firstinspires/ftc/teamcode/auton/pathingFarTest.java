package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "pathingFarTest")
public class pathingFarTest extends OpMode {
    
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    
    // Waypoints from trajectory file (trajectory 3) - organized by sequence
    private final Pose startpoint = new Pose(56, 8, Math.toRadians(90));
    private final Pose BallCollect1Start = new Pose(41, 84.5, Math.toRadians(180));
    private final Pose BallCollect1End = new Pose(24, 84.5, Math.toRadians(180));
    private final Pose OpenGate = new Pose(16, 70, Math.toRadians(180)); // Updated from trajectory (3)
    private final Pose BallShoot2 = new Pose(70.33469387755102, 74.44897959183673, Math.toRadians(180)); // Updated from trajectory (3)
    private final Pose BallCollect2Start = new Pose(42.7, 60.5, Math.toRadians(180));
    private final Pose BallCollect2End = new Pose(24, 60, Math.toRadians(180));
    private final Pose BallShoot3 = new Pose(57.9, 24, Math.toRadians(180));
    private final Pose BallCollect3 = new Pose(24, 36, Math.toRadians(180)); // Single point in trajectory (3)
    private final Pose BallShoot4 = new Pose(57.9, 24, Math.toRadians(180));
    private final Pose Park = new Pose(57.9, 39, Math.toRadians(90));
    
    // Path chains - 10 paths connecting the waypoints (trajectory 3)
    private PathChain pathToBallCollect1Start, pathBallCollect1, pathToOpenGate, pathToBallShoot2,
                      pathToBallCollect2Start, pathBallCollect2, pathToBallShoot3,
                      pathToBallCollect3, pathToBallShoot4, pathToPark;
    
    public void buildPaths() {
        // Path 1: startpoint → BallCollect1Start
        pathToBallCollect1Start = follower.pathBuilder()
                .addPath(new BezierLine(startpoint, BallCollect1Start))
                .setLinearHeadingInterpolation(startpoint.getHeading(), BallCollect1Start.getHeading())
                .build();
        
        // Path 2: BallCollect1Start → BallCollect1End
        pathBallCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(BallCollect1Start, BallCollect1End))
                .setLinearHeadingInterpolation(BallCollect1Start.getHeading(), BallCollect1End.getHeading())
                .build();
        
        // Path 3: BallCollect1End → OpenGate (with control point at 27.82, 76.99)
        pathToOpenGate = follower.pathBuilder()
                .addPath(new BezierLine(BallCollect1End, OpenGate))
                .setLinearHeadingInterpolation(BallCollect1End.getHeading(), OpenGate.getHeading())
                .build();
        
        // Path 4: OpenGate → BallShoot2
        pathToBallShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(OpenGate, BallShoot2))
                .setLinearHeadingInterpolation(OpenGate.getHeading(), BallShoot2.getHeading())
                .build();
        
        // Path 5: BallShoot2 → BallCollect2Start
        pathToBallCollect2Start = follower.pathBuilder()
                .addPath(new BezierLine(BallShoot2, BallCollect2Start))
                .setLinearHeadingInterpolation(BallShoot2.getHeading(), BallCollect2Start.getHeading())
                .build();
        
        // Path 6: BallCollect2Start → BallCollect2End
        pathBallCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(BallCollect2Start, BallCollect2End))
                .setLinearHeadingInterpolation(BallCollect2Start.getHeading(), BallCollect2End.getHeading())
                .build();
        
        // Path 7: BallCollect2End → BallShoot3
        pathToBallShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(BallCollect2End, BallShoot3))
                .setLinearHeadingInterpolation(BallCollect2End.getHeading(), BallShoot3.getHeading())
                .build();
        
        // Path 8: BallShoot3 → BallCollect3 (with control point at 44.47, 36.24)
        pathToBallCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(BallShoot3, BallCollect3))
                .setLinearHeadingInterpolation(BallShoot3.getHeading(), BallCollect3.getHeading())
                .build();
        
        // Path 9: BallCollect3 → BallShoot4
        pathToBallShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(BallCollect3, BallShoot4))
                .setLinearHeadingInterpolation(BallCollect3.getHeading(), BallShoot4.getHeading())
                .build();
        
        // Path 10: BallShoot4 → Park
        pathToPark = follower.pathBuilder()
                .addPath(new BezierLine(BallShoot4, Park))
                .setLinearHeadingInterpolation(BallShoot4.getHeading(), Park.getHeading())
                .build();
    }
    
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // 1. startpoint → BallCollect1Start
                follower.followPath(pathToBallCollect1Start, true);
                setPathState(1);
                break;
                
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
                
            case 2:
                // Wait 3 seconds before starting next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // 2. BallCollect1Start → BallCollect1End
                    follower.followPath(pathBallCollect1, true);
                    setPathState(3);
                }
                break;
                
            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
                
            case 4:
                // Wait 3 seconds before starting next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // 3. BallCollect1End → OpenGate
                    follower.followPath(pathToOpenGate, true);
                    setPathState(5);
                }
                break;
                
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
                
            case 6:
                // Wait 3 seconds before starting next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // 4. OpenGate → BallShoot2
                    follower.followPath(pathToBallShoot2, true);
                    setPathState(7);
                }
                break;
                
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
                
            case 8:
                // Wait 3 seconds before starting next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // 5. BallShoot2 → BallCollect2Start
                    follower.followPath(pathToBallCollect2Start, true);
                    setPathState(9);
                }
                break;
                
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
                
            case 10:
                // Wait 3 seconds before starting next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // 6. BallCollect2Start → BallCollect2End
                    follower.followPath(pathBallCollect2, true);
                    setPathState(11);
                }
                break;
                
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
                
            case 12:
                // Wait 3 seconds before starting next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // 7. BallCollect2End → BallShoot3
                    follower.followPath(pathToBallShoot3, true);
                    setPathState(13);
                }
                break;
                
            case 13:
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
                
            case 14:
                // Wait 3 seconds before starting next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // 8. BallShoot3 → BallCollect3
                    follower.followPath(pathToBallCollect3, true);
                    setPathState(15);
                }
                break;
                
            case 15:
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
                
            case 16:
                // Wait 3 seconds before starting next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // 9. BallCollect3 → BallShoot4
                    follower.followPath(pathToBallShoot4, true);
                    setPathState(17);
                }
                break;
                
            case 17:
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;
                
            case 18:
                // Wait 3 seconds before starting next path
                if (pathTimer.getElapsedTime() > 3000) {
                    // 10. BallShoot4 → Park
                    follower.followPath(pathToPark, true);
                    setPathState(19);
                }
                break;
                
            case 19:
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;
                
            case 20:
                // All paths complete
                break;
        }
    }
    
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("busy? ", follower.isBusy());
        telemetry.update();
    }
    
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        
        // Create follower using Constants helper method
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startpoint);
        
        buildPaths();
    }
    
    @Override
    public void init_loop() {}
    
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    
    @Override
    public void stop() {}
}

