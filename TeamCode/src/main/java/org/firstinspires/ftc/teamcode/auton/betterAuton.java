package org.firstinspires.ftc.teamcode.auton;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;
@Autonomous
public class betterAuton extends OpMode {
    private Follower follower;
    private Pose Pose;
    private Timer pathTimer, OpmodeTimer;

    private final Pose startPose=new Pose(121.71651495448634,120.59297789336802,Math.toRadians(40.0));
    private final Pose shootPose=new Pose(86.51235370611184, 85.01430429128737,Math.toRadians(40.0));
    private final Pose startIntake1=new Pose(102.61638491547464,83.89076723016905,Math.toRadians(0));
    private final Pose doneIntake1=new Pose(127.70871261378414,83.70351105331599, Math.toRadians(0));
    private final Pose shoot2= new Pose(86.51235370611184, 85.01430429128737,Math.toRadians(40.0));
    private final Pose offLine=new Pose(86.7, 117.2, Math.toRadians(40));


    public enum PathState{
        startPos_ShootPos,
        shoot1,
        ShootPos_StartIntake1Pos,
        intakeOn,

        StartIntake1Pos_FinishIntake1Pos,
        intakeOff,
        FinishIntake1Pos_ShootPos2,
        shoot2,
        ShootPos2_OffLine

    }
    PathState pathState;

    private PathChain starttoShoot, shoot1toIntake1, Intake1, intake1toShoot2, shoot2tooffline;

    public void buildPaths(){

        starttoShoot=follower.pathBuilder()
                .addPath(new BezierLine( startPose,  shootPose))
                .setLinearHeadingInterpolation( startPose.getHeading(),  shootPose.getHeading())
                .build();

        shoot1toIntake1=follower.pathBuilder()
                .addPath(new BezierLine( shootPose,  startIntake1))
                .setLinearHeadingInterpolation( shootPose.getHeading(),  startIntake1.getHeading())
                .build();

        Intake1=follower.pathBuilder()
                .addPath(new BezierLine( startIntake1,  doneIntake1))
                .setLinearHeadingInterpolation(startIntake1.getHeading(), doneIntake1.getHeading())
                .build();
        intake1toShoot2=follower.pathBuilder()
                .addPath(new BezierLine( doneIntake1, shoot2))
                .setLinearHeadingInterpolation(doneIntake1.getHeading(), shoot2.getHeading())
                .build();
        shoot2tooffline=follower.pathBuilder()
                .addPath(new BezierLine(shoot2,offLine))
                .setLinearHeadingInterpolation(shoot2.getHeading(), offLine.getHeading())
                .build();


    }

    public void StatePathUpdate(){
        switch(pathState){
            case startPos_ShootPos:
                follower.followPath(starttoShoot, true);
                setPathState(PathState.shoot1);
                break;
            case shoot1:
                if( !follower.isBusy()){
                    shootBalls();
                    setPathState(pathState.ShootPos_StartIntake1Pos);
                }
                break;
            case ShootPos_StartIntake1Pos:
                follower.followPath(shoot1toIntake1, true);
                setPathState(pathState.intakeOn);
                break;
            case intakeOn:
                if(!follower.isBusy()){
                    intakeOn();
                    setPathState(PathState.StartIntake1Pos_FinishIntake1Pos);
                }
                break;
            case StartIntake1Pos_FinishIntake1Pos:
                follower.followPath(Intake1, true);
                setPathState(pathState.intakeOff);
                break;
            case intakeOff:
                if(!follower.isBusy()){
                    intakeOff();
                    setPathState(PathState.FinishIntake1Pos_ShootPos2);
                }
                break;
            case FinishIntake1Pos_ShootPos2:
                follower.followPath(intake1toShoot2, true);
                setPathState(PathState.shoot2);
                break;
            case shoot2:
                if(!follower.isBusy()){
                    shootBalls();
                    setPathState(PathState.ShootPos2_OffLine);
                }
                break;
            case ShootPos2_OffLine:
                follower.followPath(shoot2tooffline, true);
                break;



        }

    }



    public void setPathState(PathState newState){
        pathState=newState;
        pathTimer.resetTimer();


    }




    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        pathState= PathState.startPos_ShootPos;
        pathTimer=new Timer();
        OpmodeTimer=new Timer();

        buildPaths();
        follower.setPose(startPose);

    }


    public void start(){
        OpmodeTimer.resetTimer();
        setPathState(pathState.startPos_ShootPos);


    }
    @Override
    public void loop(){
        follower.update();
        StatePathUpdate();

    }



    public void shootBalls(){


    }

    public void intakeOn(){

    }
    public void intakeOff(){

    }


}
