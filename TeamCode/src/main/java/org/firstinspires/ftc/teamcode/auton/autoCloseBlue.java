package org.firstinspires.ftc.teamcode.auton;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;
@Autonomous
public class autoCloseBlue extends OpMode {
    private Follower follower;
    private Pose Pose;
    private Timer pathTimer, OpmodeTimer;

    private final Pose startPose=new Pose(20.9,122.1,Math.toRadians(134));
    private final Pose shootPose=new Pose(51.5, 91.8,Math.toRadians(134));
    private final Pose startIntake1=new Pose(43.7,86.2,Math.toRadians(180));
    private final Pose doneIntake1=new Pose(10.1,83.9, Math.toRadians(180));
    private final Pose shoot2= new Pose(51.5, 91.8,Math.toRadians(134));
    private final Pose offLine=new Pose(67, 110.3, Math.toRadians(134));


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
    org.firstinspires.ftc.teamcode.auton.autoCloseRed.PathState pathState;

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
                setPathState(org.firstinspires.ftc.teamcode.auton.autoCloseRed.PathState.shoot1);
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
                    setPathState(org.firstinspires.ftc.teamcode.auton.autoCloseRed.PathState.StartIntake1Pos_FinishIntake1Pos);
                }
                break;
            case StartIntake1Pos_FinishIntake1Pos:
                follower.followPath(Intake1, true);
                setPathState(pathState.intakeOff);
                break;
            case intakeOff:
                if(!follower.isBusy()){
                    intakeOff();
                    setPathState(org.firstinspires.ftc.teamcode.auton.autoCloseRed.PathState.FinishIntake1Pos_ShootPos2);
                }
                break;
            case FinishIntake1Pos_ShootPos2:
                follower.followPath(intake1toShoot2, true);
                setPathState(org.firstinspires.ftc.teamcode.auton.autoCloseRed.PathState.shoot2);
                break;
            case shoot2:
                if(!follower.isBusy()){
                    shootBalls();
                    setPathState(org.firstinspires.ftc.teamcode.auton.autoCloseRed.PathState.ShootPos2_OffLine);
                }
                break;
            case ShootPos2_OffLine:
                follower.followPath(shoot2tooffline, true);
                break;



        }

    }



    public void setPathState(org.firstinspires.ftc.teamcode.auton.autoCloseRed.PathState newState){
        pathState=newState;
        pathTimer.resetTimer();


    }




    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        pathState= org.firstinspires.ftc.teamcode.auton.autoCloseRed.PathState.startPos_ShootPos;
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
