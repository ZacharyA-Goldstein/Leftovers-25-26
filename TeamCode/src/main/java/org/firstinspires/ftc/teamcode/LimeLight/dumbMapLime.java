package org.firstinspires.ftc.teamcode.LimeLight;


import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

//h
@Config
public class dumbMapLime {

    //Hardwaremaps can be used to initialize hardware and define code functions so that they can be used in \n
    // teleop(driver control) and autonomous classes. Using functions cleans up opmodes at the cost of slight
    //increase in computational load.


    //Define runtime, clock to be used in other functions
    public ElapsedTime runtime = new ElapsedTime();

    //Define the opMode
    public OpMode opMode;

    //Define all hardware

    //Define the code objects/interface that control the physical parts of the robot
    public VoltageSensor batteryVoltageSensor;

    //Regular Motors
    public DcMotor leftFront, rightFront, leftBack, rightBack, slide, slideangle;

    //Regular Servos

    //Sensors and Cameras (not all are used)
    public WebcamName bonoboCam;
    public HuskyLens huskyLens; // i2c 1
    public RevColorSensorV3 ColorSensor;
    public Limelight3A limelight;
    public DistanceSensor distanceSensor;

    //Telemetry allows the robot to print outputs from the code and sensors for debugging
    public Telemetry telemetry;

    //Defining Variables For functions in this opMode
    public double multi;
    public boolean halfSpeedToggle = false, qtrSpeedToggle = false, drivingReverse = false;

    public double drivePower;


    public boolean clawOpen = false, clawRot = false, flip = false;
    public boolean aLast1 = false, aLast2 = false, lbumpLast = false, rbumpLast = false, xLast = false, yLast = false, bLast = false, rstickpressLast = false, bonk = false, clawOpenH;

    public int currentColor = 1;
    public boolean aLast3=false;
    double slidePower;
    public int slideanglePos = -300, slidePos = 0;
    public double rstickxLast = 0, rstickyLast = 0, lTpos = 0.46;




    public dumbMapLime(OpMode opMode) {
        this.opMode = opMode;
    }

    public dumbMapLime(LinearOpMode opMode) {this.opMode = opMode;}


    //Init functions call the hardware from the configuration on the
    //robot controller and usually the basic behaviors like motor direction or braking
    //Are set in the same location.
    public void init2() {

        leftFront = this.opMode.hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = this.opMode.hardwareMap.dcMotor.get("rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack = this.opMode.hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack = this.opMode.hardwareMap.dcMotor.get("rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



//        huskyLens = this.opMode.hardwareMap.get(HuskyLens.class, "Huskylens");
//        distanceSensor = this.opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");
//        ColorSensor = this.opMode.hardwareMap.get(RevColorSensorV3.class, "colorSensor");

//        limelight = this.opMode.hardwareMap.get(Limelight3A.class, "limelight");



        VoltageSensor batteryVoltageSensor = this.opMode.hardwareMap.voltageSensor.iterator().next();

        //telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        //telemetry.update();
    }

    public void slidereset2(){

    }


    //Simple function that averages the last 'n' numbers inside an ArrayList
    public double averageLastContents(ArrayList<Double> arr, int LOOKBACK){
        int len = arr.size();
        int count = Math.min(len, LOOKBACK);
        double sum = 0;
        for(int i = len - count; i < len; i++){
            sum += arr.get(i);
        }
        return sum/count;
    }
}