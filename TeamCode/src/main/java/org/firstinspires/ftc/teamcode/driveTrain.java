package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class driveTrain extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shoot;







    public void moveDriveTrain(){
        double vertical   = -gamepad1.left_stick_y;  // forward/back
        double horizontal =  gamepad1.left_stick_x;  // strafe
        double pivot      =  gamepad1.right_stick_x; // turn


        double frontLeftPower  = vertical + horizontal + pivot;
        double frontRightPower = vertical - horizontal - pivot;
        double backLeftPower   = vertical - horizontal + pivot;
        double backRightPower  = vertical + horizontal - pivot;

        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower),  Math.abs(backRightPower))));

// Set motor powers
        frontLeft.setPower(frontLeftPower  / max);
        frontRight.setPower(frontRightPower / max);
        backLeft.setPower(backLeftPower   / max);
        backRight.setPower(backRightPower  / max);


    }

    public void shootBall(){
        if(gamepad1.a){
            shoot.setPower(1);
        }else{
            shoot.setPower(0);
        }


    }





    @Override
    public void init(){
        frontLeft=hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight=hardwareMap.get(DcMotor.class,"frontRight");
        backLeft=hardwareMap.get(DcMotor.class,"backLeft");
        backRight=hardwareMap.get(DcMotor.class,"backRight");
        shoot=hardwareMap.get(DcMotor.class, "shoot");


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);




    }


    @Override
    public void loop(){
        moveDriveTrain();
        shootBall();


    }

}

