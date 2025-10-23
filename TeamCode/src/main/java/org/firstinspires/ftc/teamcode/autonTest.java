package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="autonTest", group = "Test")
public class autonTest extends LinearOpMode{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    ElapsedTime timer= new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException{

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight=hardwareMap.get(DcMotor.class,"frontRight");
        backLeft=hardwareMap.get(DcMotor.class,"backLeft");
        backRight=hardwareMap.get(DcMotor.class,"backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()){
            return;
        }

        //PUT COMMANDS HERE







    }

    private void stopAllMotors(double time) {
        //make 0.5 sec
        timer.reset();
        while (timer.time() < time) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    private void moveForward(double time) {
        timer.reset();
        while (timer.time() < time) {
            frontLeft.setPower(1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(1);
        }
    }

    private void moveBackwards(double time) {
        timer.reset();
        while (timer.time() < time) {
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(-1);
        }
    }

    private void strafeLeft(double time) {
        timer.reset();
        while (timer.time() < time) {
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(-1);
        }
    }

    private void strafeRight(double time) {
        timer.reset();
        while (timer.time() < time) {
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(1);
        }
    }

    private void turnRight(double time) {
        timer.reset();
        while (timer.time() < time) {
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(1);
            backRight.setPower(-1);
        }
    }

    private void turnLeft(double time) {
        timer.reset();
        while (timer.time() < time) {
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backLeft.setPower(-1);
            backRight.setPower(1);
        }
    }






}
