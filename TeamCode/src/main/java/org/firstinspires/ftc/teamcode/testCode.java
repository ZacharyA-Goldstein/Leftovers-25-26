package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class testCode extends LinearOpMode {
    Servo servo;
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException{
        servo=hardwareMap.get(Servo.class, "servo");
        motor=hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo.setPosition(0.0);
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.b) {
                servo.setPosition(0.2);
                telemetry.addData("Servo Position: ", servo.getPosition());
                telemetry.update();
            }
            if (gamepad1.a) {
                servo.setPosition(0.8);
                telemetry.addData("Servo Position: ", servo.getPosition());
                telemetry.update();
            }
            if (gamepad1.x) {
                motor.setPower(0);
                telemetry.addData("Motor Power: ", motor.getPower());
                telemetry.update();
            }
            if (gamepad1.y) {
                motor.setPower(1);
                telemetry.addData("Motor Power: ", motor.getPower());
                telemetry.update();
            }
        }

    }

}





