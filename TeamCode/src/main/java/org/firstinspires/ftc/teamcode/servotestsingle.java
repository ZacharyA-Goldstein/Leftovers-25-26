package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "test")
public class servotestsingle extends LinearOpMode {

    public static double pos = 0;

    public Servo servo;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = hardwareMap.get(Servo.class, "servo" );
        servo.setPosition(1.0);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                pos += 0.001;
                sleep(50);
            }
            if (gamepad1.b) {
                pos -= 0.001;
                sleep(50);
            }
            if (gamepad1.x) {
                pos += 0.01;
                sleep(50);
            }
            if (gamepad1.y) {
                pos -= 0.01;
                sleep(50);
            }

            pos = Math.max(0.0, Math.min(1.0, pos));
            servo.setPosition(1.0 - pos);



            telemetry.addData("servoPos: ", servo.getPosition());
            telemetry.update();

        }
    }
}


