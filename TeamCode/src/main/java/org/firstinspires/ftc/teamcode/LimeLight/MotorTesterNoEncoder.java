package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dumbMap;

/**
 * Simple motor toggle: press A to turn on/off the selected motor (no encoder).
 */
@TeleOp(name = "Motor Tester No Encoder", group = "Test")
public class MotorTesterNoEncoder extends LinearOpMode {
    private dumbMapLime robot;
    private DcMotor currentMotor;
    private boolean motorOn = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        robot = new dumbMapLime(this);
        robot.initMotors();

        //currentMotor = robot.intake;
        currentMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Motor Tester No Encoder");
        telemetry.addLine("Press A to toggle motor on/off");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean aNow = gamepad1.a;
            if (aNow && !lastA) {
                motorOn = !motorOn;
                currentMotor.setPower(motorOn ? 0.5 : 0.0);
            }
            lastA = aNow;

            telemetry.addData("Motor", "Spinner");
            telemetry.addData("State", motorOn ? "ON" : "OFF");
            telemetry.addData("Power", currentMotor.getPower());
            telemetry.update();
        }

        currentMotor.setPower(0.0);
    }
}

