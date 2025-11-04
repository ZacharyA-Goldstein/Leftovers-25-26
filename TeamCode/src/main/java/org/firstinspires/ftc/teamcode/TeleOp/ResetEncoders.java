package org.firstinspires.ftc.teamcode.TeleOp;

//luh Teleop
//default position pos = 0.9, vpos = 1

/*flat positions:
pos = 0 (70 deg), vpos = 0.205, slidePos = 0.299
pos = 0.05 (72 deg), vpos = 0.21, slidePos = 0.421
pos = 0.1 (75 deg), vpos = 0.214, slidePos = 0.573
*/

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.dumbMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config

public class ResetEncoders extends OpMode {

    dumbMap dumbBot = new dumbMap(this);
    DcMotor spinner;

    @Override
    public void init() {
        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setTargetPosition(0);




    }

    @Override
    public void loop() {
        if(gamepad2.a && gamepad2.b) {
            spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

}