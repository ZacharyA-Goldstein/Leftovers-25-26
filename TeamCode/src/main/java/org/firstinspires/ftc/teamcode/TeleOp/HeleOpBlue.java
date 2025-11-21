package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HeleOp Blue", group = "TeleOp")
public class HeleOpBlue extends HeleOpBase {
    @Override
    protected int[] getTargetTagIds() {
        return new int[]{20};
    }
}

