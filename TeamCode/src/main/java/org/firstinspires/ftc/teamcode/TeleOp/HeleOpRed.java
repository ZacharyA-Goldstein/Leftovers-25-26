package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HeleOp Red", group = "TeleOp")
public class HeleOpRed extends HeleOpBase {
    @Override
    protected int[] getTargetTagIds() {
        return new int[]{24};
    }

    @Override
    protected int getAprilTagPipelineIndex() {
        return 0;
    }
}

