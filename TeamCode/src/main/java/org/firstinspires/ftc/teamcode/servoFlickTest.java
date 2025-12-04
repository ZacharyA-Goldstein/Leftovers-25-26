/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoFlickTest", group = "TeleOp")
public class servoFlickTest extends OpMode {

    private Servo servo;
    private boolean buttonPressed = false;
    private boolean isFlicking = false;
    private long flickStartTime;

    // Adjust these for your servo setup
    private final double DOWN_POS = 0.0;   // resting position
    private final double UP_POS = 0.99;     // flick position
    private final long FLICK_DURATION = 175; // milliseconds total (0.2 sec flick)

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(DOWN_POS);
        telemetry.addLine("Press A to flick servo");
    }

    @Override
    public void loop() {
        telemetry.addData("Servo Pos", servo.getPosition());

        // Detect new press
        if (gamepad1.a && !buttonPressed && !isFlicking) {
            buttonPressed = true;
            isFlicking = true;
            flickStartTime = System.currentTimeMillis();
            servo.setPosition(UP_POS); // flick up fast
        } else if (!gamepad1.a) {
            buttonPressed = false; // reset latch
        }

        // Return to down position after flick duration
        if (isFlicking) {
            long elapsed = System.currentTimeMillis() - flickStartTime;
            if (elapsed >= FLICK_DURATION) {
                servo.setPosition(DOWN_POS);
                isFlicking = false;
            }
        }

        telemetry.update();
    }
}

*/