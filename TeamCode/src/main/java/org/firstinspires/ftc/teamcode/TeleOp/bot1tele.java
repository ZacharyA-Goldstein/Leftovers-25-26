package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dumbMap;

@TeleOp(name = "Driver Control", group = "TeleOp")
public class bot1tele extends LinearOpMode {
    private dumbMap robot;
    private double drivePower = 1;
    private boolean outtakeOn = false;
    private boolean lastTriggerState = false;
    private boolean intakeOn = false;
    private boolean lastIntakeTriggerState = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private static final int SPINNER_INCREMENT = 100; // Adjust this value for larger/smaller movements

    @Override
    public void runOpMode() {
        // Initialize the robot
        robot = new dumbMap(this);
        robot.init2();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get gamepad inputs
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Apply drive power scaling
            forward *= drivePower;
            strafe *= drivePower;
            rotate *= drivePower;

            // Set motor powers
            robot.leftFront.setPower(forward + strafe + rotate);
            robot.rightFront.setPower(forward - strafe - rotate);
            robot.leftBack.setPower(forward - strafe + rotate);
            robot.rightBack.setPower(forward + strafe - rotate);

            // Toggle drive power with right bumper
            if (gamepad1.right_bumper) {
                drivePower = 0.5; // Slow mode
            } else {
                drivePower = 1;  // Normal mode
            }

            // Intake control with left trigger (controller 2) - toggle on/off
            boolean currentIntakeTrigger = gamepad2.left_trigger > 0.5;
            
            // Toggle intake on trigger press
            if (currentIntakeTrigger && !lastIntakeTriggerState) {
                intakeOn = !intakeOn; // Toggle state
            }
            lastIntakeTriggerState = currentIntakeTrigger;
            
            // Set intake power based on toggle state
            double intakePower = intakeOn ? 1.0 : 0.0;
            robot.intake.setPower(intakePower);
            
            // Outtake control with right trigger (controller 2) - toggle on/off
            boolean currentOuttakeTrigger = gamepad2.right_trigger > 0.5;
            
            // Toggle outtake on trigger press
            if (currentOuttakeTrigger && !lastTriggerState) {
                outtakeOn = !outtakeOn; // Toggle state
            }
            lastTriggerState = currentOuttakeTrigger;
            
            // Emergency stop with B button (stops both intake and outtake)
            if (gamepad2.b) {
                outtakeOn = false;
                intakeOn = false;
            }
            
            // Set outtake power based on toggle state
            double outtakePower = outtakeOn ? 1.0 : 0.0;
            robot.shooter.setPower(outtakePower);

            // Update telemetry
            telemetry.addData("Drive Power", "%.0f%%", drivePower * 100);
            telemetry.addData("Left Front Power", "%.2f", robot.leftFront.getPower());
            telemetry.addData("Right Front Power", "%.2f", robot.rightFront.getPower());
            telemetry.addData("Left Back Power", "%.2f", robot.leftBack.getPower());
            telemetry.addData("Right Back Power", "%.2f", robot.rightBack.getPower());
            telemetry.addData("Outtake", outtakeOn ? "ON" : "OFF");
            if (intakeOn) {
                telemetry.addData("Intake Status", "RUNNING - Press B to stop");
            }
            if (outtakeOn) {
                telemetry.addData("Outtake Status", "RUNNING - Press B to stop");
            }
            
            // Spinner control with D-pad (controller 2)
            boolean currentDpadLeft = gamepad2.dpad_left;
            boolean currentDpadRight = gamepad2.dpad_right;
            
            // Check for D-pad left press (edge detection)
            if (currentDpadLeft && !lastDpadLeft) {
                int newTarget = robot.spinner.getCurrentPosition() - SPINNER_INCREMENT;
                robot.spinner.setTargetPosition(newTarget);
                robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.spinner.setPower(0.5);
            }
            
            // Check for D-pad right press (edge detection)
            if (currentDpadRight && !lastDpadRight) {
                int newTarget = robot.spinner.getCurrentPosition() + SPINNER_INCREMENT;
                robot.spinner.setTargetPosition(newTarget);
                robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.spinner.setPower(0.5);
            }
            
            // Update last states for edge detection
            lastDpadLeft = currentDpadLeft;
            lastDpadRight = currentDpadRight;
            
            // Spinner telemetry
            telemetry.addData("Spinner Position", robot.spinner.getCurrentPosition());
            telemetry.addData("Spinner Target", robot.spinner.getTargetPosition());
            telemetry.addData("Spinner Busy", robot.spinner.isBusy() ? "YES" : "NO");
            
            // Servo control with D-pad up/down (controller 2)
            boolean currentDpadUp = gamepad2.dpad_up;
            boolean currentDpadDown = gamepad2.dpad_down;
            
            // Set servo to intake position (0.15) on D-pad down press
            if (currentDpadUp && !lastDpadUp) {
                if (robot.transfer != null) {
                    robot.transfer.setPosition(dumbMap.SERVO_INTAKE_POSITION);
                }
                telemetry.addData("Servo Position", "INTAKE (%.2f)", dumbMap.SERVO_INTAKE_POSITION);
            }
            
            // Set servo to transfer position (0.0) on D-pad up press
            if (currentDpadDown && !lastDpadDown) {
                if (robot.transfer != null) {
                    robot.transfer.setPosition(dumbMap.SERVO_TRANSFER_POSITION);
                }
                telemetry.addData("Servo Position", "TRANSFER (%.2f)", dumbMap.SERVO_TRANSFER_POSITION);
            }
            
            // Update last states for edge detection
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            
            // Show current servo position if not already set
            if (!currentDpadUp && !currentDpadDown) {
                double currentPos = robot.transfer != null ? robot.transfer.getPosition() : 0.0;
                String posName = Math.abs(currentPos - dumbMap.SERVO_INTAKE_POSITION) < 0.01 ? "INTAKE" : "TRANSFER";
                telemetry.addData("Servo Position", "%s (%.2f)", posName, currentPos);
            }
            telemetry.update();
        }

        // Stop all motors when opmode is stopped
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }
}
