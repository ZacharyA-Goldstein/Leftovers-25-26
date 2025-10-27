package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Full", group = "TeleOp")
public class Full extends OpMode {

    // --- Drive Motors ---
    private DcMotor frontLeft; //0
    private DcMotor frontRight; //1
    private DcMotor backLeft; //2
    private DcMotor backRight; //3

    // --- Intake & Shooter Motors ---
    private DcMotor intake; //EH0
    private DcMotor shoot; //EH1

    // --- Constants ---
    private static final double DEADBAND = 0.05;

    // --- Slow mode variables ---
    private boolean slowMode = false;
    private boolean rbWasPressed = false;

    @Override
    public void init() {
        // --- Map hardware ---
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        shoot      = hardwareMap.get(DcMotor.class, "shoot");

        // --- Motor directions ---
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- Stop all motors on init ---
        stopAllMotors();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        moveDriveTrain();
        intakeSpin();
        shooterControl();
        telemetry.update();
    }

    // --- DriveTrain Movement ---
    private void moveDriveTrain() {
        // --- Handle slow mode toggle ---
        if (gamepad1.right_bumper && !rbWasPressed) {
            slowMode = !slowMode;
        }
        rbWasPressed = gamepad1.right_bumper;

        // --- Controller inputs ---
        double rawDrive  = -gamepad1.left_stick_y;   // forward/back
        double rawTurn   =  gamepad1.right_stick_x;  // rotation
        double rawStrafe =  gamepad1.left_stick_x;   // strafe

        // --- Apply deadzones ---
        double drive  = applyDeadzone(rawDrive, DEADBAND);
        double strafe = applyDeadzone(rawStrafe, DEADBAND);
        double turn   = applyDeadzone(rawTurn, DEADBAND);

        // --- Speed scaling ---
        double DRIVE_SCALE = slowMode ? 0.4 : 1.0;
        double TURN_SCALE  = slowMode ? 0.4 : 0.7;

        drive  *= DRIVE_SCALE;
        strafe *= DRIVE_SCALE;
        turn   *= TURN_SCALE;

        // --- Mecanum math ---
        double frontLeftPower  = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower   = drive - strafe + turn;
        double backRightPower  = drive + strafe - turn;

        // --- Normalize ---
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        ));

        // --- Apply powers ---
        frontLeft.setPower(frontLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backLeft.setPower(backLeftPower / max);
        backRight.setPower(backRightPower / max);

        telemetry.addData("Mode", slowMode ? "SLOW MODE" : "NORMAL MODE");
        telemetry.addData("Drive Scale", "%.0f%%", DRIVE_SCALE * 100);
        telemetry.addData("Turn Scale", "%.0f%%", TURN_SCALE * 100);
    }

    // --- Intake Control ---
    private void intakeSpin() {
        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0);
        }
    }

    // --- Shooter Control ---
    private void shooterControl() {
        if (gamepad1.right_trigger > 0.1) {
            shoot.setPower(0.75); // Power while trigger held
        } else {
            shoot.setPower(0);
        }

        telemetry.addData("Shooter Power", shoot.getPower());
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
    }

    // --- Utility: Deadzone filter ---
    private double applyDeadzone(double value, double thresh) {
        return Math.abs(value) < thresh ? 0.0 : value;
    }

    // --- Utility: Stop all motors ---
    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intake.setPower(0);
        shoot.setPower(0);
    }

    @Override
    public void stop() {
        stopAllMotors();
    }
}
