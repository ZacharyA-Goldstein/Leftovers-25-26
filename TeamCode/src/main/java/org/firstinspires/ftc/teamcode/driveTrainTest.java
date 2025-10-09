package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "driveTrainTest", group = "TeleOp")
public class driveTrainTest extends OpMode {

    private DcMotor frontLeft; //1
    private DcMotor frontRight; //3
    private DcMotor backLeft; //0
    private DcMotor backRight; //2

    private DcMotor intake;

    // deadzone to ignore tiny stick noise
    private static final double DEADBAND = 0.05;

    @Override
    public void init() {
        // Map hardware
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intake     = hardwareMap.get(DcMotor.class,"intake");

        // Typical mecanum convention: left side reversed, right side forward.
        // If your wiring is different, swap REVERSE/FORWARD on the side that is physically reversed.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Stop all motors at start
        stopAllMotors();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        moveDriveTrain();
        workIntake();

        // Telemetry for debugging: show stick values and motor powers
        telemetry.update();
    }

    private void workIntake(){
        if (gamepad1.b){
            intake.setPower(1);
        }else{
            intake.setPower(0);
        }

    }

    private void moveDriveTrain() {
        // Inputs:
        double rawDrive  = -gamepad1.left_stick_y;   // forward/back
        double rawTurn   =  gamepad1.left_stick_x;   // rotate (left stick X)
        double rawStrafe =  gamepad1.right_stick_x;  // strafe (right stick X)

        // Apply deadzone
        double drive  = applyDeadzone(rawDrive, DEADBAND);
        double turn   = applyDeadzone(rawTurn, DEADBAND);
        double strafe = applyDeadzone(rawStrafe, DEADBAND);

        // Debug telemetry for inputs
        telemetry.addData("Inputs", "drive=%.2f turn=%.2f strafe=%.2f", drive, turn, strafe);

        // Combine motions (mecanum mixing)
        double frontLeftPower  = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower   = drive - strafe + turn;
        double backRightPower  = drive + strafe - turn;

        // Telemetry for raw (pre-normalize) powers
        telemetry.addData("RawPwr", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        // Normalize so no value exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        double fl = frontLeftPower / max;
        double fr = frontRightPower / max;
        double bl = backLeftPower / max;
        double br = backRightPower / max;

        // Telemetry for normalized powers
        telemetry.addData("NormPwr", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f", fl, fr, bl, br);

        // Apply powers
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private double applyDeadzone(double value, double thresh) {
        return Math.abs(value) < thresh ? 0.0 : value;
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void stop() {
        stopAllMotors();
    }
}
