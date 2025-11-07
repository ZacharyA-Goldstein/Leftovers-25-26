package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//full
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

    private DcMotor shooterTurn; //EH2

    // --- Servos ---

    private Servo intakeLift; //0

    private Servo transfer; //1

    // --- Constants ---
    private static final double DEADBAND = 0.05;

    // --- Slow mode variables ---
    private boolean slowMode = false;
    private boolean rbWasPressed = false;

    // --- Toggle state variables ---
    private boolean shooterOn = false;
    private boolean shooterTriggerWasPressed = false;
    private boolean intakeOn = false;
    private boolean intakeTriggerWasPressed = false;
    private boolean intakeLiftUp = false;
    private boolean xWasPressed = false;
    private boolean transferExtended = false;
    private boolean bWasPressed = false;

    // --- Smoothing state ---
    private double smoothedDrive = 0.0;
    private double smoothedStrafe = 0.0;
    private double smoothedTurn = 0.0;

    // Max rate-of-change per loop (per cycle) for smoother motion
    // Increase to make it more responsive; decrease to make it smoother
    private static final double DRIVE_SLEW_PER_CYCLE = 0.08;
    private static final double STRAFE_SLEW_PER_CYCLE = 0.08;
    private static final double TURN_SLEW_PER_CYCLE = 0.10;

    @Override
    public void init() {
        // --- Map hardware ---
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        shoot      = hardwareMap.get(DcMotor.class, "shoot");
        shooterTurn = hardwareMap.get(DcMotor.class, "shooterTurn");
        intakeLift = hardwareMap.get(Servo.class, "intakeLift");
        transfer = hardwareMap.get(Servo.class, "transfer");

        // --- Motor directions ---
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterTurn.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeLift.setPosition(0);
        // Initialize transfer to zero/center position (0.5)
        transfer.setPosition(0.75);

        // --- Stop all motors on init ---
        stopAllMotors();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        moveDriveTrain();
        intakeSpin();
        transferControl();
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
        double driveInput  = applyDeadzone(rawDrive, DEADBAND);
        double strafeInput = applyDeadzone(rawStrafe, DEADBAND);
        double turnInput   = applyDeadzone(rawTurn, DEADBAND);

        // --- Exponential shaping for finer low-speed control ---
        // curve in [0..1], higher = softer around center
        final double DRIVE_EXPO = 0.6;
        final double STRAFE_EXPO = 0.6;
        final double TURN_EXPO = 0.5;

        double drive = applyExpo(driveInput, DRIVE_EXPO);
        double strafe = applyExpo(strafeInput, STRAFE_EXPO);
        double turn = applyExpo(turnInput, TURN_EXPO);

        // --- Speed scaling ---
        double DRIVE_SCALE = slowMode ? 0.35 : 0.9;
        double TURN_SCALE  = slowMode ? 0.35 : 0.6;

        drive  *= DRIVE_SCALE;
        strafe *= DRIVE_SCALE;
        turn   *= TURN_SCALE;

        // --- Slew rate limiting (limits sudden changes) ---
        smoothedDrive = slew(smoothedDrive, drive, DRIVE_SLEW_PER_CYCLE);
        smoothedStrafe = slew(smoothedStrafe, strafe, STRAFE_SLEW_PER_CYCLE);
        smoothedTurn = slew(smoothedTurn, turn, TURN_SLEW_PER_CYCLE);

        // --- Mecanum math ---
        double frontLeftPower  = smoothedDrive + smoothedStrafe + smoothedTurn;
        double frontRightPower = smoothedDrive - smoothedStrafe - smoothedTurn;
        double backLeftPower   = smoothedDrive - smoothedStrafe + smoothedTurn;
        double backRightPower  = smoothedDrive + smoothedStrafe - smoothedTurn;

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
        telemetry.addData("Inputs", "D:%.2f S:%.2f T:%.2f", driveInput, strafeInput, turnInput);
        telemetry.addData("Smoothed", "D:%.2f S:%.2f T:%.2f", smoothedDrive, smoothedStrafe, smoothedTurn);
    }

    // --- Intake Control ---
    private void intakeSpin() {
        // Toggle intake on/off with left trigger
        if (gamepad1.left_trigger > 0.1 && !intakeTriggerWasPressed) {
            intakeOn = !intakeOn;
        }
        intakeTriggerWasPressed = gamepad1.left_trigger > 0.1;
        
        if (intakeOn) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0);
        }
        
        // X button toggles intake lift up/down
        if (gamepad1.x && !xWasPressed) {
            intakeLiftUp = !intakeLiftUp;
        }
        xWasPressed = gamepad1.x;

        intakeLift.setPosition(intakeLiftUp ? 0.2 : 0.0);
    }

    // --- Transferring Control ---
    private void transferControl() {
        // B button toggles transfer servo extend/retract
        if (gamepad1.b && !bWasPressed) {
            transferExtended = !transferExtended;
        }
        bWasPressed = gamepad1.b;

        // Zero position is 0.5 (center), toggle moves to 1.0
        double position = transferExtended ? 0.9 : 0.75;
        transfer.setPosition(position);
        
        telemetry.addData("Transfer Extended", transferExtended);
        telemetry.addData("Transfer Position", position);
        telemetry.addData("B Button", gamepad1.b);
    }

    // --- Shooter Control ---
    private void shooterControl() {
        // Toggle shooter on/off with right trigger
        if (gamepad1.right_trigger > 0.1 && !shooterTriggerWasPressed) {
            shooterOn = !shooterOn;
        }
        shooterTriggerWasPressed = gamepad1.right_trigger > 0.1;
        
        if (shooterOn) {
            shoot.setPower(1.0); // Full power when on
        } else {
            shoot.setPower(0);
        }

        // Left/Right D-pad controls shooter turn (hold down)
        if (gamepad1.dpad_left) {
            shooterTurn.setPower(-0.5); // reverse
        } else if (gamepad1.dpad_right) {
            shooterTurn.setPower(0.5);  // forward
        } else {
            shooterTurn.setPower(0);
        }

        telemetry.addData("Shooter Power", shoot.getPower());
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
    }

    // --- Utility: Deadzone filter ---
    private double applyDeadzone(double value, double thresh) {
        return Math.abs(value) < thresh ? 0.0 : value;
    }

    // --- Utility: Exponential input shaping ---
    private double applyExpo(double value, double expo) {
        double sign = Math.signum(value);
        double abs = Math.abs(value);
        double shaped = (1 - expo) * abs + expo * abs * abs * abs; // blend linear and cubic
        return sign * shaped;
    }

    // --- Utility: Slew limiter ---
    private double slew(double current, double target, double maxDeltaPerCycle) {
        double delta = target - current;
        if (delta > maxDeltaPerCycle) {
            return current + maxDeltaPerCycle;
        } else if (delta < -maxDeltaPerCycle) {
            return current - maxDeltaPerCycle;
        }
        return target;
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
