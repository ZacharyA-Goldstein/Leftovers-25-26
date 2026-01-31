package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.LimeLight.AprilTagDetector;
import org.firstinspires.ftc.teamcode.LimeLight.dumbMapLime;

import java.util.List;

@TeleOp(name = "CrunchBotBlue", group = "TeleOp")
public class CrunchBotBlue extends LinearOpMode {

    // Motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor intake;
    private DcMotorEx shooter;
    private DcMotor transfer;
    private DcMotorEx spinner;

    // Servo
    private CRServo toiletbottom;

    //LimeLight
    private dumbMapLime robot;
    private Limelight3A limelight;
    private AprilTagDetector aprilTagDetector;

    //constants

    //Limelight
    private static final double Camera_Height = 13.0;
    private static final double Camera_Angle = 17.0;
    private static final double Max_Distance = 160.0;
    private static final int Target_Tag_ID = 20;

    //turret
    private static final double Turret_Min_Power = 0.08;
    private static final double Turret_Max_Power = 0.15;
    private static final double Turret_Deadband = 3.5;
    private static final double Turret_Slow_Zone = 6.0;
    private static final double Turret_Slow_Zone_Power = 0.12;
    private static final double Turret_Very_Slow_Zone = 5.0;
    private static final double Turret_Very_Slow_Zone_Power = 0.06;
    private static final double Turret_Horizontal_Offset = 0.0; //left = -, right = +
    private static final double Turret_Direction_Flip = 1.0;

    // motor
    private static final int Shooter_Ticks_Per_Rev = 28;

    // drive
    private static final double Drive_Deadband = 0.3;

    // Intake
    private boolean intakeOn = false;
    private boolean lastIntakeTrigger = false;
    private boolean intakeTurnedOnByTransfer = false;
    private static final double INTAKE_POWER = 1.0;
    private static final double INTAKE_REVERSE_POWER = -1.0;

    // Transfer
    private static final double Transfer_Servo_Power = -1.0;

    // AutoAim
    private AprilTagDetector.AprilTagResult cachedTagResult = null;
    private int loopCounter = 0;
    private static final int LIMELIGHT_CALL_INTERVAL = 1;
    private int lostDetectionCount = 0;
    private static final int MAX_LOST_DETECTIONS = 10;

    //shooting
    private static final int Shooter_RPM = -4650;
    private boolean tagDetected = false;
    private double lastValidTx = 0.0;
    private boolean ShooterOn = false;
    private boolean manualTurretActive = false;
    private boolean lastRightTrigger = false;
    private boolean transferOn = false;
    private boolean lastAButton = false;

    @Override
    public void runOpMode() {
        initializeDriveMotors();

        initializeIntake();

        initializeTransferServos();

        initializeAutoAimShooter();

        initializeLimelight();

        telemetry.addData("Status", "Hardware initialized. Press START to begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            try {
                PlayerOne();
                PlayerTwo();
                Limelight();
                Telemetry();
                sleep(20);
            } catch (Exception e) {
                telemetry.addData("ERROR", e.getMessage());
                telemetry.update();
            }
        }
        
        StopAllMotors();
    }
    
    private void initializeDriveMotors() {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear = hardwareMap.get(DcMotor.class, "backRight");
        
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void initializeIntake() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void initializeTransferServos() {
        toiletbottom = hardwareMap.get(CRServo.class, "toiletBottom");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        if (transfer != null) {
            transfer.setDirection(DcMotor.Direction.FORWARD);
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    private void initializeAutoAimShooter() {
        robot = new dumbMapLime(this);
        robot.initMotors();
        
        if (robot.spinner != null) {
            robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.spinner.setDirection(DcMotor.Direction.FORWARD);
        }
        
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        if (shooter != null) {
            shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            shooter.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }
    
    private void initializeLimelight() {
        robot.initLimeLight();
        limelight = robot.getLimeLight();
        if (limelight != null) {
            limelight.pipelineSwitch(3);
            sleep(500);
            aprilTagDetector = new AprilTagDetector(limelight, Camera_Height, Camera_Angle, Max_Distance);
        }
    }
    
    private void PlayerOne() {
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;  // Try without inversion
        double rotate = -gamepad1.right_stick_x;
        
        if (Math.abs(forward) < Drive_Deadband) forward = 0;
        if (Math.abs(strafe) < Drive_Deadband) strafe = 0;
        if (Math.abs(rotate) < Drive_Deadband) rotate = 0;
        
        double driveScale = gamepad1.y ? 0.5 : 1.0;
        forward *= driveScale;
        strafe *= driveScale;
        rotate *= driveScale;
        
        // Mecanum drive math - swapped diagonal pairs (fl/br vs fr/bl) for strafe
        // This may be needed if mecanum wheels are mounted with opposite roller angles
        double fl = forward - strafe + rotate;  // Swapped diagonal
        double fr = forward - strafe - rotate;  // Swapped diagonal
        double bl = forward + strafe + rotate;  // Swapped diagonal
        double br = forward + strafe - rotate;  // Swapped diagonal
        
        if (leftFront != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftRear != null) leftRear.setPower(bl);
        if (rightRear != null) rightRear.setPower(br);
    }
    
    private void PlayerTwo() {
        // Intake toggle
        boolean intakeTrigger = gamepad2.left_trigger > 0.5;
        if (intakeTrigger && !lastIntakeTrigger) {
            intakeOn = !intakeOn;
            // If user manually toggles intake, clear the flag that tracks if transfer turned it on
            intakeTurnedOnByTransfer = false;
        }
        lastIntakeTrigger = intakeTrigger;
        
        if (intake != null) {
            if (gamepad2.left_bumper) {
                intake.setPower(INTAKE_REVERSE_POWER);
            } else {
                intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
            }
        }
        
        // Shooter toggle
        boolean rightTrigger = gamepad2.right_trigger > 0.5;
        if (rightTrigger && !lastRightTrigger) {
            ShooterOn = !ShooterOn;
        }
        lastRightTrigger = rightTrigger;
        
        if (shooter != null && ShooterOn) {
            double velocity = (Shooter_RPM / 60.0) * Shooter_Ticks_Per_Rev;
            shooter.setVelocity((int)Math.round(velocity));
        } else if (shooter != null) {
            shooter.setVelocity(0);
        }
        
        // Transfer toggle (independent of shooter)
        boolean aPressed = gamepad2.a;
        if (aPressed && !lastAButton) {
            transferOn = !transferOn;
            if (transferOn) {
                // When transfer turns on, also turn on intake
                intakeOn = true;
                intakeTurnedOnByTransfer = true;
            } else {
                // When transfer turns off, turn off intake if it was turned on by transfer
                if (intakeTurnedOnByTransfer) {
                    intakeOn = false;
                    intakeTurnedOnByTransfer = false;
                }
            }
        }
        lastAButton = aPressed;
        
        // Transfer control
        // Transfer servo spins when intake is on
        if (toiletbottom != null) {
            toiletbottom.setPower(intakeOn ? -Transfer_Servo_Power : 0.0);
        }
        // Transfer motor only spins when transfer is explicitly on (independent of shooter)
        if (transfer != null) {
            transfer.setPower(transferOn ? 1.0 : 0.0);
        }
    }
    
    private void Limelight() {
        if (limelight == null || aprilTagDetector == null) {
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
            return;
        }
        
        try {
            loopCounter++;
            boolean shouldCallLimelight = (loopCounter % LIMELIGHT_CALL_INTERVAL == 0);
            
            boolean freshTagDetected = false;
            double currentTx = 0.0;
            
            // Always get tx from Limelight for continuous turret alignment
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                double tx = result.getTx();
                double ta = result.getTa();
                
                // Get tx directly from target tag's fiducial result for accurate alignment
                double targetTagTx = tx; // Default to general tx
                boolean hasTag = false;
                List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid : fiducials) {
                        if (fid.getFiducialId() == Target_Tag_ID) {
                            hasTag = true;
                            // Get tx directly from the target tag for accurate alignment
                            targetTagTx = fid.getTargetXDegrees();
                            break;
                        }
                    }
                }
                
                // Always update currentTx from target tag's tx for continuous alignment
                currentTx = targetTagTx;
                
                // Only do full detection cycle (AprilTag detector) when throttled
                if (shouldCallLimelight) {
                    
                    // Use more lenient detection - only require tag to be present, not strict ta threshold
                    // This allows detection at different heights/distances
                    if (hasTag) {
                        try {
                            cachedTagResult = aprilTagDetector.getTagById(Target_Tag_ID);
                            // Accept detection if tag is found, even if distance calculation might be off
                            // The tx value from Limelight is still accurate for alignment
                            if (cachedTagResult != null && cachedTagResult.isValid) {
                                freshTagDetected = true;
                                lostDetectionCount = 0;
                                tagDetected = true;
                                lastValidTx = targetTagTx;
                            } else {
                                // If Limelight sees the tag but AprilTagDetector fails (e.g., height mismatch),
                                // still use it for alignment using direct tx from Limelight
                                // Get ty from fiducials for basic info
                                double ty = 0.0;
                                if (fiducials != null) {
                                    for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid : fiducials) {
                                        if (fid.getFiducialId() == Target_Tag_ID) {
                                            ty = fid.getTargetYDegrees();
                                            break;
                                        }
                                    }
                                }
                                freshTagDetected = true;
                                lostDetectionCount = 0;
                                tagDetected = true;
                                lastValidTx = targetTagTx;
                                // Create a minimal valid result for tracking using direct Limelight data
                                cachedTagResult = new AprilTagDetector.AprilTagResult();
                                cachedTagResult.tagId = Target_Tag_ID;
                                cachedTagResult.xDegrees = targetTagTx;
                                cachedTagResult.yDegrees = ty;
                                cachedTagResult.angle = targetTagTx;
                                cachedTagResult.distance = 60.0; // Default distance when height calculation fails
                                cachedTagResult.family = "36h11";
                                cachedTagResult.section = "BLUE";
                                cachedTagResult.isValid = true;
                            }
                        } catch (Exception e) {
                            // If AprilTagDetector throws an error but Limelight sees the tag, still use it
                            // Get ty from fiducials for basic info
                            double ty = 0.0;
                            if (fiducials != null) {
                                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fid : fiducials) {
                                    if (fid.getFiducialId() == Target_Tag_ID) {
                                        ty = fid.getTargetYDegrees();
                                        break;
                                    }
                                }
                            }
                            freshTagDetected = true;
                            lostDetectionCount = 0;
                            tagDetected = true;
                            lastValidTx = targetTagTx;
                            cachedTagResult = new AprilTagDetector.AprilTagResult();
                            cachedTagResult.tagId = Target_Tag_ID;
                            cachedTagResult.xDegrees = targetTagTx;
                            cachedTagResult.yDegrees = ty;
                            cachedTagResult.angle = targetTagTx;
                            cachedTagResult.distance = 60.0; // Default distance when height calculation fails
                            cachedTagResult.family = "36h11";
                            cachedTagResult.section = "BLUE";
                            cachedTagResult.isValid = true;
                        }
                    }
                    
                    // If no fresh tag detected this loop, increment lost count
                    if (!freshTagDetected) {
                        lostDetectionCount++;
                        if (lostDetectionCount > MAX_LOST_DETECTIONS) {
                            tagDetected = false;
                            cachedTagResult = null;
                        }
                    }
                }
            }
            
            // Update turret continuously when tag is detected
            if (tagDetected && cachedTagResult != null && cachedTagResult.isValid) {
                // Always use current tx from Limelight for continuous alignment
                // currentTx can be 0.0 (centered), which is valid
                updateTurret(currentTx);
                lastValidTx = currentTx;
            } else {
                // No tag detected - stop turret
                if (robot.spinner != null) {
                    robot.spinner.setPower(0.0);
                }
            }
        } catch (Exception e) {
            telemetry.addData("Limelight Error", e.getMessage());
            if (robot.spinner != null) {
                robot.spinner.setPower(0.0);
            }
        }
    }
    
    private void updateTurret(double tx) {
        if (robot.spinner == null) return;
        
        double adjustedTx = tx + Turret_Horizontal_Offset;
        double absTx = Math.abs(adjustedTx);
        
        if (absTx <= Turret_Deadband) {
            robot.spinner.setPower(0.0);
            return;
        }
        
        double sign = Math.signum(adjustedTx);
        double cmd;
        
        if (absTx <= Turret_Very_Slow_Zone) {
            cmd = Turret_Very_Slow_Zone_Power * sign;
        } else if (absTx <= Turret_Slow_Zone) {
            cmd = Turret_Slow_Zone_Power * sign;
        } else {
            cmd = adjustedTx * 0.02;
            if (Math.abs(cmd) > Turret_Max_Power) cmd = Turret_Max_Power * sign;
            if (Math.abs(cmd) < Turret_Min_Power) cmd = Turret_Min_Power * sign;
        }
        
        cmd *= Turret_Direction_Flip;
        
        robot.spinner.setPower(cmd);
    }
    
    private void Telemetry() {
        telemetry.addData("Tag", tagDetected ? "DETECTED" : "NOT DETECTED");
        telemetry.addData("Shooter", ShooterOn ? "ON" : "OFF");
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
        telemetry.addData("Transfer Servo", intakeOn ? "ON" : "OFF");
        telemetry.addData("Transfer Motor", transferOn ? "ON" : "OFF");
        if (toiletbottom != null) {
            telemetry.addData("Servo Power", "%.2f", intakeOn ? -Transfer_Servo_Power : 0.0);
        }
        telemetry.update();
    }
    
    private void StopAllMotors() {
        if (leftFront != null) leftFront.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
        if (leftRear != null) leftRear.setPower(0);
        if (rightRear != null) rightRear.setPower(0);
        if (intake != null) intake.setPower(0);
        if (toiletbottom != null) toiletbottom.setPower(0.0);
        if (transfer != null) transfer.setPower(0);
        if (robot != null && robot.spinner != null) robot.spinner.setPower(0.0);
        if (shooter != null) shooter.setVelocity(0);
    }








}
