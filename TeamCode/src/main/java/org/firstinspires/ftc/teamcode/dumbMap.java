package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

/**
 * Main robot hardware and control class for basic teleop operations
 */
public class dumbMap {
    // Servo positions
    public static final double SERVO_TRANSFER_POSITION = 0.08;  // Updated transfer position
    public static final double SERVO_INTAKE_POSITION = 0.22;    // Updated intake position
    public static final double HOOD_START_POSITION = 0.08;
    
    // Hardware objects
    public DcMotor leftFront, leftBack, rightFront, rightBack, shooter, spinner, intake;
    public Servo transfer, hood, flicker;
    public double drivePower = 0.5;
    
    // Sensors and other hardware
    public VoltageSensor batteryVoltageSensor;
    public WebcamName bonoboCam;
    public HuskyLens huskyLens;
    public RevColorSensorV3 ColorSensor;
    public Limelight3A limelight;
    public DistanceSensor distanceSensor;
    public BHI260IMU gyro;
    
    // Utility
    public ElapsedTime runtime = new ElapsedTime();
    public OpMode opMode;
    public double multi;
    public boolean halfSpeedToggle = false, qtrSpeedToggle = false, drivingReverse = false;

    public Telemetry telemetry;


    public dumbMap(OpMode opMode) {
        this.opMode = opMode;
    }

    public dumbMap(LinearOpMode opMode) {
        this.opMode = opMode;
    }


    /**
     * Initialize the LimeLight camera
     * Call this method in your OpMode's init() or init_loop()
     */
    public void initLimeLight() {
        try {
            // List all available Limelight devices for debugging
            this.opMode.telemetry.addData("Searching for LimeLight devices...", "");
            
            // Try to find the LimeLight by its IP address (USB connection)
            String[] allDeviceNames = this.opMode.hardwareMap.getAllNames(Limelight3A.class).toArray(new String[0]);
            
            if (allDeviceNames.length == 0) {
                this.opMode.telemetry.addData("LimeLight", "No LimeLight devices found!");
                return;
            }
            
            // Log all found devices
            this.opMode.telemetry.addData("Found LimeLight devices", String.join(", ", allDeviceNames));
            
            // Try to find a device that matches our expected name or use the first available
            String deviceName = null;
            for (String name : allDeviceNames) {
                if (name.equalsIgnoreCase("limelight") || name.contains("Ethernet")) {
                    deviceName = name;
                    break;
                }
            }
            
            // If no matching name found, use the first available device
            if (deviceName == null) {
                deviceName = allDeviceNames[0];
                this.opMode.telemetry.addData("Using first available LimeLight", deviceName);
            } else {
                this.opMode.telemetry.addData("Using LimeLight", deviceName);
            }
            
            // Initialize the LimeLight with the found device name
            limelight = this.opMode.hardwareMap.get(Limelight3A.class, deviceName);
            
            if (limelight != null) {
                // Start the camera stream
                limelight.start();
                
                // Set up basic pipeline (0 is typically the default for AprilTag detection)
                limelight.pipelineSwitch(0);
                
                // Note: Direct LED control is not available in the current API
                // LED state is typically controlled through the pipeline settings
                
                this.opMode.telemetry.addData("LimeLight", "Initialized successfully");
                
                // Get and display the current pipeline info
                try {
                    com.qualcomm.hardware.limelightvision.LLStatus status = limelight.getStatus();
                    if (status != null) {
                        this.opMode.telemetry.addData("LimeLight Pipeline", "%d (%s)", 
                            status.getPipelineIndex(), 
                            status.getPipelineType());
                        this.opMode.telemetry.addData("LimeLight Name", status.getName());
                        // Note: IP address is not directly available in the Limelight3A API
                    }
                } catch (Exception e) {
                    this.opMode.telemetry.addData("Status Error", "Could not get LimeLight status: " + e.getMessage());
                }
            } else {
                this.opMode.telemetry.addData("LimeLight", "Failed to initialize - null reference");
            }
        } catch (Exception e) {
            this.opMode.telemetry.addData("LimeLight Error", e.getMessage());
            e.printStackTrace();
        }
        this.opMode.telemetry.update();
    }
    
    /**
     * Get the LimeLight instance
     */
    public Limelight3A getLimeLight() {
        return limelight;
    }
    
    /**
     * Initialize all hardware components
     */
    public void init2() {
        try {
            // Initialize drive motors
            leftFront = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
            rightFront = opMode.hardwareMap.get(DcMotor.class, "frontRight");
            leftBack = opMode.hardwareMap.get(DcMotor.class, "backLeft");
            rightBack = opMode.hardwareMap.get(DcMotor.class, "backRight");

            // Set motor directions (adjust if needed)
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

            // Set motor modes for velocity control
            setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            

            // Reset encoders and set to run using encoder
            for (DcMotor motor : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            
            // Initialize shooter motor (fall back to old name)
            shooter = tryGetMotor("shooter");
            if (shooter == null) {
                shooter = tryGetMotor("outtake");
            }
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Initialize spinner motor
            spinner = opMode.hardwareMap.get(DcMotor.class, "spinner");
            spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Initialize intake motor
            intake = opMode.hardwareMap.get(DcMotor.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Initialize servo
            transfer = tryGetServo("transfer");
            if (transfer == null) {
                transfer = tryGetServo("flicker");
            }
            if (transfer != null) {
                transfer.setPosition(SERVO_TRANSFER_POSITION);
            }
            hood = tryGetServo("hood");
            if (hood == null) {
                hood = tryGetServo("servo");
            }
            flicker = tryGetServo("flicker");
            if (flicker == null) {
                flicker = transfer;
            }
            if (hood != null) {
                hood.setPosition(HOOD_START_POSITION);
            }
            
            // Initialize voltage sensor
            batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
            
            opMode.telemetry.addData("Status", "Hardware initialized");
            opMode.telemetry.addData("Battery", "%.1fV", batteryVoltageSensor.getVoltage());
            opMode.telemetry.update();
            
        } catch (Exception e) {
            opMode.telemetry.addData("Error", "Failed to initialize hardware: " + e.getMessage());
            opMode.telemetry.update();
        }
    }
    
    /**
     * Set all drive motors to the same run mode
     */
    public void setMotorModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }
    
    /**
     * Set all drive motors to the same zero power behavior
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }
    
    /**
     * Stop all drive motors
     */
    public void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private DcMotor tryGetMotor(String name) {
        try {
            return opMode.hardwareMap.get(DcMotor.class, name);
        } catch (Exception ignore) {
            return null;
        }
    }

    private Servo tryGetServo(String name) {
        try {
            return opMode.hardwareMap.get(Servo.class, name);
        } catch (Exception ignore) {
            return null;
        }
    }
    
    /**
     * Initialize all hardware components
     */
    public void init() {
        // Initialize voltage sensor
        batteryVoltageSensor = this.opMode.hardwareMap.voltageSensor.iterator().next();
        this.opMode.telemetry.addData("Battery Voltage", "%.1fV", batteryVoltageSensor.getVoltage());
        
        // Initialize LimeLight
        //initLimeLight();
        
        // Initialize other sensors (uncomment and modify as needed)
        /*
        try {
            huskyLens = this.opMode.hardwareMap.get(HuskyLens.class, "Huskylens");
            distanceSensor = this.opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");
            ColorSensor = this.opMode.hardwareMap.get(RevColorSensorV3.class, "colorSensor");
            gyro = this.opMode.hardwareMap.get(BHI260IMU.class, "imu");
        } catch (Exception e) {
            this.opMode.telemetry.addData("Sensor Error", e.getMessage());
        }
        */
        //servo = this.opMode.hardwareMap.get(Servo.class, "servo" );
        this.opMode.telemetry.update();
    }



    public double averageLastContents(ArrayList<Double> arr, int LOOKBACK){
        int len = arr.size();
        int count = Math.min(len, LOOKBACK);
        double sum = 0;
        for(int i = len - count; i < len; i++){
            sum += arr.get(i);
        }
        return sum/count;
    }
    
    /**
     * Calculate shooter power based on distance to target
     * @param distanceIn Distance to target in inches
     * @return Shooter motor power (0.0 to 1.0)
     */
    public double calculateShooterPower(double distanceIn) {
        // For now, return constant power
        // TODO: Add distance-based formula if you have one
        // Example: return Math.min(1.0, Math.max(0.4, basePower + (distanceIn * scaleFactor)));
        return 1.0;
    }
}