package org.firstinspires.ftc.teamcode.LimeLight;




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

//h

public class dumbMapLime {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode

    public OpMode opMode;

    //Define all hardware

    public VoltageSensor batteryVoltageSensor;

    public WebcamName bonoboCam;
    public HuskyLens huskyLens; // i2c 1
    public RevColorSensorV3 ColorSensor;
    public Limelight3A limelight;
    public double multi;



    public DistanceSensor distanceSensor;
    public BHI260IMU gyro;//Can we do it?

    public boolean halfSpeedToggle = false, qtrSpeedToggle = false, drivingReverse = false;

    // Motor declarations
    public DcMotor spinner;
    public DcMotor shooter;

    /**
     * Initialize spinner and shooter motors
     */
    public void initMotors() {
        // Initialize motors
        spinner = opMode.hardwareMap.get(DcMotor.class, "spinner");
        shooter = opMode.hardwareMap.get(DcMotor.class, "shooter");
        
        // Set motor directions
        spinner.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        
        // Set motor modes
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set zero power behavior
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double drivePower;

    public Telemetry telemetry;

    public Servo servo;


    public dumbMapLime(OpMode opMode) {
        this.opMode = opMode;
    }

    public dumbMapLime(LinearOpMode opMode) {this.opMode = opMode;}

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
                
                // Give Limelight time to start
                // Use opMode.sleep() if it's a LinearOpMode, otherwise use Thread.sleep()
                if (opMode instanceof LinearOpMode) {
                    ((LinearOpMode) opMode).sleep(200);
                } else {
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }

                // Set up basic pipeline (0 is typically the default for AprilTag detection)
                limelight.pipelineSwitch(0);
                
                // Give Limelight time to switch pipelines and start processing
                // This is critical - pipeline switching takes time
                if (opMode instanceof LinearOpMode) {
                    ((LinearOpMode) opMode).sleep(500);
                } else {
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }

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
        // Initialize voltage sensor
        batteryVoltageSensor = this.opMode.hardwareMap.voltageSensor.iterator().next();
        this.opMode.telemetry.addData("Battery Voltage", "%.1fV", batteryVoltageSensor.getVoltage());

        // Initialize LimeLight
        initLimeLight();

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
}