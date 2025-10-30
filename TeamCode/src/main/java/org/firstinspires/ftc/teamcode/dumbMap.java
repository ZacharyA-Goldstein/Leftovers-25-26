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

//h

public class dumbMap {
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

    public double drivePower;

    public Telemetry telemetry;
    public boolean clawOpen = false, clawRot = false, flip = false;
    public boolean aLast1 = false, aLast2 = false, lbumpLast = false, rbumpLast = false, xLast = false, yLast = false, bLast = false, rstickpressLast = false, bonk = false, clawOpenH;

    public int currentColor = 1;
    public boolean aLast3=false;
    double slidePower;
    public int slideanglePos = -300, slidePos = 0;
    public double rstickxLast = 0, rstickyLast = 0, lTpos = 0.46;

    public Servo servo;


    public dumbMap(OpMode opMode) {
        this.opMode = opMode;
    }

    public dumbMap(LinearOpMode opMode) {this.opMode = opMode;}

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