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
    
    /**
     * Shooter formula coefficients from 16 data points
     * Formula: distance = A * hoodPos + B * RPM + C
     * Derived using multiple linear regression on all 16 data points
     * R² = 0.972053
     */
    private static final double FORMULA_A = 164.882397;  // Hood coefficient
    private static final double FORMULA_B = -0.120907;   // RPM coefficient
    private static final double FORMULA_C = -319.979673;    // Intercept
    
    /**
     * All 16 data points from testing
     * Format: [distance, hoodPos, RPM]
     */
    private static final double[][] ALL_DATA_POINTS = {
        {71.5, 0.6810, -2250},
        {57.0, 0.6870, -2200},
        {63.0, 0.6920, -2200},
        {68.0, 0.6910, -2350},
        {77.5, 0.6930, -2400},
        {83.0, 0.6940, -2400},
        {90.0, 0.6950, -2450},
        {95.0, 0.6960, -2450},
        {101.0, 0.7000, -2500},
        {108.0, 0.7040, -2600},
        {114.5, 0.7090, -2600},
        {118.7, 0.6990, -2650},
        {126.0, 0.7030, -2700},
        {131.0, 0.7050, -2750},
        {135.0, 0.7070, -2800},
        {142.5, 0.7090, -2900},
    };
    
    // Hood and RPM limits (updated to match TeleOpBlue)
    private static final double HOOD_MIN = 0.690; // Updated to match TeleOp limits
    private static final double HOOD_MAX = 0.717;
    private static final double RPM_MIN = -6000;
    private static final double RPM_MAX = -2000;
    
    /**
     * Calculate optimal hood position and RPM using optimization
     * Finds the combination that minimizes error from the 31-point formula
     * Uses all 31 data points to determine RPM search range dynamically
     * @param distanceIn Distance to target in inches
     * @return Array with [hoodPosition, rpm]
     */
    public double[] calculateShooterSettings(double distanceIn) {
        // Clamp distance to reasonable range
        double minDist = 30.0;
        double maxDist = 160.0;
        distanceIn = Math.max(minDist, Math.min(maxDist, distanceIn));
        
        // Find RPM range from data points (dynamic based on actual data)
        double minRPMInData = Double.MAX_VALUE;
        double maxRPMInData = Double.MIN_VALUE;
        for (double[] point : ALL_DATA_POINTS) {
            double rpm = point[2];
            if (rpm < minRPMInData) minRPMInData = rpm;
            if (rpm > maxRPMInData) maxRPMInData = rpm;
        }
        
        // If no data points found, use default range
        if (minRPMInData == Double.MAX_VALUE) {
            minRPMInData = -2900;
            maxRPMInData = -2200;
        }
        
        // Expand range slightly beyond data for interpolation/extrapolation
        double rpmRange = maxRPMInData - minRPMInData;
        double searchMinRPM = Math.max(RPM_MIN, minRPMInData - rpmRange * 0.1); // 10% below min
        double searchMaxRPM = Math.min(RPM_MAX, maxRPMInData + rpmRange * 0.1); // 10% above max
        
        // Optimization: find RPM that gives valid hood position and minimizes error
        double bestRPM = (searchMinRPM + searchMaxRPM) / 2.0; // Start with middle value
        double bestHood = (HOOD_MIN + HOOD_MAX) / 2.0; // Start with middle of valid range
        double bestError = Double.MAX_VALUE;
        
        // Grid search over RPM range (coarse search with 50 RPM steps)
        // Try RPM values across the full range
        for (double testRPM = searchMinRPM; testRPM >= searchMaxRPM; testRPM -= 50) {
            // Calculate hood position from formula: hoodPos = (distance - B * RPM - C) / A
            double testHood = (distanceIn - FORMULA_B * testRPM - FORMULA_C) / FORMULA_A;
            
            // Check if within bounds
            if (testHood >= HOOD_MIN && testHood <= HOOD_MAX) {
                // Calculate predicted distance using the formula
                double predictedDist = FORMULA_A * testHood + FORMULA_B * testRPM + FORMULA_C;
                
                // Calculate error (how far off is the predicted distance from actual)
                double error = Math.abs(predictedDist - distanceIn);
                
                // Also check how close this combination is to actual data points
                double dataError = calculateDistanceToNearestDataPoint(distanceIn, testHood, testRPM);
                
                // Combined error (weighted: formula error + distance to data)
                // Heavily weight dataError to prioritize matching actual tested data points
                // At distances with many data points (like 144"), this ensures we match the actual working values
                double combinedError = error * 0.1 + dataError * 2.0; // Data proximity is 20x more important than formula error
                
                if (combinedError < bestError) {
                    bestError = combinedError;
                    bestRPM = testRPM;
                    bestHood = testHood;
                }
            }
        }
        
        // Fine-tune with smaller steps around the best value (±25 RPM in 10 RPM steps)
        for (double testRPM = bestRPM + 25; testRPM >= bestRPM - 25; testRPM -= 10) {
            double testHood = (distanceIn - FORMULA_B * testRPM - FORMULA_C) / FORMULA_A;
            
            if (testHood >= HOOD_MIN && testHood <= HOOD_MAX) {
                double predictedDist = FORMULA_A * testHood + FORMULA_B * testRPM + FORMULA_C;
                double error = Math.abs(predictedDist - distanceIn);
                double dataError = calculateDistanceToNearestDataPoint(distanceIn, testHood, testRPM);
                double combinedError = error + dataError * 0.1;
                
                if (combinedError < bestError) {
                    bestError = combinedError;
                    bestRPM = testRPM;
                    bestHood = testHood;
                }
            }
        }
        
        // Clamp final values
        bestHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, bestHood));
        bestRPM = Math.max(RPM_MIN, Math.min(RPM_MAX, bestRPM));
        
        // If no valid combination was found (bestError still MAX_VALUE), use direct formula calculation
        if (bestError == Double.MAX_VALUE) {
            // Try to find a valid RPM that gives a hood within bounds
            // Use middle RPM value and calculate hood directly
            double testRPM = (searchMinRPM + searchMaxRPM) / 2.0;
            double testHood = (distanceIn - FORMULA_B * testRPM - FORMULA_C) / FORMULA_A;
            
            // If still out of bounds, use the closest valid value
            if (testHood < HOOD_MIN) {
                bestHood = HOOD_MIN;
                // Solve for RPM: distance = A * HOOD_MIN + B * RPM + C
                bestRPM = (distanceIn - FORMULA_A * HOOD_MIN - FORMULA_C) / FORMULA_B;
            } else if (testHood > HOOD_MAX) {
                bestHood = HOOD_MAX;
                // Solve for RPM: distance = A * HOOD_MAX + B * RPM + C
                bestRPM = (distanceIn - FORMULA_A * HOOD_MAX - FORMULA_C) / FORMULA_B;
            } else {
                bestHood = testHood;
                bestRPM = testRPM;
            }
            
            // Clamp RPM
            bestRPM = Math.max(RPM_MIN, Math.min(RPM_MAX, bestRPM));
        }
        
        return new double[]{bestHood, bestRPM};
    }
    
    /**
     * Calculate distance to nearest data point in 3D space (distance, hood, RPM)
     * Used to prefer combinations close to actual tested data
     */
    private double calculateDistanceToNearestDataPoint(double distance, double hood, double rpm) {
        double minDist = Double.MAX_VALUE;
        
        for (double[] point : ALL_DATA_POINTS) {
            double pointDist = point[0];
            double pointHood = point[1];
            double pointRPM = point[2];
            
            // Normalize dimensions for comparison (distance is in inches, hood is 0-1, RPM is -6000 to -2000)
            double distDiff = (pointDist - distance) / 100.0; // Normalize by 100 inches
            double hoodDiff = (pointHood - hood) / 0.1; // Normalize by 0.1
            double rpmDiff = (pointRPM - rpm) / 1000.0; // Normalize by 1000 RPM
            
            // Euclidean distance in normalized 3D space
            double distance3D = Math.sqrt(distDiff * distDiff + hoodDiff * hoodDiff + rpmDiff * rpmDiff);
            
            if (distance3D < minDist) {
                minDist = distance3D;
            }
        }
        
        return minDist;
    }
    
    /**
     * Calculate target RPM based on distance (for backward compatibility)
     * Now uses optimization internally
     * @param distanceIn Distance to target in inches
     * @return Target RPM (negative value for reverse direction)
     */
    public double calculateShooterRPM(double distanceIn) {
        double[] settings = calculateShooterSettings(distanceIn);
        return settings[1]; // Return RPM
    }
    
    /**
     * Calculate hood position based on distance and RPM
     * Uses formula: distance = A * hoodPos + B * RPM + C
     * Solving for hoodPos: hoodPos = (distance - B * RPM - C) / A
     * @param distanceIn Distance to target in inches
     * @param rpm Target RPM (if null, will calculate from distance using optimization)
     * @return Hood servo position (0.0 to 1.0)
     */
    public double calculateHoodPosition(double distanceIn, Double rpm) {
        // If RPM not provided, use optimization to find best combination
        if (rpm == null) {
            double[] settings = calculateShooterSettings(distanceIn);
            return settings[0]; // Return hood position
        }
        
        // If RPM provided, calculate hood from formula
        double hoodPos = (distanceIn - FORMULA_B * rpm - FORMULA_C) / FORMULA_A;
        
        // Clamp to servo limits
        return Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPos));
    }
}