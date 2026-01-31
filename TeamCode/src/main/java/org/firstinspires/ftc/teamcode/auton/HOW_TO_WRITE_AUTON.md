# How to Write PedroPathing Autonomous - Step by Step

This guide will teach you how to write autonomous code using PedroPathing. We'll build it step by step.

---

## Step 1: Understand What PedroPathing Does

PedroPathing is a library that makes your robot follow precise paths on the field. Instead of saying "move forward for 2 seconds," you say "move to position (45, 33)".

**Key difference:**
- **Time-based:** "Move forward for 2 seconds" → Inconsistent (battery affects speed)
- **Path-based:** "Move to position (45, 33)" → Consistent (always reaches the same spot)

**Think of it like GPS navigation:**
- You tell it where you want to go (waypoint)
- It figures out how to get there
- It follows the path automatically

---

## Step 2: Understand Field Coordinates

Before writing code, you need to understand field coordinates:

- **X-axis:** 0 to 144 inches (0 = back wall, 144 = front wall)
- **Y-axis:** 0 to 72 inches (0 = left wall, 72 = right wall)  
- **Heading:** 0° to 360° (0° = right, 90° = forward, 180° = left, 270° = back)

**In code, heading is in radians:**
- 0° = 0 radians
- 90° = π/2 radians = `Math.toRadians(90)`
- 180° = π radians = `Math.toRadians(180)`
- 270° = 3π/2 radians = `Math.toRadians(270)`

**Example positions:**
- Starting position (Blue): `(56.5, 8, 90°)` = X=56.5, Y=8, facing forward
- Center of field: `(72, 36, 180°)` = X=72, Y=36, facing left

---

## Step 3: Plan Your Routine

Before writing code, plan your autonomous:

**Example plan:**
1. Start at (56.5, 8) facing 90°
2. Move to shooting position (56.5, 15) facing 90°
3. Shoot 2 rings
4. Move to (45, 33) facing 180°
5. Move to (20, 33) facing 180°
6. Park at (15, 10) facing 180°

**Break it into waypoints:**
- Waypoint 0: Start (56.5, 8, 90°)
- Waypoint 1: Shoot (56.5, 15, 90°)
- Waypoint 2: (45, 33, 180°)
- Waypoint 3: (20, 33, 180°)
- Waypoint 4: Park (15, 10, 180°)

**Each path connects two waypoints:**
- Path 0: Waypoint 0 → Waypoint 1
- Path 1: Waypoint 1 → Waypoint 2
- Path 2: Waypoint 2 → Waypoint 3
- Path 3: Waypoint 3 → Waypoint 4

---

## Step 4: Understand the Basic Structure

All PedroPathing autonomous code has this structure:

```java
@Autonomous(name = "My Auton", group = "Autonomous")
public class MyAuton extends OpMode {
    
    // 1. Declare PedroPathing objects
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState = 0;
    
    // 2. Declare your waypoints (positions on field)
    private final Pose startpoint = new Pose(56.5, 8, Math.toRadians(90));
    private final Pose waypoint1 = new Pose(56.5, 15, Math.toRadians(90));
    
    // 3. Declare your paths (connections between waypoints)
    private PathChain path0, path1;
    
    // 4. Declare hardware
    private DcMotor shooter;
    // ... etc
    
    @Override
    public void init() {
        // 5. Initialize PedroPathing
        // 6. Build your paths
        // 7. Initialize hardware
    }
    
    @Override
    public void start() {
        // 8. Reset timers and set starting state
    }
    
    @Override
    public void loop() {
        // 9. Update follower (REQUIRED every loop)
        // 10. Your state machine (controls the sequence)
        // 11. Update telemetry
    }
    
    // 12. Helper methods
    private void buildPaths() {
        // Build your paths here
    }
    
    private void autonomousPathUpdate() {
        // State machine here
    }
}
```

**Key differences from simple auton:**
- Uses `OpMode` (not `LinearOpMode`)
- Has `init()`, `start()`, and `loop()` methods (not just `runOpMode()`)
- Uses a state machine to control the sequence
- Must call `follower.update()` every loop

---

## Step 5: Initialize PedroPathing

In the `init()` method, set up PedroPathing:

```java
@Override
public void init() {
    // Create timers
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();
    
    // Create follower (this is the path-following controller)
    follower = Constants.createFollower(hardwareMap);
    
    // Set starting position (where robot starts on field)
    follower.setStartingPose(startpoint);
    
    // Build your paths
    buildPaths();
    
    // Initialize hardware
    initializeHardware();
}
```

**What each part does:**
- `Timer` = Stopwatch for timing things
- `Follower` = The object that makes your robot follow paths
- `Constants.createFollower()` = Helper method that sets up the follower with your drive motors
- `setStartingPose()` = Tells the follower where the robot starts
- `buildPaths()` = Creates the paths between waypoints

---

## Step 6: Build Your Paths

In `buildPaths()`, you create paths between waypoints:

```java
private void buildPaths() {
    // Path 0: startpoint → waypoint1
    path0 = follower.pathBuilder()
            .addPath(new BezierLine(startpoint, waypoint1))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
            .build();
    
    // Path 1: waypoint1 → waypoint2
    path1 = follower.pathBuilder()
            .addPath(new BezierLine(waypoint1, waypoint2))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            .build();
}
```

**Breaking it down:**
- `follower.pathBuilder()` = Start building a new path
- `.addPath(new BezierLine(start, end))` = Add a straight line path from start to end
- `.setLinearHeadingInterpolation(startHeading, endHeading)` = Robot rotates from start heading to end heading as it moves
- `.build()` = Finish building the path

**Path types:**
- `BezierLine(start, end)` = Straight line between two points
- `BezierCurve(start, control, end)` = Curved path (uses a control point)

**Example with curve:**
```java
Pose controlPoint = new Pose(41.93, 30.76, 0);
path2 = follower.pathBuilder()
        .addPath(new BezierCurve(waypoint2, controlPoint, waypoint3))
        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
        .build();
```

---

## Step 7: Understand State Machines

A state machine controls your autonomous sequence. Think of it like a flowchart:

```
State 0: Start following path 0
    ↓
State 1: Wait for path 0 to finish
    ↓
State 2: Setup shooting
    ↓
State 3: Wait for shooter RPM
    ↓
State 4: Shoot
    ↓
State 5: Start following path 1
    ↓
State 6: Wait for path 1 to finish
    ↓
Done!
```

**Each state does ONE thing:**
- State 0: Start a path
- State 1: Wait for path to finish
- State 2: Do something (shoot, intake, etc.)
- State 3: Wait for something to finish

**How it works:**
```java
private void autonomousPathUpdate() {
    switch (pathState) {
        case 0:
            // Do something
            setPathState(1); // Move to next state
            break;
            
        case 1:
            // Check if something is done
            if (condition) {
                setPathState(2); // Move to next state
            }
            break;
            
        // ... more states
    }
}
```

---

## Step 8: Write Your State Machine

Let's build a simple state machine step by step:

### State 0: Start Following Path 0

**What it does:** Tells the follower to start following path 0, then immediately moves to state 1.

```java
case 0:
    follower.followPath(path0, true);
    setPathState(1);
    break;
```

**Breaking it down:**
- `follower.followPath(path0, true)` = "Start following path 0" (true = can reverse if needed)
- `setPathState(1)` = "Move to state 1"

### State 1: Wait for Path 0 to Finish

**What it does:** Checks if the path is done. When done, moves to state 2.

```java
case 1:
    if (!follower.isBusy()) {
        setPathState(2);
    }
    break;
```

**Breaking it down:**
- `follower.isBusy()` = "Is the robot still following a path?"
- `!follower.isBusy()` = "Is the robot NOT busy?" (path is finished)
- When path finishes, move to next state

### State 2: Do Something (e.g., Setup Shooting)

**What it does:** Sets up shooting, then moves to state 3.

```java
case 2:
    setupShooting();
    setPathState(3);
    break;
```

**Breaking it down:**
- Call a helper method to do the work
- Immediately move to next state (don't wait here)

### State 3: Wait for Something (e.g., Shooter RPM)

**What it does:** Waits for shooter to reach RPM, then moves to state 4.

```java
case 3:
    if (isShooterAtRPM() || pathTimer.getElapsedTime() > 5000) {
        // Turn on transfer/intake
        transferMotor.setPower(1.0);
        pathTimer.resetTimer();
        setPathState(4);
    }
    break;
```

**Breaking it down:**
- Check condition: `isShooterAtRPM()` OR timeout (5 seconds)
- If condition is true, do something and move to next state
- `pathTimer.getElapsedTime()` = How long since timer was reset (in milliseconds)

### State 4: Do Something for a Time (e.g., Shoot)

**What it does:** Shoots for 2 seconds, then moves to state 5.

```java
case 4:
    if (pathTimer.getElapsedTime() > 2000) { // 2 seconds
        // Turn everything off
        shooterMotor.setVelocity(0);
        transferMotor.setPower(0);
        setPathState(5);
    }
    break;
```

**Breaking it down:**
- Check if enough time has passed
- When time is up, clean up and move to next state

### Complete Example:

```java
private void autonomousPathUpdate() {
    switch (pathState) {
        case 0:
            // Start following path 0
            follower.followPath(path0, true);
            setPathState(1);
            break;
            
        case 1:
            // Wait for path 0 to finish
            if (!follower.isBusy()) {
                setPathState(2);
            }
            break;
            
        case 2:
            // Setup shooting
            setupShooting();
            setPathState(3);
            break;
            
        case 3:
            // Wait for shooter RPM
            if (isShooterAtRPM() || pathTimer.getElapsedTime() > 5000) {
                transferMotor.setPower(1.0);
                pathTimer.resetTimer();
                setPathState(4);
            }
            break;
            
        case 4:
            // Shoot for 2 seconds
            if (pathTimer.getElapsedTime() > 2000) {
                shooterMotor.setVelocity(0);
                transferMotor.setPower(0);
                setPathState(5);
            }
            break;
            
        case 5:
            // Start following path 1
            follower.followPath(path1, true);
            setPathState(6);
            break;
            
        case 6:
            // Wait for path 1 to finish
            if (!follower.isBusy()) {
                setPathState(7); // Done
            }
            break;
    }
}
```

---

## Step 9: The Loop Method

The `loop()` method runs continuously (many times per second). It must:

1. Update the follower (REQUIRED)
2. Run your state machine
3. Update telemetry

```java
@Override
public void loop() {
    // REQUIRED: Update follower every loop
    follower.update();
    
    // Run your state machine
    autonomousPathUpdate();
    
    // Update telemetry (for debugging)
    updateTelemetry();
}
```

**Why `follower.update()` is required:**
- The follower needs to check the robot's position and adjust motor powers constantly
- Without this, the robot won't follow the path

---

## Step 10: Helper Methods

Helper methods do specific tasks. Here are common ones:

### Setup Shooting

**What it does:** Positions turret, sets hood, starts shooter.

```java
private void setupShooting() {
    // Position turret
    if (robot.spinner != null) {
        robot.spinner.setTargetPosition(-50);
        robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spinner.setPower(0.8);
    }
    
    // Set hood
    if (hoodServo != null) {
        hoodServo.setPosition(0.700);
    }
    
    // Start shooter at 6000 RPM
    if (shooterMotor != null) {
        double targetRPM = -6000.0;
        double velocityTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;
        shooterMotor.setVelocity((int)Math.round(velocityTicksPerSec));
    }
    
    // Reset timer for waiting
    pathTimer.resetTimer();
}
```

**Breaking it down:**
- Set turret to a specific position (encoder ticks)
- Set hood servo to a position (0.0 to 1.0)
- Set shooter to a specific RPM using velocity control
- Reset timer so you can time the wait

### Check if Shooter at RPM

**What it does:** Checks if shooter is close enough to target RPM.

```java
private static final int TICKS_PER_REVOLUTION = 28;
private static final double RPM_TOLERANCE = 50.0;

private boolean isShooterAtRPM() {
    if (shooterMotor == null) return false;
    
    try {
        double targetRPM = -6000.0;
        double currentVelocity = shooterMotor.getVelocity();
        double currentRPM = (currentVelocity / TICKS_PER_REVOLUTION) * 60.0;
        double rpmError = Math.abs(currentRPM - targetRPM);
        return rpmError <= RPM_TOLERANCE;
    } catch (Exception e) {
        return false;
    }
}
```

**Breaking it down:**
- Get current velocity from motor (in ticks per second)
- Convert to RPM: `(velocity / ticks_per_rev) * 60`
- Calculate error: `|currentRPM - targetRPM|`
- Return true if error is within tolerance (50 RPM)

### Set Path State Helper

**What it does:** Changes state and resets timer (convenience method).

```java
private void setPathState(int newState) {
    pathState = newState;
    pathTimer.resetTimer();
}
```

**Why this is useful:**
- Always resets timer when changing states
- Makes code cleaner and less error-prone

---

## Step 11: Common State Patterns

### Pattern 1: Follow a Path

**Always use two states:**
```java
case X:
    follower.followPath(pathX, true);
    setPathState(X + 1);
    break;

case X + 1:
    if (!follower.isBusy()) {
        setPathState(X + 2);
    }
    break;
```

**Why two states?**
- State X: Start the path (happens once)
- State X+1: Wait for it to finish (checks every loop)

### Pattern 2: Wait for Time

```java
case X:
    pathTimer.resetTimer();
    setPathState(X + 1);
    break;

case X + 1:
    if (pathTimer.getElapsedTime() > 2000) { // 2 seconds
        setPathState(X + 2);
    }
    break;
```

**Breaking it down:**
- State X: Reset timer and move to waiting state
- State X+1: Check if enough time has passed

### Pattern 3: Wait for Condition

```java
case X:
    // Do setup
    setupSomething();
    pathTimer.resetTimer();
    setPathState(X + 1);
    break;

case X + 1:
    if (isConditionMet() || pathTimer.getElapsedTime() > 5000) {
        // Condition met OR timeout (5 seconds)
        setPathState(X + 2);
    }
    break;
```

**Breaking it down:**
- State X: Setup and start waiting
- State X+1: Check condition OR timeout (safety)

### Pattern 4: Do Action for Time

```java
case X:
    // Start action
    motor.setPower(1.0);
    pathTimer.resetTimer();
    setPathState(X + 1);
    break;

case X + 1:
    // Keep action running
    motor.setPower(1.0); // Keep it on
    
    if (pathTimer.getElapsedTime() > 2000) { // 2 seconds
        // Stop action
        motor.setPower(0);
        setPathState(X + 2);
    }
    break;
```

**Breaking it down:**
- State X: Start the action
- State X+1: Keep it running, check if time is up

---

## Step 12: Your First PedroPathing Auton - Step by Step

Let's build a super simple one together:

### Step 1: Create the class
```java
@Autonomous(name = "My First Pathing Auton", group = "Autonomous")
public class MyFirstPathingAuton extends OpMode {
    // We'll add code here
}
```

### Step 2: Declare PedroPathing objects
```java
private Follower follower;
private Timer pathTimer, opmodeTimer;
private int pathState = 0;
```

### Step 3: Declare your waypoints
```java
private final Pose startpoint = new Pose(56.5, 8, Math.toRadians(90));
private final Pose waypoint1 = new Pose(45, 33, Math.toRadians(180));
```

### Step 4: Declare your paths
```java
private PathChain path0;
```

### Step 5: Initialize in init()
```java
@Override
public void init() {
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();
    
    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startpoint);
    
    buildPaths();
}
```

### Step 6: Build your path
```java
private void buildPaths() {
    path0 = follower.pathBuilder()
            .addPath(new BezierLine(startpoint, waypoint1))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            .build();
}
```

### Step 7: Write start() method
```java
@Override
public void start() {
    opmodeTimer.resetTimer();
    pathTimer.resetTimer();
    pathState = 0;
}
```

### Step 8: Write loop() method
```java
@Override
public void loop() {
    follower.update(); // REQUIRED
    
    autonomousPathUpdate();
}
```

### Step 9: Write state machine
```java
private void autonomousPathUpdate() {
    switch (pathState) {
        case 0:
            follower.followPath(path0, true);
            setPathState(1);
            break;
            
        case 1:
            if (!follower.isBusy()) {
                setPathState(2); // Done
            }
            break;
    }
}
```

### Step 10: Add helper method
```java
private void setPathState(int newState) {
    pathState = newState;
    pathTimer.resetTimer();
}
```

### Step 11: Test it!
Run it and see if the robot moves from startpoint to waypoint1.

---

## Step 13: Adding More Paths

Once your first path works, add more:

### Add waypoint and path:
```java
private final Pose waypoint2 = new Pose(20, 33, Math.toRadians(180));
private PathChain path0, path1;
```

### Build the new path:
```java
private void buildPaths() {
    path0 = follower.pathBuilder()
            .addPath(new BezierLine(startpoint, waypoint1))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            .build();
    
    path1 = follower.pathBuilder()
            .addPath(new BezierLine(waypoint1, waypoint2))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
}
```

### Add states to follow the new path:
```java
case 1:
    if (!follower.isBusy()) {
        setPathState(2); // Go to next path
    }
    break;

case 2:
    follower.followPath(path1, true);
    setPathState(3);
    break;

case 3:
    if (!follower.isBusy()) {
        setPathState(4); // Done
    }
    break;
```

---

## Step 14: Adding Actions Between Paths

You can do actions (shoot, intake, etc.) between paths:

```java
case 1:
    if (!follower.isBusy()) {
        setPathState(2); // Go to shooting
    }
    break;

case 2:
    // Setup shooting
    shooterMotor.setVelocity(targetRPM);
    pathTimer.resetTimer();
    setPathState(3);
    break;

case 3:
    // Wait for RPM
    if (isShooterAtRPM() || pathTimer.getElapsedTime() > 5000) {
        transferMotor.setPower(1.0);
        pathTimer.resetTimer();
        setPathState(4);
    }
    break;

case 4:
    // Shoot for 2 seconds
    if (pathTimer.getElapsedTime() > 2000) {
        shooterMotor.setVelocity(0);
        transferMotor.setPower(0);
        setPathState(5); // Go to next path
    }
    break;

case 5:
    follower.followPath(path1, true);
    setPathState(6);
    break;
```

---

## Step 15: Testing Your Code

**Test incrementally:**

1. **Test one path first:**
   ```java
   case 0:
       follower.followPath(path0, true);
       setPathState(1);
       break;
   case 1:
       if (!follower.isBusy()) {
           // Done - just stop here for testing
       }
       break;
   ```

2. **Add the next path:**
   ```java
   case 1:
       if (!follower.isBusy()) {
           setPathState(2); // Add this
       }
       break;
   case 2:
       follower.followPath(path1, true); // Add this
       setPathState(3);
       break;
   ```

3. **Add actions one at a time** (shooting, intake, etc.)

**Common issues:**
- Robot doesn't move? → Check if `follower.update()` is in `loop()`
- Robot goes wrong direction? → Check waypoint coordinates
- Robot overshoots waypoint? → Check if you're waiting for `!follower.isBusy()`
- Path doesn't start? → Check if you called `follower.followPath()` in the right state

---

## Step 16: Debugging Tips

**Use telemetry to see what's happening:**

```java
private void updateTelemetry() {
    telemetry.addData("Path State", pathState);
    telemetry.addData("Follower Busy", follower.isBusy());
    
    if (follower != null) {
        Pose currentPose = follower.getPose();
        telemetry.addData("Current X", "%.1f", currentPose.getX());
        telemetry.addData("Current Y", "%.1f", currentPose.getY());
        telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
    }
    
    telemetry.update();
}
```

**Add this to your `loop()` method:**
```java
@Override
public void loop() {
    follower.update();
    autonomousPathUpdate();
    updateTelemetry(); // Add this
}
```

**What to look for:**
- **Path State:** Which state you're in
- **Follower Busy:** Is robot still following a path?
- **Current X, Y, Heading:** Where the robot thinks it is
- Compare these to your waypoint coordinates

---

## Key Concepts to Remember

1. **Follower.update() is REQUIRED** - Must be called every loop
2. **Two states per path** - One to start, one to wait
3. **State machine controls sequence** - Each state does one thing
4. **Field coordinates matter** - Measure your starting position carefully
5. **Test incrementally** - Add one path/action at a time
6. **Always check `!follower.isBusy()`** - Before moving to next path

---

## Common Mistakes to Avoid

1. **Forgetting `follower.update()`** - Robot won't follow paths
2. **Not waiting for `!follower.isBusy()`** - Starting next path too early
3. **Wrong starting pose** - Robot thinks it's somewhere else
4. **Wrong waypoint coordinates** - Robot goes to wrong place
5. **Not resetting timer** - Timing checks won't work
6. **Starting path in wrong state** - Path might not start or might restart

---

## Practice Exercise

Try to write an autonomous that:
1. Starts at (56.5, 8) facing 90°
2. Moves to (45, 33) facing 180°
3. Waits 1 second
4. Moves to (20, 33) facing 180°
5. Stops

**You'll need:**
- 3 waypoints (start, middle, end)
- 2 paths (start→middle, middle→end)
- 5 states (start path0, wait path0, wait 1 sec, start path1, wait path1)

Good luck! Start with one path, test it, then add more.
