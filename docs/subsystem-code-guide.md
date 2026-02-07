# Team 6637 Betawolves - 2026 Chassis Bot Code Guide

> A complete walkthrough of every subsystem, how the code works, and the patterns you need to know to build new subsystems confidently.

---

## Table of Contents

1. [How This Robot Code is Organized](#1-how-this-robot-code-is-organized)
2. [The Startup Sequence: What Happens When the Robot Turns On](#2-the-startup-sequence-what-happens-when-the-robot-turns-on)
3. [The Command-Based Framework (How Robot Actions Work)](#3-the-command-based-framework-how-robot-actions-work)
4. [The IO Layer Pattern (AdvantageKit)](#4-the-io-layer-pattern-advantagekit)
5. [Subsystem Deep Dive: Swerve Drive](#5-subsystem-deep-dive-swerve-drive)
   - [5.1 SwerveDrive.java - The Brain](#51-swervedrivejava---the-brain)
   - [5.2 SwerveModule - A Single Wheel Pod](#52-swervemodule---a-single-wheel-pod)
   - [5.3 Gyro - Knowing Which Way We Face](#53-gyro---knowing-which-way-we-face)
   - [5.4 Odometry - Knowing Where We Are](#54-odometry---knowing-where-we-are)
   - [5.5 Module Angle Optimization - The Clever Shortcut](#55-module-angle-optimization---the-clever-shortcut)
6. [Subsystem Deep Dive: Vision](#6-subsystem-deep-dive-vision)
   - [6.1 Vision.java - Processing Camera Data](#61-visionjava---processing-camera-data)
   - [6.2 VisionIO Implementations](#62-visionio-implementations)
   - [6.3 How Vision Improves Our Position Estimate](#63-how-vision-improves-our-position-estimate)
7. [Commands: Making the Robot Do Things](#7-commands-making-the-robot-do-things)
   - [7.1 TeleopDriveCommand](#71-teleopdrivecommand)
   - [7.2 TestBangBang](#72-testbangbang)
8. [RobotContainer: Wiring It All Together](#8-robotcontainer-wiring-it-all-together)
9. [Constants: The Single Source of Truth](#9-constants-the-single-source-of-truth)
10. [Autonomous with PathPlanner](#10-autonomous-with-pathplanner)
11. [Logging and Debugging with AdvantageKit](#11-logging-and-debugging-with-advantagekit)
12. [How to Add a New Subsystem (Step-by-Step Guide)](#12-how-to-add-a-new-subsystem-step-by-step-guide)
13. [Key Vendor Libraries Reference](#13-key-vendor-libraries-reference)
14. [Common Pitfalls and Debugging Tips](#14-common-pitfalls-and-debugging-tips)

---

## 1. How This Robot Code is Organized

Here is the project file tree with annotations:

```
src/main/java/frc/robot/
  Main.java                          # Entry point - just boots WPILib
  Robot.java                         # Lifecycle manager - sets up logging, runs scheduler
  RobotContainer.java                # Wires subsystems, commands, and controls together
  Constants.java                     # All magic numbers live here

  commands/
    TeleopDriveCommand.java          # Translates joystick inputs into driving
    TestBangBang.java                # Simple test command (drive forward 1 meter)

  subsystems/
    swerve/
      SwerveDrive.java               # Main drivetrain subsystem
      Gyro/
        GyroIO.java                  # Interface (contract for any gyro)
        GyroPigeon.java              # Real Pigeon2 hardware implementation
        GyroSim.java                 # Simulated gyro (integrates rotation rate)
      Odometry/
        OdometryIO.java              # Interface (contract for odometry)
        OdometryIOInputs.java        # Data class for logging odometry
        OdometryReal.java            # Real wheel-based odometry
        OdometrySim.java             # Simulated odometry
      SwerveModule/
        SwerveModule.java            # Coordinator for one wheel pod
        SwerveModuleConstants.java   # Data class: CAN IDs, offsets, position
        SwerveModuleIO.java          # Interface (contract for a module)
        SwerveModuleIOInputs.java    # Data class for logging module state
        SwerveModuleIOReal.java      # Real SparkMax + CANcoder hardware
        SwerveModuleIOSim.java       # Simulated module physics

    vision/
      Vision.java                    # Main vision subsystem
      VisionConstants.java           # Camera names, transforms, thresholds
      VisionIO.java                  # Interface + data types for any camera
      VisionIOLimelight.java         # Limelight implementation (available, not active)
      VisionIOPhotonVision.java      # PhotonVision implementation (active on real robot)
      VisionIOPhotonVisionSim.java   # PhotonVision simulation

  util/
    SwerveModuleAngleOptimizer.java  # Minimizes wheel rotation (active)
    OptimizeModuleState.java         # Older version (unused)
```

**Key principle:** Every subsystem follows the same structure: an **interface** that defines what it can do, a **real** implementation for the physical robot, and a **sim** implementation for testing on your laptop.

---

## 2. The Startup Sequence: What Happens When the Robot Turns On

Understanding the boot sequence helps you know *when* your code runs and *why* things are ordered the way they are.

```
1. JVM starts
     |
2. Main.java runs: RobotBase.startRobot(Robot::new)
     |
     |  This creates a single Robot instance and starts the
     |  WPILib event loop (runs at 50Hz = every 20ms)
     |
3. Robot() constructor runs:
     |  a. Configures AdvantageKit logging
     |     - Real robot: logs to USB stick + NetworkTables
     |     - Simulation:  logs to NetworkTables only
     |     - Replay:      reads from .wpilog file
     |  b. Starts the logger
     |  c. Creates RobotContainer
     |
4. RobotContainer() constructor runs:
     |  a. Creates SwerveDrive (which creates 4 modules, gyro, odometry)
     |  b. Creates Vision (which creates camera IO objects)
     |  c. Connects Vision output -> SwerveDrive pose estimator
     |  d. Sets up joystick bindings
     |  e. Sets TeleopDriveCommand as the default swerve command
     |  f. Creates PathPlanner auto chooser
     |
5. robotInit() runs:
     |  PathPlanner warmup command is scheduled (pre-generates trajectories)
     |
6. Every 20ms: robotPeriodic() runs:
     |  CommandScheduler.run() executes:
     |    - Calls periodic() on ALL subsystems (SwerveDrive, Vision)
     |    - Runs all active commands (execute() methods)
     |    - Checks isFinished() on running commands
```

**Why this matters:** When you create a new subsystem, you construct it in `RobotContainer`. Its `periodic()` method will automatically be called every 20ms by the `CommandScheduler` - you never call it yourself.

---

## 3. The Command-Based Framework (How Robot Actions Work)

FRC robot code uses the **Command-Based** pattern. Here is the mental model:

### Subsystems = "What the robot has"
A subsystem represents a physical mechanism (drivetrain, arm, shooter, etc.). It:
- Extends `SubsystemBase`
- Has a `periodic()` method that runs every 20ms (for reading sensors, logging)
- Can only be controlled by **one command at a time** (this prevents conflicts)

### Commands = "What the robot does"
A command represents an action (drive forward, score a game piece, etc.). It:
- Extends `Command`
- Declares which subsystems it **requires** (via `addRequirements()`)
- Has a lifecycle: `initialize()` -> `execute()` (loops) -> `end()` when `isFinished()` returns true

### How they interact:

```
Joystick Input
     |
     v
TeleopDriveCommand (requires SwerveDrive)
     |
     |  execute() converts joystick values to velocities
     |  and calls swerveDrive.drive(vx, vy, omega)
     |
     v
SwerveDrive.drive() stores the desired ChassisSpeeds
     |
     |  periodic() converts ChassisSpeeds -> SwerveModuleState[]
     |  and sends them to the four modules
     |
     v
Each SwerveModule sets motor PID targets
```

### Default Commands

A default command runs on a subsystem whenever no other command is using it. In our code:
```java
// RobotContainer.java
swerveDrive.setDefaultCommand(new TeleopDriveCommand(...));
```
This means the robot is always drivable in teleop unless another command (like an auto path) takes over.

---

## 4. The IO Layer Pattern (AdvantageKit)

This is the most important architectural pattern in our codebase. Once you understand it, every subsystem will make sense.

### The Problem
You want to:
- Test code on your laptop without a real robot
- Replay logged data to debug issues after a match
- Swap hardware (e.g., Limelight to PhotonVision) without rewriting subsystem logic

### The Solution: IO Interfaces

Every piece of hardware gets an **interface** that defines *what* it can do, not *how*:

```
         GyroIO (interface)
        /         \
GyroPigeon      GyroSim
(real Pigeon2)  (math-based simulation)
```

The subsystem only talks to the interface. It never knows (or cares) whether it's talking to real hardware or a simulation:

```java
// SwerveDrive.java - doesn't care which implementation it gets
private final GyroIO gyro;

public SwerveDrive() {
    if (RobotBase.isSimulation()) {
        gyro = new GyroSim();       // Laptop testing
    } else {
        gyro = new GyroPigeon();    // Real robot
    }
}
```

### The Inputs Pattern

Each IO interface has an **Inputs** class - a simple data container that gets filled by the hardware layer and logged by AdvantageKit:

```java
// GyroIO.java
@AutoLog                           // AdvantageKit generates logging code
public static class GyroIOInputs {
    public Rotation2d yaw = new Rotation2d();
    public double yawDegrees = 0.0;
}
```

The flow every 20ms:
```
1. io.updateInputs(inputs)    // Hardware fills in current sensor values
2. Logger.processInputs(inputs) // AdvantageKit records them to the log
3. Subsystem logic reads inputs  // Your code uses the values
```

**Why this is powerful:** During replay, step 1 is skipped entirely. AdvantageKit replays the logged values from step 2, so your subsystem logic (step 3) runs on *real match data* on your laptop.

### When to use `@AutoLog` vs manual `LoggableInputs`

- **`@AutoLog`** (used for GyroIO, VisionIO): AdvantageKit generates the logging code automatically at compile time. Simpler, less boilerplate. Use this for most new subsystems.
- **Manual `LoggableInputs`** (used for OdometryIOInputs, SwerveModuleIOInputs): You write `toLog()`/`fromLog()` yourself. Only needed for types that `@AutoLog` doesn't handle automatically.

---

## 5. Subsystem Deep Dive: Swerve Drive

### 5.1 SwerveDrive.java - The Brain

**File:** `src/main/java/frc/robot/subsystems/swerve/SwerveDrive.java`

This is the central drivetrain subsystem. Think of it as the **coordinator** - it doesn't directly touch motors, but it tells each module what to do.

#### Key Fields

```java
private ChassisSpeeds desiredChassisSpeeds;  // What speed the robot WANTS to go
private final GyroIO gyro;                    // "Which way am I facing?"
private final OdometryIO odometry;            // "Where am I?" (wheels only)
private final SwerveModule[] modules;         // The four wheel pods
private final SwerveDriveKinematics kinematics; // Math: chassis speed <-> wheel speeds
private final SwerveDrivePoseEstimator poseEstimator; // Best guess of position (wheels + vision)
```

#### How Driving Works (the `drive()` method)

When a command calls `drive(vx, vy, omega)`:

```java
public void drive(double xVelocity, double yVelocity, double angVelocity) {
    // Convert field-relative speeds to robot-relative
    desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xVelocity, yVelocity, angVelocity, getHeading()
    );
}
```

**Field-relative vs robot-relative:**
- **Field-relative** (what we use): Pushing the joystick "up" always drives toward the far wall, regardless of which way the robot faces. This is more intuitive for drivers.
- **Robot-relative:** Pushing "up" drives wherever the robot's front is pointing. Used internally by PathPlanner autonomous.

The actual driving happens in `periodic()`:

```java
public void periodic() {
    if (desiredChassisSpeeds != null) {
        // Math: Convert ONE chassis velocity into FOUR individual wheel states
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(desiredChassisSpeeds);
        setModuleStates(states);
    }
    log();
    desiredChassisSpeeds = null;  // Safety: stop if no command runs next cycle
    // Update position estimate with latest sensor data
    poseEstimator.update(getHeading(), getModulePositions());
}
```

**Why set `desiredChassisSpeeds = null`?** This is a safety measure. If a command crashes or stops running, the robot will stop moving instead of continuing at the last commanded speed.

#### The Pose Estimator

The `SwerveDrivePoseEstimator` is our best guess of "where is the robot on the field?" It combines:
- **Wheel odometry** (reliable short-term, drifts over time)
- **Gyro heading** (accurate direction, no position info)
- **Vision measurements** (corrects drift, but has noise and latency)

```java
// Called by Vision subsystem when it detects AprilTags
public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {
    poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
}
```

The `stdDevs` (standard deviations) tell the estimator *how much to trust* each measurement. Lower values = more trust. Vision calculates these based on how far away the tags are and how many it sees.

#### PathPlanner Integration

The constructor configures PathPlanner's `AutoBuilder` so it can control the drivetrain during autonomous:

```java
AutoBuilder.configure(
    this::getPose,              // "Where am I?"
    this::resetPose,            // "Start here" (beginning of auto)
    this::getChassisSpeeds,     // "How fast am I going?"
    this::driveRobotRelative,   // "Go this fast" (PathPlanner's commands)
    new PPHolonomicDriveController(
        new PIDConstants(5, 0, 0),  // Translation correction
        new PIDConstants(5, 0, 0)   // Rotation correction
    ),
    config,                     // Robot physical properties
    () -> isRedAlliance(),      // Flip paths for red alliance
    this                        // This subsystem
);
```

---

### 5.2 SwerveModule - A Single Wheel Pod

Each swerve module has **two motors** and **one absolute encoder**:
- **Drive motor** (NEO via SparkMax): Spins the wheel to move the robot
- **Angle motor** (NEO via SparkMax): Rotates the wheel to point in any direction
- **CANcoder** (absolute encoder): Knows the exact wheel angle even after power-off

#### SwerveModule.java (Coordinator)

**File:** `src/main/java/frc/robot/subsystems/swerve/SwerveModule/SwerveModule.java`

This is a thin wrapper that picks the right IO implementation and delegates:

```java
public SwerveModule(SwerveModuleConstants moduleConstants) {
    if (RobotBase.isSimulation()) {
        io = new SwerveModuleIOSim(moduleConstants);
    } else {
        io = new SwerveModuleIOReal(moduleConstants);
    }
}
```

#### SwerveModuleIOReal.java (The Hardware Layer)

**File:** `src/main/java/frc/robot/subsystems/swerve/SwerveModule/SwerveModuleIOReal.java`

This is where the rubber meets the road (literally). Let's walk through the important parts:

**Motor Configuration:**

```java
private void configureDriveMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)      // Resist movement when not powered
        .inverted(constants.driveInverted)
        .smartCurrentLimit(DRIVE_CURRENT_LIMIT)  // 40A - protects motor
        .openLoopRampRate(DRIVE_RAMP);           // 0.25s to full power (smooth)

    // Tell the encoder to report in METERS and M/S (not raw rotations)
    config.encoder
        .positionConversionFactor(DRIVE_POSITION_CONVERSION)   // rotations -> meters
        .velocityConversionFactor(DRIVE_VELOCITY_CONVERSION);  // RPM -> m/s

    // PID controller runs ON the SparkMax (faster than RoboRIO loop)
    config.closedLoop
        .p(DRIVE_kP)   // 0.000215 - very small because velocity units are large
        .outputRange(-1, 1);

    driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}
```

**What are conversion factors?** The raw encoder counts rotations of the motor shaft. We want meters at the wheel. The conversion accounts for the gear ratio (6.75:1) and wheel circumference:
```
DRIVE_POSITION_CONVERSION = wheelCircumference / gearRatio
                          = 0.2985m / 6.75
                          = 0.04423 meters per motor rotation
```

**The `setDesiredState()` method - commanding a module:**

```java
public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize: maybe spin the wheel backward and rotate less
    desiredState = SwerveModuleAngleOptimizer.optimize(desiredState, getRelativeAngle());

    // Convert desired speed (m/s) to what the motor controller expects (RPM)
    double desiredDriveRPM = desiredState.speedMetersPerSecond / DRIVE_VELOCITY_CONVERSION;

    // Set the drive motor to velocity PID mode (maintains speed under load)
    driveMotor.getClosedLoopController().setReference(
        desiredDriveRPM, ControlType.kVelocity
    );

    // Set the angle motor to position PID mode (holds exact angle)
    angleMotor.getClosedLoopController().setReference(
        desiredState.angle.getRotations(),  // Target angle in rotations
        ControlType.kPosition
    );
}
```

**Why two different PID modes?**
- **Drive motor uses velocity PID:** "Maintain this speed." The PID corrects for friction, battery voltage changes, etc.
- **Angle motor uses position PID:** "Go to this angle and stay there." The PID handles getting there and holding.

**The CANcoder and absolute vs relative encoders:**

```java
public void resetToAbsolute() {
    // On startup, seed the relative encoder with the absolute position
    double adjustedAngle = getAdjustedAbsoluteAngle();
    angleMotor.getEncoder().setPosition(adjustedAngle);
}
```

Why do we need both?
- The **CANcoder** (absolute) always knows the true wheel angle, even after power cycling. But it updates slowly and can be noisy.
- The **NEO's built-in encoder** (relative) is fast and smooth, but loses its position on power-off.
- **Solution:** On startup, we read the CANcoder once to "seed" the relative encoder. Then we use the relative encoder for control (fast) and the CANcoder just for reference.

#### SwerveModuleIOSim.java (Simulation)

The sim version simplifies the physics:

```java
public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, simAngle);

    // Drive velocity is set instantly (no motor dynamics)
    simDriveVelocity = state.speedMetersPerSecond;

    // Steering angle moves toward target, capped at max rotation speed
    double angleDelta = state.angle.minus(simAngle).getRadians();
    double maxDelta = MAX_ROTATION_SPEED * LOOP_PERIOD;  // PI * 0.02 = 0.0628 rad per loop
    angleDelta = MathUtil.clamp(angleDelta, -maxDelta, maxDelta);
    simAngle = simAngle.plus(new Rotation2d(angleDelta));
}
```

This is good enough to test autonomous paths and command logic without needing the real robot.

#### SwerveModuleConstants.java (Data)

Each module is defined by a `SwerveModuleConstants` instance in `Constants.java`:

```java
public static final SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(
    12,                    // drive motor CAN ID
    11,                    // angle motor CAN ID
    13,                    // CANcoder CAN ID
    "rio",                 // CAN bus name
    true,                  // drive motor inverted
    true,                  // angle motor inverted
    Rotation2d.fromDegrees(188.877),    // CANcoder zero offset
    new Translation2d(0.288, 0.288)     // position on chassis (meters from center)
);
```

**The angle offset** is critical: it's the CANcoder reading when the wheel is pointing straight forward. Every module's offset is different because the CANcoder is mounted at a slightly different physical angle. If these offsets are wrong, the robot will crab-walk or spin unexpectedly.

---

### 5.3 Gyro - Knowing Which Way We Face

#### GyroIO.java (Interface)

```java
public interface GyroIO {
    Rotation2d getYaw();
    void zeroYaw();
    void updateInputs(GyroIOInputs inputs);
    default void updateInputs(GyroIOInputs inputs, double simRotationRate) {
        updateInputs(inputs);  // Default: ignore sim parameter
    }
}
```

The `default` method is a neat trick: real hardware ignores the `simRotationRate` parameter, but the sim implementation uses it to integrate heading over time.

#### GyroPigeon.java (Real Hardware)

Uses the CTRE Pigeon2 IMU on CAN ID 9. The Pigeon2 contains accelerometers and gyroscopes that measure rotation. `getYaw()` returns the accumulated heading since last reset.

#### GyroSim.java (Simulation)

Since there's no physical gyroscope in simulation, we calculate heading mathematically:

```java
public void updateInputs(GyroIOInputs inputs, double simRotationRateRadiansPerSecond) {
    double currentTime = Timer.getFPGATimestamp();
    double deltaTime = currentTime - lastInputsUpdateTime;

    // heading = integral of rotation rate over time
    double newYawDegrees = Math.toDegrees(simRotationRateRadiansPerSecond) * deltaTime
                         + inputs.yawDegrees;

    // Wrap to [-180, 180]
    newYawDegrees = MathUtil.inputModulus(newYawDegrees, -180, 180);
    inputs.yaw = Rotation2d.fromDegrees(newYawDegrees);
    inputs.yawDegrees = newYawDegrees;
}
```

This is basic calculus: if you know how fast you're spinning (`omega`) and how long you've been spinning (`dt`), your new angle is `angle += omega * dt`.

---

### 5.4 Odometry - Knowing Where We Are

Odometry tracks the robot's position by looking at how much each wheel has moved.

#### How Wheel Odometry Works (Conceptual)

```
Time 0: Robot is at (0, 0), heading 0 degrees
        All four wheels have driven 0 meters

Time 1: Front-left wheel drove 0.1m at 45 degrees
        Front-right wheel drove 0.1m at 45 degrees
        (etc. for all four wheels)

        Kinematics math: "That means the robot moved
        ~0.07m forward and ~0.07m left"

        New position: (0.07, 0.07)
```

WPILib's `SwerveDriveOdometry` does this math for us every 20ms. We just feed it the gyro heading and the four module positions (distance + angle).

#### OdometryReal.java / OdometrySim.java

Both implementations are actually the same - they wrap `SwerveDriveOdometry`:

```java
public void updateInputs(OdometryIOInputs inputs, Rotation2d heading,
                         SwerveModulePosition[] positions) {
    odometry.update(heading, positions);
    inputs.robotPose = odometry.getPoseMeters();
}
```

Having separate classes allows you to add simulation-specific features later (like adding noise to simulate real-world drift).

**Important:** Odometry alone drifts over time. That's why we also have the `SwerveDrivePoseEstimator` in `SwerveDrive.java` that fuses odometry with vision data for a better estimate.

---

### 5.5 Module Angle Optimization - The Clever Shortcut

**File:** `src/main/java/frc/robot/util/SwerveModuleAngleOptimizer.java`

**The problem:** If a wheel is pointing at 10 degrees and you want it at 190 degrees, should it rotate 180 degrees? No! It can just stay at 10 degrees and spin the drive wheel **backward**. Same result, half the rotation time.

```
Without optimization:        With optimization:
  Wheel at 10 deg             Wheel at 10 deg
  Target: 190 deg             Target: 190 deg
  Rotate 180 deg              Stay at 10 deg, reverse drive
  Drive forward               Drive backward
  (SLOW)                      (FAST - no rotation needed!)
```

The `optimize()` method:
1. Maps the target angle into the same 360-degree range as the current angle
2. If the difference is more than 90 degrees: reverse speed, adjust angle by 180 degrees
3. Returns the optimized state

This is why swerve robots look so smooth - the wheels minimize their rotation to change direction.

---

## 6. Subsystem Deep Dive: Vision

### 6.1 Vision.java - Processing Camera Data

**File:** `src/main/java/frc/robot/subsystems/vision/Vision.java`

The Vision subsystem detects AprilTags on the field and uses them to correct the robot's position estimate. AprilTags are like QR codes at known locations on the field - if you can see one and know its ID, you can calculate exactly where you are.

#### The Core Loop (periodic)

Every 20ms:

```
1. Update all camera inputs
2. For each camera:
   a. Get all new pose observations (AprilTag detections)
   b. For each observation, apply quality filters:
      - Reject if no tags detected
      - Reject if single tag with high ambiguity (>0.3)
      - Reject if Z-axis error is too large (>0.75m)
      - Reject if computed pose is outside field boundaries
   c. Calculate trust level (standard deviations) based on:
      - How far away the tags are
      - How many tags we see (more tags = more trust)
   d. Send accepted measurements to SwerveDrive's pose estimator
```

#### Why Filter Observations?

Not all vision measurements are good. Here are the rejection criteria and why they matter:

| Filter | Threshold | Why |
|---|---|---|
| No tags | 0 tags detected | No data to work with |
| High ambiguity | > 0.3 | Camera isn't sure which way the tag is oriented. Two possible solutions exist and it can't tell which is correct. |
| Z-axis error | > 0.75m | The robot should be on the ground (Z near 0). Large Z values mean the math went wrong. |
| Out of bounds | Outside field dimensions | If the computed pose is off the field, something is clearly wrong. |

#### Standard Deviations (Trust Levels)

```java
double factor = (averageDistance * averageDistance) / tagCount;
double linearStdDev = 0.02 * factor;   // Position uncertainty (meters)
double angularStdDev = 0.06 * factor;  // Rotation uncertainty (radians)
```

**Distance squared** is used because measurement error grows quadratically with distance (a tag at 5m is ~25x less reliable than one at 1m). **Dividing by tag count** rewards seeing multiple tags (triangulation is more accurate).

These standard deviations tell the `SwerveDrivePoseEstimator`: "I think the robot is at position X, but I could be off by this much." The estimator weighs this against its odometry-based estimate and picks the best blend.

### 6.2 VisionIO Implementations

#### VisionIOPhotonVision.java (Active on Real Robot)

Uses a PhotonVision camera (likely a USB camera with a Raspberry Pi coprocessor). The coprocessor does the heavy image processing and sends results to the RoboRIO.

**How a pose is calculated from tags:**

```
For multiple tags (most accurate):
  1. PhotonVision computes "field to camera" transform using all visible tags
  2. We apply "camera to robot center" transform (known from VisionConstants)
  3. Result: robot's position on the field

For a single tag:
  1. Look up the tag's known position on the field
  2. PhotonVision gives us "camera to tag" transform
  3. Combine: fieldToTag -> tagToCamera -> cameraToRobot
  4. Result: robot's position on the field (less accurate)
```

#### VisionIOPhotonVisionSim.java (Simulation)

Extends the real PhotonVision implementation and adds a simulated camera:

```java
public void updateInputs(VisionIOInputs inputs) {
    // Update the simulated world with our current robot pose
    visionSim.update(poseSupplier.get());
    // Then process the simulated camera output through the normal pipeline
    super.updateInputs(inputs);
}
```

This creates virtual AprilTags in the simulated world and renders what the camera would see from the robot's current position. Very useful for testing autonomous routines.

#### VisionIOLimelight.java (Available but Not Active)

A Limelight implementation that reads data via NetworkTables. Supports both MegaTag1 (standard) and MegaTag2 (orientation-enhanced) pose estimation. Not currently connected in `RobotContainer` but ready to use if you switch cameras.

### 6.3 How Vision Improves Our Position Estimate

Here is the complete data flow:

```
AprilTags on Field
     |
     v
Camera sees tags -> PhotonVision processes image
     |
     v
VisionIOPhotonVision.updateInputs() -> PoseObservation[]
     |
     v
Vision.periodic()
     |  Filters bad measurements
     |  Calculates trust levels (std devs)
     |
     v
consumer.accept(pose, timestamp, stdDevs)
     |
     |  (This is SwerveDrive::addVisionMeasurement)
     |
     v
SwerveDrivePoseEstimator.addVisionMeasurement()
     |
     |  Blends vision with wheel odometry using a Kalman filter
     |
     v
SwerveDrive.getPose() -> Best estimate of robot position
```

---

## 7. Commands: Making the Robot Do Things

### 7.1 TeleopDriveCommand

**File:** `src/main/java/frc/robot/commands/TeleopDriveCommand.java`

This is the default command that runs during teleop. It reads joystick inputs and makes the robot move.

#### The Input Pipeline

```
Raw Joystick (-1 to 1)
     |
     v
Deadband applied in RobotContainer (0.2 for translation, 0.5 for rotation)
     |  Small inputs near center are zeroed out to prevent drift
     |
     v
Cubic response curve: Math.pow(input, 3)
     |  Makes small movements precise and large movements powerful
     |  Example: 50% stick = 12.5% speed, 100% stick = 100% speed
     |
     v
Scale by max velocity (3.75 m/s translation, 3.75 rad/s rotation)
     |
     v
swerveDrive.drive(vx, vy, omega)
```

**Why cubic?** Linear joystick mapping makes the robot feel twitchy at low speeds. Cubing the input gives you a huge "precision zone" near the center while still allowing full speed at the edges.

```
Input:  0.1  0.2  0.3  0.5  0.7  1.0
Linear: 0.10 0.20 0.30 0.50 0.70 1.00
Cubic:  0.001 0.008 0.027 0.125 0.343 1.00
```

#### Speed Modifiers

The command has `speedModifier` and `rotationModifier` fields (both default to 1.0). These are ready for you to add "slow mode" and "turbo mode" buttons:

```java
// Example: Add to configureBindings() in RobotContainer
joystick.button(2).onTrue(Commands.runOnce(() -> driveCommand.setSpeedModifier(0.3)));
joystick.button(2).onFalse(Commands.runOnce(() -> driveCommand.setSpeedModifier(1.0)));
```

### 7.2 TestBangBang

**File:** `src/main/java/frc/robot/commands/TestBangBang.java`

A simple test command for validating that the drivetrain works:

```java
initialize() -> Reset pose to (0, 0, 0)
execute()    -> Drive forward at 2.0 m/s, log X position
isFinished() -> true when X position > 1.0 meter
```

This is useful for:
- Verifying odometry works (does the robot know it moved 1 meter?)
- Checking drive PID tuning (does 2.0 m/s actually produce 2.0 m/s?)
- Testing in simulation before putting code on the real robot

---

## 8. RobotContainer: Wiring It All Together

**File:** `src/main/java/frc/robot/RobotContainer.java`

This is the "glue" class. It doesn't contain logic - it creates things and connects them.

### What It Does

```java
public RobotContainer() {
    // 1. Create subsystems
    swerveDrive = new SwerveDrive();

    // 2. Create vision with appropriate IO and connect to swerve
    vision = new Vision(
        swerveDrive::addVisionMeasurement,  // <-- This connects vision to swerve!
        new VisionIOPhotonVision(camera0Name, robotToCamera0)
    );

    // 3. Set up controls
    swerveDrive.setDefaultCommand(new TeleopDriveCommand(
        swerveDrive,
        () -> -MathUtil.applyDeadband(joystick.getY(), Y_DEADBAND),  // Forward/back
        () -> -MathUtil.applyDeadband(joystick.getX(), Y_DEADBAND),  // Left/right
        () -> -MathUtil.applyDeadband(joystick.getTwist(), ANGLE_JOYSTICK_DEADBAND) // Rotation
    ));

    // 4. Create auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
}
```

**The `swerveDrive::addVisionMeasurement` pattern** is a method reference - it passes the `addVisionMeasurement` method as a callback. This means Vision can send data to SwerveDrive without having a direct reference to it (loose coupling).

**Why the negative signs on joystick axes?** WPILib convention: pushing the joystick forward gives a **negative** Y value. We negate it so "forward on the stick" = "positive X velocity" = "drive forward."

### Adding New Subsystem Controls

When you create a new subsystem, this is where you:
1. Construct it
2. Set its default command (if any)
3. Bind buttons to its commands

```java
// Example: Adding an arm subsystem
Arm arm = new Arm();
arm.setDefaultCommand(new ArmHoldCommand(arm));
joystick.button(3).whileTrue(new ArmToScoreCommand(arm));
joystick.button(4).whileTrue(new ArmToIntakeCommand(arm));
```

---

## 9. Constants: The Single Source of Truth

**File:** `src/main/java/frc/robot/Constants.java`

All "magic numbers" live here. **Never hard-code values in subsystem or command files.** If you need to change a gear ratio, PID value, or CAN ID, you change it in one place.

### Organization

```java
public class Constants {
    public static class Drivetrain { ... }      // Track width, max speed, gyro ID
    public static class Controls { ... }        // Deadbands, heading PID
    public static class SwerveConfig { ... }    // Gear ratios, PID, current limits
    public static class SwerveModules { ... }   // Per-module CAN IDs and offsets
}
```

### Key Values to Know

| Constant | Value | What It Means |
|---|---|---|
| `MAXIMUM_CHASSIS_VELOCITY` | 3.75 m/s | Top robot speed (~12.3 ft/s) |
| `DRIVE_GEAR_RATIO` | 6.75 | Motor spins 6.75x for each wheel revolution (SDS MK4i L2) |
| `ANGLE_GEAR_RATIO` | 21.43 | Motor spins 21.43x for each wheel rotation |
| `WHEEL_DIAMETER_METERS` | 0.095m | Effective diameter (nominal 4" with 0.935 fudge factor) |
| `DRIVE_kP` | 0.000215 | Drive velocity PID proportional gain |
| `ANGLE_kP` | 5.0 | Steering position PID proportional gain |
| `DRIVE_CURRENT_LIMIT` | 40A | Protects NEO motors from overheating |
| `ANGLE_CURRENT_LIMIT` | 20A | Steering needs less current |
| `fudge` | 0.935 | Empirical correction for wheel wear/compression |

**What is the fudge factor?** The nominal wheel diameter is 4 inches (0.1016m), but real wheels compress under the robot's weight and wear down over time. The fudge factor (0.935) was measured by driving a known distance and comparing expected vs actual. If odometry drifts, recalibrate this value.

---

## 10. Autonomous with PathPlanner

PathPlanner is a GUI tool for creating autonomous paths. Our integration works like this:

### Path Creation (in PathPlanner GUI)
1. Draw waypoints on the field
2. Set velocity/acceleration constraints
3. Export to JSON files in `src/main/deploy/pathplanner/paths/`

### Auto Routines (in PathPlanner GUI)
1. Chain paths together with commands
2. Export to JSON files in `src/main/deploy/pathplanner/autos/`

### Code Integration
The `AutoBuilder` in `SwerveDrive.java` tells PathPlanner how to control our robot:
- How to read the current pose
- How to reset the pose (at the start of auto)
- How to command chassis speeds (robot-relative)
- PID controllers for path following

### Current Autos
| Auto Name | What It Does |
|---|---|
| Test Auto | Follows "First Path" |
| Fancy Auto | Follows "Fancy Path" |
| Return Auto | Follows "Second Path" |

All three reset odometry at the start (so the robot knows it starts at the path's first point).

### Adding Commands During Auto

You can add subsystem commands at waypoints. For example, to score a game piece mid-path:
1. In PathPlanner GUI, add a named command at a waypoint
2. In code, register the named command:
```java
NamedCommands.registerCommand("score", new ScoreCommand(mySubsystem));
```

---

## 11. Logging and Debugging with AdvantageKit

### What Gets Logged

Every `updateInputs()` call records sensor data to the log. Our code also manually logs:
- Swerve module actual/desired angles and speeds
- Robot pose (from pose estimator)
- Odometry pose (wheel-only)
- Raw CANcoder values
- Vision accepted/rejected poses and tag detections

### Viewing Logs in AdvantageScope

See the setup instructions in `src/main/java/frc/robot/docs/AdvantageScope.txt`.

Key log paths:
| Path | What It Shows |
|---|---|
| `/Swerve/Pose` | Best position estimate |
| `/Swerve/ActualAnglesDeg` | Where wheels are actually pointing |
| `/Swerve/DesiredAnglesDeg` | Where wheels should be pointing |
| `/Swerve/ActualSpeedsMps` | Actual wheel speeds |
| `/Swerve/DesiredSpeedsMps` | Desired wheel speeds |
| `/Vision/Camera0/AcceptedPoses` | Vision measurements that passed filtering |
| `/Vision/Camera0/RejectedPoses` | Vision measurements that were thrown out |

### Replay Mode

One of AdvantageKit's best features: you can replay a match log on your laptop. Change your subsystem logic, replay the same data, and see if the behavior improves - without touching the robot.

In `Robot.java`, the replay mode is configured:
```java
case REPLAY:
    setUseTiming(false);  // Run as fast as possible
    String logPath = LogFileUtil.findReplayLog();
    Logger.setReplaySource(new WPILOGReader(logPath));
    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
```

---

## 12. How to Add a New Subsystem (Step-by-Step Guide)

Let's walk through adding a hypothetical "Elevator" subsystem. Follow this pattern for any new mechanism.

### Step 1: Define Constants

```java
// Constants.java
public static class Elevator {
    public static final int MOTOR_ID = 41;
    public static final String CAN_BUS = "rio";
    public static final double GEAR_RATIO = 10.0;
    public static final double SPOOL_DIAMETER_METERS = 0.04;
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double MAX_HEIGHT_METERS = 1.5;
    public static final double MIN_HEIGHT_METERS = 0.0;
    public static final int CURRENT_LIMIT = 40;
}
```

### Step 2: Create the IO Interface

```java
// subsystems/elevator/ElevatorIO.java
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}
    default void setPosition(double heightMeters) {}
    default void stop() {}
}
```

### Step 3: Create the Real Implementation

```java
// subsystems/elevator/ElevatorIOReal.java
public class ElevatorIOReal implements ElevatorIO {
    private final SparkMax motor;

    public ElevatorIOReal() {
        motor = new SparkMax(Constants.Elevator.MOTOR_ID, MotorType.kBrushless);
        configureMotor();
    }

    private void configureMotor() {
        // Configure PID, current limits, conversion factors, etc.
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = motor.getEncoder().getPosition();
        inputs.velocityMetersPerSecond = motor.getEncoder().getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setPosition(double heightMeters) {
        motor.getClosedLoopController().setReference(heightMeters, ControlType.kPosition);
    }

    @Override
    public void stop() {
        motor.set(0);
    }
}
```

### Step 4: Create the Sim Implementation

```java
// subsystems/elevator/ElevatorIOSim.java
public class ElevatorIOSim implements ElevatorIO {
    private double simPosition = 0.0;
    private double simVelocity = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Simple physics: integrate velocity for position
        simPosition += simVelocity * 0.02;
        inputs.positionMeters = simPosition;
        inputs.velocityMetersPerSecond = simVelocity;
    }

    @Override
    public void setPosition(double heightMeters) {
        // Simplified: move toward target
        simVelocity = (heightMeters - simPosition) * 5.0;
    }

    @Override
    public void stop() {
        simVelocity = 0.0;
    }
}
```

### Step 5: Create the Subsystem

```java
// subsystems/elevator/Elevator.java
public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator() {
        io = RobotBase.isSimulation() ? new ElevatorIOSim() : new ElevatorIOReal();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setHeight(double meters) {
        io.setPosition(MathUtil.clamp(meters, MIN_HEIGHT_METERS, MAX_HEIGHT_METERS));
    }

    public double getHeight() {
        return inputs.positionMeters;
    }

    public void stop() {
        io.stop();
    }
}
```

### Step 6: Create Commands

```java
// commands/ElevatorToHeightCommand.java
public class ElevatorToHeightCommand extends Command {
    private final Elevator elevator;
    private final double targetHeight;

    public ElevatorToHeightCommand(Elevator elevator, double targetHeight) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setHeight(targetHeight);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getHeight() - targetHeight) < 0.02; // 2cm tolerance
    }
}
```

### Step 7: Wire It Up in RobotContainer

```java
// RobotContainer.java
Elevator elevator = new Elevator();

// Button bindings
joystick.button(3).whileTrue(new ElevatorToHeightCommand(elevator, 1.0));  // High position
joystick.button(4).whileTrue(new ElevatorToHeightCommand(elevator, 0.0));  // Low position
```

That's it. The new subsystem will automatically:
- Have its `periodic()` called every 20ms
- Log all inputs via AdvantageKit
- Work in simulation
- Be controllable via commands

---

## 13. Key Vendor Libraries Reference

| Library | What It Does | Where We Use It |
|---|---|---|
| **WPILib** | Core FRC framework (kinematics, pose estimation, command scheduler) | Everywhere |
| **AdvantageKit** | Logging, replay, `@AutoLog` | Robot.java, all IO layers |
| **REVLib** | SparkMax motor controller API | SwerveModuleIOReal |
| **Phoenix6** | Pigeon2 gyro and CANcoder API | GyroPigeon, SwerveModuleIOReal |
| **PhotonLib** | PhotonVision camera API | VisionIOPhotonVision, VisionIOPhotonVisionSim |
| **PathPlanner** | Autonomous path creation and following | SwerveDrive (AutoBuilder), Robot.java (warmup) |

---

## 14. Common Pitfalls and Debugging Tips

### Swerve Modules Pointing Wrong Directions
**Cause:** CANcoder offsets are incorrect.
**Fix:** With the wheels pointed straight forward, read each CANcoder's raw value in AdvantageScope or SmartDashboard. Update the offsets in `Constants.SwerveModules`.

### Robot Drives in Circles / Crabs Sideways
**Cause:** One or more modules have swapped CAN IDs or inverted motors.
**Fix:** Test each module individually. Verify CAN IDs match the physical wiring. Check `driveInverted` and `angleInverted` settings.

### Odometry Drifts Significantly
**Cause:** The wheel diameter fudge factor is off, or wheels are worn.
**Fix:** Drive a known distance (use tape on the floor). Compare actual vs reported distance. Adjust `Constants.SwerveConfig.fudge`.

### Vision Pose Jumps Around
**Cause:** Ambiguous single-tag detections or bad camera calibration.
**Fix:** Check the rejection statistics in AdvantageScope (`/Vision/Camera0/RejectedPoses`). If many are rejected for ambiguity, your camera calibration may need improvement. You can also tighten `maxAmbiguity` in `VisionConstants`.

### Robot Doesn't Move in Auto
**Cause:** PathPlanner's `AutoBuilder` wasn't configured, or the pose wasn't reset at the start.
**Fix:** Verify `AutoBuilder.configure()` runs in the `SwerveDrive` constructor. Ensure the auto routine includes "Reset Odometry" at the start.

### "CAN Timeout" Errors in Console
**Cause:** A device on the CAN bus isn't responding (wrong ID, bad wiring, or device not powered).
**Fix:** Use Phoenix Tuner X and REV Hardware Client to scan the CAN bus and verify all devices are present with the expected IDs.

### Code Works in Sim but Not on Real Robot
**Cause:** The IO layer pattern is your friend here. The sim implementation may be too simplified.
**Fix:** Compare the sim and real IO implementations. Check that motor configurations (inversion, current limits, PID) are set correctly in the real implementation.

---

*Last updated: February 2026 | Team 6637 Betawolves*
