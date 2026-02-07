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
15. [Hardware-to-Code Guide: Every Part on Our Robot](#15-hardware-to-code-guide-every-part-on-our-robot)
    - [15.1 REV NEO Motor (REV-21-1650) + SparkMax](#151-rev-neo-motor-rev-21-1650--sparkmax)
    - [15.2 REV NEO Vortex Motor (REV-21-1652) + SparkFlex](#152-rev-neo-vortex-motor-rev-21-1652--sparkflex)
    - [15.3 SparkMax vs SparkFlex: When to Use Which](#153-sparkmax-vs-sparkflex-when-to-use-which)
    - [15.4 CTRE CANcoder (Absolute Encoder)](#154-ctre-cancoder-absolute-encoder)
    - [15.5 CTRE Pigeon2 (Gyro/IMU)](#155-ctre-pigeon2-gyroimu)
    - [15.6 REV Through-Bore Encoder](#156-rev-through-bore-encoder)
    - [15.7 External Encoders (DIO Port)](#157-external-encoders-dio-port)
    - [15.8 Integrated Motor Encoders (Built Into NEO/Vortex)](#158-integrated-motor-encoders-built-into-neovortex)
16. [Our Robot's Full Hardware Map](#16-our-robots-full-hardware-map)
    - [16.1 Complete Motor and Sensor Inventory](#161-complete-motor-and-sensor-inventory)
    - [16.2 Chassis (Swerve Drive) - Already Coded](#162-chassis-swerve-drive---already-coded)
    - [16.3 Intake - To Be Coded](#163-intake---to-be-coded)
    - [16.4 Indexer and Kicker - To Be Coded](#164-indexer-and-kicker---to-be-coded)
    - [16.5 Shooter (Turret + Flywheels + Hood) - To Be Coded](#165-shooter-turret--flywheels--hood---to-be-coded)
    - [16.6 Climber - To Be Coded](#166-climber---to-be-coded)
17. [PID Control Modes Explained (For Every Motor Situation)](#17-pid-control-modes-explained-for-every-motor-situation)
18. [Conversion Factor Cookbook](#18-conversion-factor-cookbook)

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

## 15. Hardware-to-Code Guide: Every Part on Our Robot

> **This is the section for the question: "I'm holding this motor/sensor. How do I write code for it?"**
>
> For every physical component on our robot, this section shows you the exact Java class, the import, how to construct it, how to configure it, and working code examples based on what we actually use. See also the full parts inventory in [our design guide](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md).

### The Big Picture: Physical Part to Java Object

```
PHYSICAL WORLD                     CODE WORLD                          LIBRARY
-------------------------------    --------------------------------    -----------
REV NEO motor                 -->  SparkMax object                     REVLib
REV NEO Vortex motor          -->  SparkFlex object                    REVLib
CTRE CANcoder                 -->  CANcoder object                     Phoenix6
CTRE Pigeon2 gyro             -->  Pigeon2 object                      Phoenix6
REV Through-Bore Encoder      -->  DutyCycleEncoder object             WPILib
External quadrature encoder   -->  Encoder object                      WPILib
Any motor's built-in encoder  -->  motor.getEncoder() (already there)  REVLib
PhotonVision camera           -->  PhotonCamera object                 PhotonLib
Limelight camera              -->  NetworkTable entries                 WPILib NT
```

The pattern is always the same:
1. **Find the CAN ID** (or DIO port) of the physical device
2. **Create a Java object** using that ID
3. **Configure it** (current limits, PID, inversion, conversion factors)
4. **Read from it** (get position, velocity, angle)
5. **Write to it** (set speed, set position target)

---

### 15.1 REV NEO Motor (REV-21-1650) + SparkMax

**What it is physically:** A brushless motor made by REV Robotics. It has a built-in encoder (Hall sensor) that tracks rotations. It plugs into a **SparkMax** motor controller via a data cable. The SparkMax connects to the CAN bus.

**We have 14 of these** on our robot (8 for swerve, 1 intake rotator, 1 intake rollers, 1 indexer, 1 turret, 1 shooter flywheel, 1 hood).

#### The Physical Wiring Chain
```
Battery --> PDP --> SparkMax motor controller --> NEO motor
                        |
                    CAN bus wire (connects to RoboRIO)
```

The SparkMax is the "translator" between your code and the motor. Your code talks to the SparkMax over CAN. The SparkMax does the actual electrical work of spinning the motor.

#### Creating It in Code

```java
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Constructor: new SparkMax(CAN_ID, motorType)
// The CAN_ID must match what's set in REV Hardware Client
// MotorType.kBrushless because NEO is brushless
SparkMax motor = new SparkMax(12, MotorType.kBrushless);
```

**Where does CAN ID 12 come from?** You set it using the REV Hardware Client software. Each SparkMax on the CAN bus needs a unique ID. Our convention:
- 11-13: Front-left module (angle=11, drive=12, encoder=13)
- 21-23: Front-right module
- 31-33: Back-left module
- 37-39: Back-right module
- Other subsystem IDs are TBD (see the [design guide](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md#can-bus-device-ids))

#### Configuring It (REVLib 2025+ Config API)

The 2025/2026 REVLib uses a **config object pattern** where you build up all settings, then apply them in one shot:

```java
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

SparkMaxConfig config = new SparkMaxConfig();

// --- Basic motor settings ---
config
    .inverted(true)                      // Reverse motor direction (depends on mounting)
    .idleMode(IdleMode.kBrake)           // kBrake = resist movement when not powered
                                         // kCoast = spin freely when not powered
    .smartCurrentLimit(40)               // Max amps the motor can draw (protects it)
    .openLoopRampRate(0.25);             // Seconds from 0 to full power (smooths acceleration)

// --- Built-in encoder settings ---
config.encoder
    .positionConversionFactor(0.04423)   // Converts motor rotations to YOUR units (e.g., meters)
    .velocityConversionFactor(0.000737); // Converts RPM to YOUR units (e.g., m/s)

// --- PID controller (runs on the SparkMax, not RoboRIO) ---
config.closedLoop
    .p(0.000215)                         // Proportional gain
    .i(0.0)                              // Integral gain
    .d(0.0)                              // Derivative gain
    .outputRange(-1, 1);                 // Min/max motor output (-1 to 1)

// --- Apply all settings at once ---
motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//                       ^ Reset to defaults first        ^ Save to flash (survives power cycle)
```

#### Controlling It

There are three main ways to control a SparkMax:

**1. Simple Percent Output (Open Loop) - "Just spin"**
```java
motor.set(0.5);   // 50% power forward
motor.set(-0.3);  // 30% power backward
motor.set(0.0);   // Stop
```
Use this for: Intake rollers, indexer feed - anything where you just want it spinning, and precise speed/position doesn't matter.

**2. Velocity PID (Closed Loop) - "Maintain this exact speed"**
```java
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

SparkClosedLoopController pid = motor.getClosedLoopController();
pid.setReference(2000, ControlType.kVelocity);  // Target: 2000 RPM
```
Use this for: Swerve drive motors, shooter flywheels - anything where you need consistent speed regardless of battery voltage or load.

**3. Position PID (Closed Loop) - "Go to this exact position and hold"**
```java
pid.setReference(5.0, ControlType.kPosition);  // Target: 5.0 (in your converted units)
```
Use this for: Swerve steering, turret rotation, hood angle - anything that needs to go to a specific angle/position and stay there.

#### Reading the Built-in Encoder

Every NEO motor has a Hall-effect encoder built in. You access it through the SparkMax:

```java
import com.revrobotics.RelativeEncoder;

RelativeEncoder encoder = motor.getEncoder();

double position = encoder.getPosition();    // In your converted units (meters, rotations, etc.)
double velocity = encoder.getVelocity();    // In your converted units per minute
encoder.setPosition(0.0);                   // Reset position to zero
```

**Important:** This is a **relative** encoder - it counts from zero when the robot powers on. It does NOT remember its position across power cycles. That's why we use CANcoders (absolute) on the swerve modules to seed this encoder on startup.

#### Reading Motor Status

```java
double outputPercent = motor.getAppliedOutput();  // -1.0 to 1.0
double currentAmps = motor.getOutputCurrent();     // How hard the motor is working
double busVoltage = motor.getBusVoltage();          // Battery voltage at the controller
double temperature = motor.getMotorTemperature();   // Motor temp in Celsius
```

These are great for logging and debugging. High current can indicate the mechanism is stalled or binding. Temperature warnings can prevent motor damage.

#### Complete Real-World Example: How Our Swerve Drive Motor is Set Up

This is exactly what `SwerveModuleIOReal.java` does for each swerve drive motor:

```java
// 1. Create the motor object
SparkMax driveMotor = new SparkMax(constants.driveMotorId, MotorType.kBrushless);

// 2. Build the configuration
SparkMaxConfig driveConfig = new SparkMaxConfig();
driveConfig
    .inverted(constants.driveInverted)     // true for all our modules
    .idleMode(IdleMode.kBrake)             // Brake so robot doesn't slide
    .smartCurrentLimit(40)                 // 40A limit
    .openLoopRampRate(0.25);               // Smooth acceleration

driveConfig.encoder
    .positionConversionFactor(DRIVE_POSITION_CONVERSION)   // motor rotations -> meters
    .velocityConversionFactor(DRIVE_VELOCITY_CONVERSION);  // RPM -> m/s

driveConfig.closedLoop
    .p(0.000215)
    .outputRange(-1, 1);

// 3. Apply
driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// 4. During operation (called every 20ms):
//    Convert desired m/s to RPM, then command velocity PID
double desiredRPM = desiredSpeedMetersPerSecond / DRIVE_VELOCITY_CONVERSION;
driveMotor.getClosedLoopController().setReference(desiredRPM, ControlType.kVelocity);
```

---

### 15.2 REV NEO Vortex Motor (REV-21-1652) + SparkFlex

**What it is physically:** REV's higher-power brushless motor. Faster and more powerful than the standard NEO. It plugs into a **SparkFlex** motor controller (NOT a SparkMax).

**We have 4 of these** on our robot (1 kicker, 2 shooter flywheels, 1 climber).

#### NEO vs NEO Vortex - Physical Differences

| | NEO | NEO Vortex |
|---|---|---|
| Part Number | REV-21-1650 | REV-21-1652 |
| Motor Controller | **SparkMax** | **SparkFlex** |
| Free Speed | 5,676 RPM | 6,784 RPM |
| Stall Torque | 2.6 Nm | 3.6 Nm |
| Weight | 0.425 kg | 0.348 kg |
| Best for | General purpose | High-performance (shooters, climbers) |

**The critical thing:** NEO Vortex requires a **SparkFlex**, not a SparkMax. If you plug a Vortex into a SparkMax, it won't work. The code difference is just the class name.

#### Creating It in Code

```java
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// SparkFlex instead of SparkMax - that's the only construction difference!
SparkFlex motor = new SparkFlex(41, MotorType.kBrushless);
```

#### Configuring It

SparkFlex uses `SparkFlexConfig` instead of `SparkMaxConfig`. Otherwise the API is nearly identical:

```java
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

SparkFlexConfig config = new SparkFlexConfig();
config
    .inverted(false)
    .idleMode(IdleMode.kCoast)           // Flywheels usually coast (less wear on belts)
    .smartCurrentLimit(60)               // Vortex can handle higher current than NEO
    .openLoopRampRate(0.1);              // Faster ramp for shooter responsiveness

config.encoder
    .positionConversionFactor(1.0)       // Keep as rotations for a flywheel
    .velocityConversionFactor(1.0);      // Keep as RPM for a flywheel

config.closedLoop
    .p(0.0005)                           // Tune for your specific mechanism
    .i(0.0)
    .d(0.0)
    .outputRange(-1, 1);

motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```

#### Controlling and Reading - Same as SparkMax

Everything else works the same way:

```java
// Simple percent output
motor.set(0.8);  // 80% power

// Velocity PID
motor.getClosedLoopController().setReference(4500, ControlType.kVelocity);  // 4500 RPM

// Read encoder
double rpm = motor.getEncoder().getVelocity();
double amps = motor.getOutputCurrent();
```

#### Example: Shooter Flywheel (How You'd Code It)

Based on our parts list (2x NEO Vortex flywheels + 1x NEO flywheel):

```java
// In ShooterIOReal.java constructor:
SparkFlex topFlywheel = new SparkFlex(FLYWHEEL_1_ID, MotorType.kBrushless);
SparkFlex bottomFlywheel = new SparkFlex(FLYWHEEL_2_ID, MotorType.kBrushless);
SparkMax sideFlywheel = new SparkMax(FLYWHEEL_3_ID, MotorType.kBrushless);

// Configure top flywheel
SparkFlexConfig topConfig = new SparkFlexConfig();
topConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(60);
topConfig.closedLoop.p(0.0005);
topFlywheel.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// Make bottom flywheel follow top (spin same speed, opposite direction)
SparkFlexConfig bottomConfig = new SparkFlexConfig();
bottomConfig
    .follow(topFlywheel, true);  // true = inverted (spins opposite direction)
bottomFlywheel.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// Now commanding topFlywheel automatically commands bottomFlywheel too!
topFlywheel.getClosedLoopController().setReference(4500, ControlType.kVelocity);

// Check if flywheel is up to speed (for "ready to shoot" logic)
boolean atSpeed = Math.abs(topFlywheel.getEncoder().getVelocity() - 4500) < 100;  // within 100 RPM
```

**Leader/Follower pattern:** When two motors need to spin together (like flywheels on opposite sides of a shooter), you configure one as a leader and the other as a follower. The follower automatically mirrors the leader - you only command one motor.

---

### 15.3 SparkMax vs SparkFlex: When to Use Which

This is a common point of confusion. Here's the simple rule:

```
NEO motor         --> SparkMax controller  --> SparkMax Java class
NEO Vortex motor  --> SparkFlex controller --> SparkFlex Java class
```

| | SparkMax | SparkFlex |
|---|---|---|
| **Java class** | `com.revrobotics.spark.SparkMax` | `com.revrobotics.spark.SparkFlex` |
| **Config class** | `SparkMaxConfig` | `SparkFlexConfig` |
| **Works with** | NEO, NEO 550 | NEO Vortex |
| **Max continuous current** | 40A typical | 60A typical |
| **On our robot** | 14 (all NEO motors) | 4 (all Vortex motors) |

**They share the same base API.** Both extend `SparkBase`, so methods like `.set()`, `.getEncoder()`, `.getClosedLoopController()`, `.getOutputCurrent()` work identically on both. The only differences are the class name and config class name.

#### Quick Reference: Which Subsystem Uses Which

Based on [our design guide](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md#motor--sensor-inventory):

| Subsystem | Component | Motor | Controller | Java Class |
|---|---|---|---|---|
| Chassis | Swerve Drive (x8) | NEO | SparkMax | `SparkMax` |
| Intake | Rotator | NEO | SparkMax | `SparkMax` |
| Intake | Rollers | NEO | SparkMax | `SparkMax` |
| Indexer | Feed | NEO | SparkMax | `SparkMax` |
| Kicker | Feed to Shooter | NEO Vortex | SparkFlex | `SparkFlex` |
| Shooter | Turret Rotation | NEO | SparkMax | `SparkMax` |
| Shooter | Flywheel (x2) | NEO Vortex | SparkFlex | `SparkFlex` |
| Shooter | Flywheel (x1) | NEO | SparkMax | `SparkMax` |
| Shooter | Hood | NEO | SparkMax | `SparkMax` |
| Climber | Extend/Retract | NEO Vortex | SparkFlex | `SparkFlex` |

---

### 15.4 CTRE CANcoder (Absolute Encoder)

**What it is physically:** A magnetic absolute encoder made by CTRE (Cross The Road Electronics). It attaches to a rotating shaft and always knows the exact angle, even after power cycling. It communicates over CAN bus.

**We have 4** on our robot (one per swerve module).

#### Why "Absolute" Matters

```
RELATIVE encoder (built into NEO):
  Power on  --> position = 0.0 (always starts at zero)
  Spin 90 degrees --> position = 0.25 rotations
  Power off and back on --> position = 0.0 (forgot everything!)

ABSOLUTE encoder (CANcoder):
  Power on  --> position = 0.73 rotations (knows exactly where the shaft is)
  Spin 90 degrees --> position = 0.98 rotations
  Power off and back on --> position = 0.98 rotations (remembers!)
```

This is why we use CANcoders on swerve modules - when the robot powers on, we need to know which way each wheel is already pointing.

#### Creating It in Code

```java
import com.ctre.phoenix6.hardware.CANcoder;

// Constructor: new CANcoder(CAN_ID, "canBusName")
CANcoder absoluteEncoder = new CANcoder(13, "rio");
//                                      ^    ^
//                              CAN ID  |    CAN bus name ("rio" = the RoboRIO's built-in bus)
```

#### Configuring It

```java
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

CANcoderConfiguration config = new CANcoderConfiguration();

config.MagnetSensor
    .withAbsoluteSensorDiscontinuityPoint(0.5)  // Range: [-0.5, 0.5) rotations = [-180, 180) degrees
    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
    //                   ^ Which way is "positive" rotation

absoluteEncoder.getConfigurator().apply(config);
```

#### Reading It

```java
// Get the current absolute position (in rotations, -0.5 to 0.5)
double positionRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();

// Convert to degrees if you prefer
double positionDegrees = positionRotations * 360.0;

// Convert to a WPILib Rotation2d (commonly used throughout the codebase)
Rotation2d angle = Rotation2d.fromRotations(positionRotations);
```

#### How We Use It: The Startup Seed Procedure

This is the critical workflow in `SwerveModuleIOReal.java`:

```java
// Step 1: Read the CANcoder's absolute position
double absoluteAngle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();

// Step 2: Subtract this module's offset (the reading when the wheel is straight forward)
double adjustedAngle = absoluteAngle - constants.angleOffset.getRotations();

// Step 3: Seed the NEO's relative encoder with this value
angleMotor.getEncoder().setPosition(adjustedAngle);

// Now the relative encoder "knows" where the wheel is pointing
// and we use it for all future control (it's faster and smoother)
```

**Why not just use the CANcoder for everything?** The CANcoder updates at ~100Hz over CAN, which introduces latency. The NEO's built-in encoder updates at the SparkMax's internal rate (much faster). For the tight PID control loop that keeps the swerve wheel pointed correctly, we need the faster encoder. The CANcoder is only needed once at startup to tell the relative encoder its starting position.

---

### 15.5 CTRE Pigeon2 (Gyro/IMU)

**What it is physically:** An Inertial Measurement Unit (IMU) made by CTRE. It contains gyroscopes and accelerometers that measure rotation and tilt. Mounted flat on the robot chassis.

**We have 1** on CAN ID 9.

#### What It Measures

```
               TOP VIEW
                 ^
                 | Yaw (heading) - which direction
                 | the robot is facing on the field
                 |
    <--Roll------+------Pitch-->
                 |
                 v

Yaw:   rotation around vertical axis (turning left/right)   <-- THIS IS WHAT WE USE
Pitch: tilting forward/backward
Roll:  tilting left/right
```

For a flat-driving robot, **yaw** is the only one we care about. It tells us which direction the robot is facing, which is essential for field-relative driving.

#### Creating and Using It

```java
import com.ctre.phoenix6.hardware.Pigeon2;

// In GyroPigeon.java:
Pigeon2 gyro = new Pigeon2(9, "rio");  // CAN ID 9, on the "rio" CAN bus

// Get current heading
Rotation2d heading = gyro.getRotation2d();       // As a WPILib Rotation2d
double yawDegrees = gyro.getYaw().getValueAsDouble();  // Raw degrees (can exceed 360)

// Reset heading to zero (call this when the robot is lined up with the field)
gyro.reset();
```

#### Why Resetting the Gyro Matters

The gyro doesn't know which way the field is - it only knows how far it's rotated since the last reset. If you reset the gyro when the robot is facing the far wall, then "0 degrees" means "facing the far wall" for the rest of the match.

In our code, `SwerveDrive.zeroHeading()` calls `gyro.zeroYaw()`. This is called during initialization and can be called via a button binding for driver convenience.

---

### 15.6 REV Through-Bore Encoder

**What it is physically:** An absolute encoder made by REV Robotics. It has a hollow center so a shaft passes through it. It plugs into the RoboRIO's **DIO (Digital Input/Output)** ports, NOT the CAN bus.

**We have 1** on our robot (on the turret).

#### CAN Bus vs DIO - What's the Difference?

```
CAN bus devices (SparkMax, CANcoder, Pigeon2):
  - Communicate via CAN wire daisy chain
  - Identified by CAN ID number
  - Can have many devices on one bus

DIO port devices (Through-Bore Encoder, limit switches, beam breaks):
  - Plug directly into the RoboRIO's DIO pins (ports 0-9)
  - Identified by DIO port number
  - One device per port
```

#### Creating It in Code

The Through-Bore Encoder outputs a **duty cycle** signal (PWM), so we use WPILib's `DutyCycleEncoder`:

```java
import edu.wpi.first.wpilibj.DutyCycleEncoder;

// Constructor: new DutyCycleEncoder(DIO_port_number)
DutyCycleEncoder turretEncoder = new DutyCycleEncoder(0);  // Connected to DIO port 0
```

#### Configuring and Reading

```java
// Set the range for the encoder output
// Through-Bore outputs 0 to 1 (one full rotation)
// We map this to a useful range:
turretEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

// Read the absolute position (0.0 to 1.0 = one full rotation)
double rawPosition = turretEncoder.get();  // Returns position in rotations

// Convert to radians for math
double positionRadians = rawPosition * 2.0 * Math.PI;

// Check if the encoder is connected and getting valid data
boolean connected = turretEncoder.isConnected();
```

#### Example: Turret Position Sensing

Based on [our design guide](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md#turret-subsystem), the turret has a NEO motor for rotation and a Through-Bore Encoder for absolute position:

```java
// In TurretIOReal.java:
private final SparkMax turretMotor;
private final DutyCycleEncoder absoluteEncoder;

public TurretIOReal() {
    turretMotor = new SparkMax(TURRET_MOTOR_ID, MotorType.kBrushless);
    absoluteEncoder = new DutyCycleEncoder(TURRET_ENCODER_DIO_PORT);

    // Configure the motor
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)         // Hold position when not powered
        .smartCurrentLimit(30)             // Turret doesn't need full 40A
        .inverted(false);

    // Convert motor rotations to turret rotations using gear ratio
    config.encoder
        .positionConversionFactor(1.0 / TURRET_GEAR_RATIO);  // motor rotations -> turret rotations

    config.closedLoop
        .p(TURRET_kP);

    turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Seed the relative encoder from the absolute encoder (same pattern as swerve!)
    double absolutePosition = absoluteEncoder.get();
    turretMotor.getEncoder().setPosition(absolutePosition / TURRET_GEAR_RATIO);
}

// Command the turret to a position
public void setPosition(double targetRotations) {
    turretMotor.getClosedLoopController().setReference(targetRotations, ControlType.kPosition);
}

// Read current position (for logging and aim calculations)
public double getPosition() {
    return turretMotor.getEncoder().getPosition();  // In turret rotations
}
```

**Note about the turret zero position:** Per our design guide, the turret is zeroed to the **back** of the robot, so code needs to subtract PI when converting to robot-relative angles. See the [turret aim math](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md#how-to-aim-the-math) in the design guide.

---

### 15.7 External Encoders (DIO Port)

**What it is physically:** A separate encoder (not built into the motor) that connects to the RoboRIO's DIO ports. Used when you need position feedback from a point in the mechanism that isn't at the motor shaft (e.g., after gearing).

**We have 1** on our robot (intake rotator).

#### Types of External Encoders

| Type | Signal | WPILib Class | Wiring |
|---|---|---|---|
| **Quadrature** | A + B channels | `Encoder` | 2 DIO ports |
| **Duty Cycle (PWM)** | Single PWM pulse | `DutyCycleEncoder` | 1 DIO port |
| **Analog** | 0-5V signal | `AnalogEncoder` | 1 Analog port |

#### Quadrature Encoder Example (Two-Channel)

```java
import edu.wpi.first.wpilibj.Encoder;

// Uses TWO DIO ports (channel A and channel B)
Encoder encoder = new Encoder(0, 1);     // DIO port 0 = channel A, DIO port 1 = channel B
encoder.setReverseDirection(false);       // Flip if counting backward

// Set distance per pulse to convert raw counts to useful units
// Example: 256 counts per revolution, mechanism moves 0.01m per revolution
encoder.setDistancePerPulse(0.01 / 256.0);

// Reading
double distance = encoder.getDistance();  // In your converted units
double rate = encoder.getRate();          // Units per second
int rawCounts = encoder.get();            // Raw encoder counts
encoder.reset();                          // Reset count to zero
```

#### Example: Intake Rotator

The intake has a NEO motor (SparkMax) + external encoder to track the arm position:

```java
// In IntakeIOReal.java:
private final SparkMax rotatorMotor;
private final Encoder rotatorEncoder;       // External encoder for position
private final SparkMax rollerMotor;

public IntakeIOReal() {
    rotatorMotor = new SparkMax(ROTATOR_MOTOR_ID, MotorType.kBrushless);
    rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);
    rotatorEncoder = new Encoder(ROTATOR_ENCODER_A_PORT, ROTATOR_ENCODER_B_PORT);

    // Convert encoder counts to radians
    rotatorEncoder.setDistancePerPulse((2.0 * Math.PI) / COUNTS_PER_REVOLUTION);

    // Configure rotator motor for position control
    SparkMaxConfig rotatorConfig = new SparkMaxConfig();
    rotatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    rotatorMotor.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure roller motor for simple speed control
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

// Deploy/retract uses position from external encoder
public void deploy() {
    // Use WPILib PIDController since encoder is NOT on the SparkMax
    // (can't use SparkMax's onboard PID with an external encoder)
    rotatorMotor.set(pidController.calculate(rotatorEncoder.getDistance(), DEPLOY_POSITION));
}

// Rollers are simple open-loop control
public void runRollers(double speed) {
    rollerMotor.set(speed);
}
```

**When to use an external encoder vs the motor's built-in encoder:**
- **Motor's built-in encoder:** Good enough when the motor connects directly to the mechanism (or through a gearbox). Simpler wiring.
- **External encoder:** Needed when you want position feedback at the mechanism's output shaft (after belts/chains that might slip), or when you need absolute position that survives power cycles.

---

### 15.8 Integrated Motor Encoders (Built Into NEO/Vortex)

**What it is physically:** A Hall-effect sensor built into every NEO and NEO Vortex motor. It's always there - you don't add it separately.

**We have 18** - one per motor (every NEO and Vortex has one built in).

#### How They Work

Inside the motor, magnets on the rotor pass by Hall-effect sensors. The SparkMax/SparkFlex counts these transitions to track rotation. The resolution is 42 counts per revolution for the NEO.

#### Accessing Them

You never create a separate object - you access the encoder through the motor controller:

```java
SparkMax motor = new SparkMax(12, MotorType.kBrushless);
RelativeEncoder encoder = motor.getEncoder();

// These values are in whatever units your conversion factors define
double position = encoder.getPosition();   // Default: motor rotations
double velocity = encoder.getVelocity();   // Default: RPM

// Reset to a known value (used when seeding from an absolute encoder)
encoder.setPosition(0.0);
```

#### Making the Encoder Report in Useful Units

Raw encoder values are in motor rotations and RPM. That's almost never what you want. **Conversion factors** translate to meaningful units:

```java
// Example conversions for different mechanisms:

// SWERVE DRIVE WHEEL: motor rotations -> meters traveled
config.encoder.positionConversionFactor(WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO);
// If wheel circumference = 0.3m and gear ratio = 6.75:
//   1 motor rotation = 0.3 / 6.75 = 0.0444 meters

// SWERVE STEERING: motor rotations -> wheel rotations
config.encoder.positionConversionFactor(1.0 / ANGLE_GEAR_RATIO);
// If gear ratio = 21.43:
//   1 motor rotation = 1/21.43 = 0.0467 wheel rotations

// SHOOTER FLYWHEEL: keep as RPM (no conversion needed)
config.encoder.velocityConversionFactor(1.0);

// ELEVATOR: motor rotations -> meters of height
config.encoder.positionConversionFactor(SPOOL_CIRCUMFERENCE / GEAR_RATIO);

// TURRET: motor rotations -> turret rotations (or radians)
config.encoder.positionConversionFactor(1.0 / TURRET_GEAR_RATIO);
// For radians instead:
config.encoder.positionConversionFactor(2.0 * Math.PI / TURRET_GEAR_RATIO);
```

#### The Conversion Factor Formula

```
positionConversionFactor = (output unit per mechanism revolution) / (gear ratio)

Examples:
  Wheel:    meters per wheel revolution / gear ratio = circumference / ratio
  Elevator: meters per spool revolution / gear ratio = spool_circumference / ratio
  Turret:   1 turret revolution / gear ratio         = 1.0 / ratio
  Arm:      radians per arm revolution / gear ratio   = 2*PI / ratio
```

---

## 16. Our Robot's Full Hardware Map

> This section maps every physical part on our 2026 robot to the exact code you'd write for it. For the full parts inventory and planned subsystem designs, see [000-robot-subsystems-design-guide.md](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md).

### 16.1 Complete Motor and Sensor Inventory

| # | Subsystem | Component | Motor | Controller Class | Encoder | Control Mode |
|---|---|---|---|---|---|---|
| 1-8 | Chassis | Swerve Drive + Steer (x4 modules) | 8x NEO | `SparkMax` | Integrated + CANcoder | Velocity PID (drive), Position PID (steer) |
| 9 | Intake | Rotator | NEO | `SparkMax` | External Encoder | Position PID |
| 10 | Intake | Rollers | NEO | `SparkMax` | None needed | Percent output |
| 11 | Indexer | Feed | NEO | `SparkMax` | None needed | Percent output |
| 12 | Kicker | Feed to Shooter | NEO Vortex | `SparkFlex` | None needed | Percent output |
| 13 | Shooter | Turret Rotation | NEO | `SparkMax` | Through-Bore | Position PID |
| 14-15 | Shooter | Flywheel (x2) | NEO Vortex | `SparkFlex` | Integrated | Velocity PID |
| 16 | Shooter | Flywheel (x1) | NEO | `SparkMax` | Integrated | Velocity PID |
| 17 | Shooter | Hood | NEO | `SparkMax` | Integrated | Position PID |
| 18 | Climber | Extend/Retract | NEO Vortex | `SparkFlex` | Integrated | Position PID / Percent output |

**Plus sensors:**
- 1x Pigeon2 gyro (CAN ID 9) -> `Pigeon2` class
- 4x CANcoder (CAN IDs 13, 23, 33, 39) -> `CANcoder` class
- 1x Through-Bore Encoder (turret, DIO port TBD) -> `DutyCycleEncoder` class
- 1x External Encoder (intake rotator, DIO ports TBD) -> `Encoder` class

### 16.2 Chassis (Swerve Drive) - Already Coded

**Status:** Complete and working in `subsystems/swerve/`

This is fully documented in [Section 5](#5-subsystem-deep-dive-swerve-drive). Quick summary of hardware-to-code:

```
Physical                        Code                            File
-----------------------------   ----------------------------    --------------------------
NEO (drive)    + SparkMax   --> SparkMax driveMotor             SwerveModuleIOReal.java
NEO (steer)    + SparkMax   --> SparkMax angleMotor             SwerveModuleIOReal.java
CTRE CANcoder               --> CANcoder absoluteEncoder        SwerveModuleIOReal.java
CTRE Pigeon2                --> Pigeon2 gyro                    GyroPigeon.java
```

### 16.3 Intake - To Be Coded

**Hardware:** 1x NEO (rotator) + 1x NEO (rollers) + 1x external encoder

Per the [design guide](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md#intake-subsystem), the intake has a rotator arm that deploys/retracts and rollers that spin to grab game pieces.

```java
// IntakeIOReal.java - What the code needs to create:

// ROTATOR: NEO motor for deploying/retracting the intake arm
SparkMax rotatorMotor = new SparkMax(ROTATOR_CAN_ID, MotorType.kBrushless);
// Config: Brake mode, 30A limit, position conversion for arm angle
// Control: Position PID (deploy position, retract position)

// EXTERNAL ENCODER: Tracks arm angle independently of motor
Encoder armEncoder = new Encoder(ENCODER_DIO_A, ENCODER_DIO_B);
// Or if it's a duty-cycle type:
// DutyCycleEncoder armEncoder = new DutyCycleEncoder(ENCODER_DIO_PORT);

// ROLLERS: NEO motor for spinning intake rollers
SparkMax rollerMotor = new SparkMax(ROLLER_CAN_ID, MotorType.kBrushless);
// Config: Brake mode, 30A limit
// Control: Simple percent output (motor.set(0.8) to intake, motor.set(-0.5) to eject)
```

**Why the rotator uses an external encoder:** The arm position matters for deployment/retraction limits. If the motor encoder slipped (belt skip, chain jump), the external encoder at the arm pivot still reads correctly. The external encoder also provides an absolute reference so you know where the arm is on startup.

### 16.4 Indexer and Kicker - To Be Coded

**Hardware:** 1x NEO (indexer) + 1x NEO Vortex (kicker)

Per the [design guide](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md#indexer--kicker-subsystem), the indexer feeds game pieces from the intake toward the shooter, and the kicker launches them into the flywheel.

```java
// IndexerIOReal.java - What the code needs to create:

// INDEXER: NEO motor - feeds game pieces through the robot
SparkMax indexerMotor = new SparkMax(INDEXER_CAN_ID, MotorType.kBrushless);
// Config: Brake mode, 30A limit
// Control: Simple percent output
//   indexerMotor.set(0.5)  = feed forward
//   indexerMotor.set(-0.5) = reverse (unjam)
//   indexerMotor.set(0.0)  = stop

// KICKER: NEO Vortex - launches game piece into shooter flywheels
SparkFlex kickerMotor = new SparkFlex(KICKER_CAN_ID, MotorType.kBrushless);
// Config: Brake mode, 40A limit (Vortex can handle more)
// Control: Simple percent output or timed pulse
//   kickerMotor.set(1.0) = full send into shooter
//   kickerMotor.set(0.0) = stop
```

**Why these don't need encoders or PID:** The indexer and kicker just need to spin at a set power level. There's no specific position to reach or speed to maintain. You might add a **game piece sensor** (beam break on a DIO port) later to detect when a game piece is loaded:

```java
import edu.wpi.first.wpilibj.DigitalInput;

// Beam break sensor: returns true when beam is broken (game piece present)
DigitalInput gamePieceSensor = new DigitalInput(SENSOR_DIO_PORT);
boolean hasGamePiece = !gamePieceSensor.get();  // Often inverted (broken = LOW)
```

### 16.5 Shooter (Turret + Flywheels + Hood) - To Be Coded

**Hardware:** 1x NEO (turret) + 2x NEO Vortex (flywheels) + 1x NEO (flywheel) + 1x NEO (hood) + 1x Through-Bore Encoder

This is the most complex subsystem. Per the [design guide](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md#shooter-subsystem), it may be split into separate subsystems or combined.

#### Turret (Position Control with Absolute Encoder)

```java
// TurretIOReal.java

// MOTOR: NEO for rotating the turret
SparkMax turretMotor = new SparkMax(TURRET_CAN_ID, MotorType.kBrushless);
// Config: Brake mode, 30A limit
// Conversion: motor rotations -> turret rotations (via gear ratio)
// Control: Position PID to aim at calculated angle

// ABSOLUTE ENCODER: Through-Bore Encoder on the turret shaft
DutyCycleEncoder turretEncoder = new DutyCycleEncoder(TURRET_ENCODER_DIO);
// This always knows the turret's exact angle
// Seed the motor's relative encoder from this on startup (same pattern as swerve CANcoders)

// SOFT LIMITS: Prevent over-rotation (turret can't spin 360 because of wires)
SparkMaxConfig turretConfig = new SparkMaxConfig();
turretConfig.softLimit
    .forwardSoftLimit(MAX_TURRET_ROTATIONS)     // e.g., 0.4 rotations (144 degrees)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(-MAX_TURRET_ROTATIONS)     // e.g., -0.4 rotations (-144 degrees)
    .reverseSoftLimitEnabled(true);
```

**Soft limits** are enforced by the SparkMax itself - even if your code accidentally commands past the limit, the controller refuses. This protects wires and mechanisms from wrapping.

#### Flywheels (Velocity Control with Leader/Follower)

```java
// ShooterIOReal.java

// TWO VORTEX FLYWHEELS (leader/follower pair)
SparkFlex topFlywheel = new SparkFlex(TOP_FLYWHEEL_CAN_ID, MotorType.kBrushless);
SparkFlex bottomFlywheel = new SparkFlex(BOTTOM_FLYWHEEL_CAN_ID, MotorType.kBrushless);

// Configure leader
SparkFlexConfig topConfig = new SparkFlexConfig();
topConfig
    .idleMode(IdleMode.kCoast)       // Let flywheels spin down naturally
    .smartCurrentLimit(60);          // Vortex supports higher current
topConfig.closedLoop
    .p(0.0005)                       // Tune this on the real robot
    .velocityFF(1.0 / 6784.0);      // Feedforward: 1/free_speed_RPM (gets you close, PID fine-tunes)
topFlywheel.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// Configure follower (mirrors leader, spins opposite direction)
SparkFlexConfig bottomConfig = new SparkFlexConfig();
bottomConfig.follow(topFlywheel, true);  // true = inverted
bottomFlywheel.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// THIRD FLYWHEEL (NEO, separate control for spin)
SparkMax sideFlywheel = new SparkMax(SIDE_FLYWHEEL_CAN_ID, MotorType.kBrushless);
// May run at different speed to add spin to the game piece

// Command a speed
public void setFlywheelRPM(double rpm) {
    topFlywheel.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
    // bottomFlywheel follows automatically!
    sideFlywheel.getClosedLoopController().setReference(rpm * SIDE_RATIO, ControlType.kVelocity);
}

// Check if flywheels are up to speed
public boolean atTargetSpeed(double targetRPM) {
    double currentRPM = topFlywheel.getEncoder().getVelocity();
    return Math.abs(currentRPM - targetRPM) < 100;  // Within 100 RPM tolerance
}
```

**Velocity feedforward (velocityFF):** PID alone reacts to error - it needs to fall behind before it corrects. Feedforward **predicts** what motor output is needed for a given speed. `1/freeSpeed` is a good starting estimate. The PID then makes small corrections on top of the feedforward.

#### Hood (Position Control)

```java
// Can be part of ShooterIOReal.java or a separate HoodIOReal.java

// HOOD: NEO motor for adjusting shot angle
SparkMax hoodMotor = new SparkMax(HOOD_CAN_ID, MotorType.kBrushless);

SparkMaxConfig hoodConfig = new SparkMaxConfig();
hoodConfig
    .idleMode(IdleMode.kBrake)       // Hold hood position
    .smartCurrentLimit(20);          // Hood is small/light, doesn't need much
hoodConfig.encoder
    .positionConversionFactor(2.0 * Math.PI / HOOD_GEAR_RATIO);  // motor rotations -> radians
hoodConfig.closedLoop
    .p(HOOD_kP);
hoodConfig.softLimit
    .forwardSoftLimit(MAX_HOOD_ANGLE)   // Prevent mechanical damage
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(MIN_HOOD_ANGLE)
    .reverseSoftLimitEnabled(true);

hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// Command a hood angle
public void setHoodAngle(double radians) {
    hoodMotor.getClosedLoopController().setReference(radians, ControlType.kPosition);
}
```

### 16.6 Climber - To Be Coded

**Hardware:** 1x NEO Vortex + SparkFlex

Per the [design guide](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md#climber-subsystem), the climber extends/retracts to latch onto climbing bars.

```java
// ClimberIOReal.java

// CLIMBER: NEO Vortex for extending/retracting the climbing arm
SparkFlex climberMotor = new SparkFlex(CLIMBER_CAN_ID, MotorType.kBrushless);

SparkFlexConfig config = new SparkFlexConfig();
config
    .idleMode(IdleMode.kBrake)           // CRITICAL: Must hold position while climbing!
    .smartCurrentLimit(80);              // High limit - climbing puts heavy load on the motor

// Convert motor rotations to meters of extension
config.encoder
    .positionConversionFactor(DRUM_CIRCUMFERENCE / CLIMBER_GEAR_RATIO);

config.closedLoop
    .p(CLIMBER_kP);

config.softLimit
    .forwardSoftLimit(MAX_EXTENSION_METERS)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(0.0)                    // Can't retract past zero
    .reverseSoftLimitEnabled(true);

climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// Extend to a position
public void extend() {
    climberMotor.getClosedLoopController().setReference(EXTENDED_POSITION, ControlType.kPosition);
}

// Retract (climb!)
public void retract() {
    climberMotor.getClosedLoopController().setReference(RETRACTED_POSITION, ControlType.kPosition);
}

// Manual control for testing
public void manualControl(double percent) {
    climberMotor.set(percent);
}

// Monitor current draw (high current = we've latched and are pulling the robot up)
public double getCurrentAmps() {
    return climberMotor.getOutputCurrent();
}
```

**Why 80A current limit for the climber?** Climbing means lifting the entire robot weight (~65kg). This requires significantly more torque (and therefore current) than any other mechanism. The NEO Vortex and SparkFlex are chosen specifically because they can handle this load.

---

## 17. PID Control Modes Explained (For Every Motor Situation)

Every motor on the robot uses one of these control modes. Understanding when to use each one is key to writing subsystem code.

### Percent Output (Open Loop) - "Just spin at this power level"

```java
motor.set(0.5);  // 50% of battery voltage
```

**How it works:** Sends a fixed percentage of battery voltage to the motor. No feedback loop - if the motor slows down under load, it stays at that reduced speed.

**When to use it:** Rollers, indexer feed, anything where exact speed doesn't matter. Also useful for initial testing before tuning PID.

**Our robot uses it for:** Intake rollers, indexer, kicker, climber manual control

```
Command: "Run at 50% power"
                |
Motor: applies 50% voltage -----> Wheel spins (speed varies with load)
                                   Battery at 12V? Motor gets 6V
                                   Battery at 11V? Motor gets 5.5V (slower!)
```

### Velocity PID (Closed Loop) - "Maintain this exact speed"

```java
motor.getClosedLoopController().setReference(3000, ControlType.kVelocity);  // 3000 RPM
```

**How it works:** The SparkMax reads the motor's actual speed from the encoder, compares it to your target, and adjusts voltage automatically to maintain the target speed.

**When to use it:** Swerve drive wheels, shooter flywheels - anything where consistent speed matters regardless of battery voltage or changing load.

**Our robot uses it for:** Swerve drive motors, all three shooter flywheels

```
Command: "Maintain 3000 RPM"
                |
                v
PID loop (runs on SparkMax at 1kHz):
  1. Read actual speed: 2800 RPM
  2. Error = 3000 - 2800 = 200 RPM
  3. Correction = kP * error = 0.0005 * 200 = 0.1
  4. Apply 0.1 more voltage
  5. Repeat every 1ms until error is near zero
```

**Key PID values for velocity:**
- `kP`: How aggressively to correct speed errors. Start very small (0.0001) and increase until responsive but not oscillating.
- `kFF` (feedforward): Predicts the voltage needed for a target speed. Set to `1.0 / motorFreeSpeedRPM`. This gets you 90% of the way there; PID handles the last 10%.

### Position PID (Closed Loop) - "Go to this exact position and hold"

```java
motor.getClosedLoopController().setReference(2.5, ControlType.kPosition);  // 2.5 rotations
```

**How it works:** The SparkMax reads the motor's current position from the encoder, compares it to your target position, and drives the motor toward that position.

**When to use it:** Swerve steering, turret aiming, hood angle, climber positions - anything that needs to reach and hold a specific position.

**Our robot uses it for:** Swerve angle motors, turret rotation, hood angle, climber positions

```
Command: "Go to position 2.5 rotations"
                |
                v
PID loop (runs on SparkMax at 1kHz):
  1. Read actual position: 1.8 rotations
  2. Error = 2.5 - 1.8 = 0.7 rotations
  3. Correction = kP * error = 5.0 * 0.7 = 3.5 --> clamped to 1.0 (max output)
  4. Motor drives forward at full power
  5. As position approaches 2.5, error shrinks, motor slows
  6. At position 2.5, error = 0, motor holds
```

**Key PID values for position:**
- `kP`: How aggressively to drive toward the target. Larger values mean faster response but can overshoot. Our swerve steering uses kP = 5.0 (aggressive because we want fast wheel rotation).
- `kD`: Dampens overshoot. Adds braking as the mechanism approaches the target. Start at 0 and add if you see oscillation.

### How to Choose the Right Mode

```
Do you need precise speed?
  YES --> Velocity PID (ControlType.kVelocity)
         Examples: drive wheels, flywheels
  NO  --> Do you need a precise position?
           YES --> Position PID (ControlType.kPosition)
                  Examples: turret angle, hood angle, swerve steering
           NO  --> Percent Output (motor.set())
                  Examples: rollers, feeders, simple mechanisms
```

### OnBoard PID vs RoboRIO PID

**SparkMax/SparkFlex onboard PID** (what we use):
- Runs at 1kHz (1000 times per second) on the motor controller itself
- Fast response, low latency
- Use with: `motor.getClosedLoopController().setReference(target, ControlType.k...)`
- Can only use the motor's built-in encoder for feedback

**WPILib PIDController on the RoboRIO:**
- Runs at 50Hz (50 times per second) in your `periodic()` method
- More flexible - can use ANY sensor for feedback (external encoder, camera, etc.)
- Use with: `double output = pidController.calculate(measurement, setpoint); motor.set(output);`
- Required when your feedback sensor isn't connected to the SparkMax (e.g., external encoder on DIO)

```java
// Example: Turret using RoboRIO PID with Through-Bore Encoder
import edu.wpi.first.math.controller.PIDController;

PIDController turretPID = new PIDController(kP, kI, kD);
turretPID.enableContinuousInput(-0.5, 0.5);  // Wraps around for rotational mechanisms

// In periodic() or execute():
double measurement = throughBoreEncoder.get();       // Read the external sensor
double output = turretPID.calculate(measurement, targetPosition);  // Calculate correction
turretMotor.set(output);                             // Apply to motor
```

---

## 18. Conversion Factor Cookbook

Every mechanism has a gear ratio and a physical dimension that determines how motor rotations translate to real-world units. Here's how to calculate conversion factors for every subsystem on our robot.

### The Universal Formula

```
Position conversion factor = (output units per mechanism revolution) / gear ratio

Velocity conversion factor = position conversion factor / 60
                             (because built-in velocity is in RPM = per minute)
```

### Swerve Drive Wheel (Already in our code)

```
Known values:
  Wheel diameter: 0.1016 m (4 inches) * 0.935 fudge = 0.095 m effective
  Wheel circumference: 0.095 * PI = 0.2985 m
  Gear ratio: 6.75:1

Position (motor rotations -> meters driven):
  0.2985 / 6.75 = 0.04423 meters per motor rotation

Velocity (RPM -> m/s):
  0.04423 / 60 = 0.000737 m/s per RPM
```

### Swerve Steering (Already in our code)

```
Known values:
  Output: 1 full wheel rotation = 1.0 rotations
  Gear ratio: 21.43:1

Position (motor rotations -> wheel rotations):
  1.0 / 21.43 = 0.04667 wheel rotations per motor rotation
```

### Shooter Flywheel

```
Flywheels usually have no gearing (or 1:1), so:
  Position: 1.0 (rotations stay as rotations)
  Velocity: 1.0 (RPM stays as RPM)

If there IS a gear ratio (e.g., 2:1):
  Velocity factor: 1.0 / 2.0 = 0.5 (flywheel spins at half motor speed)
  Then flywheel RPM = motor.getEncoder().getVelocity() (already converted)
```

### Turret Rotation

```
Known values:
  Output: 1 full turret revolution = 1.0 rotations (or 2*PI radians)
  Gear ratio: TBD (measure your turret gearing)

For rotations:
  Position: 1.0 / TURRET_GEAR_RATIO

For radians:
  Position: (2 * Math.PI) / TURRET_GEAR_RATIO

For degrees:
  Position: 360.0 / TURRET_GEAR_RATIO
```

### Hood Angle

```
Known values:
  Hood travel: partial rotation (e.g., 0 to 60 degrees = 0 to PI/3 radians)
  Gear ratio: TBD

Position (motor rotations -> radians):
  (2 * Math.PI) / HOOD_GEAR_RATIO
```

### Climber Extension

```
Known values:
  Drum diameter: TBD (measure the spool/drum the rope/chain wraps around)
  Drum circumference: diameter * PI
  Gear ratio: TBD

Position (motor rotations -> meters of extension):
  (drum_diameter * Math.PI) / CLIMBER_GEAR_RATIO
```

### Intake Rotator Arm

```
Known values:
  Output: 1 full arm revolution = 2*PI radians
  Gear ratio: TBD

Position (motor rotations -> arm radians):
  (2 * Math.PI) / INTAKE_GEAR_RATIO
```

### How to Measure a Gear Ratio You Don't Know

1. Mark the motor shaft and the output shaft with tape
2. Slowly rotate the output shaft exactly **1 full turn**
3. Count how many turns the motor shaft made
4. That count is your gear ratio

For example, if the motor shaft turns 15 times when the output turns once, the gear ratio is 15:1.

### Validating Your Conversion Factors

After setting conversion factors, **always validate on the real robot:**

1. Reset the encoder to zero: `motor.getEncoder().setPosition(0);`
2. Physically move the mechanism a known distance (e.g., push the robot forward 1 meter, rotate turret 90 degrees)
3. Read the encoder value
4. It should match your known distance. If it's off, adjust the conversion factor proportionally:

```
correctionFactor = actualDistance / reportedDistance
newConversionFactor = oldConversionFactor * correctionFactor
```

This is exactly how our swerve `fudge` factor (0.935) was determined.

---

*Last updated: February 2026 | Team 6637 Betawolves*
*Full parts list and subsystem designs: [betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md](../../../betawolves-hq/FRC-2026/knowledge/000-robot-subsystems-design-guide.md)*
*Software architecture overview: [betawolves-hq/FRC-2026/knowledge/000-robot-software-architecture.md](../../../betawolves-hq/FRC-2026/knowledge/000-robot-software-architecture.md)*
