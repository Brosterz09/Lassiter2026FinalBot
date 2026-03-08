# Robot Code Improvement Plan

Post-competition code review and prioritized fix list for the Lassiter 2026 robot.

---

## Problem 1 & 2: Controls Invert / Change Mid-Match

### Root Cause

The vision system uses **MegaTag1** (`getBotPose2d_wpiBlue`) which computes a full 6-DOF pose including heading/yaw. This heading estimate is fed into the Kalman filter pose estimator every cycle (every 20ms) for both cameras.

MegaTag1 with a single AprilTag is geometrically ambiguous -- there are often two valid pose solutions approximately **180 degrees apart**. The Limelight can flip between them frame-to-frame. Even with 2 tags the heading can be wrong under certain tag geometries.

The current theta (heading) standard deviations of **0.7 rad (2 tags)** and **1.5 rad (1 tag)** allow the Kalman filter to gradually pull the robot's estimated heading toward the Limelight's potentially incorrect heading. If the heading gets pulled ~180 degrees off, field-centric driving inverts completely.

There are **no sanity checks** on the vision pose before it enters the Kalman filter -- no guard against heading disagreement, poses outside the field, or degenerate default poses.

**Relevant code:** `CommandSwerveDrivetrain.java` lines 368-387

### Why Sim Appears Opposite

The Pigeon2 mount pose is set to `MountPoseYaw = -90` in `TunerConstants.java` line 81-83. If the simulation does not model this mount offset correctly, or if the initial heading reference differs between sim and real, the sim will appear mirrored. A commented-out config at lines 84-87 shows `MountPoseYaw = 180` was tried previously -- this uncertainty suggests the mount pose may not have been validated.

### Fix A: Switch from MegaTag1 to MegaTag2 (PRIMARY FIX)

MegaTag2 uses the gyro heading as an input constraint and only solves for x/y position, completely eliminating heading ambiguity.

**Step 1:** Feed gyro heading to both Limelights every cycle. Add to the top of `periodic()` in `CommandSwerveDrivetrain.java`, before the `updateVisionFromCamera` calls:

```java
double yaw = getState().Pose.getRotation().getDegrees();
LimelightHelpers.setRobotOrientation("limelight-front", yaw, 0, 0, 0, 0, 0);
LimelightHelpers.setRobotOrientation("limelight-back", yaw, 0, 0, 0, 0, 0);
```

**Step 2:** Replace the `updateVisionFromCamera` method entirely:

```java
private void updateVisionFromCamera(String cameraName) {
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    if (mt2 == null || mt2.tagCount == 0) return;

    // Reject measurements during fast rotation (MegaTag2 is unreliable above ~720 deg/s)
    if (Math.abs(getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(720)) return;

    // Reject poses outside the field boundary
    if (mt2.pose.getX() < 0 || mt2.pose.getX() > 16.5
        || mt2.pose.getY() < 0 || mt2.pose.getY() > 8.2) return;

    // Set theta std dev to effectively infinite -- never correct heading from vision
    if (mt2.tagCount >= 2) {
        addVisionMeasurement(mt2.pose, mt2.timestampSeconds,
            VecBuilder.fill(0.5, 0.5, 9999999));
    } else {
        addVisionMeasurement(mt2.pose, mt2.timestampSeconds,
            VecBuilder.fill(1.0, 1.0, 9999999));
    }
}
```

### Fix B: Re-enable the seedFieldCentric Button (SAFETY NET)

In `RobotContainer.java` line 168, `seedFieldCentric` is commented out. There is currently **no way for the driver to reset heading during a match**. Uncomment and bind to the `start` button:

```java
joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
```

This gives the driver a panic button: point the robot toward the opposing alliance wall and press `start` to reset "forward."

### Fix C: Validate Pigeon2 Mount Pose

Use Tuner X self-test to confirm the Pigeon2's actual mounting orientation. The robot's forward direction should read 0 degrees when facing the red alliance wall (WPILib blue-alliance convention). Set `MountPoseYaw` in `TunerConstants.java` line 82 to match the measured physical offset.

---

## Problem 3: Brownouts / Swerve Stuttering

### Root Cause

The theoretical peak current draw across all motors is approximately **525A**. A standard FRC battery can only sustain ~120-150A before voltage collapses below the roboRIO's brownout threshold (6.8V).

Contributing factors:

1. **Steer motors have unnecessarily high current limits (40A each).** They rarely need more than 20A.
2. **No supply current limits anywhere.** Only stator current limits are configured. Stator limits prevent motor heating but do not directly prevent battery voltage collapse.
3. **Shooter motor allows 60A**, indexer allows 55A -- both are high.
4. **The SparkMax hang motor has NO configuration at all** -- no current limits, no brake mode, no ramp rate. It can draw unlimited current.
5. **Open-loop voltage drive mode** (`RobotContainer.java` line 40) allows massive transient current spikes during rapid acceleration, pushing, or direction changes.

### Fix A: Reduce Current Limits

**`TunerConstants.java` -- Steer motors (line 70-77):**
```java
// Change from 40A to 20A
.withStatorCurrentLimit(Amps.of(20))
.withStatorCurrentLimitEnable(true)
```

**`TunerConstants.java` -- Drive motors (line 64-69), add supply limit:**
```java
.withStatorCurrentLimit(Amps.of(40))
.withStatorCurrentLimitEnable(true)
.withSupplyCurrentLimit(Amps.of(35))
.withSupplyCurrentLimitEnable(true)
```

**`ShooterSubsystem.java` line 48:** Reduce from 60A to 40A:
```java
config.CurrentLimits.StatorCurrentLimit = 40;
```

**`IndexSubsystem.java` line 37:** Reduce from 55A to 35A:
```java
config.CurrentLimits.StatorCurrentLimit = 35;
```

**`IntakeSubsystem.java` line 60 (arm motor):** Reduce from 45A to 30A:
```java
m_armConfig.CurrentLimits.StatorCurrentLimit = 30;
```

### Fix B: Configure the Hang Motor

In `HangSubsystem.java`, the SparkMax is constructed at line 17 but never configured. Add:

```java
public HangSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(30);       // 30A current limit
    config.idleMode(IdleMode.kBrake);   // Hold position when not commanded
    HangMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}
```

### Fix C: Consider Closed-Loop Drive Control (Optional / Future)

In `RobotContainer.java` line 40, change `OpenLoopVoltage` to `Velocity` for more controlled current draw:
```java
.withDriveRequestType(DriveRequestType.Velocity)
```
This gives smoother acceleration and more predictable current draw but may feel slightly different to the driver. Test before competition.

### Revised Current Budget After Fixes

| Motor             | Count | Stator Limit | Max Total |
|-------------------|-------|--------------|-----------|
| Drive             | 4     | 40A          | 160A      |
| Steer             | 4     | 20A          | 80A       |
| Shooter           | 1     | 40A          | 40A       |
| Indexer           | 1     | 35A          | 35A       |
| Intake            | 1     | 45A          | 45A       |
| Intake Arm        | 1     | 30A          | 30A       |
| Hang              | 1     | 30A          | 30A       |
| **Total**         |       |              | **420A**  |

This is still high in theory, but in practice not all motors peak simultaneously. The key improvement is steer motors dropping from 160A to 80A total, and the hang motor getting a limit at all.

---

## Problem 4: Intake Arm Slams Down

### Root Cause

`SetIntakeArmDown()` in `IntakeSubsystem.java` line 97-101 uses `PositionVoltage` with Slot 1 gains:
- `kP = 0.3`, `kD = 0`, `kV = 5.3`, `kS = 0`

The `kV` feedforward of **5.3** is the main culprit. This value commands substantial voltage proportional to the target velocity of the motion. Since **gravity assists the downward motion**, the motor is actively driving the arm downward on top of gravity instead of braking against it.

By comparison, Slot 0 (used for arm UP) has `kP = 3.0`, `kV = 3.6`, `kS = 8` -- the high `kS` provides gravity compensation going up, which is correct.

### Fix A: Switch to MotionMagic (RECOMMENDED)

MotionMagic limits the velocity and acceleration of the motion, creating a smooth trapezoidal profile instead of commanding full speed immediately.

**Step 1:** Add MotionMagic config in the `IntakeSubsystem` constructor after applying the arm config:

```java
m_armConfig.MotionMagic.MotionMagicCruiseVelocity = 4;   // rotations per second
m_armConfig.MotionMagic.MotionMagicAcceleration = 8;      // rot/s^2
m_armConfig.MotionMagic.MotionMagicJerk = 40;             // rot/s^3 (smooths start/stop)
IntakeArmMotor.getConfigurator().apply(m_armConfig);
```

**Step 2:** Replace `PositionVoltage` with `MotionMagicVoltage`:

```java
// Change the field declaration:
private final MotionMagicVoltage m_motionMagicArm = new MotionMagicVoltage(0);

// In SetIntakeArmDown():
IntakeArmMotor.setControl(m_motionMagicArm.withPosition(ARM_DOWN_POSITION).withSlot(1));

// In SetIntakeArmUp():
IntakeArmMotor.setControl(m_motionMagicArm.withPosition(ARM_UP_POSITION).withSlot(0));
```

**Step 3:** Retune Slot 1 gains for MotionMagic downward travel:

```java
m_armConfig.Slot1.kP = 0.3;
m_armConfig.Slot1.kI = 0;
m_armConfig.Slot1.kD = 0.1;   // ADD derivative to brake near target
m_armConfig.Slot1.kV = 1.5;   // REDUCE from 5.3 -- gravity does most of the work
m_armConfig.Slot1.kS = 0;
```

The MotionMagicCruiseVelocity and Acceleration values will need tuning on the actual robot. Start conservative (lower values) and increase until the motion is fast enough but still controlled.

### Fix B: Quick Tuning Fix (If No Time for MotionMagic)

If MotionMagic is too big a change before the next event, just retune Slot 1:

```java
m_armConfig.Slot1.kP = 0.3;    // keep
m_armConfig.Slot1.kD = 0.1;    // ADD -- brakes the motion near the target
m_armConfig.Slot1.kV = 1.5;    // REDUCE from 5.3 -- stop actively driving into gravity
m_armConfig.Slot1.kS = 0;      // keep
```

---

## General Code Quality Improvements

### Joystick Axis Negation Inconsistency

The default drive command in `RobotContainer.java` lines 89-91 negates the joystick axes:
```java
drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
```

But `aimAtHub` and `aimAtAllianceSide` receive non-negated suppliers from `RobotContainer.java` lines 116-117, then negate inside the method at `CommandSwerveDrivetrain.java` lines 405-408. This works but is fragile. Standardize: always negate at the call site, or always inside the method.

### Rotation Rate Sign

`RobotContainer.java` line 91: `withRotationalRate(joystick.getRightX() * MaxAngularRate)` is **not negated** unlike X and Y. The comment says "counterclockwise with negative X" but the code uses the raw positive value. Verify the rotation direction is correct for your driver's expectation.

### Duplicate Button Bindings

`RobotContainer.java` lines 96-97 and 136-137 both bind `povUp` and `povDown` to the same hang commands. Remove the duplicate at lines 136-137.

### System.out.println in Hot Loops

These print every 20ms during active use and create garbage collection pressure:
- `CommandSwerveDrivetrain.java` line 399 (inside `aimAtHub`) -- prints error value
- `ShooterSubsystem.java` line 72 (inside `MoveShooterWithDistance`) -- prints distance

Replace with SmartDashboard calls or remove entirely:
```java
SmartDashboard.putNumber("AimError", error);
SmartDashboard.putNumber("ShooterDistance", distance);
```

### Unused Imports and Dead Code to Clean Up

| File | What to Remove |
|------|----------------|
| `CommandSwerveDrivetrain.java` | `ArrayList`, `List` imports (lines 36-37); unused `VisionTimer` field (line 49) |
| `ShooterSubsystem.java` | `ObjectInputFilter.Config` import (line 24); `java.util.Timer` import (line 26) |
| `IntakeSubsystem.java` | `AbstractQueue` import (line 12) |
| All subsystems | Boilerplate `exampleCondition()` methods |

### Auto Path Validation

`RobotContainer.java` line 175 calls `AutoBuilder.buildAuto("empty")`. Verify that this PathPlanner auto file exists and has a valid starting pose. PathPlanner calls `resetPose` at auto start -- if the "empty" auto has a bad pose, it will corrupt the heading for the entire match.

---

## Implementation Priority

| Priority | Item | Fixes Problem | Difficulty |
|----------|------|---------------|------------|
| **P0** | Switch to MegaTag2 vision | 1, 2 | Medium |
| **P0** | Add seedFieldCentric button | 1, 2 | Easy |
| **P0** | Validate Pigeon2 mount pose | 1 | Easy |
| **P1** | Lower steer motor current to 20A | 3 | Easy |
| **P1** | Add supply current limits | 3 | Easy |
| **P1** | Lower shooter/indexer current limits | 3 | Easy |
| **P1** | Configure hang SparkMax with current limit | 3 | Easy |
| **P1** | Switch intake arm to MotionMagic, retune Slot 1 | 4 | Medium |
| **P2** | Remove System.out.println from hot loops | General | Easy |
| **P2** | Clean up unused imports and dead code | General | Easy |
| **P2** | Standardize joystick negation convention | General | Easy |
| **P2** | Remove duplicate povUp/povDown bindings | General | Easy |
| **P3** | Consider closed-loop velocity drive mode | 3 | Medium |
