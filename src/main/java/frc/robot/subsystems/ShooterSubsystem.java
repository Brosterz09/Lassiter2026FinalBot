// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterSubsystem extends SubsystemBase {
  private Supplier<Pose2d> m_poseSupplier;
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);
  //private final double TARGET_RPS = 56.0;
  private final double TARGET_RPS = 53;
  public Translation2d blueHubPosition = new Translation2d(4.625, 4.025);
  public Translation2d redHubPosition = new Translation2d(11.913, 4.025);

  // Offset of the shooter (under limelight-front) from the robot center, in robot-relative coords.
  // Forward = +x, left = +y in WPILib convention.
  private static final Translation2d kShooterOffset = new Translation2d(0.15748, -0.1353312);

  /** Returns the shooter's position in field coordinates given the current robot pose. */
  private static Translation2d shooterFieldPosition(Pose2d pose) {
    return pose.getTranslation().plus(kShooterOffset.rotateBy(pose.getRotation()));
  }
  private boolean running = false;
  private boolean unJamRunning = false;
  private boolean autoRunning = false;
  private double m_targetRPS = TARGET_RPS;
  private boolean m_reachedSpeed = false;
  public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
  m_poseSupplier = poseSupplier;
  TalonFXConfiguration config = new TalonFXConfiguration();
  config.Slot0.kP = 0.5;
  config.Slot0.kI = 0;
  config.Slot0.kD = 0;
  config.Slot0.kV = 0.126;
  config.Slot0.kS = 0.0;
  config.CurrentLimits.StatorCurrentLimit = 100;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  ShooterMotor.getConfigurator().apply(config);

  }
  TalonFX ShooterMotor = new TalonFX(15);
  
  
  
  double speed = 12;
  VoltageOut voltageRequest = new VoltageOut(0.0);
  /**
   * Shooter command factory method.
   *
   * @return a command
   */
  public Command MoveShooterWithDistance(Supplier<Pose2d> poseSupplier) {
    Translation2d blueHub = new Translation2d(4.625, 4.025);
    Translation2d redHub = new Translation2d(11.913, 4.025);
    return run(() -> {
        Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? blueHub : redHub;
        double distance = shooterFieldPosition(poseSupplier.get()).getDistance(hub);
        getSpeedForDistance(distance);
        System.out.println(distance);
        setShooterVelocity(speed);
    }).finallyDo(interrupted -> endMove());
}


  public Command AutoMoveShooter(Supplier<Pose2d> poseSupplier) {
    return runEnd(
        () -> {
            Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? blueHubPosition : redHubPosition;
            double distance = shooterFieldPosition(poseSupplier.get()).getDistance(hub);
            getSpeedForDistance(distance);
            setShooterVelocity(speed);
        },
        () -> endMove()
    ).withTimeout(7.0);
}
  public Command BackShots() {
    return runEnd(
        () -> {
          setShooterVelocity(m_targetRPS);
        },
        () -> endMove()
    ).withTimeout(.5);
}

  public void getSpeedForDistance(double distanceMeters) {
    double KP = 10;
    speed =  distanceMeters*KP + 20
    ;
  }
  
  public boolean atSpeed() {
    return getShooterVelocity() >= .98 * m_targetRPS;
  }

  /** Latching speed flag: becomes true once up to speed, stays true until the shooter is stopped. */
  public boolean reachedSpeed() {
    if (atSpeed()) m_reachedSpeed = true;
    return m_reachedSpeed;
  }
  public Command JustShoot() {
    return run(
        () -> {
            running = true;
            setShooterVelocity(m_targetRPS);
        }).finallyDo(interrupted->endMove());
      }
  public Command CrackCocaineShooter() {
    return run(
        () -> {
            setShooterVelocity(m_targetRPS*1.2);
        }).finallyDo(interrupted->endMove());
      }

  public Command AutoJustShoot() {
    return runEnd(
        () -> {
            autoRunning = true;
            setShooterVelocity(m_targetRPS);
        },
        () -> endMove()).withTimeout(5.5);
      }
  public Command unJamShooter() {
    return run(
        () -> {
            unJamRunning = true;
            setShooterVelocity(100);
        }).finallyDo(interrupted->endMove());
      }

  public void setShooterVelocity(double targetRPS){
    ShooterMotor.setControl(m_velocity.withVelocity(targetRPS));
  }

  public double getShooterVelocity() {
    return ShooterMotor.getVelocity().getValueAsDouble();
  }

  public void stop(){
    ShooterMotor.setControl(new com.ctre.phoenix6.controls.NeutralOut());
  }

  public Command velocityIncrease() {
    return runOnce(
      () -> m_targetRPS *= 1.1
    );
  }

  public Command velocityDecrease() {
    return runOnce(
      () -> m_targetRPS /= 1.1
    );
  }

  public Command velocityReset() {
    return runOnce(
      () -> m_targetRPS = TARGET_RPS
    );
  }

  /**
   * An Shooter method querying a boolean state of the subsystem (for Shooter, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean ShooterCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? blueHubPosition : redHubPosition;
        double distance = m_poseSupplier.get().getTranslation().getDistance(hub);
      SmartDashboard.putNumber("DistanceToHub", distance);
    double velocity = getShooterVelocity();
    SmartDashboard.putNumber("ShooterVelocity", velocity);
    SmartDashboard.putBoolean("ShooterAtSpeed", atSpeed());
    SmartDashboard.putBoolean("ShooterReachedSpeed", m_reachedSpeed);
    SignalLogger.writeDouble("Shooter/VelocityRPS", velocity, "rotations per second");
    SignalLogger.writeDouble("Shooter/TargetRPS", m_targetRPS, "rotations per second");
    SignalLogger.writeBoolean("Shooter/AtSpeed", atSpeed());
    SignalLogger.writeBoolean("Shooter/ReachedSpeed", m_reachedSpeed);
    if(running == true) {

    }
    else if(unJamRunning == true) {

    }
    else if (autoRunning == true) {

    }
    else {
      setShooterVelocity(20);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void endMove() {
    m_reachedSpeed = false;
    unJamRunning = false;
    autoRunning = false;
    running = false;
    stop();
  }
}
 