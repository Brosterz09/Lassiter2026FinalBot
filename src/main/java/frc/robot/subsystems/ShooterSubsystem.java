// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
  private final double TARGET_RPS = 90.0;
  public Translation2d blueHubPosition = new Translation2d(4.625, 3.775);
  public Translation2d redHubPosition = new Translation2d(11.915,3.775);
  private double m_targetRPS = TARGET_RPS;
  public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
  m_poseSupplier = poseSupplier;
  TalonFXConfiguration config = new TalonFXConfiguration();
  config.Slot0.kP = 0.12;
  config.Slot0.kI = 0;
  config.Slot0.kD = 0;
  config.Slot0.kV = 0.126;
  config.Slot0.kS = 0.0;
  config.CurrentLimits.StatorCurrentLimit = 40;
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
    Translation2d blueHub = new Translation2d(4.625, 3.8);
    Translation2d redHub = new Translation2d(11.915, 3.8);
    return run(() -> {
        Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? blueHub : redHub;
        double distance = poseSupplier.get().getTranslation().getDistance(hub);
        // getSpeedForDistance(distance);
        setShooterVelocity(m_targetRPS);
    }).finallyDo(interrupted -> endMove());
}


  public Command AutoMoveShooter(Supplier<Pose2d> poseSupplier) {
    return runEnd(
        () -> {
            Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? blueHubPosition : redHubPosition;
            // double distance = poseSupplier.get().getTranslation().getDistance(hub);
            // getSpeedForDistance(distance);
            setShooterVelocity(m_targetRPS);
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
    double KP = .27827842218;
    speed =  distanceMeters*KP + 12;
  }

  public boolean atSpeed() {
    return getShooterVelocity() >= .9 * m_targetRPS;
  }
  public Command JustShoot() {
    return run(
        () -> {
            setShooterVelocity(m_targetRPS);
        }).finallyDo(interrupted->endMove());
      }
  public Command AutoJustShoot() {
    return runEnd(
        () -> {
            setShooterVelocity(m_targetRPS);
        },
        () -> setShooterVelocity(0)).withTimeout(7.0);
      }
  public Command ReverseShooter() {
    return run(
        () -> {
            setShooterVelocity(-100);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void endMove() {
    stop();
  }
  public void setMotorVoltage(double volts) {
    ShooterMotor.setControl(voltageRequest.withOutput(volts));
  }
}
 