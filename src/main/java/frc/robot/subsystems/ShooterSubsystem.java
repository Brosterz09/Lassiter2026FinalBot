// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import java.util.Timer;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterSubsystem extends SubsystemBase {
  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();

  public ShooterSubsystem() {
  }
  TalonFX ShooterMotor = new TalonFX(16);
  
  double speed = 12;
  VoltageOut voltageRequest = new VoltageOut(0.0);
  /**
   * Shooter command factory method.
   *
   * @return a command
   */
  public Command MoveShooterWithDistance(Supplier<Pose2d> poseSupplier) {
    Translation2d blueHub = new Translation2d(4.6, 4.0);
    Translation2d redHub = new Translation2d(11.9, 4.0);
    return run(() -> {
        Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? blueHub : redHub;
        double distance = poseSupplier.get().getTranslation().getDistance(hub);
        getSpeedForDistance(distance);
        ShooterMotor.setControl(new VoltageOut(-speed));
    }).finallyDo(interrupted -> endMove());
}

  public Command AutoMoveShooter(Supplier<Pose2d> poseSupplier) {
    Translation2d blueHubPosition = new Translation2d(4.6, 4.0);
    Translation2d redHubPosition = new Translation2d(11.9,4.0);
    return runEnd(
        () -> {
            Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? blueHubPosition : redHubPosition;
            double distance = poseSupplier.get().getTranslation().getDistance(hub);
            getSpeedForDistance(distance);
            ShooterMotor.setControl(new VoltageOut(-speed));
        },
        () -> endMove()
    ).withTimeout(4.0);
}

  public void getSpeedForDistance(double distanceMeters) {
    double KP = .27827842218;
    speed =  distanceMeters*KP + 1.2;
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void endMove() {
    ShooterMotor.set(0);
  }
  public void setMotorVoltage(double volts) {
    ShooterMotor.setControl(voltageRequest.withOutput(volts));
  }
}
