// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;

public class IndexSubsystem extends SubsystemBase {
  TalonFX Indexer = new TalonFX(17);
 private final InterpolatingDoubleTreeMap indexerSpeedMap = new InterpolatingDoubleTreeMap();
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);

  public IndexSubsystem() {
    
    TalonFXConfiguration config = new TalonFXConfiguration();
  config.Slot0.kP = 0.11;
  config.Slot0.kI = 0;
  config.Slot0.kD = 0;
  config.Slot0.kV = 0.12;
  config.CurrentLimits.StatorCurrentLimit = 60;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  Indexer.getConfigurator().apply(config);
  }

  
  public Command RunSpindexer() {
    return run(
        () -> {
            setIndexerVelocity(4000);
         }
    ).finallyDo(interrupted->stop());
}
  public Command AutoRunSpindexer() {
    return runEnd(
        () -> Indexer.set(.2),
        () -> Indexer.set(0)
    ).withTimeout(3.5);
  }
   public void setIndexerVelocity(double targetRPS){
    Indexer.setControl(m_velocity.withVelocity(targetRPS));
  }
  public void stop(){
    Indexer.setControl(new com.ctre.phoenix6.controls.NeutralOut());
  }

  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
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
}
