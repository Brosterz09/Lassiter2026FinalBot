// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}
  TalonFX ShooterMotor = new TalonFX(15);
  
  double speed = 12;
  VoltageOut voltageRequest = new VoltageOut(0.0);
  /**
   * Shooter command factory method.
   *
   * @return a command
   */
  public Command MoveMotor() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          ShooterMotor.setControl(new VoltageOut(-speed));
          //continuance action goes here */
        }).finallyDo(interrupted->endMove());
        
  }
  public Command raiseSpeed() {
    return runOnce(
        () -> {
            speed += 1;
          System.out.println("new speed: "+ speed);
        });
  }

  public Command lowerSpeed() {
    return runOnce(
        () -> {
            speed -= 1;     
           System.out.println("new speed: "+ speed);
        });
        
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
