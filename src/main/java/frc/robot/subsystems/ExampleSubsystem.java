// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}
  TalonFX Motor1 = new TalonFX(15);
  double speed = .3;
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command MoveMotor() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
         Motor1.set(-speed);

          //continuance action goes here */
        }).finallyDo(interrupted->endMove());
        
  }
  public Command raiseSpeed() {
    return runOnce(
        () -> {
          if (speed < 1) {
            speed -= .05;
          }
        });
  }

  public Command lowerSpeed() {
    return runOnce(
        () -> {
           if (speed > 0) {
            speed += .05;
           }         
        });
        
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
  public void endMove() {
    Motor1.set(0);
  }
}
