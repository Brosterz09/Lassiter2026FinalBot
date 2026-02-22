// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;

public class HangSubsystem extends SubsystemBase {
  SparkMax HangMotor = new SparkMax(18, SparkMax.MotorType.kBrushless);


  public HangSubsystem() {}

  
  public Command HangRobotUp() {
    return run(
        () -> {
            HangMotor.set(.7);
         }
    ).finallyDo(interrupted->HangMotor.set(0));
}

  public Command HangRobotDown() {
    return run(
        () -> {
            HangMotor.set(-.7);
         }
    ).finallyDo(interrupted->HangMotor.set(0));
}

  public Command AutoHangBot() {
    return runEnd(
        () -> HangMotor.set(.7),
        () -> HangMotor.set(0)
    ).withTimeout(2.5);
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
