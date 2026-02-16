// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class IntakeSubsystem extends SubsystemBase {
  
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {}

    TalonFX IntakeLEVERMotor = new TalonFX(25);
    TalonFX IntakeMotor = new TalonFX(14);

  private boolean reversed = false;

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
  public Command LowerIntakeDOWN() {
    return run(
        () -> {
            IntakeLEVERMotor.set(.5);
        }).finallyDo(interrupted->endMove());
      }

  public Command BringIntakeUP() {
    return run(
        () -> {
            IntakeLEVERMotor.set(-.5);
        }).finallyDo(interrupted->endMove());
    }

  
  public Command RunIntake() {
    return run(
      () -> {
        IntakeMotor.set(.4);

      }
    ).finallyDo(interrupted->endMove());
  }

  public Command AutoRunIntake() {
    return runEnd(
        () -> IntakeMotor.set(.4),
        () -> endMove()
        ).withTimeout(3.5);
      }

  public Command AutoBringIntakeUP() {
    return runEnd(
        () -> IntakeLEVERMotor.set(-.5),
        () -> endMove()
        ).withTimeout(.6);
    }

  public Command AutoLowerIntakeDOWN() {
    return runEnd(
        () -> IntakeLEVERMotor.set(.5),
        () -> endMove()
        ).withTimeout(.6);
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
  public void endMove(){
    IntakeMotor.set(0);
    IntakeLEVERMotor.set(0);
    
  }
}
