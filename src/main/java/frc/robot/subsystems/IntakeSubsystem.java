// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {}
  //TalonFX Rotate1 = new TalonFX(14);
  //TalonFX Rotate2 = new TalonFX(15);
  // TalonFX IntakeMotor = new TalonFX(15);
  SparkMax Rotate = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);

  private boolean reversed = false;

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    public Command Forwards() {
    return run(
        () -> {
            Rotate.set(.55);
        }).finallyDo(interrupted->Rotate.set(0));
}
    public Command Backwards() {
    return run(
        () -> {
            Rotate.set(-.55);
        }).finallyDo(interrupted->Rotate.set(0));
}

  
  // public Command RunIntake() {
  //   return run(
  //     () -> {
  //       IntakeMotor.set(.1);

  //     }
  //   ).finallyDo(interrupted->endMove());
  // }

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
  // public void endMove(){
  //   IntakeMotor.set(0);
  // }
}
