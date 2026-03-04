// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.AbstractQueue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class IntakeSubsystem extends SubsystemBase {
  
  TalonFX IntakeArmMotor = new TalonFX(25);
  TalonFX IntakeMotor = new TalonFX(14);

  private final VelocityVoltage m_velocityIntake = new VelocityVoltage(0);
  private final PositionVoltage m_positionArm = new PositionVoltage(0);
  private final double ARM_DOWN_POSITION = 6.7;
  private final double ARM_UP_POSITION = 0;

  private final TalonFXConfiguration m_armConfig;
  public boolean Intaking = false;
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = 0.098;
      config.Slot0.kI = 0;
      config.Slot0.kD = 0;
      config.Slot0.kV = 0.098;
      config.Slot0.kS = 0.0;
      config.CurrentLimits.StatorCurrentLimit = 60;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      IntakeMotor.getConfigurator().apply(config);
      m_armConfig = new TalonFXConfiguration();
      m_armConfig.Slot0.kP = 2.0;
      m_armConfig.Slot0.kI = 0;
      m_armConfig.Slot0.kD = 0;
      m_armConfig.Slot0.kV = 3.0;
      m_armConfig.Slot0.kS = 8;
      m_armConfig.Slot1.kP = .3;
      m_armConfig.Slot1.kI = 0;
      m_armConfig.Slot1.kD = 0;
      m_armConfig.Slot1.kV = 0.3;
      m_armConfig.Slot1.kS = 0;
      m_armConfig.CurrentLimits.StatorCurrentLimit = 60;
      m_armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      m_armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      m_armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      IntakeArmMotor.getConfigurator().apply(m_armConfig);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
  // public Command LowerIntakeDOWN() {
  //   return run(
  //       () -> {
  //           IntakeArmMotor.set(.5);
  //       }).finallyDo(interrupted->endLeverMove());
  //     }

  // public Command BringIntakeUP() {
  //   return run(
  //       () -> {
  //           IntakeArmMotor.set(-.5);
  //       }).finallyDo(interrupted->endLeverMove());
  //   }

    public void setIntakeVelocity(double targetRPS){
      IntakeMotor.setControl(m_velocityIntake.withVelocity(targetRPS));
    }

    public void stopIntake(){
      IntakeMotor.setControl(new com.ctre.phoenix6.controls.NeutralOut());
    }

    public Command SetIntakeArmDown() {
    return run(
      () -> {
        IntakeArmMotor.setControl(m_positionArm.withPosition(ARM_DOWN_POSITION).withSlot(1));
      }).finallyDo(Interrupted -> stopIntake());
    }
        // () -> {
        //   IntakeArmMotor.set(.25);
        // }).until(() -> Math.abs(IntakeArmMotor.getPosition().getValueAsDouble()+.47) <= .5)
        // .finallyDo(interrupted ->IntakeArmMotor.set(0));

  public Command SetIntakeArmUp() {
    return run(
      () -> {
        IntakeArmMotor.setControl(m_positionArm.withPosition(ARM_UP_POSITION).withSlot(0));
      }).finallyDo(Interrupted -> stopIntake());
    }

  //   public Command IntakeArmDown() {
  //   return run(
  //       () -> {
  //           // double leverPosition = IntakeArmMotor.getPosition().getValueAsDouble();
  //           IntakeArmMotor.set(.5);
  //       }).finallyDo(interrupted->endLeverMove());
  //     }

  // public Command IntakeArmUp() {
  //   return run(
  //       () -> {
  //           IntakeArmMotor.set(-.5);
  //       }).finallyDo(interrupted->endLeverMove());
  //   }

  public Command RunIntake() {
    return runOnce(
      () -> {
        if(Intaking == false) {
          Intaking = true;
          setIntakeVelocity(-60);
        }
        else {
          Intaking = false;
          stopIntake();
        }
      }
    );
  }

  public Command AutoRunIntake() {
    return runEnd(
        () -> IntakeMotor.set(.6),
        () -> endIntakeMove()
        ).withTimeout(5);
      }

  public Command AutoBringIntakeUP() {
    return runEnd(
        () -> IntakeArmMotor.set(-.5),
        () -> endLeverMove()
        ).withTimeout(.6);
    }

  public Command AutoLowerIntakeDOWN() {
    return runEnd(
        () -> IntakeArmMotor.set(.5),
        () -> endLeverMove()
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
  public void endLeverMove(){
    IntakeArmMotor.set(0);
  }

  public void endIntakeMove() {
    IntakeMotor.set(0);
  }
}
