// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
    public double MaxSpeed = .8 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);

    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(() -> drivetrain.getState().Pose);
    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    public final IndexSubsystem m_IndexSubsystem = new IndexSubsystem(m_shooterSubsystem);
    public final HangSubsystem m_HangSubsystem = new HangSubsystem();
    public final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
    public static final Rotation2d k180deg = new Rotation2d();

    // Auto chooser — automatically picks up all PathPlanner autos
    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Shoot", m_shooterSubsystem.AutoJustShoot());
        NamedCommands.registerCommand("ShootForever", m_shooterSubsystem.AutoJustShootForever());
        NamedCommands.registerCommand("TrigIntake", m_IntakeSubsystem.RunIntake());
        NamedCommands.registerCommand("BackShots", m_shooterSubsystem.BackShots());
        NamedCommands.registerCommand("Spindex", m_IndexSubsystem.AutoRunSpindexer());
        NamedCommands.registerCommand("SpindexForever", m_IndexSubsystem.AutoRunSpindexerForever());
        NamedCommands.registerCommand("Intake", m_IntakeSubsystem.AutoRunIntake());
        NamedCommands.registerCommand("MoveIntakeDOWN", m_IntakeSubsystem.AutoLowerIntakeDOWN());
        NamedCommands.registerCommand("MoveIntakeUP", m_IntakeSubsystem.AutoBringIntakeUP());
        NamedCommands.registerCommand("Hang", m_HangSubsystem.AutoHangBot());

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-front",
            0.15748, -0.1353312, 0.35306,
            0.0, 0.0, 0.0
        );
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-back",
            -0.321, -0.15, 0.4699,
            0, 0, 0
        );

        drivetrain.configureAutoBuilder();

        // Must be built AFTER configureAutoBuilder()
        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Selector", m_autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        joystick.leftTrigger().whileTrue(m_IntakeSubsystem.RunIntake());
        joystick.leftBumper().whileTrue(m_IntakeSubsystem.RunIntakeReverse());
        joystick.povUp().whileTrue(m_HangSubsystem.HangRobotUp());
        joystick.povDown().whileTrue(m_HangSubsystem.HangRobotDown());
        joystick.b().onTrue(m_IntakeSubsystem.SetIntakeArmDown());
        joystick.a().onTrue(m_IntakeSubsystem.SetIntakeArmUp());
        joystick.povLeft().whileTrue(m_IndexSubsystem.RunSpindexerWOShooter());
        joystick.povRight().whileTrue(
            Commands.parallel(
                m_shooterSubsystem.CrackCocaineShooter(),
                m_IndexSubsystem.RunSpindexer()
            ));
        joystick.x().whileTrue(
            Commands.parallel(
                m_shooterSubsystem.JustShoot(),
                m_IndexSubsystem.RunSpindexer()
            ));
        joystick.y().whileTrue(
            Commands.parallel(
                m_shooterSubsystem.unJamShooter(),
                m_IndexSubsystem.RunSpindexerReverse()
            ));
        joystick.rightTrigger().whileTrue(
            Commands.parallel(
                drivetrain.aimAtHub(
                    drive,
                    () -> -joystick.getLeftY() * MaxSpeed,
                    () -> -joystick.getLeftX() * MaxSpeed,
                    MaxAngularRate
                ),
                m_shooterSubsystem.MoveShooterWithDistance(() -> drivetrain.getState().Pose),
                m_IndexSubsystem.RunSpindexer()
            )
        );
        joystick.rightBumper().whileTrue(
            Commands.parallel(
                drivetrain.aimAtAllianceSide(
                    drive,
                    () -> -joystick.getLeftY() * MaxSpeed,
                    () -> -joystick.getLeftX() * MaxSpeed,
                    MaxAngularRate
                ),
                m_shooterSubsystem.MoveShooterWithDistance(() -> drivetrain.getState().Pose),
                m_IndexSubsystem.RunSpindexer()
            )
        );

        joystick2.x().onTrue(m_shooterSubsystem.velocityIncrease());
        joystick2.y().onTrue(m_shooterSubsystem.velocityDecrease());
        joystick2.a().onTrue(m_IndexSubsystem.velocityIncrease());
        joystick2.b().onTrue(m_IndexSubsystem.velocityDecrease());
        joystick2.rightTrigger().onTrue(m_IntakeSubsystem.SpeedIncrease());
        joystick2.leftTrigger().onTrue(m_IntakeSubsystem.SpeedDecrease());
        joystick2.leftBumper().onTrue(m_shooterSubsystem.velocityReset());
        joystick2.rightBumper().onTrue(m_IndexSubsystem.velocityReset());

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Keep the PDH switchable channel (ch 23) always on
        pdh.setSwitchableChannel(true);

        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(k180deg)));

        joystick.back().onTrue(Commands.runOnce(() -> {
            if (LimelightHelpers.getTV("limelight-front")) {
                Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-front");
                drivetrain.resetPose(visionPose);
                System.out.println("Pose reset from front camera: " + visionPose);
            } else if (LimelightHelpers.getTV("limelight-back")) {
                Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-back");
                drivetrain.resetPose(visionPose);
                System.out.println("Pose reset from back camera: " + visionPose);
            } else {
                System.out.println("WARNING: No vision targets seen, pose not reset");
            }
        }, drivetrain));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
            // Briefly lock wheels so steer motors reach their initial heading
            // before drive motors spin up, preventing the auto-start lurch.
            drivetrain.applyRequest(() -> brake).withTimeout(0.25),
            m_autoChooser.getSelected()
        );
    }
}