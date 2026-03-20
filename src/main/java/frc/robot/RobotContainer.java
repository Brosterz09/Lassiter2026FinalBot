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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    public double MaxSpeed = .8 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    // Limits how fast the rotation command can change to better match real robot feel in sim
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(MaxAngularRate * 2);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(() -> drivetrain.getState().Pose);
    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    public final IndexSubsystem m_IndexSubsystem = new IndexSubsystem(m_shooterSubsystem);
    public final HangSubsystem m_HangSubsystem = new HangSubsystem();
    


    public RobotContainer() {
        NamedCommands.registerCommand("Shoot", m_shooterSubsystem.AutoJustShoot());
        NamedCommands.registerCommand("TrigIntake", m_IntakeSubsystem.RunIntake());
        NamedCommands.registerCommand("BackShots", m_shooterSubsystem.BackShots());
        NamedCommands.registerCommand("Spindex", m_IndexSubsystem.AutoRunSpindexer());
        NamedCommands.registerCommand("Intake", m_IntakeSubsystem.AutoRunIntake());
        NamedCommands.registerCommand("MoveIntakeDOWN", m_IntakeSubsystem.AutoLowerIntakeDOWN());
        NamedCommands.registerCommand("MoveIntakeUP", m_IntakeSubsystem.AutoBringIntakeUP());
        NamedCommands.registerCommand("Hang", m_HangSubsystem.AutoHangBot());
        //  NamedCommands.registerCommand("Hang", m_hangSubsystem.HangUp());
        
        LimelightHelpers.setCameraPose_RobotSpace(
    "limelight-front",
    0.15748, -0.1353312, 0.35306,
    0.0, 0.0, 0.0
        );
        LimelightHelpers.setCameraPose_RobotSpace(
    "limelight-back",
    -0.328, -0.254, 0.35,
    180.0, 0, 180.0
        );
        drivetrain.configureAutoBuilder();
    
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(m_rotLimiter.calculate(-joystick.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
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
            )
            );
        joystick.y().whileTrue(
            Commands.parallel(
                m_shooterSubsystem.ReverseShooter(),
                m_IndexSubsystem.RunSpindexerReverse()
            )
            );
        joystick.rightTrigger().whileTrue(
            Commands.parallel(
                drivetrain.aimAtHub(
                    drive,
                    () ->  -joystick.getLeftY() * MaxSpeed,
                    () ->  -joystick.getLeftX() * MaxSpeed,
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
                    () ->  -joystick.getLeftY() * MaxSpeed,
                    () ->  -joystick.getLeftX() * MaxSpeed,
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
        // joystick.rightBumper().onTrue(drivetrain.CenterBot(drive, MaxAngularRate));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Keep the PDH switchable channel (ch 23) always on.
        pdh.setSwitchableChannel(true);

        // joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Re-apply alliance-correct operator perspective on start button press.
        // seedFieldCentric() is NOT used here because it resets the pose heading to 0°
        // and ignores the alliance perspective, causing controls to flip or pose to be wrong
        // depending on which direction the robot faces when pressed.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Reset pose from vision on select (back) button press.
        // Tries limelight-front first, falls back to limelight-back, no-ops if no targets.
    //     joystick.back().onTrue(Commands.runOnce(() -> {
    //         if (LimelightHelpers.getTV("limelight-front")) {
    //             Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-front");
    //             drivetrain.resetPose(visionPose);
    //             System.out.println("Pose reset from front camera: " + visionPose);
    //         } else if (LimelightHelpers.getTV("limelight-back")) {
    //             Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-back");
    //             drivetrain.resetPose(visionPose);
    //             System.out.println("Pose reset from back camera: " + visionPose);
    //         } else {
    //             System.out.println("WARNING: No vision targets seen, pose not reset");
    //         }
    //     }, drivetrain));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
            // Briefly lock wheels in X-pattern so steer motors reach their initial
            // path heading before the drive motors spin up, preventing the auto-start lurch.
            drivetrain.applyRequest(() -> brake).withTimeout(0.25),
            AutoBuilder.buildAuto("empty"));
            //AutoBuilder.buildAuto("TestAuto"));/
            //AutoBuilder.buildAuto("CenterAutoLEFT"));
    }
}

