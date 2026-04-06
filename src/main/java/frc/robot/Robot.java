// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        double matchTime = DriverStation.getMatchTime();

        // Same voltage source the Driver Station uses
        double voltage = RobotController.getBatteryVoltage();

        String robotMode;
        if (DriverStation.isAutonomous()) {
            robotMode = "Autonomous";
        } else if (DriverStation.isTeleop()) {
            robotMode = "Teleop";
        } else if (DriverStation.isDisabled()) {
            robotMode = "Disabled";
        } else {
            robotMode = "Unknown";
        }

        // ── Voltage ───────────────────────────────────────────────────────
        SmartDashboard.putNumber("Battery Voltage", voltage);

        // ── Auto result (only available once FMS sends game data in teleop) ──
        String gameData = DriverStation.getGameSpecificMessage();
        if (!gameData.isEmpty()) {
            boolean redInactiveFirst = gameData.charAt(0) == 'R';
            var alliance = DriverStation.getAlliance();
            boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            boolean weWonAuto = isRed ? !redInactiveFirst : redInactiveFirst;
            SmartDashboard.putBoolean("Won Auto", weWonAuto);
            SmartDashboard.putString("Auto Result", weWonAuto ? "WON AUTO" : "LOST AUTO");
        } else {
            SmartDashboard.putString("Auto Result", "Waiting...");
        }

        // ── COMP layout keys ──────────────────────────────────────────────
        SmartDashboard.putNumber("Match Time", matchTime);
        SmartDashboard.putString("Robot Mode", robotMode);
        updatePeriodTimings(matchTime, "Hub Status", "Time Until Next Change (s)", "Time Until Our Period (s)", "Our Period Active");

        // ── PRACTICE layout keys ──────────────────────────────────────────
        SmartDashboard.putNumber("PRACTICE_MatchTime", matchTime);
        SmartDashboard.putString("PRACTICE_RobotMode", robotMode);
        updatePeriodTimings(matchTime, "PRACTICE_HubStatus", "PRACTICE_TimeUntilNextChange", "PRACTICE_TimeUntilOurPeriod", "PRACTICE_OurPeriodActive");
    }

    /**
     * Shared logic for computing and publishing hub period timing.
     * Called once for the comp layout and once for the practice layout.
     */
    private void updatePeriodTimings(
        double matchTime,
        String hubStatusKey,
        String nextChangeKey,
        String ourPeriodKey,
        String ourPeriodActiveKey
    ) {
        if (!DriverStation.isTeleopEnabled()) {
            SmartDashboard.putString(hubStatusKey, "N/A");
            SmartDashboard.putNumber(nextChangeKey, -1);
            SmartDashboard.putNumber(ourPeriodKey, -1);
            SmartDashboard.putBoolean(ourPeriodActiveKey, false);
            return;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) {
            SmartDashboard.putString(hubStatusKey, "Waiting for game data...");
            SmartDashboard.putNumber(nextChangeKey, -1);
            SmartDashboard.putNumber(ourPeriodKey, -1);
            SmartDashboard.putBoolean(ourPeriodActiveKey, false);
            return;
        }

        // 'R' = red hub goes inactive first (red loses shifts 1 & 3)
        boolean redInactiveFirst = gameData.charAt(0) == 'R';

        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        // Shift 1 active alliance = the one that WON auto
        boolean weAreActiveInShift1 = isRed ? !redInactiveFirst : redInactiveFirst;

        // Shift boundaries — matchTime counts DOWN from 135
        // >130 = transition, 130-105 = Shift 1, 105-80 = Shift 2,
        //  80-55 = Shift 3, 55-30 = Shift 4, <30 = End Game
        double[] boundaries = {130.0, 105.0, 80.0, 55.0, 30.0};

        int currentShift;
        if      (matchTime > 130) currentShift = 0; // transition
        else if (matchTime > 105) currentShift = 1;
        else if (matchTime > 80)  currentShift = 2;
        else if (matchTime > 55)  currentShift = 3;
        else if (matchTime > 30)  currentShift = 4;
        else                      currentShift = 5; // end game

        // Odd shifts (1,3) = shift1 winner; even shifts (2,4) = other alliance
        boolean weAreActiveNow;
        if (currentShift == 0 || currentShift == 5) {
            weAreActiveNow = true; // always active in transition & end game
        } else {
            weAreActiveNow = (currentShift % 2 == 1) ? weAreActiveInShift1 : !weAreActiveInShift1;
        }

        // Time until the next shift boundary
        double timeUntilNextChange = -1;
        for (double boundary : boundaries) {
            if (matchTime > boundary) {
                timeUntilNextChange = matchTime - boundary;
                break;
            }
        }

        // If we're inactive, next change = our period starts (shifts alternate)
        double timeUntilOurPeriod = weAreActiveNow ? 0 : timeUntilNextChange;

        SmartDashboard.putString(hubStatusKey, weAreActiveNow ? "OUR PERIOD" : "Their Period");
        SmartDashboard.putNumber(nextChangeKey, Math.max(0, timeUntilNextChange));
        SmartDashboard.putNumber(ourPeriodKey, Math.max(0, timeUntilOurPeriod));
        SmartDashboard.putBoolean(ourPeriodActiveKey, weAreActiveNow);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // Continuously seed pose from vision while disabled so that by the time
        // auto starts the odometry already reflects the robot's true field position.
        m_robotContainer.drivetrain.seedPoseFromVision();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // Final vision pose seed right before auto starts, in case the most recent
        // disabled-period update was stale. resetOdom is disabled in the auto files
        // so PathPlanner will not override this pose.
        if (!m_robotContainer.drivetrain.seedPoseFromVision()) {
            System.out.println("WARNING: No vision targets seen, starting pose not seeded from vision.");
        }
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
            System.out.println("Auto command scheduled");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}