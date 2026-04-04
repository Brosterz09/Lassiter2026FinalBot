package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final Servo blinkin = new Servo(0); // the 0 is hte PWM port the LED controller is plugged into on the roboRIO

    private final SendableChooser<Double> ledChooser = new SendableChooser<>();

    public LEDSubsystem() {
        ledChooser.setDefaultOption("Rainbow",      0.77);
        ledChooser.addOption("Fire",                0.57);
        ledChooser.addOption("Blue Chase",         -0.29);
        ledChooser.addOption("Red Heartbeat",       0.25);
        ledChooser.addOption("Confetti",            0.81);
        ledChooser.addOption("Solid Green",         0.73);
        ledChooser.addOption("Knight Rider",        0.35);
        ledChooser.addOption("Solid Red",           0.61);
        ledChooser.addOption("Solid Blue",          0.87);
        ledChooser.addOption("Off",                 0.99);

        SmartDashboard.putData("LED Mode", ledChooser);
    }

    @Override
    public void periodic() {
        blinkin.set(ledChooser.getSelected());
    }
}
