package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private Vision vision;
    private Swerve drivetrain;
    @Override
    public void robotInit() {
        drivetrain = new Swerve();
        vision = new Vision(drivetrain::addVisionMeasurement);
    }
    @Override
    public void robotPeriodic() {
        vision.periodic();
    }
}
