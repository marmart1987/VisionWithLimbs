package frc.robot;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putNumber("Robot X Position", drivetrain.poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Robot Y Position", drivetrain.poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Robot Rotation", drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }
}
