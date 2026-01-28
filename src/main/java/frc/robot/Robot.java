package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private Vision vision;
    @Override
    public void robotInit() {
        vision = new Vision();
        
    }

    @Override
    public void robotPeriodic() {
        vision.periodic();
        // DriverStation.reportWarning(""+vision.getEstimationStdDevs().get(1, 1), null);
    }

    // @Override
    // public void disabledPeriodic() {
    // }

    // @Override
    // public void teleopInit() {
    //     resetPose();
    // }

    // @Override
    // public void teleopPeriodic() {
    // }

    // public void resetPose() {
    // }
}
