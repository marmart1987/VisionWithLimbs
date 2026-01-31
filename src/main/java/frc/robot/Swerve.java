package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MAXSwerveModule;

public class Swerve extends SubsystemBase {
    public AHRS m_gyro;
    public final SwerveDrivePoseEstimator poseEstimator;

    // Construct the swerve modules with their respective constants.
    // The SwerveModule class will handle all the details of controlling the
    // modules.
    private final MAXSwerveModule[] swerveMods = {
            new MAXSwerveModule(Constants.Swerve.ModuleConstants.FL),
            new MAXSwerveModule(Constants.Swerve.ModuleConstants.FR),
            new MAXSwerveModule(Constants.Swerve.ModuleConstants.BL),
            new MAXSwerveModule(Constants.Swerve.ModuleConstants.BR)
    };

    // Kinematics for the swerve drive, based on the module locations.
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            swerveMods[0].getModuleConstants().centerOffset,
            swerveMods[1].getModuleConstants().centerOffset,
            swerveMods[2].getModuleConstants().centerOffset,
            swerveMods[3].getModuleConstants().centerOffset);
    
    public Swerve() {
        // Initialize the gyro (NavX) on the USB port.
        m_gyro = new AHRS(NavXComType.kUSB1);
        // Reset the gyro to zero heading.
        m_gyro.reset();


        // Define the standard deviations for the pose estimator, which determine how
        // fast the pose estimate converges to the vision measurement. This should
        // depend on the
        // vision measurement noise and how many or how frequently vision measurements
        // are applied to the pose estimator.

        //stateStdDevs Standard deviations of the pose estimate (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust your state estimate less
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        //Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust the vision pose measurement less.
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroYaw(), getModulePositions(), new Pose2d(),
                stateStdDevs,
                visionStdDevs);
    }

    public void periodic() {
        // Update the pose estimator using the wheel encoders and gyro.
        poseEstimator.update(getGyroYaw(), getModulePositions());
    }
    /**
     * 
     * @return The current positions of all four swerve modules as an array of SwerveModulePosition.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = swerveMods[0].getPosition();
        positions[1] = swerveMods[1].getPosition();
        positions[2] = swerveMods[2].getPosition();
        positions[3] = swerveMods[3].getPosition();

        return positions;
    }

    /**
     * 
     * @return The current yaw of the robot from the gyro as a Rotation2d.
     */
    public Rotation2d getGyroYaw() {
        return m_gyro.getRotation2d();
    }

    /**
     * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
     */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
        DriverStation.reportWarning("" + visionMeasurement.getX() + ", " + visionMeasurement.getY() + ", " + visionMeasurement.getRotation().getDegrees(), false);
    }

    /**
     * See
     * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds,
        stdDevs);
        DriverStation.reportWarning("" + visionMeasurement.getX() + ", " + visionMeasurement.getY() + ", " + visionMeasurement.getRotation().getDegrees(), false);
    }

}
