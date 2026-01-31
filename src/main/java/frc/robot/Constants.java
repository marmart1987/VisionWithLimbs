package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants.driveGearRatios;

public class Constants {

        public static class Swerve {
                public static final COTSNeoSwerveConstants chosenModule = COTSNeoSwerveConstants
                                .SDSMK4i(driveGearRatios.SDSMK4i_L2);
                public static final double angleGearRatio = chosenModule.angleGearRatio;
                public static final double kMaxSpeedMetersPerSecond = 4.8;
                public static final double kTrackWidth = Units.inchesToMeters(18.5);
                public static final double kTrackLength = Units.inchesToMeters(18.5);
                public static final double kRobotWidth = Units.inchesToMeters(25 + 3.25 * 2);
                public static final double kRobotLength = Units.inchesToMeters(25 + 3.25 * 2);
                public static final double kMaxLinearSpeed = Units.feetToMeters(15.5);
                public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
                public static final double kWheelDiameter = Units.inchesToMeters(4);
                public static final double kWheelCircumference = kWheelDiameter * Math.PI;

                public static final double kDriveGearRatio = 6.75; // 6.75:1 SDS MK4 L2 ratio
                public static final double kSteerGearRatio = 12.8; // 12.8:1

                public static final double kDriveDistPerPulse = kWheelCircumference / 1024 / kDriveGearRatio;
                public static final double kSteerRadPerPulse = 2 * Math.PI / 1024;

                public enum ModuleConstants {
                        // Configuration for each swerve module
                        FL( // Front left
                                        10, 20, 0, 0, kTrackLength / 2, kTrackWidth / 2),
                        FR( // Front Right
                                        11, 21, 1, 0, kTrackLength / 2, -kTrackWidth / 2),
                        BL( // Back Left
                                        12, 22, 2, 0, -kTrackLength / 2, kTrackWidth / 2),
                        BR( // Back Right
                                        13, 23, 3, 0, -kTrackLength / 2, -kTrackWidth / 2);


                        public final int drivermotorID0;
                        public final int anglemotorID0;
                        public final int camcoderID;
                        public final double offset0;
                        public final Translation2d centerOffset;

                        /**
                         * Constructor for module constants
                         * @param drivermotorID0 Driver motor ID
                         * @param anglemotorID0 Angle motor ID
                         * @param camcoderID CANCoder ID
                         * @param offset0 Angle offset
                         * @param xOffset X offset from robot center
                         * @param yOffset Y offset from robot center
                         */
                        private ModuleConstants(
                                        int drivermotorID0,
                                        int anglemotorID0,
                                        int camcoderID,
                                        double offset0,
                                        double xOffset,
                                        double yOffset) {
                                this.drivermotorID0 = drivermotorID0;
                                this.anglemotorID0 = anglemotorID0;
                                this.camcoderID = camcoderID;
                                this.offset0 = offset0;
                                centerOffset = new Translation2d(xOffset, yOffset);

                        }
                }
        }

        public static final class MotorConstants {
                public static final int kDrivingMotorPinionTeeth = 14;

                public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

                public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                                * kWheelCircumferenceMeters)
                                / kDrivingMotorReduction;
        }

        public static class Vision {
                public static final String kCameraName = "MainCam";
                // Cam mounted facing forward, half a meter forward of center, half a meter up
                // from center.
                public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                                new Rotation3d(0, 0, 0));

                // The layout of the AprilTags on the field
                public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                                .loadField(AprilTagFields.k2026RebuiltWelded);

                // The standard deviations of our vision estimated poses, which affect
                // correction rate
                // (Fake values. Experiment and determine estimation noise on an actual robot.)
                public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        }

        public static final class NeoMotorConstants {
                public static final double kFreeSpeedRpm = 5676;
        }
}