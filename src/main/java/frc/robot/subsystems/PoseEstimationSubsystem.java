// Slightly modified code from TigerLib 2023
// Some of this code was copied from Team 7028 - Binary Battalion's swerve-test repository
// https://github.com/STMARobotics/swerve-test/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java

package frc.robot.subsystems;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class PoseEstimationSubsystem extends SubsystemBase {
  
  private final DriveSubsystem driveSubsystem;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final NetworkTable networkTable; 
  private final NetworkTableEntry botPoseNetworkTableEntry;
  private final NetworkTableEntry jsonDumpNetworkTableEntry;

  /* EDIT CODE BELOW HERE */
  
  private static final String limelightNetworktableName = "limelight";
 
  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(DriveConstants.X_POS_TRUST, DriveConstants.Y_POS_TRUST, Units.degreesToRadians(DriveConstants.ANGLE_TRUST));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(VisionConstants.VISION_X_POS_TRUST,
   VisionConstants.VISION_Y_POS_TRUST, Units.degreesToRadians(VisionConstants.VISION_ANGLE_TRUST));
  
  // IMPORTANT: Make sure your driveSubsystem has the methods getPose, getRotation2d, getModulePositions, and resetOdometry
  
  /* EDIT CODE ABOVE HERE (ONLY TOUCH THE REST OF THE CODE IF YOU KNOW WHAT YOU'RE DOING) */

  private double lastTimeStampSeconds = 0;

  public PoseEstimationSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVE_KINEMATICS,
      driveSubsystem.getRotation2d(),
      driveSubsystem.getModulePositions(),
      new Pose2d(), // This is the position for where the robot starts the match, use setPose() to set it in autonomous init
      stateStdDevs,
      visionMeasurementStdDevs
    );

    networkTable = NetworkTableInstance.getDefault().getTable(limelightNetworktableName);
    botPoseNetworkTableEntry = networkTable.getEntry("botpose");
    jsonDumpNetworkTableEntry = networkTable.getEntry("json");
  }

  @Override
  public void periodic() {
    // Gets the robot's pose from the network table with an empty array as the default value
    double[] botPose = botPoseNetworkTableEntry.getDoubleArray(new double[]{});
    String jsonDump = jsonDumpNetworkTableEntry.getString("{}");  

    double currentTimeStampSeconds = lastTimeStampSeconds;
    // Attempts to get the time stamp for when the robot pose was calculated
    try {
      ObjectMapper mapper = new ObjectMapper();
      JsonNode jsonNodeData = mapper.readTree(jsonDump);
      double timeStampValue = jsonNodeData.path("Results").path("ts").asDouble();
      SmartDashboard.putNumber("tsValue", timeStampValue);
      if (timeStampValue != 0) {
        // Converts from milleseconds to seconds
        currentTimeStampSeconds = timeStampValue / 1000;
      }
    } catch (JsonProcessingException e) {
      SmartDashboard.putString("Json Parsing Error", e.getStackTrace().toString());
    }

    // Updates the pose estimator's position if limelight position data was recieved with a new time stamp
    if (botPose.length != 0 && currentTimeStampSeconds > lastTimeStampSeconds) {
      double robotX = botPose[0] + FieldConstants.FIELD_LENGTH_METERS / 2;
      double robotY = botPose[1] + FieldConstants.FIELD_WIDTH_METERS / 2;
      Rotation2d robotRotation = Rotation2d.fromDegrees(botPose[5]);
      Pose2d limelightVisionMeasurement = new Pose2d(robotX, robotY, robotRotation);
      poseEstimator.addVisionMeasurement(limelightVisionMeasurement, currentTimeStampSeconds);
      SmartDashboard.putString("Limelight Pose", limelightVisionMeasurement.toString());
    }

    lastTimeStampSeconds = currentTimeStampSeconds;

    // Uses the swerve's sensors to update the pose estimator
    poseEstimator.update(
      driveSubsystem.getRotation2d(),
      driveSubsystem.getModulePositions()
    );

    Pose2d pose = getPose();
    // Updates the odometry to the pose estimator's pose
    driveSubsystem.resetOdometry(pose);
    SmarterDashboardRegistry.setPose(pose);

    //returns pose in meters
    SmartDashboard.putString("Estimated Pose", pose.toString()); 
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator to the specified pose. Keeps the current
   * rotation and module positions.
   * @param newPose The Pose2d object
   */
  public void setPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      driveSubsystem.getRotation2d(),
      driveSubsystem.getModulePositions(),
      newPose
    );
  }
  
  /**
   * Resets the position on the field to 0,0, 0-degrees, with forward being downfield. 
   * This resets what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setPose(new Pose2d());
  }

  public void addPoseEstimatorSwerveMeasurement() {
    poseEstimator.update(
      driveSubsystem.getRotation2d(),
      driveSubsystem.getModulePositions()
    );
  }
  
  public void addPoseEstimatorVisionMeasurement(Pose2d visionMeasurement, double currentTimeStampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
    SmarterDashboardRegistry.setLimelightPose(visionMeasurement);
  }

  public void resetOdometryAndRotation(Pose2d pose, double angle) {
    driveSubsystem.zeroHeading();
    driveSubsystem.setGyroOffset(angle);
    poseEstimator.resetPosition(pose.getRotation(), driveSubsystem.getModulePositions(), pose);
  }

  public void setPoseEstimatorVisionConfidence(double xStandardDeviation, double yStandardDeviation,
    double thetaStandardDeviation) {
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }
}