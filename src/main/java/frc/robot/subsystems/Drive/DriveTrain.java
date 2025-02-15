// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simulation.Field;
import frc.robot.simulation.SimulatableSparkMax;

public class DriveTrain extends SubsystemBase {

  private RobotConfig config;

  SimulatableSparkMax leftFront, leftRear, rightFront, rightRear;
  AHRS gyro;
  DifferentialDrive drive;
 // RelativeEncoder leftEncoder, rightEncoder;

  public static DriveTrain instance;
  private final Field field = Field.getInstance();
  private double x = 7.889, y = 5.963;
  private Rotation2d heading = new Rotation2d();

  private double estimatedX = 0.0;
  private double estimatedY = 0.0;
  private double lastTimeStamp = Timer.getFPGATimestamp();

  private static final double kTrackWidth = Units.inchesToMeters(20.75);
  private static final double kWheelRadius = Units.inchesToMeters(3.0);
  private static final double kGearRatio = 10.71;

    private final LinearSystem<N2, N2, N2> mDrivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
    private final DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
      mDrivetrainSystem, DCMotor.getCIM(2), kGearRatio, kTrackWidth, kWheelRadius, null);

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);

 // private final DifferentialDriveOdometry odometry;

  public static DriveTrain getInstance() {
    if(instance == null) {
      instance = new DriveTrain();
    } 
    return instance;
  }


  /** Creates a new DriveTrain. */
  public DriveTrain() {

    gyro = new AHRS(NavXComType.kMXP_SPI);
    resetGyro();

    leftFront = new SimulatableSparkMax(DriveConstants.kLeftFrontPort, MotorType.kBrushed);
    leftRear = new SimulatableSparkMax(DriveConstants.kLeftRearPort, MotorType.kBrushed);
    rightFront = new SimulatableSparkMax(DriveConstants.kRightFrontPort, MotorType.kBrushed);
    rightRear = new SimulatableSparkMax(DriveConstants.kRightRearPort, MotorType.kBrushed);

    leftFront.setCANTimeout(250);
    leftRear.setCANTimeout(250);
    rightFront.setCANTimeout(250);
    rightRear.setCANTimeout(250);

    drive = new DifferentialDrive(leftFront, rightFront);

    //leftEncoder = leftFront.getEncoder();
    //rightEncoder = rightFront.getEncoder();

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig LFconfig = new SparkMaxConfig();
    SparkMaxConfig LRconfig = new SparkMaxConfig();
    SparkMaxConfig RFconfig = new SparkMaxConfig();
    SparkMaxConfig RRconfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(DriveConstants.smartCurrentLimit).voltageCompensation(DriveConstants.voltageCompensation);
    LFconfig.apply(globalConfig);
    LRconfig.apply(globalConfig).follow(DriveConstants.kLeftFrontPort);
    RFconfig.apply(globalConfig).inverted(true);//.encoder.inverted(true);
    RRconfig.apply(globalConfig).follow(DriveConstants.kRightFrontPort).inverted(true);

    leftFront.configure(LFconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftRear.configure(LRconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFront.configure(RFconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightRear.configure(RRconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //leftEncoder.setPosition(0);
    //rightEncoder.setPosition(0);

    //odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putData("Field", field);
    Logger.recordMetadata("Subsystem", "DriveTrain olusturuldu.");
    Logger.recordOutput("NavX baglanti durumu", isGyroConnected());

     // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getEstimatedPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveWithChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

public Command createPathFollowingCommand(PathPlannerTrajectory trajectory) {
    return new FunctionalCommand(
        () -> {}, 
        () -> {
            double currentTime = Timer.getFPGATimestamp();
            PathPlannerTrajectoryState desiredState = trajectory.sample(currentTime);

           // Pose2d currentPose = getPose();

            // Robotun şimdiki pozisyonu ile hedef pozisyon arasındaki farkı bul
            ChassisSpeeds speeds = kinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(
                    desiredState.linearVelocity,
                    desiredState.linearVelocity
                )
            );

            driveWithChassisSpeeds(speeds);
        }, 
        (interrupted) -> arcadeDriveVolts(0, 0), 
        () -> false, 
        this
    );
}


  public void drive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
    Logger.recordOutput("DriveTrain", "Robot hizi: " + speed);
    Logger.recordOutput("DriveTrain", "Robot donus hizi: " + rotation);

  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    double estimatedSpeed = (leftFront.get() + rightFront.get()) / 2.0;
    double estimatedRotation = Math.toRadians(gyro.getRate());

    return new ChassisSpeeds(estimatedSpeed, 0, estimatedRotation);
}


  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    arcadeDriveVolts(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  public void updatePosition(double forwardSpeed, double rotationSpeed) {
    x += forwardSpeed * Math.cos(heading.getRadians()) * 0.02; 
    y += forwardSpeed * Math.sin(heading.getRadians()) * 0.02; 

    heading = heading.plus(Rotation2d.fromDegrees(rotationSpeed * 5));
    field.setRobotPose(new Pose2d(new Translation2d(x, y), heading));
  }

  public Pose2d getEstimatedPose() {
    double deltaTime = Timer.getFPGATimestamp() - lastTimeStamp;
    lastTimeStamp = Timer.getFPGATimestamp();

    // Motor çıkışlarına dayalı tahmini hız belirleme
    double estimatedSpeed = 1.0 * (leftFront.get() + rightFront.get()) / 2.0; // Buradaki katsayıyı ayarlayabilirsiniz.

    // Gyro'dan açıyı al
    double headingRad = Math.toRadians(gyro.getAngle());
    Rotation2d rotation = new Rotation2d(headingRad);

    // X ve Y tahminini güncelle (Dead Reckoning)
    estimatedX += estimatedSpeed * deltaTime * Math.cos(headingRad);
    estimatedY += estimatedSpeed * deltaTime * Math.sin(headingRad);

    return new Pose2d(estimatedX, estimatedY, rotation);
}

  public void resetPose(Pose2d newPose) {
    // Gyro'yu verilen pozisyon açısına sıfırla
    gyro.setAngleAdjustment(newPose.getRotation().getDegrees());

    // X ve Y pozisyonlarını sıfırla
    estimatedX = newPose.getX();
    estimatedY = newPose.getY();

    // Son zaman damgasını sıfırla
    lastTimeStamp = Timer.getFPGATimestamp();

    Logger.recordOutput("DriveTrain", "Pozisyon sıfırlandı: " + newPose);
  }


 /*  
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }

  public void updateOdometry() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }
*/

  public double getHeading() {
    return Math.toRadians(gyro.getAngle());
  }

  public void setGyroAngle(double angle) {
    gyro.setAngleAdjustment(angle);
    Logger.recordOutput("DriveTrain", "Gyro acisi ayarlandi: " + angle);
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public double getGyroYaw() {
    return gyro.getYaw();
  }

  public double getGyroPitch() {
    return gyro.getPitch();
  }

  public double getGyroRoll() {
    return gyro.getRoll();
  }

  public void resetGyro() {
    gyro.reset();
    Logger.recordOutput("DriveTrain", "Gyro sifirlandi.");
  }

  public boolean isGyroConnected() {
    return gyro.isConnected();
  }

  public void arcadeDriveVolts(double leftVolts, double rightVolts) {
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
    //drive.feed();
  }

  public void getAccel() {
    Logger.recordOutput("DriveTrain", "X ivmesi: " + gyro.getWorldLinearAccelX());
    Logger.recordOutput("DriveTrain", "Y ivmesi: " + gyro.getWorldLinearAccelY());
    Logger.recordOutput("DriveTrain", "Z ivmesi: " + gyro.getWorldLinearAccelZ());
  }

  @Override
  public void periodic() {
    Rotation2d currentHeading = Rotation2d.fromDegrees(gyro.getAngle());
    field.setRobotPose(new Pose2d(new Translation2d(x, y), currentHeading));

    Logger.recordOutput("NavX/Angle", gyro.getAngle());
    Logger.recordOutput("NavX/Yaw", gyro.getYaw());
    Logger.recordOutput("NavX/Pitch", gyro.getPitch());
    Logger.recordOutput("NavX/Roll", gyro.getRoll());
    getAccel();
  }

  @Override
  public void simulationPeriodic() {
      drivetrainSim.setInputs(
        leftFront.get() * RobotController.getInputVoltage(),
        rightFront.get() * RobotController.getInputVoltage()
      );
      drivetrainSim.update(0.02);
  }
}
