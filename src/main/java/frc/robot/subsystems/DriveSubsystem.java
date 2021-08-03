// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class DriveSubsystem extends SubsystemBase {
  // // The motors on the left side of the drive.
  // private final SpeedControllerGroup m_leftMotors =
  //     new SpeedControllerGroup(
  //         new PWMVictorSPX(Constants.DriveConstants.kLeftMotor1Port),
  //         new PWMVictorSPX(Constants.DriveConstants.kLeftMotor2Port));

  // // The motors on the right side of the drive.
  // private final SpeedControllerGroup m_rightMotors =
  //     new SpeedControllerGroup(
  //         new PWMVictorSPX(Constants.DriveConstants.kRightMotor1Port),
  //         new PWMVictorSPX(Constants.DriveConstants.kRightMotor2Port));
  private final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backLeftMotor = new CANSparkMax(Constants.DriveConstants.BACK_LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backRightMotor = new CANSparkMax(Constants.DriveConstants.BACK_RIGHT_MOTOR, MotorType.kBrushless);
  
  // private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  // private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);
  private int update = 0;

  // The robot's drive
  // private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public final CANEncoder m_leftEncoder = frontLeftMotor.getEncoder();  
  public final CANEncoder m_rightEncoder = frontRightMotor.getEncoder();
  // The left-side drive encoder
  /**
   * Code below was code frome xample
   */
  // private final Encoder m_leftEncoder =
  //     new Encoder(
  //         Constants.DriveConstants.kLeftEncoderPorts[0],
  //         Constants.DriveConstants.kLeftEncoderPorts[1],
  //         Constants.DriveConstants.kLeftEncoderReversed);

  // // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(
  //         Constants.DriveConstants.kRightEncoderPorts[0],
  //         Constants.DriveConstants.kRightEncoderPorts[1],
  //         Constants.DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  // public static AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private String devicekeyfrontleft = "SPARK MAX [4]";
  private String devicekeybackleft = "SPARK MAX [5]";
  private String devicekeyfrontright = "SPARK MAX [2]";
  private String devicekeybackright = "SPARK MAX [3]";
  private String fieldKeyPosition = "Position";
  private String fieldKeyVelocity = "Velocity";
  private SimDeviceSim m_frontleftEncoderSim;
  private SimDeviceSim m_frontrightEncoderSim;
  private SimDeviceSim m_backleftEncoderSim;
  private SimDeviceSim m_backrightEncoderSim;
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;
  private ADXRS450_GyroSim m_gyroSim;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    //CAN encoders for NEO have different set up than below
    // m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    


    frontRightMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();
    frontLeftMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();

    frontRightMotor.setIdleMode(IdleMode.kBrake);
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kCoast);
    backLeftMotor.setIdleMode(IdleMode.kCoast);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    frontRightMotor.setInverted(true);

    m_leftEncoder.setPositionConversionFactor(Constants.DriveConstants.DISTANCE_PER_REVOLUTION / Constants.DriveConstants.kDriveGearing);
    m_rightEncoder.setPositionConversionFactor(Constants.DriveConstants.DISTANCE_PER_REVOLUTION / Constants.DriveConstants.kDriveGearing);
    m_gyro.reset();

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              Constants.DriveConstants.kDrivetrainPlant,
              Constants.DriveConstants.kDriveGearbox,
              Constants.DriveConstants.kDriveGearing,
              Constants.DriveConstants.kTrackwidthMeters,
              Constants.DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

            // Constants.DriveConstants.kDriveGearbox,
            // Constants.DriveConstants.kDriveGearing,               //gearbox reduction
            // 7.5,                //MOI, don't know how to get
            // 68.0,               //mass of robot
            // Constants.DriveConstants.PULSES_PER_REVOLUTION, //6" radius wheels
            // Constants.DriveConstants.kTrackwidthMeters,    //track width
            //  // The standard deviations for measurement noise:
            // // x and y:          0.001 m
            // // heading:          0.001 rad
            // // l and r velocity: 0.1   m/s
            // // l and r position: 0.005 m
            // VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      // m_leftEncoderSim = new SimDeviceSim(devicekeyleft);
      // m_rightEncoderSim = new SimDeviceSim(devicekeyright);
      m_frontleftEncoderSim = new SimDeviceSim(devicekeyfrontleft);
      m_frontrightEncoderSim = new SimDeviceSim(devicekeyfrontright);
      m_backleftEncoderSim = new SimDeviceSim(devicekeybackleft);
      m_backrightEncoderSim = new SimDeviceSim(devicekeybackright);
      m_gyroSim = new ADXRS450_GyroSim(m_gyro);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());
    m_fieldSim.setRobotPose(getPose());
    // m_drive.feed();
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    update += 1;
    m_drivetrainSimulator.setInputs(
        frontLeftMotor.get() * RobotController.getBatteryVoltage(),
        frontRightMotor.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);
    
    SimDouble positionFrontLeft = m_frontleftEncoderSim.getDouble(fieldKeyPosition);
    positionFrontLeft.get();
    positionFrontLeft.set(m_drivetrainSimulator.getLeftPositionMeters());
    SimDouble VelocityFrontLeft = m_frontleftEncoderSim.getDouble(fieldKeyVelocity);
    VelocityFrontLeft.get();
    VelocityFrontLeft.set(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

    SimDouble positionBackLeft = m_backleftEncoderSim.getDouble(fieldKeyPosition);
    positionBackLeft.get();
    positionBackLeft.set(m_drivetrainSimulator.getLeftPositionMeters());
    SimDouble VelocityBackLeft = m_backleftEncoderSim.getDouble(fieldKeyVelocity);
    VelocityBackLeft.get();
    VelocityBackLeft.set(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

    
    SimDouble positionFrontRight = m_frontrightEncoderSim.getDouble(fieldKeyPosition);
    positionFrontRight.get();
    positionFrontRight.set(m_drivetrainSimulator.getRightPositionMeters());
    SimDouble VelocityFrontRight = m_frontrightEncoderSim.getDouble(fieldKeyVelocity);
    VelocityFrontRight.get();
    VelocityFrontRight.set(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    SimDouble positionBackRight = m_backrightEncoderSim.getDouble(fieldKeyPosition);
    positionBackRight.get();
    positionBackRight.set(m_drivetrainSimulator.getRightPositionMeters());
    SimDouble VelocityBackRight = m_backrightEncoderSim.getDouble(fieldKeyVelocity);
    VelocityBackRight.get();
    VelocityBackRight.set(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    // if(update > 50) {
    //   update = 0;
    //   System.out.println("\r\nRight Position: ");
    //   System.out.print(m_rightEncoder.getPosition());
    //   System.out.println("\r\nRight Velocity: ");
    //   System.out.print(m_rightEncoder.getVelocity());
    //   System.out.println("\r\nLeft Position: ");
    //   System.out.print(m_leftEncoder.getPosition());
    //   System.out.println("\r\nLeft Velocity: ");
    //   System.out.print(m_leftEncoder.getVelocity());
    // }


    // m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    // m_rightEncoderSim.set(m_drivetrainSimulator.getRightPositionMeters());
    // m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity()/60, m_rightEncoder.getVelocity()/60);
    
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    
    // if(RobotContainer.driverJoystick.getRawButton(11))
    // {
    //   frontLeftMotor.set(0.2 * (fwd + rot));
    //   frontRightMotor.set(fwd - rot);
    // }
    // else if(RobotContainer.driverJoystick.getRawButton(12))
    // {
    //   frontLeftMotor.set(fwd + rot);
    //   frontRightMotor.set(0.2 * (fwd - rot));
    // }
    // else {
      frontLeftMotor.set(fwd + rot);
      frontRightMotor.set(fwd - rot);
      if(update > 50) {
        update = 0;
        System.out.print("Left motor: ");
        System.out.print(fwd+rot);
        System.out.print(" ");
        System.out.println(frontLeftMotor.get());
      }
    // }
  }
  public void HelixDrive(double throttle, double twist) {
    // double throttle = RobotContainer.driverJoystick.getY();
    // double twist = RobotContainer.driverJoystick.getZ() * -0.65;

    double saturatedInput;
    double greaterInput = Math.max(Math.abs(twist), Math.abs(throttle));
    double lesserInput = Math.min(Math.abs(twist), Math.abs(throttle));

    if (greaterInput > 0.0) 
      saturatedInput = (lesserInput/greaterInput) + 1.0;
    else 
      saturatedInput = 1.0;

    throttle = throttle / saturatedInput;
    twist = twist/saturatedInput;
    if(Math.abs(throttle) < 0.1) 
      throttle = 0;
    if(Math.abs(twist) < 0.1) 
      twist = 0;
    
    arcadeDrive(throttle, twist);
  }

  

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  // public void tankDriveVolts(double leftVolts, double rightVolts) {
  //   var batteryVoltage = RobotController.getBatteryVoltage();
  //   if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
  //     leftVolts *= batteryVoltage / 12.0;
  //     rightVolts *= batteryVoltage / 12.0;
  //   }
  //   frontLeftMotor.setVoltage(leftVolts);
  //   frontRightMotor.setVoltage(-rightVolts);
  //   // m_drive.feed();
  // }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  // public void setMaxOutput(double maxOutput) {
  //   frontLeftMotor.set
  //   m_drive.setMaxOutput(maxOutput);
  // }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360)
        * (Constants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
