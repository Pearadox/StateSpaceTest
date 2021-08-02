// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    /**
     * These values below are used for Pearadox code
     */
    public static final int FRONT_RIGHT_MOTOR = 2;
    public static final int BACK_RIGHT_MOTOR = 3;

    public static final int FRONT_LEFT_MOTOR = 4;
    public static final int BACK_LEFT_MOTOR = 5;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6.0d); // Meters
    public static final double DISTANCE_PER_REVOLUTION = WHEEL_DIAMETER * Math.PI;
    public static final double PULSES_PER_REVOLUTION = 42 * 5.6;

    /**
     * The values below are used in simulation
     */
    // public static final int kLeftMotor1Port = 0;
    // public static final int kLeftMotor2Port = 1;
    // public static final int kRightMotor1Port = 2;
    // public static final int kRightMotor2Port = 3;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 1.4; ///1.4 is Pearadox bot, 0.69 was original value;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    // public static final int kEncoderCPR = 1024; not used in Pearadox set up
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6.0d); //Pearadox value , original value is 0.15;
    
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        //original value (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        DISTANCE_PER_REVOLUTION / PULSES_PER_REVOLUTION;

    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.23f; //from Pearadox FRC-Characterization, original 0.22;
    public static final double kvVoltSecondsPerMeter = 3.61; //1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.529; // 0.2;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // These two values are "angular" kV and kA
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);

    // Example values only -- use what's on your physical robot!
    public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);
    public static final double kDriveGearing = 13.8;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kintakeSolenoid = 1;
    public static final int kArmMotorID = 13;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 6;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class ArmConstants {
    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from
     * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
     * configuration.
     */
    public static final int kSlotIdx = 0;
  
    /**
     * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;
  
    /**
     * set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 30;
  
    /**
     * Gains used in Motion Magic, to be adjusted accordingly
       * Gains(kp, ki, kd, kf, izone, peak output);
       */
    public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);

  }

}
