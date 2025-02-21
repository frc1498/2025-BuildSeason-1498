// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */


 

public final class Constants {
  public static final boolean kWristRotateMotorEnabled = false;
  public static final boolean kWristSpinMotorEnabled = false;
  public static final boolean kClimberRotateMotorEnabled = true;
  public static final boolean kElevatorExtendMotorEnabled = false;
  public static final boolean kCoralIntakeSpinMotorEnabled = false;
  public static final boolean kCoralIntakerotateMotorEnabled = false;
  public static final boolean kArmRotateMotorEnabled = false;


  public static final boolean kSloMo = true;
  public static final double kSloMoFactor = 0.1;



  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;


  }


}
