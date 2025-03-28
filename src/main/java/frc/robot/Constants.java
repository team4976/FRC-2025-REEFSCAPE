// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  // Elevator Constants
  public static final int Elevator_Leader_Id = 30;
  public static final int Elevator_Follower_Id = 31;
   
  public static final double MaxMotorPosition = 31.8;
  public static final double MinMotorPosition = 0.5;
  public static final int elevatorLimitSwitchPort = 0;


  // Arm Constants
  public static final int armId = 50;

  // Auto Aim Constants
  public static final double leftOffset = -0.19;
  public static final double leftForwardOffest = 0.3;//0.3 was

  public static final double rightOffset = 0.145;
  public static final double rightForwardOffset = 0.4;

  public static final double poseAmbigMax = 0.5;
  public static final double rotationalrightoffset = -2.3;
  public static final double rotationalleftoffset = -1.25;//was -1.25 was -6.5?
  public static final double maxRotationalOutput = 0.1;
  public static final double maxTranslationalOutput = 0.5;
  public static final double modeChangeLimit = 0.2;
  public static final double scaleRotation = 4;
  public static final double scaleTranslation = 4;
  
  
  public static final double LeftOffset = -0.19;//was -0.14
  

  public static final double RightOffset = 0.15;// was 0.14

  public static final double ForwardOffset = 0.35;//was 0.34 at home


  public static final double L4Elevator = 29;//was 30.6 // was 30.45
  public static final double L4Arm = 0.346;//was 0.346 //was 0.364

  public static final double L3Elevator = 15;//was 15.7// was 15.9
  public static final double L3Arm = 0.346;//was 0.346
  
  public static final double L2Elevator = 5.5;//was 7.4 // was 6.2
  public static final double L2Arm = 0.346;//was 0.346


  // Rotational Offsets
  public static final HashMap<Integer, Double> tagIdtoRotation = new HashMap<Integer, Double>();

  static public void initTags(){
    tagIdtoRotation.put(1, 126.0);//feeder
    tagIdtoRotation.put(2, 234.0);//feeder

    tagIdtoRotation.put(3, 0.0);//irrelevant from amp
    tagIdtoRotation.put(4, 0.0);
    tagIdtoRotation.put(5, 0.0);
    
    tagIdtoRotation.put(6, 120.0);
    tagIdtoRotation.put(7, 180.0);
    tagIdtoRotation.put(8, 240.0);
    tagIdtoRotation.put(9, 300.0);
    tagIdtoRotation.put(10, 0.0);
    tagIdtoRotation.put(11, 60.0);

    
    tagIdtoRotation.put(13, tagIdtoRotation.get(1));
    tagIdtoRotation.put(12, tagIdtoRotation.get(2));
    tagIdtoRotation.put(17, tagIdtoRotation.get(8));
    tagIdtoRotation.put(18, tagIdtoRotation.get(7));
    tagIdtoRotation.put(19, tagIdtoRotation.get(6));
    tagIdtoRotation.put(20, tagIdtoRotation.get(11));
    tagIdtoRotation.put(21, tagIdtoRotation.get(10));
    tagIdtoRotation.put(22, tagIdtoRotation.get(9));


    tagIdtoRotation.put(15, tagIdtoRotation.get(4));
    tagIdtoRotation.put(14, tagIdtoRotation.get(5));
    tagIdtoRotation.put(16, tagIdtoRotation.get(3));
  }
  

  
  
}
