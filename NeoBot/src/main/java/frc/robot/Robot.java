/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  //sets up CAN IDs for all of the Spark Maxes
  private static final int frontLeftDeviceID = 4;
  private static final int rearLeftDeviceID = 3;
  private static final int frontRightDeviceID = 1;
  private static final int rearRightDeviceID = 2;
  private CANSparkMax m_frontLeftMotor;
  private CANSparkMax m_frontRightMotor;
  private CANSparkMax m_rearLeftMotor;
  private CANSparkMax m_rearRightMotor;

  //Sets joystick channels and variables
  private static final int kLeftJoystickChannel = 0;
  private static final int kRightJoystickChannel = 1;
  private Joystick m_LeftStick;
  private Joystick m_RightStick;
  
  //initiailizes the Mecanum class
  private MecanumDrive m_robotDrive;

  //Sets up variables for the adjusted joysticks (for z axis control)
  private double adjustedLeftJoystickY;
  private double adjustedLeftJoystickX;
  private double adjustedRightJoystickX;
  private double deadbandLimits = 0.1;

  @Override
  public void robotInit() {
    //Initializes each of the Spark Maxes
    m_frontLeftMotor = new CANSparkMax(frontLeftDeviceID, MotorType.kBrushless);
    m_rearLeftMotor = new CANSparkMax(rearLeftDeviceID, MotorType.kBrushless);
    m_frontRightMotor = new CANSparkMax(frontRightDeviceID, MotorType.kBrushless);
    m_rearRightMotor = new CANSparkMax(rearRightDeviceID, MotorType.kBrushless);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    m_frontLeftMotor.setInverted(true);
    m_rearLeftMotor.setInverted(true);

    //sets each motro back to factory defaults
    m_frontLeftMotor.restoreFactoryDefaults();
    m_frontRightMotor.restoreFactoryDefaults();
    m_rearLeftMotor.restoreFactoryDefaults();
    m_rearRightMotor.restoreFactoryDefaults();

    //sets up m_robotDrive with the MecanumDrive
    m_robotDrive = new MecanumDrive(m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor, m_rearRightMotor);

    //sets up joysticks
    m_LeftStick = new Joystick(kLeftJoystickChannel);
    m_RightStick = new Joystick(kRightJoystickChannel);
  }

  @Override
  public void teleopPeriodic() {
    //This maps the z axis to the x and y of both joysticks, speed control
    adjustedLeftJoystickX = m_LeftStick.getX() * (m_LeftStick.getZ()/2 + 0.5);
    adjustedLeftJoystickY = m_LeftStick.getY() * (m_LeftStick.getZ()/2 + 0.5);
    adjustedRightJoystickX = -1 * m_RightStick.getX() * (m_LeftStick.getZ()/2 + 0.5);

    if(adjustedLeftJoystickX < deadbandLimits && adjustedLeftJoystickX > -(deadbandLimits)){
      adjustedLeftJoystickX = 0;
    }

    if(adjustedLeftJoystickY < deadbandLimits && adjustedLeftJoystickY > -(deadbandLimits)){
      adjustedLeftJoystickY = 0;
    }

    if(adjustedRightJoystickX < deadbandLimits && adjustedRightJoystickX > -(deadbandLimits)){
      adjustedRightJoystickX = 0;
    }

    //Drives the robto
    m_robotDrive.driveCartesian(adjustedLeftJoystickX, adjustedLeftJoystickY, adjustedRightJoystickX, 0.0);
  }
}
