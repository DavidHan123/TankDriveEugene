/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// import edu.wpi.first.wpilibj.IterativeRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import com.ctre.phoenix.motorcontrol.can.*;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */

///////////////// Iterative Class???

public class Robot extends TimedRobot {
  // private Joystick m_leftStick;
  // private Joystick m_rightStick;

  // Joystick scorpion = new Joystick(0);
	Joystick driveStick = new Joystick(0);
  WPI_TalonSRX leftFront = new WPI_TalonSRX(12); //front
	WPI_TalonSRX leftBack = new WPI_TalonSRX(13); //back
	WPI_TalonSRX rightFront = new WPI_TalonSRX(15); //front
	WPI_TalonSRX rightBack = new WPI_TalonSRX(0); //back
  SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFront, leftBack);
	SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive m_robotDrive = new DifferentialDrive(leftDrive, rightDrive);

  @Override
  public void robotInit() {
    // m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    // m_leftStick = new Joystick(0);
    // m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    // m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());

    //gives left and right drive motor speeds bound[-1,1]
		m_robotDrive.tankDrive(Math.max(-1,Math.min(1.0,0 - driveStick.getY() + driveStick.getX())),Math.max(-1,Math.min(1.0,0 - driveStick.getY() - driveStick.getX())));
  }
}
