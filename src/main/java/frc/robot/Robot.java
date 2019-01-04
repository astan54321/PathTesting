/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import easypath.EasyPath;
import easypath.EasyPathConfig;
import easypath.FollowPath;
import easypath.PathUtil;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;

public class Robot extends TimedRobot {

  public Drive drive = new Drive();
  public VictorSP coil = new VictorSP(RobotMap.COIL_MOTOR_CHANNEL);
  public Joystick driver = new Joystick(0);

  Command m_autonomousCommand;

  @Override
  public void robotInit() {

    EasyPathConfig config;
    config = new EasyPathConfig(drive, // the subsystem itself
        drive::setLeftRightMotorSpeeds, // function to set left/right speeds
        // function to give EasyPath the length driven
        () -> PathUtil.defaultLengthDrivenEstimator(drive::getLeftDistance, drive::getRightDistance), drive::getHeading,
        drive::reset, // function to reset your encoders to 0
        0.07 // kP value for P loop
    );

    config.setSwapDrivingDirection(false);
    config.setSwapTurningDirection(false);

    EasyPath.configure(config);

    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override
  public void autonomousInit() {
    // This drives 36 inches in a straight line, driving at 25% speed the first 50%
    // of the path,
    // and 75% speed in the remainder.
    // x is the percentage completion of the path, between 0 and 1.
    m_autonomousCommand = new FollowPath(PathUtil.createStraightPath(36.0), x -> {
      if (x < 0.5)
        return 0.25;
      else
        return 0.75;
    });
    m_autonomousCommand.start();

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    double speedInput = driver.getRawAxis(2) - driver.getRawAxis(3);
    double rotationInput = driver.getRawAxis(0);
    drive.drive(speedInput, rotationInput);
    coil.set(driver.getRawAxis(5));
  }

  @Override
  public void testPeriodic() {
    drive.setLeftRightMotorSpeeds(-driver.getRawAxis(1), driver.getRawAxis(5));
  }
}
