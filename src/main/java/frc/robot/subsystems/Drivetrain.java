/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetArcadeDrive;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends SubsystemBase {

  public Drivetrain() {
    setDefaultCommand(new SetArcadeDrive());
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  @Override
  public void periodic() {

  }
}
