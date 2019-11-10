/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.kMaxAngularSpeed;
import static frc.robot.subsystems.Drivetrain.kMaxSpeed;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetArcadeDriveSpeed extends CommandBase {
    private final Drivetrain m_drivetrain;

    public SetArcadeDriveSpeed(Drivetrain drivetrain) {
        // Use requires() here to declare subsystem dependencies
        this.m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double throttle = RobotContainer.leftJoystick.getY() * kMaxSpeed;
        double turn = RobotContainer.leftJoystick.getX() * kMaxAngularSpeed;

        m_drivetrain.drive(throttle, turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
    return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {

    }
}
