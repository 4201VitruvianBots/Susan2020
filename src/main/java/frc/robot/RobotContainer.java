
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetArcadeDrive;
import frc.robot.commands.SetArcadeDriveSpeed;
import frc.robot.subsystems.Drivetrain;
import vitruvianlib.DriverJoystick;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Drivetrain m_drivetrain = new Drivetrain();


    public static DriverJoystick leftJoystick = new DriverJoystick(Constants.leftJoystick);
    public static DriverJoystick rightJoystick = new DriverJoystick(Constants.rightJoystick);

    private final Command m_autoCommand = new ExampleCommand(m_drivetrain);
    private final Command m_arcadeDrive = new SetArcadeDrive(m_drivetrain);
    private final Command m_arcadeDriveSpeed = new SetArcadeDriveSpeed(m_drivetrain);

    SendableChooser<Command> m_chooser = new SendableChooser<>();
    static SendableChooser<Command> m_driveChooser = new SendableChooser<>();
    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set default commands here
        m_drivetrain.setDefaultCommand(new SetArcadeDrive(m_drivetrain));

        // Configure the button bindings
        configureButtonBindings();

        // Configure the SendableChooser with auto commands and put it on the SmartDashboard
        m_chooser.setDefaultOption("Default Auto", m_autoCommand);
        SmartDashboard.putData("Auto mode", m_chooser);

        m_driveChooser.setDefaultOption("Arcade Drive", m_arcadeDrive);
        m_driveChooser.setDefaultOption("Arcade Drive Speed", m_arcadeDriveSpeed);
        SmartDashboard.putData("Drive Mode", m_driveChooser);


    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autoCommand;
    }

    public Drivetrain getDriveTrain() {
        return m_drivetrain;
    }
}
