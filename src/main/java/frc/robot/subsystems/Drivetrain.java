/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends SubsystemBase {

    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double kTrackWidth = 0.381 * 2; // meters
    private static final double kWheelRadius = 0.0508; // meters
    private static final int kEncoderResolution = 4096;

    TalonSRX[] driveMotors = {
        new TalonSRX(Constants.leftFrontMotor),
        new TalonSRX(Constants.leftRearMotor),
        new TalonSRX(Constants.rightFrontMotor),
        new TalonSRX(Constants.rightRearMotor),
    };

    public AHRS navX = new AHRS(SerialPort.Port.kMXP);

    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_kinematics);

    public Drivetrain() {
        for(TalonSRX motor:driveMotors) {
            motor.configFactoryDefault();
        }
        driveMotors[0].setInverted(false);
        driveMotors[0].setInverted(false);
        driveMotors[0].setInverted(true);
        driveMotors[0].setInverted(true);

        driveMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        driveMotors[0].setSensorPhase(false);
        driveMotors[2].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        driveMotors[2].setSensorPhase(true);


        driveMotors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
        driveMotors[2].set(ControlMode.Follower, driveMotors[2].getDeviceID());

        navX.reset();
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public double getSensorVelocity(int controllerIdx){
        try {
            return driveMotors[controllerIdx].getSelectedSensorVelocity();
        } catch(Exception e) {
            DriverStation.reportWarning("4201 Error: getSensorVelocity() - Invalid controllerIdx!", false);
            return 0;
        }
    }

    public double getSensorRate(int controllerIdx){
        try {
            return (2 * Math.PI * kWheelRadius / kEncoderResolution) * getSensorVelocity(controllerIdx);
        } catch(Exception e) {
            DriverStation.reportWarning("4201 Error: getSensorVelocity() - Invalid controllerIdx!", false);
            return 0;
        }
    }

    public double getSensorPosition(int controllerIdx){
        try {
            return driveMotors[controllerIdx].getSelectedSensorPosition();
        } catch(Exception e) {
            DriverStation.reportWarning("4201 Error: getSensorPosition() - Invalid controllerIdx!", false);
            return 0;
        }
    }

    public double getNavXAngle() {
        try {
            return navX.getAngle();
        } catch(Exception e) {
            DriverStation.reportWarning("4201 Error: getAngle() - Could not get angle from Navx!", false);
            return 0;
        }
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-getNavXAngle());
    }

    public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
        return new DifferentialDriveWheelSpeeds(getSensorRate(0), getSensorRate(2));
    }

    public void setMotorArcadeDrive(double throttle, double turn) {
        double leftPWM = throttle + turn;
        double rightPWM = throttle - turn;

        if(rightPWM > 1.0) {
            leftPWM -= rightPWM - 1.0;
            rightPWM = 1.0;
        } else if(rightPWM < -1.0) {
            leftPWM -= rightPWM + 1.0;
            rightPWM = -1.0;
        } else if(leftPWM > 1.0) {
            rightPWM -= leftPWM - 1.0;
            leftPWM = 1.0;
        } else if(leftPWM < -1.0) {
            rightPWM -= leftPWM + 1.0;
            leftPWM = -1.0;
        }

//        if(Robot.climber.climbMode == 1)
//            setMotorCurrentOutput(20 *leftPWM, 20 * rightPWM);
//        else
        setMotorOutputs(leftPWM, rightPWM);
    }

    public void setMotorOutputs(double leftOutput, double rightOutput) {
        driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
        driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
    }

    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftOutput = m_leftPIDController.calculate(getSensorRate(0),
                speeds.leftMetersPerSecond);
        double rightOutput = m_rightPIDController.calculate(getSensorRate(2),
                speeds.rightMetersPerSecond);

        setMotorOutputs(leftOutput, rightOutput);
    }

    public void updateOdometry() {
        m_odometry.update(getAngle(), getCurrentSpeeds());
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
