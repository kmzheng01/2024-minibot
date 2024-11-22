package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utils.NetworkTableUtils;

public class MovementCommand extends Command {
    private DriveTrainSubsystem driveTrainSubsystem;
    private XboxController xboxController;
    private AHRS gyro;
    private double leftVelocity = 0;
    private double rightVelocity = 0;
    private double robotTurnAngle;
    private PIDController leftMotorVelocity = new PIDController(0.1, 0, 0);
    private PIDController rightMotorVelocity = new PIDController(0.1, 0, 0);
    private PIDController robotTurn = new PIDController(0.1, 0 ,0);
    private boolean doneTurning = false;
    private final NetworkTableUtils table = new NetworkTableUtils("Debug");
    public MovementCommand(DriveTrainSubsystem driveTrainSubsystem, XboxController xboxController, AHRS gyro) {
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.xboxController = xboxController;
        this.gyro = gyro;
        addRequirements(driveTrainSubsystem);
        robotTurn.setTolerance(0.4);
    }

    @Override
    public void execute() {
        robotTurnAngle = Math.atan2(xboxController.getLeftY(), xboxController.getLeftX());
        robotTurn.setSetpoint(robotTurnAngle + this.gyro.getAngle());
        if (!robotTurn.atSetpoint()) {
            double turnOutput = robotTurn.calculate(gyro.getAngle(), robotTurnAngle);
            driveTrainSubsystem.setLeftSideMotorVoltage(turnOutput);
            driveTrainSubsystem.setRightSideMotorVoltage(turnOutput);
        } else {
            leftVelocity = Math.abs(xboxController.getLeftY()) * 2;
            rightVelocity = Math.abs(xboxController.getLeftY()) * 2;
          

            driveTrainSubsystem.setLeftSideMotorVoltage(leftMotorVelocity.calculate(driveTrainSubsystem.getLeftSideEncoderVelocity() / 1700, leftVelocity));
            driveTrainSubsystem.setRightSideMotorVoltage(rightMotorVelocity.calculate(-driveTrainSubsystem.getRightSideEncoderVelocity() / 1700, rightVelocity));
        }

    }
}
