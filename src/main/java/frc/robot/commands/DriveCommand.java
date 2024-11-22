package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveCommand extends Command {
    private DriveTrainSubsystem driveTrainSubsystem;
    private XboxController xboxController;
    private double leftVelocity = 0;
    private double rightVelocity = 0;
    private PIDController leftMotorVelocity = new PIDController(0.6, 0, 0);
    private PIDController rightMotorVelocity = new PIDController(0.6, 0, 0);
    public DriveCommand(DriveTrainSubsystem driveTrainSubsystem, XboxController xboxController) {
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.xboxController = xboxController;
        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void execute() {
        leftVelocity = xboxController.getLeftY() * 5 + xboxController.getLeftX() * 5;
        rightVelocity = xboxController.getLeftY() * -5 + xboxController.getLeftX() * 5;

        driveTrainSubsystem.setLeftSideMotorVoltage(leftMotorVelocity.calculate(-driveTrainSubsystem.getLeftSideEncoderVelocity() / 1700, leftVelocity));
        driveTrainSubsystem.setRightSideMotorVoltage(rightMotorVelocity.calculate(driveTrainSubsystem.getRightSideEncoderVelocity() / 1700, rightVelocity));
        NetworkTableInstance.getDefault().getTable("Debug").getEntry("RPM R").setDouble(-driveTrainSubsystem.getRightSideEncoderVelocity());
        NetworkTableInstance.getDefault().getTable("Debug").getEntry("RPM L").setDouble(driveTrainSubsystem.getLeftSideEncoderVelocity());

    }
}
