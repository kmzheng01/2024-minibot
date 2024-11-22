package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utils.NetworkTableUtils;

public class TurnCommand extends Command {
    private AHRS gyro;
    private DriveTrainSubsystem driveTrainSubsystem;
    private double robotTurnAngle; // = Math.toRadians(20);
    private PIDController robotTurn = new PIDController(0.1, 0, 0);
    private final NetworkTableUtils table = new NetworkTableUtils("Debug");

    public TurnCommand(DriveTrainSubsystem driveTrainSubsystem, AHRS gyro) {
        this.gyro = gyro;
        this.driveTrainSubsystem = driveTrainSubsystem;
    }

    @Override
    public void initialize() {
//        robotTurn.enableContinuousInput(-Math.PI, Math.PI);
        robotTurnAngle = gyro.getAngle() + 90;
    }

    @Override
    public void execute() {
        double pidOutput = robotTurn.calculate(gyro.getAngle(), robotTurnAngle);

        driveTrainSubsystem.setLeftSideMotorVoltage(pidOutput);
        driveTrainSubsystem.setRightSideMotorVoltage(pidOutput);



    }
}

