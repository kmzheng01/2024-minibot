package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
    private final CANSparkMax leftSideMotor = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightSideMotor = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);

    public DriveTrainSubsystem(){

    };

    // set motor voltage
    public void setLeftSideMotorVoltage(double volts) {
        this.leftSideMotor.setVoltage(volts);
    }
    public void setRightSideMotorVoltage(double volts) {
        this.rightSideMotor.setVoltage(volts);
    }
    public double getLeftSideEncoderVelocity() {
        return this.leftSideMotor.getEncoder().getVelocity();
    }
    public double getRightSideEncoderVelocity() {
        return this.rightSideMotor.getEncoder().getVelocity();
    }

}


