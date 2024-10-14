package frc.robot.Subsystems.Testing;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private final CANSparkMax feederMotor;

    public enum FeederState {
        STOP,
        IDLE,
        SPEAKER
    }

    private FeederState state;

    public Feeder() {
        feederMotor = new CANSparkMax(18, MotorType.kBrushless);
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(IdleMode.kCoast);
    }

    @Override 
    public void periodic() {
        runState();
    }

    public FeederState getState() {
        return state;
    }

    private void runState() {
        SmartDashboard.putString("Feeder_State", state.name());
        switch(state) {
            case STOP:
            // Actions to stop de motor.
            break;
            case IDLE:
            // Actions to keep motor turning.
            break;
            case SPEAKER:
            // Actions to make the motor turn.
            break;
        }
    }
}
