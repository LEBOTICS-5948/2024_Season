package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LiftingArms;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.DriveTrain.DriveTrainState;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.LiftingArms.LiftingArmsState;
import frc.robot.Subsystems.Shooter.ShooterState;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final DriveTrain swerveDrive = DriveTrain.getInstance();
    private final LiftingArms liftingArms = LiftingArms.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    public RobotContainer(){
        configureBindings();
    }

    private void configureBindings(){
        // Driver Controls
        swerveDrive.setJoystickSuppliers(
            () -> driverController.getLeftX(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX()
        );
        
        driverController.x()
            .toggleOnTrue(swerveDrive.setRotationSuplier(true, -90d))
            .toggleOnFalse(swerveDrive.setRotationSuplier(false, null)); 
        driverController.y()
            .toggleOnTrue(swerveDrive.setRotationSuplier(true, 135d))
            .toggleOnFalse(swerveDrive.setRotationSuplier(false, null)); 

        // Operator Conrols
        operatorController.a().toggleOnTrue(intake.setState(IntakeState.DOWN));
        operatorController.b().toggleOnTrue(intake.setState(IntakeState.STOP));
        operatorController.x().toggleOnTrue(intake.setState(IntakeState.UP));
        operatorController.y().toggleOnTrue(intake.setState(IntakeState.AMP));  

        operatorController.leftTrigger()
            .toggleOnTrue(shooter.setState(ShooterState.START))
            .toggleOnFalse(shooter.setState(ShooterState.STOP));
        (operatorController.rightTrigger().and(() -> shooter.isReady && !intake.isIntaking))
            .toggleOnTrue(intake.setState(IntakeState.SPEAKER));

        operatorController.povUp()
            .toggleOnTrue(liftingArms.setState(LiftingArmsState.UP))
            .toggleOnFalse(liftingArms.setState(LiftingArmsState.STOP));
        operatorController.povDown()
            .toggleOnTrue(liftingArms.setState(LiftingArmsState.DOWN))
            .toggleOnFalse(liftingArms.setState(LiftingArmsState.STOP)); 
    }
}
