package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LiftingArms;
import frc.robot.Subsystems.Shooter;
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

    private final SendableChooser<Command> autoChooser;

    public RobotContainer(){
        NamedCommands.registerCommand("shoot", Commands.sequence(
            intake.setState(IntakeState.SPEAKER),
            Commands.waitSeconds(0.9)
        ));
        NamedCommands.registerCommand("startShooter", Commands.sequence(
            shooter.setState(ShooterState.START_HIGHT)
        ));
        NamedCommands.registerCommand("stopShooter", Commands.sequence(
            shooter.setState(ShooterState.STOP)
        ));
        NamedCommands.registerCommand("take", Commands.sequence(
            shooter.setState(ShooterState.STOP),
            Commands.waitSeconds(0.5)
        ));
        NamedCommands.registerCommand("amp", Commands.sequence(
            intake.setState(IntakeState.AMP),
            Commands.waitSeconds(0.5)
        ));
        NamedCommands.registerCommand("retract", Commands.waitSeconds(0.5).andThen(shooter.setState(ShooterState.START_HIGHT)).andThen(retractifneeded()));
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);  
    }

    private Command retractifneeded(){
        if(intake.isLoaded && !intake.isIntaking){
            return new Command(){};
        }else{
            return Commands.sequence(
                intake.setState(IntakeState.STOP),
                Commands.waitSeconds(0.5)
            );
        }
    }

    private void configureBindings(){
        // Driver Controls
        swerveDrive.setJoystickSuppliers(
            () -> driverController.getLeftX(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX()
        );

        // Operator Conrols
        //operatorController.a().toggleOnTrue(intake.setState(IntakeState.DOWN));
        //operatorController.x().toggleOnTrue(intake.setState(IntakeState.UP));
        operatorController.b().toggleOnTrue(intake.setState(IntakeState.STOP));
        operatorController.y().toggleOnTrue(intake.setState(IntakeState.AMP));  

        /* operatorController.leftBumper()
            .toggleOnTrue(shooter.setState(ShooterState.START_LOW))
            .toggleOnFalse(shooter.setState(ShooterState.STOP)); */
        operatorController.leftTrigger()
            .toggleOnTrue(shooter.setState(ShooterState.START_HIGHT))
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

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Runnable controlLoop() {
		return () -> {

		};
	}
}
