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
        
        NamedCommands.registerCommand("stopShooter", Commands.sequence(
            shooter.setState(ShooterState.STOP)
        ));
        NamedCommands.registerCommand("take", Commands.sequence(
            shooter.setState(ShooterState.STOP),
            Commands.waitSeconds(0.5)
        ));
        
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
        operatorController.a().and(() -> !shooter.isLoaded)
            .toggleOnTrue(intake.setState(IntakeState.IN)
            .alongWith(shooter.setState(ShooterState.LOAD))
        );
        operatorController.a().and(() -> shooter.isLoaded)
            .toggleOnTrue(shooter.setState(ShooterState.STOP)
            //.toggleOnTrue(intake.setState(IntakeState.STOP)
        );
        operatorController.a()
            .toggleOnFalse(intake.setState(IntakeState.STOP)
            .alongWith(shooter.setState(ShooterState.STOP))
        );
        operatorController.b()
            .toggleOnTrue(intake.setState(IntakeState.OUT))
            .toggleOnFalse(intake.setState(IntakeState.STOP));
        //operatorController.x().toggleOnTrue(shooter.setState(ShooterState.LOAD));
        operatorController.y().toggleOnTrue(intake.setState(IntakeState.STOP));
        
        operatorController.leftTrigger()
            .toggleOnTrue(shooter.setState(ShooterState.AIM))
            .toggleOnFalse(shooter.setState(ShooterState.STOP));
        (operatorController.rightTrigger().and(() -> shooter.isReady))
            .toggleOnTrue(shooter.setState(ShooterState.SHOOT)
        );
        
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
