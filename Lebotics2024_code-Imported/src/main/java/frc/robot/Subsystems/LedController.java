package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TejuinoBoard;

public class LedController extends SubsystemBase {
    private TejuinoBoard tejuinoBoard;  

    // Toda la tabla de las funciones de las leds.
    public enum LedMode {
        AUTOMONUS,
        RF_LAUNCH,
        DEFFAULT,
        TEST_MODE,
        OFF
    }

    // Esto sirve para apagar las leds del robot al iniciarse.
    LedMode mode = LedMode.OFF;

    public LedController() {
        tejuinoBoard = new TejuinoBoard();
        //tejuinoBoard.turn_off_all_leds(0);
    }

    @Override
    public void periodic() {
        UpdateLeds();
        UpdateStrip();
    }

    private void UpdateLeds() {
        if (Intake.getInstance().isLoaded == true) {
            mode = LedMode.RF_LAUNCH;
        } else if (DriverStation.isTeleop()) {
            mode = LedMode.DEFFAULT;
        } else if (DriverStation.isAutonomousEnabled()) {
            mode = LedMode.AUTOMONUS;
        } else if (DriverStation.isTestEnabled()) {
            mode = LedMode.TEST_MODE;
        } else {
            mode = LedMode.OFF;
        }
    }

    // Estas lineas de codigo define de que color las leds se pondran en caso de X o Y.
    private void UpdateStrip() {
        switch (mode) {
            case DEFFAULT:
            tejuinoBoard.rainbow_effect(1);
            break; 
            case AUTOMONUS:
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                tejuinoBoard.all_leds_blue(1);
            } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                tejuinoBoard.all_leds_red(1);
            } else {
                tejuinoBoard.rainbow_effect(1);
            }
            break;
            case RF_LAUNCH:
            tejuinoBoard.all_leds_green(1);
            break;
            case TEST_MODE:
            tejuinoBoard.all_leds_yellow(1);
            case OFF:
            tejuinoBoard.turn_off_all_leds(1);
            break;
        }   
    }
}
