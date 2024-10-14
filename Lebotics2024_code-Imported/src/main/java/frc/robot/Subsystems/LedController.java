package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;

//import edu.wpi.first.networktables.NetworkTableInstance;

public class LedController {
    private final PWM blinkinPWM;
    private final int LedPin = 9; // El pin al que se va a conectar las leds. (PWM)

    // Esto definira los valores de cada color en el PWM. (Libreria: https://1166281274-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-ME3KPEhFI6-MDoP9nZD%2Fuploads%2FMOYJvZmWgxCVKJhcV5fn%2FREV-11-1105-LED-Patterns.pdf?alt=media&token=e8227890-6dd3-498d-834a-752fa43413fe)
    private final double STOPPED_PATTERN = -0.21;  // Ejemplo: color blanco pulsante
    private final double DEFAULT_PATTERN = -0.07;  // Ejemplo: color oro pulsante
    private final double AUTOMONUS_BLUE_PATTERN = 0.85; // Ejemplo: azul sólido
    private final double AUTOMONUS_RED_PATTERN = 0.59;  // Ejemplo: rojo sólido
    private final double LOW_BATTERY_PATTERN = -0.11;   // Ejemplo: rojo oscuro pulsante
    private final double TAKING_PATTERN = 0.77;         // Ejemplo: verde sólido
    private final double ERROR_PATTERN = 0.69;          // Ejemplo: amarillo sólido
    private final double TEST_MODE = -0.13;             // Ejemplo: Gris claro pulsante.
    private final double OFF_PATTERN = 0.99;            // Apaga el Blinkin (negro) | Testing deffault pattern. (-1.0 should be off but it is rainbow.)
    private final double FIRE_PATTERN = -0.57;           // Simula el patron del fuego.


    // Toda la tabla de las funciones de las leds.
    public enum LedMode {
        OFF,
        STOPPED,
        AUTOMONUS,
        TAKING,
        RF_LAUNCH,
        ERROR,
        LOW_BATTERY,
        DEFFAULT,
        TEST_MODE
    }

    // Esto sirve para apagar las leds del robot al iniciarse.
    private LedMode mode = LedMode.OFF;

    // Esto es el controlador de las leds para comenzar a utilizarlas.
    public LedController() {
        blinkinPWM = new PWM(LedPin); 
    }

    // Esto actualizara las leds dependiendo de lo que este pasando en el robot.
    private void UpdateLeds() {
        if (DriverStation.isEStopped()) {
            mode = LedMode.STOPPED;
        } else if (DriverStation.isTest()) {
            mode = LedMode.TEST_MODE;
        } else if (RobotController.getBatteryVoltage() < 8.4) {
            mode = LedMode.LOW_BATTERY;
        } else if (Intake.getInstance().isIntaking) {
            mode = LedMode.TAKING;
        } else if (Intake.getInstance().isLoaded) {
            mode = LedMode.RF_LAUNCH;
        } else if (DriverStation.isAutonomousEnabled()) {
            mode = LedMode.AUTOMONUS;
        } else if (DriverStation.isTeleopEnabled()) {
            mode = LedMode.DEFFAULT;
        } else {
            mode = LedMode.ERROR;
        }
    }

    // Estas lineas de codigo define de que color las leds se pondran en caso de X o Y.
    private void UpdateStrip() {
        switch (mode) {
            case STOPPED:
            blinkinPWM.setSpeed(STOPPED_PATTERN);
            break;
            case DEFFAULT:
            blinkinPWM.setSpeed(DEFAULT_PATTERN);
            break; 
            case AUTOMONUS:
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                blinkinPWM.setSpeed(AUTOMONUS_BLUE_PATTERN);
            } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                blinkinPWM.setSpeed(AUTOMONUS_RED_PATTERN);
            } else {
                blinkinPWM.setSpeed(DEFAULT_PATTERN);
            }
            break;
            case LOW_BATTERY:
            blinkinPWM.setSpeed(LOW_BATTERY_PATTERN);
            break;
            case TAKING:
            blinkinPWM.setSpeed(TAKING_PATTERN);
            break;
            case RF_LAUNCH:
            blinkinPWM.setSpeed(FIRE_PATTERN);
            break;
            case ERROR:
            blinkinPWM.setSpeed(ERROR_PATTERN);
            break;
            case TEST_MODE:
            blinkinPWM.setSpeed(TEST_MODE);
            break;
            case OFF:
            blinkinPWM.setSpeed(OFF_PATTERN);
            break;
        }   
    }

    // Funcion publica para que se ejecuten las funciones anteriores.
    public void LedsFuncionar() {
        UpdateLeds();
        UpdateStrip();
    }
}