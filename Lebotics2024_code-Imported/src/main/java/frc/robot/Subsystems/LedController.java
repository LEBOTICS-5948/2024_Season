package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

//import edu.wpi.first.networktables.NetworkTableInstance;

public class LedController {
    private final PWM blinkinPWM;
    private final int LedPin = 9; // El pin al que se va a conectar las leds. (PWM)
    int LedNums = 68; // Numero de leds en la tira utilizada.

    // Esto definira los valores de cada color en el PWM.
    private final double STOPPED_PATTERN = -0.05;  // Ejemplo: color blanco sólido
    private final double DEFFAULT_PATTERN = 0.25;  // Ejemplo: amarillo
    private final double AUTOMONUS_BLUE_PATTERN = 0.87; // Ejemplo: azul
    private final double AUTOMONUS_RED_PATTERN = 0.61;  // Ejemplo: rojo
    private final double LOW_BATTERY_PATTERN = -0.11;   // Ejemplo: rojo oscuro
    private final double TAKING_PATTERN = 0.77;         // Ejemplo: azul oscuro
    private final double ERROR_PATTERN = 0.69;          // Ejemplo: amarillo intermitente
    private final double OFF_PATTERN = -1.0;            // Apaga el Blinkin (negro)

    // TODO (Finish fire effect for the shooter.)
    private final double FIRE_PATTERN = 0.57;           // Simula el patron del fuego.


    // Toda la tabla de las funciones de las leds.
    public enum LedMode {
        STOPPED,
        AUTOMONUS,
        TAKING,
        RF_LAUNCH,
        ERROR,
        LOW_BATTERY,
        DEFFAULT,
        OFF
    }

    // Esto sirve para apagar las leds del robot al iniciarse.
    LedMode mode = LedMode.OFF;

    // Declara las direcciones de cada led para poder ser utilizada.

    boolean AutoLastlyEnabled = false;
    double LastTime = 0.0;

    // Esto es el controlador de las leds para comenzar a utilizarlas.
    public LedController() {
        blinkinPWM = new PWM(LedPin); 
    }

    // Esto actualizara las leds dependiendo de lo que este pasando en el robot.
    private void UpdateLeds() {
        AutoLastlyEnabled = DriverStation.isAutonomous();
        LastTime = Timer.getFPGATimestamp();

        if (DriverStation.isEStopped()) {
            mode = LedMode.STOPPED;
        } else if (DriverStation.isTeleop()) {
            mode = LedMode.DEFFAULT;
        } else if (DriverStation.isAutonomous()) {
            mode = LedMode.AUTOMONUS;
        } else if (RobotController.getBatteryVoltage() < 9) {
            mode = LedMode.LOW_BATTERY;
        } else if (Intake.getInstance().isIntaking == true) {
            mode = LedMode.TAKING;
        } else if (Intake.getInstance().isLoaded == true) {
            mode = LedMode.RF_LAUNCH;
        } else {
            mode = LedMode.OFF;
        }

    }

    // Estas lineas de codigo define de que color las leds se pondran en caso de X o Y.
    private void UpdateStrip() {
        switch (mode) {
            case STOPPED:
            blinkinPWM.setSpeed(STOPPED_PATTERN);
            break;
            case DEFFAULT:
            blinkinPWM.setSpeed(DEFFAULT_PATTERN);
            break; 
            case AUTOMONUS:
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                blinkinPWM.setSpeed(AUTOMONUS_BLUE_PATTERN);
            } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                blinkinPWM.setSpeed(AUTOMONUS_RED_PATTERN);
            } else {
                blinkinPWM.setSpeed(DEFFAULT_PATTERN);
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