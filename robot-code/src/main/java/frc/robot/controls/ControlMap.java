package frc.robot.controls;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ControlMap{
    //driver ports
    private static int DRIVER_PORT = 0;
    private static int GUNNER_PORT = 1;
    private static int DRIVER_PORT_BUTTONS = 2;
    private static int GUNNER_PORT_BUTTONS = 3;
    
    //Controller objects
    public static Joystick driver_joystick = new Joystick(DRIVER_PORT);
    public static Joystick gunner_joystick = new Joystick(GUNNER_PORT);
    public static Joystick driver_buttons = new Joystick(DRIVER_PORT_BUTTONS);
    public static Joystick gunner_buttons = new Joystick(GUNNER_PORT_BUTTONS);


    public static JoystickButton blue1 = new JoystickButton(gunner_buttons, 1);
    public static JoystickButton blue2 = new JoystickButton(gunner_buttons, 2);
    public static JoystickButton green2 = new JoystickButton(gunner_buttons, 3);
    public static JoystickButton red4 = new JoystickButton(gunner_buttons, 4);
    public static JoystickButton red5 = new JoystickButton(gunner_buttons, 5);
    public static JoystickButton red6 = new JoystickButton(gunner_buttons, 6);

    // public static JoystickButton yellow_1 = new JoystickButton(driver_buttons,1);
    // public static JoystickButton yellow_2 = new JoystickButton(driver_buttons,2);
    // public static JoystickButton green_1 = new JoystickButton(driver_buttons,3);
    // public static JoystickButton red_1 = new JoystickButton(driver_buttons,4);
    // public static JoystickButton red_2 = new JoystickButton(driver_buttons,5);
    // public static JoystickButton red_3 = new JoystickButton(driver_buttons,6);
    




   
    // public static JoystickButton GUNNER_A = new JoystickButton(gunner_joystick, 1); //A
    // public static JoystickButton GUNNER_B = new JoystickButton(gunner_joystick, 2); //B
    // public static JoystickButton GUNNER_X = new JoystickButton(gunner_joystick, 3); //X
    // public static JoystickButton GUNNER_Y = new JoystickButton(gunner_joystick, 4); //Y
    // public static JoystickButton GUNNER_LB = new JoystickButton(gunner_joystick, 5); //LB
    // public static JoystickButton GUNNER_RB = new JoystickButton(gunner_joystick, 6); //RB
    // public static JoystickButton GUNNER_BACK = new JoystickButton(gunner_joystick, 7); 
    // public static JoystickButton GUNNER_START = new JoystickButton(gunner_joystick, 8); 
    // public static JoystickButton GUNNER_LJ_BUTTON = new JoystickButton(gunner_joystick, 9); //
    // public static JoystickButton GUNNER_RJ_BUTTON = new JoystickButton(gunner_joystick, 10); //
    // public static POVButton GUNNER_DUP = new POVButton(gunner_joystick, 0);
    // public static POVButton GUNNER_DDOWN = new POVButton(gunner_joystick, 180);

    // public static JoystickButton DRIVER_A = new JoystickButton(driver_joystick, 1); //A
    // public static JoystickButton DRIVER_B = new JoystickButton(driver_joystick, 2); //B
    // public static JoystickButton DRIVER_X = new JoystickButton(driver_joystick, 3); //X
    // public static JoystickButton DRIVER_Y = new JoystickButton(driver_joystick, 4); //Y
    // public static JoystickButton DRIVER_LB = new JoystickButton(driver_joystick, 5); //LB
    // public static JoystickButton DRIVER_RB = new JoystickButton(driver_joystick, 6); //RB
    // public static JoystickButton DRIVER_BACK = new JoystickButton(driver_joystick, 7); 
    // public static JoystickButton DRIVER_START = new JoystickButton(driver_joystick, 8); 
    // public static JoystickButton DRIVER_LJ_BUTTON = new JoystickButton(driver_joystick, 9); //
    // public static JoystickButton DRIVER_RJ_BUTTON = new JoystickButton(driver_joystick, 10); //
    // public static POVButton DRIVER_DUP = new POVButton(driver_joystick, 0);

    
}