package vitruvianlib;

import edu.wpi.first.wpilibj.Joystick;

public class DriverJoystick {
    private Joystick m_joystick;

    public DriverJoystick(int port) {
        m_joystick = new Joystick(port);
    }

    public double getY() {
        return -m_joystick.getY();
    }

    public double getX() {
        return m_joystick.getX();
    }
}
