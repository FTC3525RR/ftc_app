package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

public class TeleopLibrary {

    public void drivingMotorPowers(Gamepad gamepad1) {

        leftJoystickToMotorPower(gamepad1);
    }

    public void leftJoystickToMotorPower(Gamepad gamepad1) {

        double yValue = gamepad1.left_stick_y;

    }

    public void scaleInput(double input){


    }
}
