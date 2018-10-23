package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

public class TeleopLibrary {

    RobotHardware robot;

    public void drivingMotorPowers(Gamepad Gamepad1) {

        leftJoystickToMotorPower(Gamepad1);
    }

    public void leftJoystickToMotorPower(Gamepad gamepad1) {

        double yValue = gamepad1.left_stick_y;

    }

    public void scaleInput(double input, LinearOpMode caller, Gamepad gamepad1) {

        while (caller.opModeIsActive()) {
            robot.liftMotor.setDirection(DcMotor.Direction.FORWARD);

            if (gamepad1.dpad_up) {
                if (robot.liftMotor.getCurrentPosition() < 100) {
                    robot.liftMotor.setPower(1);
                }
                else {robot.liftMotor.setPower(0.0);}
            } else if (gamepad1.dpad_down) {
                if (robot.liftMotor.getCurrentPosition() > -100) {
                    robot.liftMotor.setDirection(DcMotor.Direction.REVERSE);
                    robot.liftMotor.setPower(1);
                }
                else {robot.liftMotor.setPower(0.0);}
            }
            else {robot.liftMotor.setPower(0);}
        }
    }
}

