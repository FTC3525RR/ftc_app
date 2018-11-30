package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

@TeleOp(name = "TeleOp")
//@Disabled
public class TeleopDriverTest extends OpMode {

    RobotHardware rh = new RobotHardware();
    double frPower;
    double flPower;
    double brPower;
    double blPower;
    /*double backRightPower;
    double backLeftPower;*/
    public double clockwiseRotation = 0;
    public double counterclockwiseRotation = 0;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        rh.init(hardwareMap, telemetry);
        rh.runMethod();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        drivingMotorPowers(gamepad1);
        raiseLift(gamepad1);
        lowerLift(gamepad1);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public void drivingMotorPowers(Gamepad gamepad1) {

        rightJoystickToRotation(gamepad1);
        leftJoystickToMotorPower(gamepad1);

        rh.frontLeftMotor.setPower(-(Range.clip((clockwiseRotation + flPower) * Math.abs((clockwiseRotation + flPower)), -1, 1)));
        rh.frontRightMotor.setPower(-(Range.clip((counterclockwiseRotation + frPower) * Math.abs((counterclockwiseRotation + frPower)), -1, 1)));
        rh.backRightMotor.setPower(-(Range.clip((counterclockwiseRotation + brPower) * Math.abs((counterclockwiseRotation + brPower)), -1, 1)));
        rh.backLeftMotor.setPower(-(Range.clip((clockwiseRotation + flPower) * Math.abs((clockwiseRotation + blPower)), -1, 1)));
    }

    public void leftJoystickToMotorPower(Gamepad gamepad1) {

        double yValue = gamepad1.left_stick_y;
        frPower = scaleInput(Range.clip((yValue + gamepad1.left_stick_x), -1, 1));
        flPower = scaleInput(Range.clip((yValue - gamepad1.left_stick_x), -1, 1));
        brPower = scaleInput(Range.clip((yValue + gamepad1.left_stick_x), -1, 1));
        blPower = scaleInput(Range.clip((yValue - gamepad1.left_stick_x), -1, 1));

    }

    public void rightJoystickToRotation (Gamepad gamepad1){

        float HorizontalInput = Range.clip(gamepad1.right_stick_x, -1, 1);
        clockwiseRotation = scaleInput(HorizontalInput);
        counterclockwiseRotation = scaleInput(-HorizontalInput);
    }

    public void liftMotor(LinearOpMode caller, Gamepad gamepad1) {

        while (caller.opModeIsActive()) {
            rh.liftMotor.setDirection(DcMotor.Direction.FORWARD);

            if (gamepad1.dpad_up) {
                if (rh.liftMotor.getCurrentPosition() < 1000) {
                    rh.liftMotor.setPower(1);
                }
                else {rh.liftMotor.setPower(0);}
            } else if (gamepad1.dpad_down) {
                if (rh.liftMotor.getCurrentPosition() >= 0) {
                    rh.liftMotor.setPower(-1);
                }
                else {rh.liftMotor.setPower(0);}
            }
            else {rh.liftMotor.setPower(0);}
        }
    }

    private static double scaleInput(double dVal)  {
        /**
         * Converts raw input into values that can be used as power arguments for motors and servos
         */

        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        int index = (int) (dVal * 16.0);

        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale;

        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

    public void lowerLift(Gamepad gamepad1){

        rh.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       try {
           while (gamepad1.a) {
               rh.liftMotor.setPower(-1);
            /*if (rh.liftMotor.getCurrentPosition() >= 20000){
                rh.liftMotor.setPower(0);
            }*/
           }

       }catch (Exception e) {
           rh.liftMotor.setPower(0);
       }


        rh.liftMotor.setPower(0);
    }

    public void raiseLift(Gamepad gamepad1) {

        try {
            rh.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (gamepad1.b && !gamepad1.a) {
                rh.liftMotor.setPower(1);

                if (!rh.liftMotorTouchSensor.getState() == true) {
                    rh.liftMotor.setPower(0);
                    rh.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
            rh.liftMotor.setPower(0);

        }catch (Exception e) {
            rh.liftMotor.setPower(0);
        }
    }
}
