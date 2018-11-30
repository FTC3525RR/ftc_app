package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

@Autonomous (name= "Auto test")

public class AutonomousTest extends LinearOpMode{
    RobotHardware rh;
    Orientation angles;
    Orientation startAngles;

    public void runOpMode(){

        rh = new RobotHardware();
        rh.init(hardwareMap, telemetry);
//        AutonomousLibrary al = new AutonomousLibrary();
//        CommonLibrary cl = new CommonLibrary();
        waitForStart();

        runMethod();
//        turnOnWheels();

        while(opModeIsActive()){
            //turnOnWheels();
            //encoderTicksToInchesTest(telemetry);
            //resetlift();
            //liftDown(telemetry);
//            turnOnLift();
            //testSensor(telemetry);
            //dropMarker();
            turnRelativeToStart(45, telemetry);
            if (isStopRequested()){break;}
            break;
        }

    }

    public void runMethod(){

        rh.frontRightMotor.setPower(0);
        rh.frontLeftMotor.setPower(0);
        rh.backLeftMotor.setPower(0);
        rh.backRightMotor.setPower(0);
        //liftMotor.setPower(0);

//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        //liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnOnWheels(){

        rh.frontRightMotor.setPower(1);
        rh.backRightMotor.setPower(1);
        rh.backLeftMotor.setPower(1);
        rh.frontLeftMotor.setPower(1);
    }

    public void encoderTicksToInchesTest (Telemetry telemetry) {

        while (rh.frontRightMotor.getCurrentPosition() <= 1000){
            rh.frontRightMotor.setPower(1);
            rh.frontLeftMotor.setPower(1);
            rh.backRightMotor.setPower(1);
            rh.backLeftMotor.setPower(1);
            telemetry.addData("ticks", rh.backRightMotor.getCurrentPosition());
            telemetry.update();

            if (isStopRequested()) {break;}
        }

        rh.frontRightMotor.setPower(0);
        rh.frontLeftMotor.setPower(0);
        rh.backRightMotor.setPower(0);
        rh.backLeftMotor.setPower(0);
        telemetry.addData("done", "");
        telemetry.update();
    }

    public void driveAuto(double angle, double distance) {

        resetMotorEncoders();

        double driveAngle = angle * Math.PI / 180;

        double xInput = Math.cos(driveAngle);
        double yInput = Math.sin(driveAngle);
        double flPower = Range.clip((yInput - xInput), -1, 1);
        double frPower = Range.clip((yInput + xInput), -1, 1);
        double blPower = Range.clip((yInput - xInput), -1, 1);
        double brPower = Range.clip((yInput + xInput), -1, 1);

        double frontLeftMotorRatio = 1 / Math.sin(driveAngle);
        double frontRightMotorRatio = 1 / Math.sin(driveAngle);
        double rearLeftMotorRatio = 1 / Math.sin(driveAngle);
        double rearRightMotorRatio = 1 / Math.sin(driveAngle);

        //Put target stuff here, don't run until that happens. Delete comment when done.

        rh.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void resetMotorEncoders(){

        rh.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runWithoutEncoders(){

        rh.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runWithEncoder(){
        rh.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rh.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rh.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rh.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void liftDown(Telemetry telemetry){

        rh.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(rh.liftMotor.getCurrentPosition() < 20500) {

            rh.liftMotor.setPower(1);
            telemetry.addData("Lift Ticks", rh.liftMotor.getCurrentPosition());
            telemetry.update();
            if(isStopRequested()){
                break;
            }
        }

        rh.liftMotor.setPower(0);
        while(rh.frontRightMotor.getCurrentPosition() < 500){
            rh.frontRightMotor.setPower(-1);
            rh.frontLeftMotor.setPower(-1);
            rh.backRightMotor.setPower(-1);
            rh.backLeftMotor.setPower(-1);
        }
        rh.frontRightMotor.setPower(0);
        rh.frontLeftMotor.setPower(0);
        rh.backRightMotor.setPower(0);
        rh.backLeftMotor.setPower(0);

    }

    public void resetlift(){
        while (rh.liftMotorTouchSensor.getState() == true){
            rh.liftMotor.setPower(-1);
            if (!rh.liftMotorTouchSensor.getState() == true){
                rh.liftMotor.setPower(0);
                break;
            }
        }
        telemetry.addData("lift motor state", rh.liftMotorTouchSensor.getState());
        telemetry.update();
        rh.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turnOnLift(){

        rh.liftMotor.setPower(1);
    }
    public void testSensor(Telemetry telemetry){

        telemetry.addData("True or false?",rh.liftMotorTouchSensor.getState());
        telemetry.update();
    }
    public void dropMarker(){
        rh.markerServo.setPosition(1);
    }
    public void turnRelativeToStart(int angle, Telemetry telemetry) {

        runWithoutEncoders();
        if (angle >= 360) {
            angle = angle - 360;
        }
        if (angle <= -360) {
            angle = angle + 360;
        }
        double targetAngle = startAngles.firstAngle + angle;
        if (targetAngle >= 360) {
            targetAngle = targetAngle - 360;
        }
        if (targetAngle <= -360) {
            targetAngle = targetAngle + 360;
        }
        double acceptableError = 0.5;
        double currentError = 1;
        double power;
        while (Math.abs(currentError) > acceptableError && !isStopRequested()) {

            angles = rh.imu.getAngularOrientation();
            double currentAngle = angles.firstAngle;
            currentError = targetAngle - currentAngle;
            telemetry.addData("Current angle", currentAngle);
            if (currentError > 180) {
                currentError = currentError - 360;
            }
            if (currentError <= -180) {
                currentError = currentError + 360;
            }
            telemetry.addLine();
            telemetry.addData("Current error", currentError);
            power = currentError * 0.01;
            if (power > 0.75) {
                power = 0.75;
            }
            if (power < 0.17 && power > 0) {
                power = 0.17;
            }
            if (power > -0.17 && power < 0) {
                power = -0.17;
            }
            if (power < -0.75) {
                power = -0.75;
            }
            telemetry.addLine();
            telemetry.addData("Power", power);
            telemetry.update();
            rh.frontLeftMotor.setPower(-power);
            rh.frontRightMotor.setPower(power);
            rh.backRightMotor.setPower(power);
            rh.backLeftMotor.setPower(-power);
            if (isStopRequested()) {
                break;
            }
        }
        rh.frontLeftMotor.setPower(0);
        rh.frontRightMotor.setPower(0);
        rh.backRightMotor.setPower(0);
        rh.backLeftMotor.setPower(0);
        telemetry.addData("done", "done");
        telemetry.update();
    }
}
