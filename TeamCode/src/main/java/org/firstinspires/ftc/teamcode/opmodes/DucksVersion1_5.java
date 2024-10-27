package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;

@TeleOp
public class DucksVersion1_5 extends OpMode{
    DucksProgrammingBoard1_4 board = new DucksProgrammingBoard1_4();
    double gear = 1.0;
    double armconstant = 360 * 30/125 * 30/125 ;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    @Override
    public void init() {
        board.init(hardwareMap);
        //initialArmRotation = board.getArmMotorRotations();
    }
    @Override
    public void loop() {

        double forwardSpeed = gear * -gamepad1.right_stick_y;
        double sideSpeed = gear * gamepad1.right_stick_x;
        double rotateSpeed = gear * gamepad1.left_stick_x;
        double armSpeed = -gamepad2.left_stick_y;
        double millimeters = Math.PI * 80 * board.getMotorRotations();

        leftDrive  = hardwareMap.get(DcMotor.class, "hangingarm_1");
        rightDrive = hardwareMap.get(DcMotor.class, "hangingarm_2");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        double leftPower;
        double rightPower;

        double leftMotor = 0.87 * gamepad2.right_stick_y;
        double rightMotor  = 1.0 * gamepad2.right_stick_y;
        leftPower    = Range.clip(leftMotor, -1.0, 1.0) ;
        rightPower   = Range.clip(rightMotor, -1.0, 1.0) ;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        if (gamepad2.right_stick_y < -0.1){
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }


        if(gamepad1.right_bumper) {
            gear += .01;
            if (gear > 3) {
                gear =3;
            }
        }
        if(gamepad1.left_bumper){
            gear -= .01;
            if (gear < 0.001) {
                gear = 0.001;
            }
        }

        if ((Math.abs(forwardSpeed) > Math.abs(sideSpeed) && (Math.abs(forwardSpeed) > Math.abs(rotateSpeed)))){
            board.setForwardSpeed(forwardSpeed);
        }
        else if ((Math.abs(sideSpeed) > Math.abs(rotateSpeed) && (Math.abs(forwardSpeed) < Math.abs(sideSpeed)))) {
            board.setSideMotorSpeed(sideSpeed);
        }
        else {
            board.setRotateSpeed(rotateSpeed);
        }
        if (gamepad1.a){
            board.launchDrone();
        }

        board.setArmSpeed(armSpeed);

        if(gamepad2.a){
            //board.setServoDown();
            //board.setServoDown();
            //board.setRotationDirectionFORWARD();
            board.setClawRotation(0.0);
        }
        if(gamepad2.y){
            //board.setServoUp();
            //board.setRotationDirectionREVERSE();
            board.setClawRotation(0.25);
        }
        if(gamepad2.x){
            board.setClaw_1Active();
        }
        if(gamepad2.b){
            board.setClaw_2Active();
        }
        if(gamepad2.right_bumper){
            board.setClaw_1Inactive();
            board.setClaw_2Inactive();
        }
        if(gamepad2.dpad_up){
            MoveArmDegrees(15, 0.3);
        }
        if(gamepad2.dpad_down){
            MoveArmDegrees(-15, 0.3);
        }
        if(gamepad2.dpad_left){
            board.setClawRotation(0.2);
        }
        if(gamepad2.dpad_right){
            board.setClawRotation(0.15);
        }

        telemetry.addData("Arm Speed: ", armSpeed);
        telemetry.addData("Motor speed: ", forwardSpeed);
        telemetry.addData("Motor rotations: ", board.getMotorRotations());
        telemetry.addData("Distance: ", millimeters);
        telemetry.addData("gear: ", gear);
        telemetry.update();
    }
    public void MoveArmDegrees(double degrees, double Speed){
        double initialArmRotation = board.getArmMotorRotations();
        double angleDeg = Math.abs(armconstant * (board.getArmMotorRotations() - initialArmRotation));
        if (degrees > 0) {
            while (angleDeg < degrees) {
                //elevatorheight();
                board.setArmSpeed(Speed);
                angleDeg = (armconstant * (board.getArmMotorRotations() - initialArmRotation));
                telemetry.addData("arm angle (degrees) ", angleDeg);
                telemetry.update();
            }
        }
        else if (degrees < 0) {
            while (angleDeg > degrees) {
                //elevatorheight();
                board.setArmSpeed(-Speed);
                angleDeg = (armconstant * (board.getArmMotorRotations() - initialArmRotation));
                telemetry.addData("arm angle (degrees) ", angleDeg);
                telemetry.update();
            }
        }
        board.setArmSpeed(0);
        telemetry.addData("degrees?", angleDeg);
        telemetry.update();
    }
}


