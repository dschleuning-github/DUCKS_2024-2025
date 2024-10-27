package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DucksProgrammingBoard1_4 {
    //initializing objects:
    //private DigitalChannel touchSensor;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private double ticksPerRotation;
    private DcMotor motor_arm_0;
    private Servo clawRotation;
    private Servo servo;
    private double armTicksPerRotation;
    private Servo CLAW_1;
    private Servo CLAW_2;
    private Servo droneLauncher;
    /*
    //private Servo claw_left;
    //private Servo claw_right;
    //private Servo DuckServo_0;
    //private Servo DuckServo_1;
    //private Servo DuckServo_2;
    //private Servo DuckServo_3;
    */


    public void init(HardwareMap hwMap) {
        //touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        //touchSensor.setMode(DigitalChannel.Mode.INPUT);
        motor0 = hwMap.get(DcMotor.class, "motor0");
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1 = hwMap.get(DcMotor.class, "motor1");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2 = hwMap.get(DcMotor.class, "motor2");
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3 = hwMap.get(DcMotor.class, "motor3");
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor1.getMotorType().getTicksPerRev();
        motor_arm_0 = hwMap.get(DcMotor.class, "motor_arm_0");
        motor_arm_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTicksPerRotation = motor_arm_0.getMotorType().getTicksPerRev();
        clawRotation = hwMap.get(Servo.class, "clawRotation");
        CLAW_1 = hwMap.get(Servo.class, "CLAW_1");
        CLAW_2 = hwMap.get(Servo.class, "CLAW_2");
        droneLauncher = hwMap.get(Servo.class, "droneLauncher");
        droneLauncher.setDirection(Servo.Direction.REVERSE);
        //claw_left = hwMap.get(Servo.class, "claw_left");
        //claw_right = hwMap.get(Servo.class, "claw_right");
        //claw_right.setDirection(Servo.Direction.REVERSE);
        //DuckServo_0 = hwMap.get(Servo.class, "DuckServo_0");
        //DuckServo_1 = hwMap.get(Servo.class, "DuckServo_1");
        //DuckServo_2 = hwMap.get(Servo.class, "DuckServo_2");
        //DuckServo_3 = hwMap.get(Servo.class, "DuckServo_3");
    }


    //public boolean isTouchSensorPressed()
    //{
    // return !touchSensor.getState();
    //}


    //functions for movement (forward/backward, sideways, rotation)
    //motor 0 RF
    //motor 1 LF
    //motor 2 RB
    //motor 3 LB
    public void setForwardSpeed(double speed) {
        motor0.setPower(-speed);
        motor1.setPower(speed);
        motor2.setPower(speed);
        motor3.setPower(speed);
    }
    public void setSideMotorSpeed(double speed) {
        motor0.setPower(speed);
        motor1.setPower(speed);
        motor2.setPower(speed);
        motor3.setPower(-speed);
    }
    public void setRotateSpeed(double speed) {
        motor0.setPower(speed);
        motor1.setPower(speed);
        motor2.setPower(-speed);
        motor3.setPower(speed);
    }
    public void launchDrone(){
        droneLauncher.setPosition(180);
    }


    //functions for arm movement
    public void setArmSpeed(double armspeed) {
        motor_arm_0.setPower(armspeed);
    }
    public void setClawRotation(double position) {
        if (position >= 0){
            clawRotation.setDirection(Servo.Direction.FORWARD);
            clawRotation.setPosition(position);
        }
        else if (position < 0) {
            clawRotation.setDirection(Servo.Direction.REVERSE);
            clawRotation.setPosition(-position);
        }
    }

    //public void setServoDown(){
        //servo.setPosition(0.0);
    //}
    //public void setServoUp(){
        //servo.setPosition(0.5);
    //}

    //public void setClawPosition(double CLAWposition){
        //CLAW.setDirection(Servo.Direction.FORWARD);
        //CLAW.setPosition(CLAWposition);
    //}

    public void setClaw_1Position(double CLAW_1position) {
        CLAW_1.setPosition(CLAW_1position);
    }
    public void setClaw_2Position(double CLAW_2position) {
        CLAW_2.setPosition(CLAW_2position);
    }

    public void setClaw_1Active() {
        setClaw_1Position(-0.5);
    }
    public void setClaw_2Active() {
        setClaw_2Position(0.1);
    }
    public void setClaw_1Inactive() {
        setClaw_1Position(0.15);

    }
    public void setClaw_2Inactive() {
        setClaw_2Position(-0.5);

    }
    public void setClawZero(){
        setClaw_1Position(0.01);
    }

    public void setRotationDirectionFORWARD(){
        CLAW_1.setDirection(Servo.Direction.FORWARD);
    }
    public void setRotationDirectionREVERSE(){
        CLAW_1.setDirection(Servo.Direction.REVERSE);
    }




    //functions for claw movement
    //public void setClawClosed() {
        //claw_left.setPosition(0.45);
        //claw_right.setPosition(0.33);
    //}
    //public void setClawOpen() {
        //claw_left.setPosition(0.6);
        //claw_right.setPosition(0.45);
    //}
    //neutral makes the claws point straight
    //public void setClawNeutral() {
        //claw_left.setPosition(0.55);
        //claw_right.setPosition(0.40);
    //}

    public void rotateDucks() {
        //DuckServo_0.setPosition(180);
        //DuckServo_1.setPosition(180);
        //DuckServo_2.setPosition(180);
        //DuckServo_3.setPosition(180);
    }
    public void setDuckStraight() {
        //DuckServo_0.setPosition(0);
        //DuckServo_1.setPosition(0);
        //DuckServo_2.setPosition(0);
        //DuckServo_3.setPosition(0);
    }

    //motor position return statements
    public double getMotorRotations() {
        return motor1.getCurrentPosition() / ticksPerRotation;
    }
    public double getArmMotorRotations() {
        return motor_arm_0.getCurrentPosition() / armTicksPerRotation;
    }
}
