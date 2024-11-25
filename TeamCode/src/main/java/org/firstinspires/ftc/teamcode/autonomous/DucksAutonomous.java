package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;

import java.util.List;

@Autonomous
//@Disabled
public class DucksAutonomous extends OpMode {
    DucksProgrammingBoard1_4 board = new DucksProgrammingBoard1_4();
    double forwardconstant = Math.PI * 75 * 523.875 / 457.2 * 514.35 / 457.2 * 417.5125 / 457.2 * 665 / 635 * 641 / 635 * 638 / 635;
    double rotationConstant = 360 * ((75 * Math.PI) / (545 * Math.PI)) * 92 / 90 * 90.7 / 90 * 88.8103 / 90 * 177 / 180;
    double sideconstant = Math.PI * 75 * 534 / 508 * 510 / 508 * 512 / 508;
    int state;

    public DcMotor armMotor    = null; //the arm motor
    public CRServo intake      = null; //the active intake servo
    public Servo wrist       = null; //the wrist servo


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;
    final double WRIST_ZERO = 0.0;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;


    //DucksProgrammingBoard1_4 board = new DucksProgrammingBoard1_4();
    //double gear = 1.0;
    double armconstant = 360 * 30/125 * 30/125 ;


    @Override
    public void init() {

        board.init(hardwareMap);

        /* Define and Initialize Motors */
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_front_drive"); //the left drivetrain motor
        //rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //the right drivetrain motor
        armMotor   = hardwareMap.get(DcMotor.class, "left_arm"); //the arm motor


        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        //leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(10);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(0);
        wrist.setPosition(0.8333);


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        intake = hardwareMap.get(CRServo.class, "intake");
        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (state == 0) {
            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            MoveForwardDistance(400, 0.4);
            state = 1;
        } else if (state == 1) {
            ducksSleepMilliSec(3000);
            state = 2;
        } else if (state == 2) {
            intake.setPower(1.0);
            ducksSleepMilliSec(1500);
            intake.setPower(0);
            state = 6;
        } else if (state == 6) {
            MoveForwardDistance(-50, 0.3);
            MoveSidewaysDistance(-250, 0.4);
            ducksSleepMilliSec(500);
            state = 7;
        } else if (state == 7) {
            MoveRotateDegrees(-90, 0.2);
            ducksSleepMilliSec(500);
            MoveSidewaysDistance(100, 0.2);
            state = 8;
        } else if (state == 8) {
            armPosition = ARM_COLLECT;
            wrist.setPosition(0.5);
            ducksSleepMilliSec(1000);
            intake.setPower(-1.0);
            state = 9;

        } else if (state == 9) {
            MoveForwardDistance(100, 0.2);
            ducksSleepMilliSec(1000);
            intake.setPower(0);
            state = 10;
        } else if (state == 10) {
            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            ducksSleepMilliSec(300);
            MoveForwardDistance(200, 0.4);
            ducksSleepMilliSec(300);
            MoveRotateDegrees(-45, 0.2);
            ducksSleepMilliSec(1000);
            MoveSidewaysDistance(200, 0.3);
            state = 11;
        } else if (state == 11) {
            MoveForwardDistance(300, 0.4);
            intake.setPower(1.0);
            ducksSleepMilliSec(1000);
            intake.setPower(0);
            state = 12;
        } else if (state == 12) {
            //board.setClaw_2Inactive();
            //ducksSleepMilliSec(300);
            state = 13;
        } else if (state == 13) {
            //MoveForwardDistance(-100, 0.4);
            //ducksSleepMilliSec(200);
            //board.setClawRotation(0.0);
            state = 14;
        }


        telemetry.update();

    }

    public void MoveRotateDegrees(double degrees, double rotateSpeed) {
        double initialWheelRotation = board.getMotorRotations();
        double deg_tolerance = 0.5;
        double delta_deg;
        delta_deg = degrees - (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
        while (Math.abs(delta_deg) > deg_tolerance) {
            if (delta_deg > 0) {
                board.setRotateSpeed(rotateSpeed);
            } else if (delta_deg < 0) {
                board.setRotateSpeed(-rotateSpeed);
            }
            delta_deg = degrees - (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
            if (Math.abs(delta_deg) < deg_tolerance) {
                board.setRotateSpeed(0);
                ducksSleepMilliSec(100);
                delta_deg = degrees - (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
                rotateSpeed = 0.1;
            }
        }
        board.setRotateSpeed(0);
    }

    /*
        public void MoveRotateDegrees(double degrees, double rotateSpeed) {
            double initialWheelRotation = board.getMotorRotations();
            double mm = (rotationConstant * (board.getMotorRotations()-initialWheelRotation));
            if (degrees > 0) {
                while (mm < degrees) {
                    //elevatorheight();
                    board.setRotateSpeed(rotateSpeed);
                    mm = (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
    //                telemetry.addData("rotations (mm?)=", mm);
    //                telemetry.update();
                }
            }
            else if (degrees < 0) {
                while (mm > degrees) {
                    //elevatorheight();
                    board.setRotateSpeed(-rotateSpeed);
                    mm = (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
     //               telemetry.addData("rotations (mm?)=", mm);
    //                telemetry.update();
                }
            }
            board.setRotateSpeed(0);
        }
     */
    public void ducksSleepMilliSec(int sleepTime) {
        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void MoveForwardDistance(double distance, double forwardSpeed) {
        double initialWheelRotation = board.getMotorRotations();
        double mm_tolerance = 10;
        double delta_mm;
        delta_mm = distance - (forwardconstant * (board.getMotorRotations() - initialWheelRotation));
        while (Math.abs(delta_mm) > mm_tolerance) {
            if (delta_mm > 0) {
                board.setForwardSpeed(forwardSpeed);
            } else if (delta_mm < 0) {
                board.setForwardSpeed(-forwardSpeed);
            }
            delta_mm = distance - (forwardconstant * (board.getMotorRotations() - initialWheelRotation));
            if (Math.abs(delta_mm) < mm_tolerance) {
                board.setForwardSpeed(0);
                ducksSleepMilliSec(100);
                delta_mm = distance - (forwardconstant * (board.getMotorRotations() - initialWheelRotation));
                forwardSpeed = 0.1;
            }

        }
        board.setForwardSpeed(0);
    }
    /*public void MoveForwardDistance(double distance, double forwardSpeed){
//        telemetry.addData("rotations forward", board.getMotorRotations());
        telemetry.update();
        double initialWheelRotation = board.getMotorRotations();
        double millimeters = (forwardconstant * (board.getMotorRotations()-initialWheelRotation));
//        telemetry.addData("millimeters", millimeters);
//        telemetry.update();
        if (distance > 0) {
            while (millimeters < distance) {
                //elevatorheight();
                if (millimeters > distance* 3/4){
                    forwardSpeed = .2;
                }

                board.setForwardSpeed(forwardSpeed);
                millimeters = (forwardconstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("millimeter slow", millimeters);
                telemetry.update();
            }
        }
        else if (distance < 0) {
            while (millimeters > distance) {
                //elevatorheight();
                board.setForwardSpeed(-forwardSpeed);
                millimeters = (forwardconstant * (board.getMotorRotations() - initialWheelRotation));
//                telemetry.addData("millimeter slow", millimeters);
//                telemetry.update();
            }
        }
        board.setForwardSpeed(0);
    }*/


    public void MoveSidewaysDistance(double distance, double sideSpeed) {
        double initialWheelRotation = board.getMotorRotations();
        double side_tolerance = 2;
        double delta_side;
        delta_side = distance - (sideconstant * (board.getMotorRotations() - initialWheelRotation));
        while (Math.abs(delta_side) > side_tolerance) {
            if (delta_side > 0) {
                board.setSideMotorSpeed(sideSpeed);
            } else if (delta_side < 0) {
                board.setSideMotorSpeed(-sideSpeed);
            }
            delta_side = distance - (sideconstant * (board.getMotorRotations() - initialWheelRotation));
            if (Math.abs(delta_side) < side_tolerance) {
                board.setSideMotorSpeed(0);
                ducksSleepMilliSec(100);
                delta_side = distance - (sideconstant * (board.getMotorRotations() - initialWheelRotation));
                sideSpeed = 0.1;
            }
        }
        board.setSideMotorSpeed(0);
    }
}
