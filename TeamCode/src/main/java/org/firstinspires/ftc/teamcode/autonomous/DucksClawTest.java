package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;

@Autonomous
public class DucksClawTest extends LinearOpMode{
    DucksProgrammingBoard1_4 board = new DucksProgrammingBoard1_4();
    int state = 0;
    @Override
    public void runOpMode() {
        board.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (state == 0) {
                //board.setClawRotation(0.1);
                state=1;
            }
            if (state == 1){
                state=2;
            }
        }
    }
}
