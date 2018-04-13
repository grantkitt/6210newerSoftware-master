/*
AutoMain_v1
9/18/2017
6210 Software
- William Fisher
- Rohit Chawla
- Nihal Kyasa

Controls robot with methods from AutoLibrary class in the
autonomous period of FTC's Relic Recovery competition.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous (name="AutoBlueInner_v3", group="Auto")
public class AutoBlueInner_v3 extends AutoLibrary_v2{

    private int targetColumn;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        double angle = getAngle();
        sleep(250);
        targetColumn = getSymbol();
        sleep(250);
        extendGem(1300, true);
        sleep(250);
        getGem(5, false);
        sleep(250);
        extendGem(1300, false);
        sleep(250);
        move_encoder(.25, 0, 35);
        sleep(250);
        move_advanced_x(-.35, angle, .86, 2000);
        sleep(250);
        telemetry.addLine("Starting Turn");
        telemetry.update();
        sleep(250);
        turn_gyro(.3, 90, 15); // .4, 90, 2
        sleep(250);
        angle = getAngle();
        telemetry.addData("Final Angle", angle);
        sleep(250);
        move_advanced_x(-.35, angle, .86, 350); //650
        sleep(250);
        //RIGHT
        if (targetColumn == 3)
        {
            move_advanced_x(-.35, angle, .86, 1025); //1000
            telemetry.addLine("Move : ToRight : Complete");
            telemetry.update();
        }
        //CENTER
        else if (targetColumn == 2)
        {
            move_advanced_x(-.35, angle, .86, 515); //530
            telemetry.addLine("Move : ToCenter : Complete");
            telemetry.update();
        }
        //LEFT or CENTER
        else
        {
            telemetry.addLine("Move : ToLeft or Unknown : Complete");
            telemetry.update();
        }
        sleep(250);
        move_encoder(-.25, 0, 250);
        startOutput(-1);
        sleep(1500);
        move_encoder(.25, 0, 200);
        sleep(100);
        move_encoder(-.2, 0, 200);
        sleep(100);
        move_encoder(.25, 0, 400);
        stopOutput();
        sleep(100);
    }
}