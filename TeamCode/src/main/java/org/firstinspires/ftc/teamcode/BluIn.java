/*
AutoMain_v1
9/18/2017
6210 Software
- William Fisher
- Grant Kitlowski

Controls robot with methods from AutoLibrary class in the
autonomous period of FTC's Relic Recovery competition.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="BluIn", group="Auto")
public class BluIn extends AutoLibrary_v2{

    private int targetColumn;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        double angle = getAngle();
        sleep(250);
        targetColumn = getSymbol();
        sleep(250);
        setRelic();
        sleep(250);
        extendGem(1300, true);
        sleep(250);
        getGem(5, false);
        sleep(250);
        extendGem(700, false);
        sleep(500);
        extendGem(650, false);
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
            move_advanced_x(-.35, angle, .86, 1300); //UNTESTED DISTANCE VALUE
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
        move_encoder(-.50, 0, 300);
        sleep(100);
        unfoldRobo();
        sleep(100);
        move_encoder(.50, 0, 100);
        sleep(100);
        move_encoder(-.50, 0, 200);
        sleep(100);
        move_encoder(.50, 0, 300);
        telemetry.addLine("Drivers, take it from here ;D");
        telemetry.update();
        sleep(100);

         /*
        ================================== FUTURE 2 GLYPH AUTO ====================================
        move_encoder(-.50, 0, 300);
        sleep(100);
        unfoldRobo();
        sleep(100);
        move_encoder(.50, 0, 100);
        sleep(100);
        move_encoder(0, -.5, 2000);
        sleep(100);
        startIntake();
        move_encoder(.5, 0, 1000);
        sleep(100);
        move_encoder(-.5, 0, 1000);
        sleep(100);
        move_encoder(0, .5, 2000);
        sleep(50);
        move_encoder(-.5, 0, 300);
        sleep(50);
        move_encoder(.5, 0, 100);
        sleep(100);
        stopIntake();
        sleep(100);
        unfoldRobo();

        */
    }
}