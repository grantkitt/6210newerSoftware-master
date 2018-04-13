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

@Autonomous (name="BluOut2x", group="Auto")
public class BluOut2x extends AutoLibrary_v2 {

    private int targetColumn;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        double angle = getAngle();
        sleep(250);
        targetColumn = getSymbol();
        sleep(250);
        setRelic();

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
        move_advanced_x(-.35, angle, .86, 1850);
        telemetry.addLine("Move : ToLine : Complete");
        telemetry.update();
        sleep(250);
        if (targetColumn == 3) {
            move_advanced_x(-.35, angle, .86, 1050); //1000
            telemetry.addLine("Move : ToRight : Complete");
            telemetry.update();
        }
        //CENTER
        else if (targetColumn == 2) {
            move_advanced_x(-.35, angle, .86, 500); //525
            telemetry.addLine("Move : ToCenter : Complete");
            telemetry.update();
        }
        //LEFT or UNKNOWN
        else {
            telemetry.addLine("Move : ToLeft or Unknown : Complete");
            telemetry.update();
        }
        sleep(250);
        move_encoder(-.50, 0, 300);
        sleep(100);
        unfoldRobo();
        sleep(250);
        startIntake(1);
        //moving to the glyph pit
        move_encoder(.50, 0, 1000); //UNTESTED DISTANCE VALUE <<==============
        telemetry.addLine("Going for another glyph boys >:D");
        telemetry.update();
        sleep(400);
        //returning to the cryptobox
        move_encoder(-.25, 0, 1000); //UNTESTED DISTANCE VALUE <<===============
        telemetry.addLine("Drivers, take it from here ;D");
        telemetry.update();
        stopIntake();
        unfoldRobo();
        sleep(100);
        move_encoder(.25, 0, 300);
    }
}