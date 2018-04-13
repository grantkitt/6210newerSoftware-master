/*
TeleOP_1P
2/23/2018
6210 Software
- William Fisher
- Grant Kitlowski

Allows one driver to control the robot using a single gamepad during
the driver controlled period of FTC's Relic Recovery competition.
 */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp 1P", group="TeleOp")
public class TeleOp_1P extends TeleOpLibrary_v2
{
    private double drivePowerMod;
    private boolean tank;
    private boolean hold;
    private double reverse2;
    private boolean relic_claw;

    @Override
    public void init() {
        drivePowerMod = 1;
        tank = false;
        hold = false;
        relic_claw = true;
        reverse2 = 1;
        initialize();
        //RelicArm.setPosition(.1);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
    }
    */

    /*
     * Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        runtime.reset();
    }
    */

    @Override
    public void loop() {

        // Mecanum Drive
        if (!tank)
        {
            drive_mecanum(drivePowerMod);
        }
        //tank drive
        else
        {
            drive_tank(drivePowerMod);
        }
        topTrackManual(gamepad1.left_stick_y, hold);
        intake(gamepad1.right_bumper, gamepad1.left_bumper, gamepad2.right_bumper, gamepad2.left_bumper);
        relic(1, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.dpad_down, gamepad1.dpad_up, relic_claw); //henlo my guy
        hold = toggle(hold, gamepad1.y);
        tank = toggle(tank, gamepad1.b);
        relic_claw = toggle(relic_claw, gamepad1.a);
        //pushglyph(gamepad1.a);
        drivePowerMod = toggleDouble(drivePowerMod, gamepad1.x, .8, .4);
        reverse2 = toggleDouble(reverse2, gamepad1.y, 1, -1);
        setReverse(reverse2);


    }
    @Override
    public void stop() {
    }

}
