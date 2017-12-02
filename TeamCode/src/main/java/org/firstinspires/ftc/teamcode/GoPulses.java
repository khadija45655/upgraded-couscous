package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by khadija on 11/25/2017.
 */

@Autonomous(name = "GoColumn" , group ="Concept")

public class GoPulses extends Processor{
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();


        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
        sleep(1000);

        runtime.reset();
        while(runtime.milliseconds()<800) {
            bot.slideMotor.setPower(.2);
        }
        bot.slideMotor.setPower(0);
        runtime.reset();

        sleep(1000);

        bot.columnToScore = RelicRecoveryVuMark.CENTER;
        gotoColumnRight();

        score();

    }
}