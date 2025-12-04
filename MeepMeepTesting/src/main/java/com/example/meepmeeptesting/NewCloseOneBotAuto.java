package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class NewCloseOneBotAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark()) // set our bot to be red
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // old starting pos new Pose2d(-10, -14.5, Math.toRadians(180))
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-59,-42, Math.toRadians(233)))

                /*

                 This code below takes 6.3 seconds on meep meep and is being replaced fo rthe spline version which takes 5.8 seconds on meep meep, to be tested and determine which one to use

                .strafeToLinearHeading(new Vector2d(-44,-28),Math.toRadians(240)) // go to shoot first batch
                .waitSeconds(1.5)

                .strafeToLinearHeading(new Vector2d(-12,-27), Math.toRadians(270)) // go to get first set of artifacts
                .strafeToLinearHeading(new Vector2d(-12,-50), Math.toRadians(270)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(-2,-58), Math.toRadians(270)) // push classifier gate


                 */



                .strafeToLinearHeading(new Vector2d(-35,-24),Math.toRadians(230)) // go to shoot preload
                .waitSeconds(2)

                .splineToLinearHeading(new Pose2d(-13,-29, Math.toRadians(270)), Math.toRadians(270)) // go to first set of artifacts
                .strafeToLinearHeading(new Vector2d(-12,-50), Math.toRadians(270)) // drive into first set of artifacts
                .splineToLinearHeading(new Pose2d(-2,-58, Math.toRadians(270)), Math.toRadians(270)) //push classifier gate
                .waitSeconds(1)// wait for gate




                .strafeToLinearHeading(new Vector2d(-40,-28),Math.toRadians(240)) // go to shoot first batch
                .waitSeconds(2)



//                .strafeToLinearHeading(new Vector2d(12,-30), Math.toRadians(270)) // go to second set of artifacts
//                .strafeToLinearHeading(new Vector2d(12,-50), Math.toRadians(270)) // drive into second set of artifacts
//
//
//                .strafeToLinearHeading(new Vector2d(-44,-28),Math.toRadians(240)) // go to shoot second batch\
//                .waitSeconds(1.5)


                //trying to replace the drive into artifacts + go to shoot from 2 strafes to a spline and a strafe

                .strafeToLinearHeading(new Vector2d(5,-30), Math.toRadians(270)) // go to second set of artifacts
                .splineToLinearHeading(new Pose2d(7,-54,Math.toRadians(270)),Math.toRadians(170))


                .strafeToLinearHeading(new Vector2d(-40,-28),Math.toRadians(240)) // go to shoot second batch\
                .waitSeconds(2)





                .strafeToLinearHeading(new Vector2d(27,-30), Math.toRadians(270)) // go to third set of artifacts




                .splineToLinearHeading(new Pose2d(30, -56, Math.toRadians(270)),Math.toRadians(160)) //drive into third row
                .strafeToLinearHeading(new Vector2d(-40,-28),Math.toRadians(240)) // go to shoot
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-50,-28),Math.toRadians(240)) // get off launch line





                .build());







        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}