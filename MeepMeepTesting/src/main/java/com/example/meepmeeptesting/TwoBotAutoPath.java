package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TwoBotAutoPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark()) // set our bot to be red
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -15, Math.PI))
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(195)) // position to shoot zero batch
                .waitSeconds(2)

                .strafeToLinearHeading(new Vector2d(35,-30), Math.toRadians(270)) // go to first set of artifacts
                .strafeToLinearHeading(new Vector2d(35,-50), Math.toRadians(270)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(195)) // go back after grabbing first set of artifacts to shoot
                .waitSeconds(2)


                .strafeToLinearHeading(new Vector2d(25,-63), Math.toRadians(180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(195))
                .waitSeconds(2)


                // extra not part of main auto plan
                .strafeToLinearHeading(new Vector2d(40,-63), Math.toRadians(180))// go grab anywehre
                .strafeToLinearHeading(new Vector2d(20,-63), Math.toRadians(180))// go grab anywehre
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(195)) // go to shoot
                .waitSeconds(2)


                .strafeToLinearHeading(new Vector2d(40,-15), Math.toRadians(180)) // get off launch line


                .build());


        RoadRunnerBotEntity alliancePartnerBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark()) // set the other bot to be blue
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        alliancePartnerBot.runAction(alliancePartnerBot.getDrive().actionBuilder(new Pose2d(-48, -52, Math.toRadians(232)))
                .strafeToLinearHeading(new Vector2d(-25,-25), Math.toRadians(232)) // go to shoot pre load
                .waitSeconds(2)

                .strafeToLinearHeading(new Vector2d(-12,-30), Math.toRadians(270)) // go to get first set of artifacts
                .strafeToLinearHeading(new Vector2d(-12,-50), Math.toRadians(270)) // drive into first set of artifacts


                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-2,-58), Math.toRadians(270)) // push classifier gate
                .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(-25,-25), Math.toRadians(232)) // go to shoot first set of artifacts
                .waitSeconds(2)

                .strafeToLinearHeading(new Vector2d(12,-30), Math.toRadians(270)) // go to second set of artifacts
                .strafeToLinearHeading(new Vector2d(12,-50), Math.toRadians(270)) // drive into second set of artifacts


                .strafeToLinearHeading(new Vector2d(-25,-25), Math.toRadians(232))
                .waitSeconds(2)

                .strafeToLinearHeading(new Vector2d(0,-25), Math.toRadians(90))




                .build());





        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(alliancePartnerBot)
                .start();
    }
}