// File:          Ground_java.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes

import com.cyberbotics.webots.controller.*;

import java.util.Arrays;

// This class defines how to initialize and how to run your controller.
public class Ground_java {

    static final int TIME_STEP = 64;

    static final int X = 0;
    static final int Z = 2;

    // size of the ground
    static final double GROUND_X = 4.9;
    static final double GROUND_Z = 4.9;

    public static double getVelocityFromVector(double[] vector) {
        return Math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    }

    // Main function of the controller.
    public static void main(String[] args) {

        Supervisor supervisor = new Supervisor();

        // First we get a handler to devices
        Display display = supervisor.getDisplay("ground_display");

        // get the properties of the Display
        int width = display.getWidth();
        int height = display.getHeight();

        // prepare stuff to get the
        // Robot(IROBOT_CREATE).translation field
        Node mybot = supervisor.getFromDef("IROBOT_CREATE");
        Field translationField = mybot.getField("translation");

        // set the background (otherwise an empty ground is displayed at this step)
        ImageRef background = display.imageLoad("../../worlds/textures/dirty.jpg");
        display.imagePaste(background, 0, 0, false);

        // set the pen to remove the texture
        display.setAlpha(0.0);

        // Main loop:
        // - perform simulation steps until Webots is stopping the controller
        while (supervisor.step(TIME_STEP) != -1) {

            //TODO sijainnin ja nopeuden ym. kirjaaminen csv-tiedostoon
            System.out.println("---");
            System.out.println("LOG");
            System.out.println("Velocity vector: " + Arrays.toString(mybot.getVelocity()));
            System.out.println("Velocity: " + getVelocityFromVector(mybot.getVelocity()));
            System.out.println("Translation: " + Arrays.toString(translationField.getSFVec3f()));

            // Update the translation field
            double[] translation = translationField.getSFVec3f();

            int intX = (int) (width * (translation[X] + GROUND_X / 2) / GROUND_X);
            int intZ = (int) (height * (translation[Z] + GROUND_Z / 2) / GROUND_Z);

            // display the robot position
            display.fillOval(intX, intZ, 14, 14);
        }

        // Enter here exit cleanup code.
    }
}
