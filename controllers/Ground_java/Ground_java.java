// File:          Ground_java.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes

import com.cyberbotics.webots.controller.*;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

// This class defines how to initialize and how to run your controller.
public class Ground_java {

    static final int TIME_STEP = 64;

    static final int X = 0;
    static final int Z = 2;

    //delay tulostusten välissä sekunteina
    static final int DELAY = 1;

    // size of the ground
    static final double GROUND_X = 4.9;
    static final double GROUND_Z = 4.9;

    private static final DecimalFormat df = new DecimalFormat("0.000");

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

        //long time = System.currentTimeMillis();
        double time = supervisor.getTime();

        List<String[]> data = new ArrayList<>();

        //otsikkorivi
        data.add(new String[]{"time", "velocity", "x", "y"});

        // Main loop:
        // - perform simulation steps until Webots is stopping the controller
        while (supervisor.step(TIME_STEP) != -1) {

            if (time + DELAY < supervisor.getTime()) {
                time = supervisor.getTime();

                data.add(new String[]{
                        Double.toString(supervisor.getTime()).split("\\.")[0],
                        df.format(getVelocityFromVector(mybot.getVelocity())),
                        df.format(translationField.getSFVec3f()[0]),
                        df.format(translationField.getSFVec3f()[2]),
                });

                //TODO tähän jokin ehto, jolla tunnistetaan että robotti lopettanut
                if (supervisor.getTime() > 600) {
                    break;
                }

                //System.out.println("---");
                //System.out.println("LOG");
                //System.out.println("Velocity vector: " + Arrays.toString(mybot.getVelocity()));
                //System.out.println("TIME: " + supervisor.getTime());
                //System.out.println("Velocity: " + df.format(getVelocityFromVector(mybot.getVelocity())));
                //System.out.println("Translation: " + Arrays.toString(translationField.getSFVec3f()));
                //System.out.println("X: " + df.format(translationField.getSFVec3f()[0]));
                //System.out.println("Y: " + df.format(translationField.getSFVec3f()[2]));
            }

            // Update the translation field
            double[] translation = translationField.getSFVec3f();

            int intX = (int) (width * (translation[X] + GROUND_X / 2) / GROUND_X);
            int intZ = (int) (height * (translation[Z] + GROUND_Z / 2) / GROUND_Z);

            // display the robot position
            display.fillOval(intX, intZ, 14, 14);
        }

        //kirjaillaan data tiedostoon
        try {
            CSVUtils.writeToCSVFile(data, "export.csv");
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
        // Enter here exit cleanup code.

    }

    static class CSVUtils {

        private CSVUtils() {
        }

        /**
         * Lista merkkijonotaulukoita csv-tiedostoksi
         *
         * @param lines datalista
         */
        public static void writeToCSVFile(List<String[]> lines, String filename) throws IOException {
            //luodaan tiedosto
            File csvOutput = new File(filename);

            //kirjoitetaan
            try (PrintWriter pw = new PrintWriter(csvOutput)) {
                lines.stream()
                        .map(CSVUtils::convertLineToCSV)
                        .forEach(pw::println);
            }
        }

        /**
         * Muunnetaan merkkijonotaulukko csv-riviksi
         *
         * @param data taulukko
         * @return csv-rivi merkkijonomuotoisena
         */
        public static String convertLineToCSV(String[] data) {
            return Arrays.stream(data)
                    .map(element -> element.replace(",", "."))
                    .collect(Collectors.joining(","));
        }

    }

}
