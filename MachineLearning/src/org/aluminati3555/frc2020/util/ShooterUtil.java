/**
 * Copyright (c) 2020 Team 3555
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.aluminati3555.frc2020.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

import org.encog.engine.network.activation.ActivationSigmoid;
import org.encog.mathutil.randomize.ConsistentRandomizer;
import org.encog.ml.data.MLData;
import org.encog.ml.data.MLDataPair;
import org.encog.ml.data.MLDataSet;
import org.encog.ml.data.basic.BasicMLDataSet;
import org.encog.neural.networks.BasicNetwork;
import org.encog.neural.networks.layers.BasicLayer;
import org.encog.neural.networks.training.propagation.resilient.ResilientPropagation;
import org.encog.persist.EncogDirectoryPersistence;

/**
 * This class provides utilities for the shooter system. This includes distance
 * to rpm calculations.
 * 
 * This class uses a neural network. There is one input and one output. The
 * input is the target height in degrees divided by 100, and the output is the
 * rpm / 10,000.
 * 
 * @author Caleb Heydon
 */
public class ShooterUtil {
    private static final int HIDDEN_LAYERS = 2;
    private static final int NEURONS_PER_LAYER = 6;
    private static final double MAX_ERROR = 0.0000001;

    private static BasicNetwork basicNetwork;

    /**
     * This method calculates a target rpm from the target height
     */
    public static double calculateRPM(double targetHeight) {
        if (basicNetwork == null) {
            return 0;
        }

        double[][] inputs = new double[1][1];
        double[][] outputs = new double[1][1];

        inputs[0][0] = targetHeight / 100;

        MLDataSet data = new BasicMLDataSet(inputs, outputs);
        MLDataPair dataPair = data.get(0);

        MLData output = basicNetwork.compute(dataPair.getInput());

        return output.getData(0) * 10000;
    }

    /**
     * Loads the neural network from a file
     */
    public static void load(String path) {
        basicNetwork = (BasicNetwork) EncogDirectoryPersistence.loadObject(new File(path));
    }

    /**
     * Trains a neural network from a text file of training data
     * 
     * @throws FileNotFoundException
     */
    public static void create(String path) throws FileNotFoundException {
        // Create network and add input layer
        BasicNetwork basicNetwork = new BasicNetwork();
        basicNetwork.addLayer(new BasicLayer(null, true, 1));

        // Add hidden layers
        for (int i = 0; i < HIDDEN_LAYERS; i++) {
            basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), true, NEURONS_PER_LAYER));
        }

        // Add output layer
        basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), false, 1));

        // Final setup
        basicNetwork.getStructure().finalizeStructure();
        basicNetwork.reset();

        // Randomize weights
        new ConsistentRandomizer(-1, 1, 100).randomize(basicNetwork);

        // Train the neural network

        ArrayList<Double> inputs = new ArrayList<Double>();
        ArrayList<Double> outputs = new ArrayList<Double>();

        Scanner input = new Scanner(new File(path));

        while (input.hasNextLine()) {
            String line = input.nextLine();
            String[] components = line.split(" ");

            if (components.length < 2) {
                continue;
            }

            try {
                inputs.add(Double.parseDouble(components[0]) / 100);
            } catch (NumberFormatException e) {
                continue;
            }

            try {
                outputs.add(Double.parseDouble(components[1]) / 10000);
            } catch (NumberFormatException e) {
                inputs.remove(inputs.size() - 1);
                continue;
            }
        }

        input.close();

        double[][] inputArray = new double[inputs.size()][1];
        double[][] outputArray = new double[outputs.size()][1];

        for (int i = 0; i < inputs.size(); i++) {
            inputArray[i][0] = inputs.get(i);
            outputArray[i][0] = outputs.get(i);
        }

        MLDataSet data = new BasicMLDataSet(inputArray, outputArray);

        // Train network
        ResilientPropagation resilientPropagation = new ResilientPropagation(basicNetwork, data);

        double currentError = 1;
        do {
            resilientPropagation.iteration();

            currentError = resilientPropagation.getError();
            System.out.println("Current error: " + currentError);
        } while (currentError > MAX_ERROR);
        resilientPropagation.finishTraining();

        ShooterUtil.basicNetwork = basicNetwork;
    }

    /**
     * Saves the neural network to a file
     */
    public static void save(String path) {
        EncogDirectoryPersistence.saveObject(new File(path), basicNetwork);
    }
}
