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

package org.aluminati3555.lib.auto;

import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class manages autonomous mode selection
 * 
 * @author Caleb Heydon
 */
public class AluminatiAutoSelector extends Thread {
    private static final int MAX_PACKET_LENGTH = 64;

    private int port;
    private ArrayList<Entry> entries;

    private DatagramSocket socket;
    private DatagramPacket packet;

    private AluminatiAutoTask autoMode;

    @Override
    public String toString() {
        return "[AutoSelector] port: " + port;
    }

    /**
     * Publishes the auto modes to network tables
     */
    private void publish() {
        String[] modes = new String[entries.size()];
        for (int i = 0; i < entries.size(); i++) {
            modes[i] = entries.get(i).name;
        }

        NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Auto List").setStringArray(modes);
    }

    /**
     * Finds a mode by the name
     */
    private AluminatiAutoTask getModeByName(String name) {
        for (int i = 0; i < entries.size(); i++) {
            if (entries.get(i).name.equals(name)) {
                return entries.get(i).mode;
            }
        }

        return null;
    }

    /**
     * Selects an auto mode
     */
    private void select(String mode) {
        autoMode = getModeByName(mode);
    }

    /**
     * Returns the currently selected auto mode (does not lock in network tables
     * mode if udp mode does not exist)
     */
    public AluminatiAutoTask getEarlySelected() {
        synchronized (this) {
            if (autoMode != null) {
                return autoMode;
            }

            String auto = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Auto Selector")
                    .getString(null);

            if (auto == null) {
                return null;
            }

            return getModeByName(auto);
        }
    }

    /**
     * Returns the selected mode (This method sets the auto mode as the network
     * tables selected one if there is no udp selected mode)
     */
    public AluminatiAutoTask getSelected() {
        synchronized (this) {
            if (autoMode != null) {
                return autoMode;
            }

            String auto = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Auto Selector")
                    .getString(null);

            if (auto == null) {
                return null;
            }

            select(auto);
            return autoMode;
        }
    }

    @Override
    public void run() {
        while (true) {
            try {
                socket.receive(packet);

                ByteArrayInputStream byteInput = new ByteArrayInputStream(packet.getData());
                DataInputStream input = new DataInputStream(byteInput);

                String data = input.readUTF();
                input.close();

                synchronized (this) {
                    select(data);
                }
            } catch (IOException e) {
                Timer.delay(1);
                continue;
            }
        }
    }

    public AluminatiAutoSelector(int port, Entry... entries) {
        System.out.println("Starting UDP listener for AutoSelector on port " + port + "...");

        this.port = port;
        try {
            this.socket = new DatagramSocket(port);
        } catch (SocketException e) {
            DriverStation.reportError("Unable to start UDP listener for AutoSelector on port " + port, false);
            return;
        }
        this.packet = new DatagramPacket(new byte[MAX_PACKET_LENGTH], MAX_PACKET_LENGTH);

        super.setName("Auto-Selector");
        super.setPriority(Thread.MIN_PRIORITY);
        super.start();

        this.entries = new ArrayList<Entry>();
        for (int i = 0; i < entries.length; i++) {
            this.entries.add(entries[i]);
        }

        publish();
    }

    public static class Entry {
        public String name;
        public AluminatiAutoTask mode;

        public Entry(String name, AluminatiAutoTask mode) {
            this.name = name;
            this.mode = mode;
        }
    }
}
