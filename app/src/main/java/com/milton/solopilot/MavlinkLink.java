package com.milton.solopilot;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

/**
 * Talks MAVLink to the Solo through the controller's WiFi.
 * The controller forwards the drone's telemetry to UDP port 14550 on the phone.
 * Version 1 is listen-only: the only thing it sends is a "ground station is here"
 * heartbeat. It never sends any command that can move the drone.
 */
public class MavlinkLink {

    public interface TextListener {
        void onStatusText(String text);
    }

    public static final String CONTROLLER_IP = "10.1.1.1";
    public static final int PORT = 14550;

    /** Latest values from the drone. Read from the screen thread. */
    public static class Telemetry {
        public volatile long lastPacketMs;
        public volatile long lastHeartbeatMs;
        public volatile boolean armed;
        public volatile long customMode = -1;
        public volatile double battVolts = -1;
        public volatile double battAmps = -1;
        public volatile int battPct = -1;
        public volatile int gpsFix = -1;
        public volatile int sats = -1;
        public volatile boolean hasPos;
        public volatile double lat;
        public volatile double lon;
        public volatile double relAltM;
        public volatile boolean hasHud;
        public volatile double groundSpeed;
        public volatile double climb;
        public volatile int heading = -1;
        public volatile double roll;
        public volatile double pitch;
    }

    // Per-message checksum seeds from the MAVLink "common" message set.
    private static final int[] CRC_EXTRA = new int[256];

    static {
        Arrays.fill(CRC_EXTRA, -1);
        CRC_EXTRA[0] = 50;    // HEARTBEAT
        CRC_EXTRA[1] = 124;   // SYS_STATUS
        CRC_EXTRA[24] = 24;   // GPS_RAW_INT
        CRC_EXTRA[30] = 39;   // ATTITUDE
        CRC_EXTRA[33] = 104;  // GLOBAL_POSITION_INT
        CRC_EXTRA[74] = 20;   // VFR_HUD
        CRC_EXTRA[253] = 83;  // STATUSTEXT
    }

    public final Telemetry t = new Telemetry();
    private final TextListener listener;
    private volatile boolean running;
    private DatagramSocket socket;
    private int seq;

    public MavlinkLink(TextListener listener) {
        this.listener = listener;
    }

    public void start() {
        running = true;
        try {
            socket = new DatagramSocket(null);
            socket.setReuseAddress(true);
            socket.bind(new InetSocketAddress(PORT));
            socket.setSoTimeout(1000);
        } catch (Exception e) {
            running = false;
            say("Could not open telemetry port: " + e.getMessage());
            return;
        }
        new Thread(this::receiveLoop, "mav-rx").start();
        new Thread(this::heartbeatLoop, "mav-tx").start();
    }

    public void stop() {
        running = false;
        if (socket != null) {
            socket.close();
        }
    }

    private void say(String s) {
        if (listener != null) {
            listener.onStatusText(s);
        }
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ignored) {
        }
    }

    private void receiveLoop() {
        byte[] buf = new byte[4096];
        DatagramPacket pkt = new DatagramPacket(buf, buf.length);
        while (running) {
            try {
                pkt.setLength(buf.length);
                socket.receive(pkt);
                t.lastPacketMs = System.currentTimeMillis();
                parse(buf, pkt.getLength());
            } catch (SocketTimeoutException e) {
                // nothing this second, keep listening
            } catch (Exception e) {
                if (!running) {
                    break;
                }
                sleep(200);
            }
        }
    }

    /** Sends a ground-station heartbeat once a second so the controller knows we're here. */
    private void heartbeatLoop() {
        while (running) {
            try {
                InetAddress dest = InetAddress.getByName(CONTROLLER_IP);
                byte[] p = new byte[9];
                ByteBuffer b = ByteBuffer.wrap(p).order(ByteOrder.LITTLE_ENDIAN);
                b.putInt(0);          // custom_mode
                b.put((byte) 6);      // type = GCS
                b.put((byte) 8);      // autopilot = INVALID (we're not an autopilot)
                b.put((byte) 0);      // base_mode
                b.put((byte) 0);      // system_status
                b.put((byte) 3);      // mavlink_version
                byte[] frame = frameV1(0, p);
                socket.send(new DatagramPacket(frame, frame.length, dest, PORT));
            } catch (Exception ignored) {
            }
            sleep(1000);
        }
    }

    private byte[] frameV1(int msgId, byte[] payload) {
        byte[] f = new byte[payload.length + 8];
        f[0] = (byte) 0xFE;
        f[1] = (byte) payload.length;
        f[2] = (byte) (seq++ & 0xFF);
        f[3] = (byte) 255;   // our system id (ground station)
        f[4] = (byte) 190;   // our component id (mission planner)
        f[5] = (byte) msgId;
        System.arraycopy(payload, 0, f, 6, payload.length);
        int crc = 0xFFFF;
        for (int i = 1; i < 6 + payload.length; i++) {
            crc = crcAcc(f[i] & 0xFF, crc);
        }
        crc = crcAcc(CRC_EXTRA[msgId], crc);
        f[6 + payload.length] = (byte) (crc & 0xFF);
        f[7 + payload.length] = (byte) ((crc >> 8) & 0xFF);
        return f;
    }

    static int crcAcc(int b, int crc) {
        int tmp = (b ^ (crc & 0xFF)) & 0xFF;
        tmp = (tmp ^ (tmp << 4)) & 0xFF;
        return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
    }

    private static boolean checkCrc(byte[] d, int start, int count, int msgId, byte lo, byte hi) {
        int extra = CRC_EXTRA[msgId];
        if (extra < 0) {
            return false; // a message we don't use
        }
        int crc = 0xFFFF;
        for (int i = start; i < start + count; i++) {
            crc = crcAcc(d[i] & 0xFF, crc);
        }
        crc = crcAcc(extra, crc);
        return (crc & 0xFF) == (lo & 0xFF) && ((crc >> 8) & 0xFF) == (hi & 0xFF);
    }

    /** Splits one UDP packet into MAVLink frames (version 1 or 2). */
    void parse(byte[] d, int n) {
        int i = 0;
        while (i < n) {
            int stx = d[i] & 0xFF;
            if (stx == 0xFE) {
                if (i + 8 > n) {
                    return;
                }
                int len = d[i + 1] & 0xFF;
                int total = len + 8;
                if (i + total > n) {
                    return;
                }
                int comp = d[i + 4] & 0xFF;
                int msgId = d[i + 5] & 0xFF;
                if (checkCrc(d, i + 1, 5 + len, msgId, d[i + 6 + len], d[i + 7 + len])) {
                    handle(comp, msgId, d, i + 6, len);
                }
                i += total;
            } else if (stx == 0xFD) {
                if (i + 12 > n) {
                    return;
                }
                int len = d[i + 1] & 0xFF;
                int incompat = d[i + 2] & 0xFF;
                int total = len + 12 + ((incompat & 0x01) != 0 ? 13 : 0);
                if (i + total > n) {
                    return;
                }
                int comp = d[i + 6] & 0xFF;
                int msgId = (d[i + 7] & 0xFF) | ((d[i + 8] & 0xFF) << 8) | ((d[i + 9] & 0xFF) << 16);
                if (msgId < 256 && checkCrc(d, i + 1, 9 + len, msgId, d[i + 10 + len], d[i + 11 + len])) {
                    handle(comp, msgId, d, i + 10, len);
                }
                i += total;
            } else {
                i++;
            }
        }
    }

    private void handle(int comp, int msgId, byte[] d, int off, int len) {
        // MAVLink 2 trims trailing zeros, so copy into a zero-filled buffer.
        byte[] p = new byte[Math.max(64, len)];
        System.arraycopy(d, off, p, 0, len);
        ByteBuffer b = ByteBuffer.wrap(p).order(ByteOrder.LITTLE_ENDIAN);
        long now = System.currentTimeMillis();

        switch (msgId) {
            case 0: { // HEARTBEAT - only trust the flight controller (component 1)
                int type = p[4] & 0xFF;
                if (comp != 1 || type == 6) {
                    break;
                }
                t.customMode = b.getInt(0) & 0xFFFFFFFFL;
                t.armed = (p[6] & 0x80) != 0;
                t.lastHeartbeatMs = now;
                break;
            }
            case 1: { // SYS_STATUS
                if (comp != 1) {
                    break;
                }
                int mv = b.getShort(14) & 0xFFFF;
                if (mv != 0xFFFF) {
                    t.battVolts = mv / 1000.0;
                }
                int ca = b.getShort(16);
                t.battAmps = ca < 0 ? -1 : ca / 100.0;
                t.battPct = p[30];
                break;
            }
            case 24: { // GPS_RAW_INT
                t.gpsFix = p[28] & 0xFF;
                t.sats = p[29] & 0xFF;
                break;
            }
            case 30: { // ATTITUDE
                t.roll = Math.toDegrees(b.getFloat(4));
                t.pitch = Math.toDegrees(b.getFloat(8));
                break;
            }
            case 33: { // GLOBAL_POSITION_INT
                double lat = b.getInt(4) / 1e7;
                double lon = b.getInt(8) / 1e7;
                t.lat = lat;
                t.lon = lon;
                t.relAltM = b.getInt(16) / 1000.0;
                t.hasPos = lat != 0 || lon != 0;
                int hdg = b.getShort(26) & 0xFFFF;
                if (hdg != 0xFFFF) {
                    t.heading = hdg / 100;
                }
                break;
            }
            case 74: { // VFR_HUD
                t.groundSpeed = b.getFloat(4);
                t.climb = b.getFloat(12);
                t.heading = b.getShort(16);
                t.hasHud = true;
                break;
            }
            case 253: { // STATUSTEXT
                int end = 1;
                while (end < 51 && p[end] != 0) {
                    end++;
                }
                String text = new String(p, 1, end - 1, StandardCharsets.US_ASCII).trim();
                if (!text.isEmpty()) {
                    say(text);
                }
                break;
            }
            default:
                break;
        }
    }

    public static String copterMode(long m) {
        switch ((int) m) {
            case 0: return "Stabilize";
            case 1: return "Acro";
            case 2: return "Manual (Alt Hold)";
            case 3: return "Auto";
            case 4: return "Guided";
            case 5: return "Fly (Loiter)";
            case 6: return "Return Home";
            case 7: return "Circle";
            case 9: return "Land";
            case 11: return "Drift";
            case 13: return "Sport";
            case 15: return "AutoTune";
            case 16: return "PosHold";
            case 17: return "Brake";
            case 18: return "Throw";
            case 21: return "Smart RTL";
            default: return "Mode " + m;
        }
    }

    public static String gpsFixName(int fix) {
        switch (fix) {
            case 0:
            case 1: return "No fix";
            case 2: return "2D fix";
            case 3: return "3D fix";
            case 4: return "DGPS";
            case 5: return "RTK float";
            case 6: return "RTK";
            default: return "--";
        }
    }
}
