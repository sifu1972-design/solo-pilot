package com.milton.solopilot;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.provider.Settings;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

import java.util.ArrayDeque;
import java.util.Locale;

public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {

    private static final double M_TO_FT = 3.28084;
    private static final double MPS_TO_MPH = 2.23694;

    private TextView tvStatus, tvMode, tvBattery, tvGps, tvAlt, tvSpeed, tvVideo, tvMessages, tvHint;
    private final Handler ui = new Handler(Looper.getMainLooper());
    private final ArrayDeque<String> messages = new ArrayDeque<>();

    private ConnectivityManager cm;
    private ConnectivityManager.NetworkCallback netCallback;
    private volatile Network wifiNetwork;
    private WifiManager.MulticastLock multicastLock;

    private MavlinkLink link;
    private VideoReceiver video;
    private Surface surface;
    private boolean running;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        tvStatus = findViewById(R.id.status);
        tvMode = findViewById(R.id.mode);
        tvBattery = findViewById(R.id.battery);
        tvGps = findViewById(R.id.gps);
        tvAlt = findViewById(R.id.altitude);
        tvSpeed = findViewById(R.id.speed);
        tvVideo = findViewById(R.id.video_status);
        tvMessages = findViewById(R.id.messages);
        tvHint = findViewById(R.id.center_hint);

        TextureView videoView = findViewById(R.id.video);
        videoView.setSurfaceTextureListener(this);

        Button wifi = findViewById(R.id.wifi_button);
        wifi.setOnClickListener(v -> startActivity(new Intent(Settings.ACTION_WIFI_SETTINGS)));

        cm = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
        WifiManager wm = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        if (wm != null) {
            multicastLock = wm.createMulticastLock("solopilot");
            multicastLock.setReferenceCounted(false);
        }

        String version = "";
        try {
            version = getPackageManager().getPackageInfo(getPackageName(), 0).versionName;
        } catch (Exception ignored) {
        }
        addMessage("Solo Pilot " + version + " (view only - can't move the drone)");
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) {
            getWindow().getDecorView().setSystemUiVisibility(
                    View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                            | View.SYSTEM_UI_FLAG_FULLSCREEN
                            | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                            | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                            | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                            | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN);
        }
    }

    @Override
    protected void onStart() {
        super.onStart();
        running = true;
        if (multicastLock != null) {
            multicastLock.acquire();
        }
        requestWifi();
        ui.post(tick);
    }

    @Override
    protected void onStop() {
        super.onStop();
        running = false;
        ui.removeCallbacks(tick);
        stopLinks();
        if (netCallback != null) {
            try {
                cm.unregisterNetworkCallback(netCallback);
            } catch (Exception ignored) {
            }
            netCallback = null;
        }
        cm.bindProcessToNetwork(null);
        wifiNetwork = null;
        if (multicastLock != null && multicastLock.isHeld()) {
            multicastLock.release();
        }
    }

    /**
     * The SoloLink WiFi has no internet, so Android would normally send our traffic
     * out over cellular instead. This asks for the WiFi network specifically and
     * routes everything this app does through it.
     */
    private void requestWifi() {
        NetworkRequest req = new NetworkRequest.Builder()
                .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
                .removeCapability(NetworkCapabilities.NET_CAPABILITY_NOT_RESTRICTED)
                .removeCapability(NetworkCapabilities.NET_CAPABILITY_TRUSTED)
                .removeCapability(NetworkCapabilities.NET_CAPABILITY_NOT_VPN)
                .build();
        netCallback = new ConnectivityManager.NetworkCallback() {
            @Override
            public void onAvailable(Network network) {
                ui.post(() -> useWifi(network));
            }

            @Override
            public void onLost(Network network) {
                ui.post(() -> {
                    if (network.equals(wifiNetwork)) {
                        wifiNetwork = null;
                        cm.bindProcessToNetwork(null);
                        stopLinks();
                    }
                });
            }
        };
        try {
            // Listen only - don't ask Android to change networks for us.
            cm.registerNetworkCallback(req, netCallback);
        } catch (Exception e) {
            addMessage("WiFi watch failed: " + e.getMessage());
        }
        findWifi();
    }

    /** Backup check: look through every network the phone has and pick the WiFi one. */
    private void findWifi() {
        if (wifiNetwork != null) {
            return;
        }
        try {
            for (Network n : cm.getAllNetworks()) {
                NetworkCapabilities caps = cm.getNetworkCapabilities(n);
                if (caps != null && caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)) {
                    useWifi(n);
                    return;
                }
            }
        } catch (Exception ignored) {
        }
    }

    private void useWifi(Network network) {
        if (!running || network.equals(wifiNetwork)) {
            return;
        }
        wifiNetwork = network;
        boolean bound = false;
        try {
            bound = cm.bindProcessToNetwork(network);
        } catch (Exception ignored) {
        }
        addMessage(bound ? "Found WiFi, talking to controller..." : "Found WiFi but couldn't lock onto it");
        stopLinks();
        startLinks();
        runDiagnostics(network);
    }

    /** Checks whether the phone can actually reach the controller, and reports in yellow text. */
    private void runDiagnostics(Network network) {
        new Thread(() -> {
            String addr = "unknown";
            try {
                android.net.LinkProperties lp = cm.getLinkProperties(network);
                if (lp != null) {
                    StringBuilder sb = new StringBuilder();
                    for (android.net.LinkAddress la : lp.getLinkAddresses()) {
                        if (la.getAddress() instanceof java.net.Inet4Address) {
                            sb.append(la.getAddress().getHostAddress()).append(' ');
                        }
                    }
                    if (sb.length() > 0) {
                        addr = sb.toString().trim();
                    }
                }
            } catch (Exception ignored) {
            }
            final String phoneAddr = addr;
            ui.post(() -> addMessage("Phone's WiFi address: " + phoneAddr));

            int[] ports = {22, 5502};
            for (int port : ports) {
                String result;
                java.net.Socket s = null;
                try {
                    s = network.getSocketFactory().createSocket();
                    s.connect(new java.net.InetSocketAddress(MavlinkLink.CONTROLLER_IP, port), 3000);
                    result = "open";
                } catch (Exception e) {
                    result = "closed (" + e.getClass().getSimpleName() + ")";
                } finally {
                    if (s != null) {
                        try {
                            s.close();
                        } catch (Exception ignored) {
                        }
                    }
                }
                final String msg = "Controller port " + port + ": " + result;
                ui.post(() -> addMessage(msg));
            }
        }, "diag").start();
    }

    private void startLinks() {
        if (link == null) {
            link = new MavlinkLink(text -> ui.post(() -> addMessage(text)));
            link.start();
        }
        if (video == null) {
            video = new VideoReceiver();
            video.setSurface(surface);
            video.start();
        }
    }

    private void stopLinks() {
        if (link != null) {
            link.stop();
            link = null;
        }
        if (video != null) {
            video.stop();
            video = null;
        }
    }

    private void addMessage(String text) {
        messages.addLast(text);
        while (messages.size() > 8) {
            messages.removeFirst();
        }
        StringBuilder sb = new StringBuilder();
        for (String m : messages) {
            if (sb.length() > 0) {
                sb.append('\n');
            }
            sb.append(m);
        }
        tvMessages.setText(sb.toString());
    }

    private final Runnable tick = new Runnable() {
        @Override
        public void run() {
            updateScreen();
            if (running) {
                ui.postDelayed(this, 300);
            }
        }
    };

    private void updateScreen() {
        if (wifiNetwork == null) {
            findWifi();
        }
        long now = System.currentTimeMillis();
        MavlinkLink l = link;
        MavlinkLink.Telemetry t = l != null ? l.t : null;

        boolean droneOk = t != null && now - t.lastHeartbeatMs < 3000;
        boolean controllerOk = t != null && now - t.lastPacketMs < 3000;

        if (wifiNetwork == null) {
            setStatus("Not on SoloLink WiFi", "#FF5555");
            tvHint.setText("Turn on the controller and the Solo.\nThen tap WiFi (top right) and join SoloLink_xxxx.\nIf the phone says the WiFi has no internet, choose to stay connected.");
        } else if (!controllerOk) {
            setStatus("WiFi OK - no data yet", "#FFAA33");
            tvHint.setText("Connected to WiFi, waiting for the controller...\nTelemetry packets received: " + (t != null ? t.packetCount : 0));
        } else if (!droneOk) {
            setStatus("Controller OK - no Solo", "#FFAA33");
            tvHint.setText("Controller found. Waiting for the Solo to link up...\n(Make sure the Solo's battery is on.)");
        } else if (t.armed) {
            setStatus("Solo - ARMED", "#FF3333");
        } else {
            setStatus("Solo connected", "#55FF55");
        }

        VideoReceiver v = video;
        boolean live = v != null && v.isLive();
        tvVideo.setText(v != null ? v.getStatus() : "Video --");
        if (droneOk) {
            tvHint.setText(live ? "" : "No video yet - a GoPro must be in the gimbal and turned on.");
        }
        tvHint.setVisibility(live ? View.GONE : View.VISIBLE);

        if (!droneOk) {
            tvMode.setText("Mode --");
            tvBattery.setText("Batt --");
            tvGps.setText("GPS --");
            tvAlt.setText("Alt --");
            tvSpeed.setText("Speed --");
            return;
        }

        tvMode.setText(t.customMode >= 0 ? MavlinkLink.copterMode(t.customMode) : "Mode --");

        if (t.battVolts > 0) {
            String pct = t.battPct >= 0 ? t.battPct + "%" : "--%";
            tvBattery.setText(String.format(Locale.US, "Batt %s  %.1fV", pct, t.battVolts));
            tvBattery.setTextColor(t.battPct >= 0 && t.battPct < 25 ? Color.parseColor("#FF5555") : Color.WHITE);
        } else {
            tvBattery.setText("Batt --");
        }

        if (t.gpsFix >= 0) {
            String sats = (t.sats >= 0 && t.sats < 255) ? t.sats + " sats" : "";
            tvGps.setText("GPS " + MavlinkLink.gpsFixName(t.gpsFix) + "  " + sats);
            tvGps.setTextColor(t.gpsFix >= 3 ? Color.WHITE : Color.parseColor("#FFAA33"));
        } else {
            tvGps.setText("GPS --");
        }

        tvAlt.setText(t.hasPos ? String.format(Locale.US, "Alt %.0f ft", t.relAltM * M_TO_FT) : "Alt --");

        if (t.hasHud) {
            tvSpeed.setText(String.format(Locale.US, "Speed %.0f mph   Climb %.1f ft/s   Hdg %d°",
                    t.groundSpeed * MPS_TO_MPH, t.climb * M_TO_FT, t.heading));
        } else {
            tvSpeed.setText("Speed --");
        }
    }

    private void setStatus(String text, String color) {
        tvStatus.setText(text);
        tvStatus.setTextColor(Color.parseColor(color));
    }

    // --- video surface ---

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture st, int width, int height) {
        surface = new Surface(st);
        if (video != null) {
            video.setSurface(surface);
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture st, int width, int height) {
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture st) {
        if (video != null) {
            video.setSurface(null);
        }
        if (surface != null) {
            surface.release();
            surface = null;
        }
        return true;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture st) {
    }
}
