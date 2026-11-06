package com.milton.solopilot;

import android.media.MediaCodec;
import android.media.MediaFormat;
import android.view.Surface;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;

/**
 * Live camera video from the Solo.
 * Opening a TCP connection to the controller on port 5502 asks it to send video.
 * The video then arrives as H.264 in RTP packets on UDP port 5600.
 */
public class VideoReceiver {

    private static final String HOST = "10.1.1.1";
    private static final int TCP_PORT = 5502;
    private static final int UDP_PORT = 5600;
    private static final int MAX_FRAME = 2 * 1024 * 1024;

    private volatile boolean running;
    private volatile Socket tcp;
    private DatagramSocket udp;

    private final Object lock = new Object();
    private Surface surface;
    private MediaCodec decoder;
    private final MediaCodec.BufferInfo info = new MediaCodec.BufferInfo();
    private long pts;

    private byte[] sps;
    private byte[] pps;
    private final ByteArrayOutputStream fu = new ByteArrayOutputStream();
    private boolean fuActive;
    private final ByteArrayOutputStream au = new ByteArrayOutputStream();

    private volatile String status = "Video: waiting";
    private volatile long lastFrameMs;

    public String getStatus() {
        if (lastFrameMs > 0 && System.currentTimeMillis() - lastFrameMs < 2000) {
            return "Video: live";
        }
        return status;
    }

    public boolean isLive() {
        return lastFrameMs > 0 && System.currentTimeMillis() - lastFrameMs < 2000;
    }

    public void setSurface(Surface s) {
        synchronized (lock) {
            if (s == surface) {
                return;
            }
            releaseDecoder();
            surface = s;
        }
    }

    public void start() {
        running = true;
        new Thread(this::tcpLoop, "video-tcp").start();
        new Thread(this::udpLoop, "video-udp").start();
    }

    public void stop() {
        running = false;
        Socket s = tcp;
        if (s != null) {
            try {
                s.close();
            } catch (Exception ignored) {
            }
        }
        if (udp != null) {
            udp.close();
        }
        synchronized (lock) {
            releaseDecoder();
        }
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ignored) {
        }
    }

    /** Keeps the "please send video" connection open, reconnecting if it drops. */
    private void tcpLoop() {
        while (running) {
            Socket s = new Socket();
            try {
                s.connect(new InetSocketAddress(HOST, TCP_PORT), 3000);
                tcp = s;
                if (lastFrameMs == 0) {
                    status = "Video: asked controller, waiting for picture";
                }
                InputStream in = s.getInputStream();
                byte[] b = new byte[256];
                while (running && in.read(b) >= 0) {
                    // the controller doesn't need us to say anything; just stay connected
                }
            } catch (Exception e) {
                if (running && lastFrameMs == 0) {
                    status = "Video: controller not answering";
                }
            } finally {
                try {
                    s.close();
                } catch (Exception ignored) {
                }
                tcp = null;
            }
            if (running) {
                sleep(2000);
            }
        }
    }

    private void udpLoop() {
        try {
            udp = new DatagramSocket(null);
            udp.setReuseAddress(true);
            udp.setReceiveBufferSize(1024 * 1024);
            udp.bind(new InetSocketAddress(UDP_PORT));
            udp.setSoTimeout(1000);
        } catch (Exception e) {
            status = "Video: can't open port " + UDP_PORT;
            return;
        }
        byte[] buf = new byte[65536];
        DatagramPacket pkt = new DatagramPacket(buf, buf.length);
        while (running) {
            try {
                pkt.setLength(buf.length);
                udp.receive(pkt);
                handleRtp(buf, pkt.getLength());
            } catch (SocketTimeoutException e) {
                // keep waiting
            } catch (Exception e) {
                if (!running) {
                    break;
                }
                sleep(100);
            }
        }
    }

    private void handleRtp(byte[] d, int len) {
        if (len < 13) {
            return;
        }
        int b0 = d[0] & 0xFF;
        if ((b0 >> 6) != 2) {
            return; // not RTP version 2
        }
        int cc = b0 & 0x0F;
        boolean ext = (b0 & 0x10) != 0;
        boolean marker = (d[1] & 0x80) != 0;
        int off = 12 + cc * 4;
        if (ext) {
            if (off + 4 > len) {
                return;
            }
            int extLen = ((d[off + 2] & 0xFF) << 8) | (d[off + 3] & 0xFF);
            off += 4 + extLen * 4;
        }
        int end = len;
        if ((b0 & 0x20) != 0) {
            end -= d[len - 1] & 0xFF; // padding
        }
        if (off >= end) {
            return;
        }

        int nalType = d[off] & 0x1F;
        if (nalType >= 1 && nalType <= 23) {
            nal(d, off, end - off);
        } else if (nalType == 24) { // STAP-A: several small NAL units in one packet
            int p = off + 1;
            while (p + 2 <= end) {
                int size = ((d[p] & 0xFF) << 8) | (d[p + 1] & 0xFF);
                p += 2;
                if (size == 0 || p + size > end) {
                    break;
                }
                nal(d, p, size);
                p += size;
            }
        } else if (nalType == 28) { // FU-A: one NAL unit split across packets
            if (end - off < 2) {
                return;
            }
            int fh = d[off + 1] & 0xFF;
            boolean first = (fh & 0x80) != 0;
            boolean last = (fh & 0x40) != 0;
            if (first) {
                fu.reset();
                fu.write((d[off] & 0xE0) | (fh & 0x1F));
                fuActive = true;
            }
            if (fuActive) {
                fu.write(d, off + 2, end - off - 2);
                if (last) {
                    byte[] n = fu.toByteArray();
                    fuActive = false;
                    nal(n, 0, n.length);
                }
            }
        }

        if (marker) {
            flushFrame();
        }
    }

    private void nal(byte[] d, int off, int n) {
        int type = d[off] & 0x1F;
        if (type == 7) {
            sps = copy(d, off, n);
        } else if (type == 8) {
            pps = copy(d, off, n);
        }
        if (au.size() > MAX_FRAME) {
            au.reset();
        }
        au.write(0);
        au.write(0);
        au.write(0);
        au.write(1);
        au.write(d, off, n);
    }

    private static byte[] copy(byte[] d, int off, int n) {
        byte[] out = new byte[n];
        System.arraycopy(d, off, out, 0, n);
        return out;
    }

    private static byte[] withStartCode(byte[] nal) {
        byte[] out = new byte[nal.length + 4];
        out[3] = 1;
        System.arraycopy(nal, 0, out, 4, nal.length);
        return out;
    }

    private void flushFrame() {
        if (au.size() == 0) {
            return;
        }
        byte[] frame = au.toByteArray();
        au.reset();

        synchronized (lock) {
            if (surface == null) {
                return;
            }
            if (decoder == null) {
                if (sps == null || pps == null) {
                    status = "Video: receiving, waiting for key frame";
                    return;
                }
                if (!createDecoder()) {
                    return;
                }
            }
            try {
                int idx = decoder.dequeueInputBuffer(10000);
                if (idx >= 0) {
                    ByteBuffer in = decoder.getInputBuffer(idx);
                    if (in != null && frame.length <= in.capacity()) {
                        in.clear();
                        in.put(frame);
                        decoder.queueInputBuffer(idx, 0, frame.length, pts, 0);
                    } else {
                        decoder.queueInputBuffer(idx, 0, 0, pts, 0);
                    }
                    pts += 33333;
                }
                while (true) {
                    int out = decoder.dequeueOutputBuffer(info, 0);
                    if (out >= 0) {
                        decoder.releaseOutputBuffer(out, true);
                        lastFrameMs = System.currentTimeMillis();
                    } else if (out == MediaCodec.INFO_TRY_AGAIN_LATER) {
                        break;
                    }
                }
            } catch (Exception e) {
                status = "Video: decoder hiccup, restarting";
                releaseDecoder();
            }
        }
    }

    private boolean createDecoder() {
        try {
            MediaFormat f = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, 1280, 720);
            f.setByteBuffer("csd-0", ByteBuffer.wrap(withStartCode(sps)));
            f.setByteBuffer("csd-1", ByteBuffer.wrap(withStartCode(pps)));
            decoder = MediaCodec.createDecoderByType(MediaFormat.MIMETYPE_VIDEO_AVC);
            decoder.configure(f, surface, null, 0);
            decoder.start();
            status = "Video: starting";
            return true;
        } catch (Exception e) {
            status = "Video: can't start decoder";
            releaseDecoder();
            return false;
        }
    }

    private void releaseDecoder() {
        if (decoder != null) {
            try {
                decoder.stop();
            } catch (Exception ignored) {
            }
            try {
                decoder.release();
            } catch (Exception ignored) {
            }
            decoder = null;
        }
    }
}
