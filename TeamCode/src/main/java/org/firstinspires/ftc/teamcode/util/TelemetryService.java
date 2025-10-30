package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;

/**
 * Centralises telemetry output to FTControl Panels, the driver station, and optional PsiKit logging.
 * OpModes create a single instance and share its {@link TelemetryPublisher} across subsystems.
 */
public class TelemetryService {

    private final boolean enablePsiKitLogging;
    private final PsiKitAdapter psiKitLogger;
    private final TelemetryPublisher publisher;

    private TelemetryManager panelsTelemetry;
    private boolean sessionActive = false;

    public TelemetryService(boolean enablePsiKitLogging) {
        this.enablePsiKitLogging = enablePsiKitLogging;
        this.psiKitLogger = enablePsiKitLogging ? new PsiKitAdapter() : null;
        this.publisher = new TelemetryPublisher(null, psiKitLogger);
    }

    public TelemetryService() {
        this(false);
    }

    /**
     * Prepares Panels and (optionally) starts PsiKit logging. Call once when the OpMode starts.
     */
    public void startSession() {
        if (!sessionActive) {
            panelsTelemetry = PanelsBridge.preparePanels();
            publisher.setTelemetryManager(panelsTelemetry);
            if (enablePsiKitLogging && psiKitLogger != null) {
                psiKitLogger.startSession();
            }
            sessionActive = true;
        }
    }

    /**
     * Stops any active logging session and flushes resources.
     */
    public void stopSession() {
        if (sessionActive && psiKitLogger != null) {
            psiKitLogger.stopSession();
        }
        publisher.setTelemetryManager(null);
        panelsTelemetry = null;
        sessionActive = false;
    }

    public TelemetryPublisher publisher() {
        return publisher;
    }

    public TelemetryManager panelsTelemetry() {
        return panelsTelemetry;
    }

    public void updateDriverStation(Telemetry telemetry) {
        if (panelsTelemetry != null) {
            panelsTelemetry.update(telemetry);
        }
    }

    /**
     * Provides access to the optional PsiKit adapter so that background loggers can stream
     * metrics without blocking the OpMode loop. This will be {@code null} when PsiKit logging
     * is disabled via {@link TelemetrySettings#enablePsiKitLogging}.
     */
    public PsiKitAdapter psiKitLogger() {
        return psiKitLogger;
    }
}
