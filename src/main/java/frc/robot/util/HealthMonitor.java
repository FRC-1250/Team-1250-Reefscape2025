package frc.robot.util;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

public class HealthMonitor {
    private static HealthMonitor instance;
    private final Map<String, Map<String, HealthStatus>> healthStatus = new ConcurrentHashMap<>();
    private final Map<String, Map<String, PhoenixHealthChecker>> subsystems = new ConcurrentHashMap<>();
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    private final long checkInterval;
    private boolean paused = true;

    private HealthMonitor(long checkInterval) {
        this.checkInterval = checkInterval;
    }

    public static synchronized HealthMonitor getInstance() {
        if (instance == null) {
            instance = new HealthMonitor(3); // Default 3 seconds interval
        }
        return instance;
    }

    public HealthMonitor addComponent(String subsystemName, String deviceName, TalonFX talonFX) {
        return addComponent(deviceName, new TalonHealthChecker(talonFX, subsystemName));
    }

    public HealthMonitor addComponent(String subsystemName, String deviceName, CANcoder cancoder) {
        return addComponent(deviceName, new CANcoderHealthChecker(cancoder, subsystemName));
    }

    public HealthMonitor addComponent(String subsystemName, String deviceName, Pigeon2 pigeon2) {
        return addComponent(deviceName, new PigeonHealthChecker(pigeon2, subsystemName));
    }

    private HealthMonitor addComponent(String deviceName, PhoenixHealthChecker e) {
        subsystems.computeIfAbsent(e.subsystemName, k -> new ConcurrentHashMap<>()).put(deviceName, e);
        healthStatus.computeIfAbsent(e.subsystemName, k -> new ConcurrentHashMap<>()).put(deviceName,
                HealthStatus.IS_OK);
        return this;
    }

    public void start() {
        scheduler.scheduleAtFixedRate(this::checkHealth, 0, checkInterval, TimeUnit.SECONDS);
    }

    public void pause() {
        paused = true;
    }

    public void unpause() {
        paused = false;
    }

    public void stop() {
        scheduler.shutdown();
        try {
            if (!scheduler.awaitTermination(5, TimeUnit.SECONDS)) {
                scheduler.shutdownNow();
            }
        } catch (InterruptedException e) {
            scheduler.shutdownNow();
        }
    }

    private void checkHealth() {
        if(!paused) {
            for (var subsystem : subsystems.entrySet()) {
                for (var component : subsystem.getValue().entrySet()) {
                    if (component.getValue().isDeviceHealthy()) {
                        healthStatus.get(subsystem.getKey()).put(component.getKey(), HealthStatus.IS_OK);
                    } else {
                        healthStatus.get(subsystem.getKey()).put(component.getKey(), HealthStatus.ERROR);
                    }
                }
            }
        }
    }

    public HealthStatus getSubsystemStatus(String subsystemName) {
        if (paused) {
            return HealthStatus.IS_OK;
        } else {
            if (healthStatus.containsKey(subsystemName)) {
                for (var component : healthStatus.get(subsystemName).values()) {
                    if (component == HealthStatus.ERROR)
                        return HealthStatus.ERROR;
                }
            }
            return HealthStatus.IS_OK;
        }
    }
}