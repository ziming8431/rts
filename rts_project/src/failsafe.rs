// ============================================================================
// Fail-Safe Module (Advanced Feature)
// ============================================================================
// Implements fail-safe mechanisms for graceful degradation:
// - Monitors system health metrics
// - Triggers safe mode when thresholds are violated
// - Manages recovery from degraded states
// ============================================================================

use crate::config::*;
use std::time::{Duration, Instant};

// ----------------------------------------------------------------------------
// Fail-Safe State Machine
// ----------------------------------------------------------------------------

/// States of the fail-safe system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailSafeState {
    /// Normal operation
    Normal,
    /// Warning state - monitoring closely
    Warning,
    /// Degraded operation - reduced functionality
    Degraded,
    /// Critical - minimal safe operation
    Critical,
    /// Recovery - attempting to return to normal
    Recovery,
}

/// Reasons for entering fail-safe mode
#[derive(Debug, Clone)]
pub enum FailSafeReason {
    /// Too many missed deadlines
    MissedDeadlines(usize),
    /// Too many sensor anomalies
    SensorAnomalies(usize),
    /// Communication failure
    CommunicationFailure,
    /// Actuator error
    ActuatorError(String),
    /// Manual trigger
    ManualTrigger,
    /// System overload
    SystemOverload,
}

// ----------------------------------------------------------------------------
// Fail-Safe Manager
// ----------------------------------------------------------------------------

/// Manages system fail-safe behavior
pub struct FailSafeManager {
    /// Current fail-safe state
    state: FailSafeState,
    /// Reason for current state (if not normal)
    reason: Option<FailSafeReason>,
    /// Consecutive missed deadlines counter
    missed_deadline_count: usize,
    /// Consecutive anomaly counter
    anomaly_count: usize,
    /// Time when fail-safe was entered
    failsafe_entered_at: Option<Instant>,
    /// Minimum time to stay in fail-safe before recovery
    min_failsafe_duration: Duration,
    /// Time when recovery started
    recovery_started_at: Option<Instant>,
    /// Recovery duration requirement
    recovery_duration: Duration,
    /// Thresholds
    missed_deadline_threshold: usize,
    anomaly_threshold: usize,
    /// History of state transitions
    state_history: Vec<(Instant, FailSafeState, Option<FailSafeReason>)>,
}

impl FailSafeManager {
    /// Create a new fail-safe manager
    pub fn new() -> Self {
        Self {
            state: FailSafeState::Normal,
            reason: None,
            missed_deadline_count: 0,
            anomaly_count: 0,
            failsafe_entered_at: None,
            min_failsafe_duration: Duration::from_secs(2),
            recovery_started_at: None,
            recovery_duration: Duration::from_secs(1),
            missed_deadline_threshold: FAILSAFE_MISSED_DEADLINE_THRESHOLD,
            anomaly_threshold: FAILSAFE_ANOMALY_THRESHOLD,
            state_history: Vec::new(),
        }
    }

    /// Get current state
    pub fn get_state(&self) -> FailSafeState {
        self.state
    }

    /// Check if system is in fail-safe mode
    pub fn is_failsafe_active(&self) -> bool {
        matches!(
            self.state,
            FailSafeState::Degraded | FailSafeState::Critical
        )
    }

    /// Check if system is operating normally
    pub fn is_normal(&self) -> bool {
        self.state == FailSafeState::Normal
    }

    /// Report a missed deadline
    pub fn report_missed_deadline(&mut self) -> FailSafeState {
        self.missed_deadline_count += 1;
        self.check_thresholds()
    }

    /// Report a successful deadline
    pub fn report_deadline_met(&mut self) {
        // Gradually decrease counter on success
        if self.missed_deadline_count > 0 {
            self.missed_deadline_count -= 1;
        }
        
        // Check if we can start recovery
        if self.state == FailSafeState::Degraded && self.missed_deadline_count == 0 {
            self.start_recovery();
        }
    }

    /// Report an anomaly detected
    pub fn report_anomaly(&mut self) -> FailSafeState {
        self.anomaly_count += 1;
        self.check_thresholds()
    }

    /// Report normal sensor reading
    pub fn report_normal_reading(&mut self) {
        if self.anomaly_count > 0 {
            self.anomaly_count -= 1;
        }
    }

    /// Check thresholds and update state
    fn check_thresholds(&mut self) -> FailSafeState {
        let previous_state = self.state;

        // Check for critical conditions
        if self.missed_deadline_count >= self.missed_deadline_threshold * 2 {
            self.enter_state(
                FailSafeState::Critical,
                Some(FailSafeReason::MissedDeadlines(self.missed_deadline_count)),
            );
        } else if self.anomaly_count >= self.anomaly_threshold * 2 {
            self.enter_state(
                FailSafeState::Critical,
                Some(FailSafeReason::SensorAnomalies(self.anomaly_count)),
            );
        }
        // Check for degraded conditions
        else if self.missed_deadline_count >= self.missed_deadline_threshold {
            self.enter_state(
                FailSafeState::Degraded,
                Some(FailSafeReason::MissedDeadlines(self.missed_deadline_count)),
            );
        } else if self.anomaly_count >= self.anomaly_threshold {
            self.enter_state(
                FailSafeState::Degraded,
                Some(FailSafeReason::SensorAnomalies(self.anomaly_count)),
            );
        }
        // Check for warning conditions
        else if self.missed_deadline_count >= self.missed_deadline_threshold / 2
            || self.anomaly_count >= self.anomaly_threshold / 2
        {
            if self.state == FailSafeState::Normal {
                self.enter_state(FailSafeState::Warning, None);
            }
        }

        if previous_state != self.state {
            self.record_transition();
        }

        self.state
    }

    /// Enter a new fail-safe state
    fn enter_state(&mut self, new_state: FailSafeState, reason: Option<FailSafeReason>) {
        if new_state != self.state {
            self.state = new_state;
            self.reason = reason;
            
            if matches!(new_state, FailSafeState::Degraded | FailSafeState::Critical) {
                self.failsafe_entered_at = Some(Instant::now());
                self.recovery_started_at = None;
            }
        }
    }

    /// Start recovery process
    fn start_recovery(&mut self) {
        if self.recovery_started_at.is_none() {
            // Check if we've been in fail-safe long enough
            if let Some(entered) = self.failsafe_entered_at {
                if entered.elapsed() >= self.min_failsafe_duration {
                    self.state = FailSafeState::Recovery;
                    self.recovery_started_at = Some(Instant::now());
                    self.record_transition();
                }
            }
        }
    }

    /// Update recovery progress
    pub fn update_recovery(&mut self) -> bool {
        if self.state != FailSafeState::Recovery {
            return false;
        }

        if let Some(started) = self.recovery_started_at {
            if started.elapsed() >= self.recovery_duration {
                // Recovery complete
                self.state = FailSafeState::Normal;
                self.reason = None;
                self.failsafe_entered_at = None;
                self.recovery_started_at = None;
                self.missed_deadline_count = 0;
                self.anomaly_count = 0;
                self.record_transition();
                return true;
            }
        }

        false
    }

    /// Manually trigger fail-safe mode
    pub fn trigger_failsafe(&mut self, reason: FailSafeReason) {
        self.enter_state(FailSafeState::Critical, Some(reason));
        self.record_transition();
    }

    /// Force exit from fail-safe (manual override)
    pub fn force_exit_failsafe(&mut self) {
        self.state = FailSafeState::Normal;
        self.reason = None;
        self.failsafe_entered_at = None;
        self.recovery_started_at = None;
        self.missed_deadline_count = 0;
        self.anomaly_count = 0;
        self.record_transition();
    }

    /// Record state transition
    fn record_transition(&mut self) {
        self.state_history
            .push((Instant::now(), self.state, self.reason.clone()));
        
        // Keep only last 100 transitions
        if self.state_history.len() > 100 {
            self.state_history.remove(0);
        }
    }

    /// Get safe actuator output value for fail-safe mode
    pub fn get_safe_output(&self) -> f64 {
        match self.state {
            FailSafeState::Normal | FailSafeState::Warning => FAILSAFE_ACTUATOR_VALUE,
            FailSafeState::Degraded => FAILSAFE_ACTUATOR_VALUE * 0.5,
            FailSafeState::Critical => FAILSAFE_ACTUATOR_VALUE,
            FailSafeState::Recovery => FAILSAFE_ACTUATOR_VALUE * 0.25,
        }
    }

    /// Get output scaling factor for current state
    pub fn get_output_scale(&self) -> f64 {
        match self.state {
            FailSafeState::Normal => 1.0,
            FailSafeState::Warning => 0.9,
            FailSafeState::Degraded => 0.5,
            FailSafeState::Critical => 0.1,
            FailSafeState::Recovery => 0.7,
        }
    }

    /// Get current reason for fail-safe
    pub fn get_reason(&self) -> Option<&FailSafeReason> {
        self.reason.as_ref()
    }

    /// Get state history
    pub fn get_history(&self) -> &[(Instant, FailSafeState, Option<FailSafeReason>)] {
        &self.state_history
    }

    /// Print fail-safe status
    pub fn print_status(&self) {
        println!("\n=== Fail-Safe Status ===");
        println!("  Current State:   {:?}", self.state);
        println!("  Missed Deadlines: {} / {}", 
            self.missed_deadline_count, self.missed_deadline_threshold);
        println!("  Anomaly Count:   {} / {}", 
            self.anomaly_count, self.anomaly_threshold);
        
        if let Some(reason) = &self.reason {
            println!("  Reason:          {:?}", reason);
        }
        
        if let Some(entered) = self.failsafe_entered_at {
            println!("  Time in Fail-Safe: {:.2}s", entered.elapsed().as_secs_f64());
        }
        
        println!("  State History Count: {}", self.state_history.len());
    }
}

impl Default for FailSafeManager {
    fn default() -> Self {
        Self::new()
    }
}

// ----------------------------------------------------------------------------
// Health Monitor
// ----------------------------------------------------------------------------

/// Monitors overall system health metrics
pub struct HealthMonitor {
    /// CPU usage estimate (0.0 to 1.0)
    cpu_usage: f64,
    /// Memory usage estimate (0.0 to 1.0)
    memory_usage: f64,
    /// Message queue depth
    queue_depth: usize,
    /// Maximum acceptable queue depth
    max_queue_depth: usize,
    /// Recent latencies for averaging
    recent_latencies: Vec<u64>,
    /// Maximum latency window
    latency_window: usize,
    /// Health score (0.0 to 1.0)
    health_score: f64,
}

impl HealthMonitor {
    /// Create a new health monitor
    pub fn new() -> Self {
        Self {
            cpu_usage: 0.0,
            memory_usage: 0.0,
            queue_depth: 0,
            max_queue_depth: 50,
            recent_latencies: Vec::with_capacity(100),
            latency_window: 100,
            health_score: 1.0,
        }
    }

    /// Update CPU usage estimate
    pub fn update_cpu_usage(&mut self, usage: f64) {
        self.cpu_usage = usage.clamp(0.0, 1.0);
        self.recalculate_health();
    }

    /// Update queue depth
    pub fn update_queue_depth(&mut self, depth: usize) {
        self.queue_depth = depth;
        self.recalculate_health();
    }

    /// Record a latency measurement
    pub fn record_latency(&mut self, latency_ns: u64) {
        self.recent_latencies.push(latency_ns);
        if self.recent_latencies.len() > self.latency_window {
            self.recent_latencies.remove(0);
        }
        self.recalculate_health();
    }

    /// Recalculate overall health score
    fn recalculate_health(&mut self) {
        let mut score = 1.0;

        // CPU factor
        if self.cpu_usage > 0.8 {
            score -= (self.cpu_usage - 0.8) * 2.0;
        }

        // Queue depth factor
        let queue_ratio = self.queue_depth as f64 / self.max_queue_depth as f64;
        if queue_ratio > 0.5 {
            score -= (queue_ratio - 0.5) * 0.5;
        }

        // Latency factor
        if !self.recent_latencies.is_empty() {
            let avg_latency: f64 = self.recent_latencies.iter().sum::<u64>() as f64
                / self.recent_latencies.len() as f64;
            let latency_ms = avg_latency / 1_000_000.0;
            if latency_ms > 1.0 {
                score -= (latency_ms - 1.0).min(0.5) * 0.2;
            }
        }

        self.health_score = score.clamp(0.0, 1.0);
    }

    /// Get current health score
    pub fn get_health_score(&self) -> f64 {
        self.health_score
    }

    /// Check if system is healthy
    pub fn is_healthy(&self) -> bool {
        self.health_score > 0.7
    }

    /// Get average latency
    pub fn get_average_latency_ns(&self) -> f64 {
        if self.recent_latencies.is_empty() {
            0.0
        } else {
            self.recent_latencies.iter().sum::<u64>() as f64
                / self.recent_latencies.len() as f64
        }
    }

    /// Print health status
    pub fn print_status(&self) {
        println!("\n=== Health Monitor Status ===");
        println!("  Health Score:    {:.2}", self.health_score);
        println!("  CPU Usage:       {:.1}%", self.cpu_usage * 100.0);
        println!("  Queue Depth:     {} / {}", self.queue_depth, self.max_queue_depth);
        println!("  Avg Latency:     {:.3} µs", self.get_average_latency_ns() / 1000.0);
        println!("  Status:          {}", if self.is_healthy() { "HEALTHY" } else { "DEGRADED" });
    }
}

impl Default for HealthMonitor {
    fn default() -> Self {
        Self::new()
    }
}
