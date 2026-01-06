// ============================================================================
// IPC Module - Inter-Process Communication
// ============================================================================
// Implements efficient communication channels between sensor and actuator
// modules. Uses crossbeam channels for high-performance message passing.
// ============================================================================

use crate::config::*;
use crate::types::*;
use crossbeam_channel::{bounded, unbounded, Receiver, Sender, TryRecvError};
use std::sync::Arc;
use std::time::{Duration, Instant};

// ----------------------------------------------------------------------------
// Channel Types
// ----------------------------------------------------------------------------

/// Channel for sending processed sensor data from sensor to actuator module
pub struct SensorDataChannel {
    sender: Sender<ProcessedSensorData>,
    receiver: Receiver<ProcessedSensorData>,
}

impl SensorDataChannel {
    /// Create a new bounded channel with specified capacity
    pub fn new(capacity: usize) -> Self {
        let (sender, receiver) = bounded(capacity);
        Self { sender, receiver }
    }

    /// Create an unbounded channel (unlimited capacity)
    pub fn unbounded() -> Self {
        let (sender, receiver) = unbounded();
        Self { sender, receiver }
    }

    /// Get a clone of the sender
    pub fn get_sender(&self) -> SensorDataSender {
        SensorDataSender {
            sender: self.sender.clone(),
        }
    }

    /// Get a clone of the receiver
    pub fn get_receiver(&self) -> SensorDataReceiver {
        SensorDataReceiver {
            receiver: self.receiver.clone(),
        }
    }
}

/// Sender end of the sensor data channel
#[derive(Clone)]
pub struct SensorDataSender {
    sender: Sender<ProcessedSensorData>,
}

impl SensorDataSender {
    /// Send sensor data (blocking if channel is full)
    pub fn send(&self, data: ProcessedSensorData) -> Result<(), String> {
        self.sender
            .send(data)
            .map_err(|e| format!("Failed to send sensor data: {}", e))
    }

    /// Try to send without blocking
    pub fn try_send(&self, data: ProcessedSensorData) -> Result<(), String> {
        self.sender
            .try_send(data)
            .map_err(|e| format!("Failed to send sensor data: {}", e))
    }

    /// Send with timeout
    pub fn send_timeout(&self, data: ProcessedSensorData, timeout: Duration) -> Result<(), String> {
        self.sender
            .send_timeout(data, timeout)
            .map_err(|e| format!("Send timeout: {}", e))
    }
}

/// Receiver end of the sensor data channel
#[derive(Clone)]
pub struct SensorDataReceiver {
    receiver: Receiver<ProcessedSensorData>,
}

impl SensorDataReceiver {
    /// Receive sensor data (blocking)
    pub fn recv(&self) -> Result<ProcessedSensorData, String> {
        self.receiver
            .recv()
            .map_err(|e| format!("Failed to receive sensor data: {}", e))
    }

    /// Try to receive without blocking
    pub fn try_recv(&self) -> Result<ProcessedSensorData, TryRecvError> {
        self.receiver.try_recv()
    }

    /// Receive with timeout
    pub fn recv_timeout(&self, timeout: Duration) -> Result<ProcessedSensorData, String> {
        self.receiver
            .recv_timeout(timeout)
            .map_err(|e| format!("Receive timeout: {}", e))
    }

    /// Check if there are pending messages
    pub fn is_empty(&self) -> bool {
        self.receiver.is_empty()
    }

    /// Get number of pending messages
    pub fn len(&self) -> usize {
        self.receiver.len()
    }
}

// ----------------------------------------------------------------------------
// Feedback Channel (Actuator to Sensor)
// ----------------------------------------------------------------------------

/// Channel for sending feedback from actuator back to sensor module
pub struct FeedbackChannel {
    sender: Sender<ActuatorFeedback>,
    receiver: Receiver<ActuatorFeedback>,
}

impl FeedbackChannel {
    /// Create a new bounded feedback channel
    pub fn new(capacity: usize) -> Self {
        let (sender, receiver) = bounded(capacity);
        Self { sender, receiver }
    }

    /// Get a clone of the sender
    pub fn get_sender(&self) -> FeedbackSender {
        FeedbackSender {
            sender: self.sender.clone(),
        }
    }

    /// Get a clone of the receiver
    pub fn get_receiver(&self) -> FeedbackReceiver {
        FeedbackReceiver {
            receiver: self.receiver.clone(),
        }
    }
}

/// Sender end of feedback channel
#[derive(Clone)]
pub struct FeedbackSender {
    sender: Sender<ActuatorFeedback>,
}

impl FeedbackSender {
    /// Send feedback (blocking)
    pub fn send(&self, feedback: ActuatorFeedback) -> Result<(), String> {
        self.sender
            .send(feedback)
            .map_err(|e| format!("Failed to send feedback: {}", e))
    }

    /// Try to send without blocking
    pub fn try_send(&self, feedback: ActuatorFeedback) -> Result<(), String> {
        self.sender
            .try_send(feedback)
            .map_err(|e| format!("Failed to send feedback: {}", e))
    }

    /// Send with timeout (for real-time constraints)
    pub fn send_timeout(&self, feedback: ActuatorFeedback, timeout: Duration) -> Result<(), String> {
        self.sender
            .send_timeout(feedback, timeout)
            .map_err(|e| format!("Feedback send timeout: {}", e))
    }
}

/// Receiver end of feedback channel
#[derive(Clone)]
pub struct FeedbackReceiver {
    receiver: Receiver<ActuatorFeedback>,
}

impl FeedbackReceiver {
    /// Receive feedback (blocking)
    pub fn recv(&self) -> Result<ActuatorFeedback, String> {
        self.receiver
            .recv()
            .map_err(|e| format!("Failed to receive feedback: {}", e))
    }

    /// Try to receive without blocking
    pub fn try_recv(&self) -> Result<ActuatorFeedback, TryRecvError> {
        self.receiver.try_recv()
    }

    /// Receive with timeout
    pub fn recv_timeout(&self, timeout: Duration) -> Result<ActuatorFeedback, String> {
        self.receiver
            .recv_timeout(timeout)
            .map_err(|e| format!("Feedback receive timeout: {}", e))
    }

    /// Check if there are pending messages
    pub fn is_empty(&self) -> bool {
        self.receiver.is_empty()
    }
}

// ----------------------------------------------------------------------------
// Command Channel (for control commands)
// ----------------------------------------------------------------------------

/// Commands that can be sent to control the system
#[derive(Debug, Clone)]
pub enum ControlCommand {
    /// Start the system
    Start,
    /// Stop the system gracefully
    Stop,
    /// Enter fail-safe mode
    EnterFailSafe,
    /// Exit fail-safe mode
    ExitFailSafe,
    /// Update configuration
    UpdateConfig(String, f64),
    /// Request status report
    RequestStatus,
    /// Emergency shutdown
    EmergencyStop,
}

/// Channel for sending control commands
pub struct CommandChannel {
    sender: Sender<ControlCommand>,
    receiver: Receiver<ControlCommand>,
}

impl CommandChannel {
    pub fn new() -> Self {
        let (sender, receiver) = bounded(10);
        Self { sender, receiver }
    }

    pub fn get_sender(&self) -> CommandSender {
        CommandSender {
            sender: self.sender.clone(),
        }
    }

    pub fn get_receiver(&self) -> CommandReceiver {
        CommandReceiver {
            receiver: self.receiver.clone(),
        }
    }
}

impl Default for CommandChannel {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone)]
pub struct CommandSender {
    sender: Sender<ControlCommand>,
}

impl CommandSender {
    pub fn send(&self, cmd: ControlCommand) -> Result<(), String> {
        self.sender
            .send(cmd)
            .map_err(|e| format!("Failed to send command: {}", e))
    }
}

#[derive(Clone)]
pub struct CommandReceiver {
    receiver: Receiver<ControlCommand>,
}

impl CommandReceiver {
    pub fn try_recv(&self) -> Option<ControlCommand> {
        self.receiver.try_recv().ok()
    }
}

// ----------------------------------------------------------------------------
// IPC Manager - Coordinates all channels
// ----------------------------------------------------------------------------

/// Central manager for all IPC channels
pub struct IpcManager {
    /// Sensor data channel
    pub sensor_channel: SensorDataChannel,
    /// Feedback channel
    pub feedback_channel: FeedbackChannel,
    /// Command channel
    pub command_channel: CommandChannel,
    /// Transmission timing statistics
    pub transmission_times: Arc<parking_lot::Mutex<Vec<u64>>>,
}

impl IpcManager {
    /// Create a new IPC manager with all channels
    pub fn new() -> Self {
        Self {
            sensor_channel: SensorDataChannel::new(CHANNEL_BUFFER_SIZE),
            feedback_channel: FeedbackChannel::new(CHANNEL_BUFFER_SIZE),
            command_channel: CommandChannel::new(),
            transmission_times: Arc::new(parking_lot::Mutex::new(Vec::new())),
        }
    }

    /// Get sensor data sender
    pub fn get_sensor_sender(&self) -> SensorDataSender {
        self.sensor_channel.get_sender()
    }

    /// Get sensor data receiver
    pub fn get_sensor_receiver(&self) -> SensorDataReceiver {
        self.sensor_channel.get_receiver()
    }

    /// Get feedback sender
    pub fn get_feedback_sender(&self) -> FeedbackSender {
        self.feedback_channel.get_sender()
    }

    /// Get feedback receiver
    pub fn get_feedback_receiver(&self) -> FeedbackReceiver {
        self.feedback_channel.get_receiver()
    }

    /// Get command sender
    pub fn get_command_sender(&self) -> CommandSender {
        self.command_channel.get_sender()
    }

    /// Get command receiver
    pub fn get_command_receiver(&self) -> CommandReceiver {
        self.command_channel.get_receiver()
    }

    /// Record a transmission time
    pub fn record_transmission_time(&self, time_ns: u64) {
        self.transmission_times.lock().push(time_ns);
    }

    /// Get transmission statistics
    pub fn get_transmission_stats(&self) -> PerformanceStats {
        let times = self.transmission_times.lock();
        let deadline_ns = TRANSMISSION_DEADLINE.as_nanos() as u64;
        let missed = times.iter().filter(|&&t| t > deadline_ns).count();
        let total_time = times.iter().sum::<u64>() as f64 / 1_000_000_000.0;
        PerformanceStats::from_measurements(&times, missed, total_time)
    }
}

impl Default for IpcManager {
    fn default() -> Self {
        Self::new()
    }
}

// ----------------------------------------------------------------------------
// Timed Send/Receive Wrappers
// ----------------------------------------------------------------------------

/// Wrapper that times how long a send operation takes
pub fn timed_send<T>(
    sender: &Sender<T>,
    data: T,
    deadline: Duration,
) -> (Result<(), String>, u64, bool)
where
    T: Send,
{
    let start = Instant::now();
    let result = sender.send(data);
    let elapsed = start.elapsed();
    let elapsed_ns = elapsed.as_nanos() as u64;
    let met_deadline = elapsed <= deadline;

    (
        result.map_err(|e| format!("Send failed: {}", e)),
        elapsed_ns,
        met_deadline,
    )
}

/// Wrapper that times how long a receive operation takes
pub fn timed_recv<T>(receiver: &Receiver<T>, timeout: Duration) -> (Result<T, String>, u64)
where
    T: Send,
{
    let start = Instant::now();
    let result = receiver.recv_timeout(timeout);
    let elapsed = start.elapsed();
    let elapsed_ns = elapsed.as_nanos() as u64;

    (
        result.map_err(|e| format!("Receive failed: {}", e)),
        elapsed_ns,
    )
}
