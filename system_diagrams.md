# System Design Diagrams

Here are the diagrams for your report. You can take screenshots of these rendered diagrams.

## Figure 1: High-Level System Architecture

This diagram shows the decoupled Sensor and Actuator modules, the IPC channels, and the Shared Resource container.

```mermaid
graph TD
    subgraph "Component A: Sensor Module"
        S1[Force Sensor]
        S2[Position Sensor]
        S3[Temp Sensor]
        Agg[Data Aggregator]
        Filter[Moving Average Filter]
        Anomaly[Anomaly Detector]
    end

    subgraph "Component B: Actuator Module"
        PID_Bank[PID Controller Bank]
        Predict[Predictive Model]
        Act1[Gripper]
        Act2[Motor]
        Act3[Stabilizer]
    end

    subgraph "Shared Resources Container"
        Log[Diagnostic Log (Mutex)]
        Config[Config Buffer (RwLock)]
        Status[Status Memory (Atomic)]
    end

    %% Data Flow
    S1 & S2 & S3 --> Agg
    Agg --> Filter
    Filter --> Anomaly
    Anomaly == "Processed Data (Channel)" ==> PID_Bank
    
    PID_Bank <--> Predict
    PID_Bank --> Act1 & Act2 & Act3
    
    %% Feedback
    Act1 & Act2 & Act3 -.->|Feedback Channel| Agg

    %% Shared Resource Interaction
    Anomaly -.-> Log
    PID_Bank -.-> Config
    Filter -.-> Status
    Act1 -.-> Status
```

---

## Figure 2: Fail-Safe State Machine

This diagram illustrates the 5-state fail-safe logic managed by the supervisor.

```mermaid
stateDiagram-v2
    [*] --> Normal
    
    Normal --> Warning : Missed Deadlines > 3
    Normal --> Warning : Minor Anomalies
    
    Warning --> Normal : Stability Timer Expired (No Errors)
    Warning --> Degraded : Missed Deadlines > 5
    Warning --> Degraded : Persistent Anomalies
    
    Degraded --> Recovery : Errors Cleared
    Degraded --> Critical : Safety Stop Triggered
    Degraded --> Critical : Unexpected Exception
    
    Critical --> Recovery : Manual Reset / System Check
    
    Recovery --> Normal : Stability Confirmation (100 cycles)
    Recovery --> Critical : Recursion Error
    
    note right of Normal
        Full Performance
        Logging: Standard
    end note
    
    note right of Degraded
        Actuator Speed: 50%
        Logging: Verbose
    end note
    
    note right of Critical
        Actuators: HALTED
        System: Safe Mode
    end note
```

---

## Figure 3: Control Loop Sequence

This shows the strict timing sequence of a single 5ms cycle.

```mermaid
sequenceDiagram
    participant Timer as System Timer (5ms)
    participant Sensor as Sensor Module
    participant Channel as IPC Channel
    participant Actuator as Actuator Module
    participant Shared as Shared State

    Timer->>Sensor: Tick (Start Cycle)
    activate Sensor
    Sensor->>Sensor: Generate Data (0.13µs)
    Sensor->>Sensor: Filter & Anomaly Check (0.53µs)
    Sensor->>Shared: Update Status
    Sensor->>Channel: Send Processed Frame
    deactivate Sensor

    Channel->>Actuator: Receive Frame
    activate Actuator
    Actuator->>Actuator: Predictive PID Calc (1.01µs)
    Actuator->>Shared: Read PID Gains (RwLock)
    Actuator->>Shared: Log Actuation Event
    Actuator->>Sensor: Send Feedback (Ack)
    deactivate Actuator
    
    Note over Timer, Shared: Total Loop Latency < 2.59µs (Async)
```
