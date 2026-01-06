# Research Report: Design and Implementation of a Rust-Based Real-Time Control System for Automated Manufacturing

**Author:** [Student Name/ID]
**Date:** December 31, 2025
**Course:** Real-Time Systems (RTS2509)

---

## Abstract

This report presents the comprehensive design, implementation, and rigorous evaluation of a simulated real-time control system specifically engineered for an automated manufacturing environment. The system was developed using the Rust programming language to address the critical and often conflicting requirements of memory safety, high performance, and concurrency in safety-critical industrial applications. Traditional systems often rely on C or C++ which introduce risks related to memory management, or managed languages that suffer from unpredictable garbage collection latencies. This project proposes a novel architecture that leverages Rust's ownership model to guarantee thread safety and memory safety at compile time without the overhead of a runtime garbage collector. The developed system integrates a sophisticated multi-sensor monitoring module and a PID-controlled actuator module which verify the effectiveness of the design. These components communicate via a bounded Inter-Process Communication mechanism that ensures predictable message delivery latencies. Key contributions of this work include the implementation of a five-state fail-safe finite state machine, the application of predictive control algorithms to mitigate actuator lag, and a detailed comparative analysis of asynchronous versus operating system thread-based concurrency models. Extensive performance benchmarks conducted using the Criterion framework demonstrate that the asynchronous implementation achieves an eighty-seven percent reduction in average latency compared to the multi-threaded approach while adhering strictly to five millisecond deadline constraints under simulated central processing unit loads of up to sixty percent. The findings suggest that Rust is not only a viable candidate but a superior choice for next-generation real-time industrial control systems.

---

## 1. Introduction

### 1.1 Problem Statement
The domain of industrial automation is characterized by strict temporal constraints where the correctness of a system depends not only on the logical result of computation but also on the time at which the results are produced. Modern automated manufacturing lines utilize high-speed robotic arms and conveyor systems that require control loops to execute within millisecond-level deadlines. A failure to meets these deadlines can result in catastrophic consequences ranging from production defects and expensive equipment damage to severe safety hazards for human operators.

Reviewing the current landscape reveals significant challenges in the software engineering of these systems. The industry standard languages, primarily C and C++, offer the necessary low-level control and performance but are fraught with dangers related to manual memory management. Common vulnerabilities such as buffer overflows, dangling pointers, and data races are persistent issues that compromise system reliability and security. On the other hand, higher-level managed languages like Java or Python introduce a garbage collector that manages memory automatically. While this simplifies development, it introduces non-deterministic pauses in execution that are unacceptable for hard real-time constraints where predictability is paramount.

There is a distinct gap in the available technology stack for a language that provides the low-level control and zero-cost abstractions of C++ while guaranteeing the memory safety and type safety typically associated with managed languages. This project explores the application of Rust to fill this gap. Rust offers a unique ownership model that enforces memory safety rules at compile time, theoretically eliminating the need for a garbage collector while preventing data races in concurrent environments.

### 1.2 Objectives
The primary objective of this research is to design, implement, and evaluate a robust Real-Time System simulation using the Rust programming language. The goal is to demonstrate that it is possible to achieve hard real-time performance guarantees while maintaining a high level of code safety and maintainability.

The specific objectives that guide this work are detailed as follows.

First, the research aims to design a concurrent Sensor-Actuator architecture that decouples data acquisition from control logic. This involves creating a robust mechanism for shared resource synchronization that does not introduce significant locking contention or priority inversion issues.

Second, the project seeks to implement comprehensive fault tolerance mechanisms. Real-world industrial environments are noisy and prone to hardware glitches. Therefore, the system must include sophisticated anomaly detection algorithms and a fail-safe state machine that can gracefully degrade system performance rather than failing catastrophically when faults are detected.

Third, the research intends to implement advanced control strategies. A standard Proportional-Integral-Derivative controller will be developed and then extended with predictive capabilities to handle the inherent processing delays in the sensor-actuator loop.

Fourth, and perhaps most importantly, the project aims to conduct a rigorous evaluation of concurrency models. The study will empirically compare the performance of standard Operating System threading against modern asynchronous concurrency models powered by the Tokio runtime. This comparison provides critical insights into the optimal architectural choices for future real-time systems development.

---

## 2. Related Work

### 2.1 The Rise of Rust in Real-Time Systems
The emergence of Rust has sparked significant interest in the real-time systems community. Levy et al. (2017) demonstrated that Rust’s ownership model allows for the development of operating system kernels that are as efficient as those written in C but free from memory safety bugs. Their work highlights that the ownership type system allows developers to explicitly manage distinct memory regions without the overhead of runtime checks.

In the specific context of embedded systems, recent studies have shown that Rust’s zero-cost abstractions allow for high-level programming concepts, such as iterators and closures, to compile down to machine code that is identical to hand-optimized C. This is a crucial advantage for resource-constrained controllers used in manufacturing. Blandy and Orendorff (2018) emphasize that Rust utilizes compile-time static analysis to prevent data races. This is particularly relevant for this project, as data races are a notorious source of non-deterministic bugs in concurrent real-time software. By preventing these errors at compile time, Rust significantly reduces the verification and validation burden for safety-critical systems.

### 2.2 Real-Time Scheduling and Concurrency Models
The scheduling of tasks is the heartbeat of any real-time system. Traditional literature focuses heavily on algorithmic approaches such as Fixed-Priority Preemptive Scheduling (FPPS) and Earliest Deadline First (EDF). However, the implementation of these algorithms depends heavily on the underlying concurrency model provided by the language and operating system.

There are two primary models of concurrency relevant to this domain. The first is the one-to-one threading model, where each user-level thread maps directly to a kernel-level Operating System thread. This model offers predictable preemption and is well-understood, but it incurs a relatively high overhead for context switching. The Operating System must save and restore the entire register set and thread stack, which consumes valuable processor cycles.

The second model is the many-to-many or asynchronous model, often referred to as green threading or user-level tasks. In this model, a runtime scheduler manages many lightweight tasks onto a smaller number of OS threads. Abbott and Garcia-Molina (1992) explored the performance implications of scheduling real-time transactions and noted that minimizing blocking operations is key to throughput. Modern asynchronous runtimes like Tokio utilize cooperative multitasking, where tasks voluntarily yield control. This can theoretically offer much higher throughput and lower latency, but it requires careful design to ensuring that long-running computations do not block the executor and starve other critical tasks. This report contributes to this body of knowledge by providing empirical data comparing these two models in a simulated industrial context.

### 2.3 Fault Tolerance and Feedback Control
Feedback control loops are ubiquitous in automation. The Proportional-Integral-Derivative controller remains the industry standard due to its versatile performance and ease of tuning. However, the implementation of such controllers in Cyber-Physical Systems is complicated by network effects. Networked control systems must account for variable delays, packet loss, and jitter, all of which can destabilize a standard PID loop.

To address these reliability concerns, robust systems typically employ architectural patterns such as the Simplex architecture or state-machine-based supervisors. Sha (2001) argued for using simplicity to control complexity, advocating for a simple, verified safety core that can take over control when complex, high-performance components fail. This project adopts a similar philosophy by implementing a hierarchical fail-safe state machine. This supervisor monitors the health of the system—tracking metrics like deadline misses and sensor anomalies—and transitions the system through distinct states of operation, from Normal to Degraded and finally to Critical or Recovery modes. This ensures that the system maintains safety properties even when partial failures occur.

---

## 3. System Design

### 3.1 Architecture Overview
The system architecture was meticulously designed to simulate a realistic manufacturing cell. The core design philosophy relies on the decoupling of components to ensure that failures or delays in one module do not immediately propagate to others. The system consists of two primary active components, the Sensor Module and the Actuator Module, which run concurrently and communicate via bounded message channels.

[Insert Figure 1: High-Level System Architecture Diagram. The diagram should show the Sensor Module on the left and the Actuator Module on the right. Arrows indicate data flowing from Sensor to Actuator via an IPC Channel. Another set of arrows shows both modules interacting with a central block labeled "Shared State" which contains the Log, Status, and Config components. A final feedback loop arrow returns from Actuator to Sensor.]

The Sensor Module, designated as Component A, is responsible for the perception of the environment. It simulates three distinct types of physical sensors: Force sensors, Position sensors, and Temperature sensors. Each sensor type has unique noise characteristics and signal behaviors. The module performs data acquisition, cleanses the signal using a Moving Average filter to remove high-frequency noise, and performs anomaly detection using statistical Z-Score analysis. All of these operations are constrained within a strict two hundred microsecond processing budget.

The Actuator Module, designated as Component B, is responsible for acting upon the environment. It receives the processed data frames from the Sensor Module and computes the necessary control signals. It manages a bank of virtual actuators, including a high-speed Gripper, a precision Motor, and a Stabilizer. The control signals are derived using a logic bank of PID controllers which are tuned specifically for the dynamics of each actuator type.

Supporting these two active modules is a robust Shared Resources container. In a multi-threaded real-time system, managing shared state is a critical challenge. We implemented a container holding three synchronized structures. First is the DiagnosticLog, a ring buffer protected by a Mutex implementation that utilizes thread parking to minimize CPU usage during contention. Second is the StatusMemory which utilizes lock-free atomic integers. This allows high-speed status checks, such as verifying the current operating mode or incrementing cycle counters, without the risk of deadlock or thread sleeping. Third is the ConfigBuffer, which is protected by a Read-Write Lock. This design choice implies that system parameters such as PID gains can be read concurrently by multiple threads frequently, while updates happen infrequently, optimizing for the common read case.

### 3.2 Scheduling and Inter-Process Communication
The temporal behavior of the system determines its correctness. The manufacturing simulation operates on a fundamental hyperperiod of five milliseconds. This means that every five milliseconds, a new set of sensor readings is generated, processed, transmitted, and acted upon.

For Inter-Process Communication, the design utilizes `crossbeam-channel`. This library provides multi-producer multi-consumer channels that are optimized for high performance. We selected a bounded channel capacity to provide backpressure. If the Actuator Module falls behind, the channel fills up, and the Sensor Module is forced to wait. This backpressure mechanism prevents the accumulation of stale data, ensuring that the actuators are always operating on the most recent valid state of the system.

Deadline management is enforced through instrumentation. Every operational step is bracketed by high-resolution timers. If the processing of a sensor frame exceeds its allocated budget, for instance two hundred microseconds, a deadline miss is explicitly recorded in the shared status memory. This allows the system to self-diagnose performance issues in real time.

The system implements a sophisticated fail-safe state machine to manage health. A dedicated manager monitors the cumulative count of deadline misses and sensor anomalies. It transitions the system through a directed graph of five states. The Normal state represents full nominal operation. If the error count exceeds a threshold, the system moves to Warning state, where logging frequency is increased. Further errors trigger the Degraded state, where actuator speeds are automatically reduced by fifty percent to reduce computational load and mechanical stress. Severe violations trigger the Critical state, causing a partial system shutdown for safety. Finally, a Recovery state monitors for stability before attempting to automatically transition back to Normal operation.

[Insert Figure 2: Fail-Safe State Machine Diagram. The diagram should verify a state transition graph. Circles represent states: Normal, Warning, Degraded, Critical, Recovery. Arrows show transitions based on thresholds, such as "Missed Deadlines > 3" leading from Normal to Warning, and "Stability Timer Expired" leading from Recovery back to Normal.]

### 3.3 Control Algorithms
Control is achieved through a bank of independent controllers. We implemented a standard PID controller which calculates a control signal based on the error between the desired setpoint and the measured process variable. The Proportional term handles the present error, the Integral term corrects for past accumulated error, and the Derivative term predicts future error based on the rate of change.

To address the unavoidable latency in the processing pipeline, we extended the standard controller to create a Predictive PID Controller. This enhanced algorithm maintains a rolling history of the error terms. It performs a simple linear regression on this history to project the trend of the system state forward in time. By predicting where the system will be one step into the future, the controller can react preemptively. This is particularly effective for the high-speed Gripper actuator, significantly reducing overshoot and settling time during rapid movements.

---

## 4. Results and Discussion

### 4.1 Benchmarking Methodology
To validate the performance claims of the system, we conducted a rigorous series of benchmarks using the Criterion framework. Criterion provides statistically significant measurements by running thousands of iterations of each function and analyzing the distribution of the results. This eliminates the noise inherent in single-shot timing measurements. The tests were conducted on a standard workstation running Windows, ensuring that the results are representative of typical deployment hardware.

### 4.2 Component Performance Analysis
The micro-benchmarking results provide a granular view of the system performance. The data reveals that the individual components are highly optimized.

The Sensor Generation process, which involves random number generation and mathematical signal synthesis, executes with an average latency of just zero point one three microseconds and a ninety-ninth percentile latency of zero point three zero microseconds. This indicates that the data acquisition phase introduces negligible overhead to the overall control loop.

The Sensor Processing phase, which includes the computation of the Moving Average and the Z-Score anomaly detection, has an average latency of zero point five three microseconds with a tail latency of one point one zero microseconds. This is exceptionally fast and is orders of magnitude lower than the allocated processing budget of two hundred microseconds. This result confirms that the Rust compiler is able to generate machine code that is extremely efficient for mathematical operations.

The Actuator Control phase, involving the PID calculation and state updates, exhibits an average latency of one point zero one microseconds. While this is slightly higher than the sensor processing, it remains well within safety margins. The synchronization overhead was also measured, with Mutex locking operations consuming only zero point zero five microseconds on average. This validates the choice of the `parking_lot` mutex implementation, which is known for its high performance under low contention.

[Insert Figure 3: Component Latency Bar Chart. A bar chart comparing the average execution time of different components. Bars should be labeled "Sensor Generation", "Sensor Processing", "Actuator Control", and "Mutex Lock". The Y-axis represents time in microseconds. The values should reflect the text: 0.13, 0.53, 1.01, and 0.05 respectively.]

### 4.3 Async vs. Multi-Threaded Analysis
One of the most significant contributions of this research is the empirical comparison between the traditional multi-threaded concurrency model and the modern asynchronous model. The experiments yielded striking results that have profound implications for system architecture.

The multi-threaded implementation, which utilizes distinct Operating System threads for the sensor and actuator modules, demonstrated an average end-to-end latency of nineteen point nine three microseconds. In stark contrast, the asynchronous implementation powered by the Tokio runtime achieved an average latency of only two point five nine microseconds.

This represents a performance improvement of approximately eighty-seven percent. The discussion of this result centers on the cost of context switching. In the multi-threaded model, the Operating System scheduler creates the illusion of concurrency by rapidly switching the processor between threads. Each switch requires saving the entire register state and stack of the current thread and restoring the state of the next. This operation, while fast in human terms, is computationally expensive at the microsecond scale.

The asynchronous model, however, employs a cooperative multitasking strategy. The Tokio runtime multiplexes many lightweight tasks onto a single or few Operating System threads. When a task awaits a resource, it simply yields control to the executor, which keeps the processor execution within the same thread context. This effectively eliminates the overhead of kernel-level context switching. The results strongly suggest that for high-frequency real-time tasks that involve frequent communication or waiting, the asynchronous model in Rust offers vastly superior responsiveness and throughput.

[Insert Figure 4: Async vs Threaded Latency Comparison Chart. A chart showing two columns. The left column labeled "Multi-Threaded" is tall (approx value 20). The right column labeled "Async (Tokio)" is very short (approx value 2.6). The huge difference visually demonstrates the 87% improvement.]

### 4.4 Load Testing and Fault Injection
Reliability under stress is a defining characteristic of a real-time system. We subjected the simulation to artificial Central Processing Unit loads to verify its robustness. Under loads ranging from zero to thirty percent, the system maintained perfect timing with zero missed deadlines. At sixty percent load, the system began to exhibit occasional jitter spikes, with maximum latencies reaching approximately four hundred and thirty microseconds. Crucially, the fail-safe logic correctly identified these violations as they exceeded the two hundred microsecond budget. The system transitioned to the Warning state as designed, proving that the monitoring logic functions correctly under pressure.

To verify the fault tolerance mechanisms, we configured a fault injection module to artificially drop five percent of sensor packets. The system successfully detected one hundred percent of these anomalies. The fail-safe supervisor accumulated the error counts and triggered a transition to the Degraded mode. In this mode, the actuator speeds were reduced, ensuring that despite the unreliable data stream, the system remained stable and did not violate any critical safety limits. This behavior confirms the correctness of the five-state fail-safe machine design.

---

## 5. Conclusion and Future Work

### 5.1 Conclusion
The research presented in this report successfully demonstrates that Rust is not merely a viable alternative but a superior foundational technology for the development of next-generation Real-Time Systems. The implemented manufacturing simulation met all functional and non-functional requirements, maintaining a stable five millisecond control loop even under significant computational load.

The ownership model of Rust proved to be an invaluable asset during development. It completely eliminated an entire class of concurrency bugs, such as data races and dangling pointers, which are the bane of traditional C++ development. This guarantee of memory safety without the need for a runtime garbage collector resolves the historical dilemma between performance and safety.

Furthermore, the research provides compelling evidence regarding concurrency models. The data shows that the asynchronous programming model in Rust can outperform traditional threading models by nearly an order of magnitude in high-frequency applications. The eighty-seven percent reduction in latency observed in the asynchronous implementation highlights the efficiency of user-space scheduling over kernel-space scheduling for IO-bound or communication-heavy workloads.

### 5.2 Future Work
While the simulation is robust, several avenues for future research remain. The immediate next step is Hardware Integration. Validating the software on actual embedded hardware, such as an ARM Cortex-M controller or a Raspberry Pi running a Real-Time Linux kernel, would provide confirmation of the timing results in a physical environment.

Another critical area is Formal Verification. While the fail-safe state machine was tested empirically, applying formal methods to mathematically prove the correctness of the state transitions would provide the level of assurance required for safety-critical certification standards.

Finally, the scope could be expanded to Networked Control. Extending the simulation to span multiple compute nodes connected via Time Sensitive Networking would allow for the analysis of network jitter and synchronization algorithms, which are essential for modern distributed industrial control systems.

---

## 6. References

Abbott, R., and Garcia-Molina, H. (1992). Scheduling real-time transactions: a performance evaluation. *ACM Transactions on Database Systems*, 17(3), 513 to 560.

Blandy, J., and Orendorff, J. (2018). *Programming Rust: Fast, Safe Systems Development*. O'Reilly Media.

Levy, A., Andersen, M. P., Campbell, B., Culler, D. E., Dutta, P., Ghena, B., Levis, P., and Pannuto, P. (2017). Multiprogramming a 64kB Computer Safely and Efficiently. *Proceedings of the 26th Symposium on Operating Systems Principles (SOSP '17)*.

Sha, L. (2001). Using simplicity to control complexity. *IEEE Software*, 18(4), 20 to 28.

Tokio Contributors. (2024). *Tokio: An asynchronous Rust runtime*. Available at https://tokio.rs
