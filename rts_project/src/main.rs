// ============================================================================
// RTS Manufacturing System - Main Entry Point
// ============================================================================
// Real-Time Sensor-Actuator System for Automated Manufacturing
// 
// This program demonstrates:
// - Multi-threaded sensor-actuator integration
// - PID control with predictive algorithms
// - Shared resource synchronization (mutex, RwLock, atomics)
// - Fault injection and fail-safe mechanisms
// - Comprehensive performance benchmarking
// ============================================================================

use rts_manufacturing::actuator::*;
use rts_manufacturing::benchmark::*;
use rts_manufacturing::config::*;
use rts_manufacturing::failsafe::*;
use rts_manufacturing::fault_injection::*;
use rts_manufacturing::ipc::*;
use rts_manufacturing::sensor::*;
use rts_manufacturing::shared_resource::*;
use rts_manufacturing::types::*;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

// ----------------------------------------------------------------------------
// Main System Integration
// ----------------------------------------------------------------------------

fn main() {
    println!("============================================================");
    println!("  RTS Manufacturing System - Real-Time Control Simulation");
    println!("============================================================");
    println!();

    // Run the main demonstration
    run_demonstration();
}

/// Main demonstration function
fn run_demonstration() {
    println!("Starting system demonstration...\n");

    // 1. Run basic multi-threaded integration
    println!("=== Part 1: Multi-Threaded Integration ===");
    run_multithreaded_system(500); // Run for 500 cycles

    // 2. Run with fault injection enabled
    println!("\n=== Part 2: Fault Injection Testing ===");
    run_with_fault_injection(300);

    // 3. Run under CPU load simulation
    println!("\n=== Part 3: CPU Load Simulation ===");
    run_under_load(200);

    // 4. Run detailed benchmarks
    println!("\n=== Part 4: Performance Benchmarking ===");
    run_benchmarks();

    // 5. Demonstrate async implementation (comparison)
    println!("\n=== Part 5: Async vs Threaded Comparison ===");
    run_async_comparison();

    println!("\n============================================================");
    println!("  Demonstration Complete");
    println!("============================================================");
}

// ----------------------------------------------------------------------------
// Multi-Threaded System Integration
// ----------------------------------------------------------------------------

/// Run the complete multi-threaded system
fn run_multithreaded_system(num_cycles: usize) {
    println!("Initializing multi-threaded system...");

    // Create shared resources
    let shared = SharedResources::new();

    // Create IPC channels
    let ipc = IpcManager::new();

    // Create running flag
    let running = Arc::new(AtomicBool::new(true));

    // Create sensor module
    let sensor_sender = ipc.get_sensor_sender();
    let feedback_receiver = ipc.get_feedback_receiver();
    let sensor_shared = shared.clone();
    let sensor_running = Arc::clone(&running);

    // Create actuator module
    let sensor_receiver = ipc.get_sensor_receiver();
    let feedback_sender = ipc.get_feedback_sender();
    let actuator_shared = shared.clone();
    let actuator_running = Arc::clone(&running);

    // Track timing for the complete system
    let start_time = Instant::now();
    let mut cycle_times: Vec<u64> = Vec::with_capacity(num_cycles);

    // Spawn sensor thread
    let sensor_handle = thread::Builder::new()
        .name("sensor-module".into())
        .spawn(move || {
            let mut sensor = SensorModule::new(
                sensor_sender,
                feedback_receiver,
                sensor_shared,
                sensor_running,
            );
            
            for _ in 0..num_cycles {
                if let Err(e) = sensor.run_cycle() {
                    eprintln!("Sensor error: {}", e);
                }
                thread::sleep(Duration::from_millis(1));
            }
            
            sensor.get_stats()
        })
        .expect("Failed to spawn sensor thread");

    // Spawn actuator thread
    let actuator_handle = thread::Builder::new()
        .name("actuator-module".into())
        .spawn(move || {
            let mut actuator = ActuatorModule::new(
                sensor_receiver,
                feedback_sender,
                actuator_shared,
                actuator_running,
            );
            
            for _ in 0..num_cycles {
                if let Err(e) = actuator.run_cycle() {
                    eprintln!("Actuator error: {}", e);
                }
                thread::sleep(Duration::from_micros(500));
            }
            
            actuator.get_stats()
        })
        .expect("Failed to spawn actuator thread");

    // Wait for threads to complete
    let sensor_stats = sensor_handle.join().expect("Sensor thread panicked");
    let actuator_stats = actuator_handle.join().expect("Actuator thread panicked");

    // Calculate total runtime
    let total_time = start_time.elapsed();

    // Print results
    println!("\n--- Multi-Threaded System Results ---");
    println!("Total Runtime: {:.2} seconds", total_time.as_secs_f64());
    println!("Total Sensor Cycles: {}", sensor_stats.total_cycles);
    println!("Total Actuator Cycles: {}", actuator_stats.total_cycles);
    
    println!("\nSensor Performance:");
    println!("  Avg Generation Time:   {:.3} µs", sensor_stats.generation.avg_latency_ns / 1000.0);
    println!("  Avg Processing Time:   {:.3} µs", sensor_stats.processing.avg_latency_ns / 1000.0);
    println!("  Avg Transmission Time: {:.3} µs", sensor_stats.transmission.avg_latency_ns / 1000.0);
    println!("  Missed Deadlines:      {}", sensor_stats.missed_deadlines);

    println!("\nActuator Performance:");
    println!("  Avg Reception Time:    {:.3} µs", actuator_stats.reception.avg_latency_ns / 1000.0);
    println!("  Avg Control Time:      {:.3} µs", actuator_stats.control.avg_latency_ns / 1000.0);
    println!("  Avg Feedback Time:     {:.3} µs", actuator_stats.feedback.avg_latency_ns / 1000.0);
    println!("  Missed Deadlines:      {}", actuator_stats.missed_deadlines);
    println!("  Fail-Safe State:       {:?}", actuator_stats.failsafe_state);

    // Print shared resource statistics
    shared.print_sync_stats();
}

// ----------------------------------------------------------------------------
// Fault Injection Testing
// ----------------------------------------------------------------------------

/// Run system with fault injection enabled
fn run_with_fault_injection(num_cycles: usize) {
    println!("Initializing fault injection test...");

    let shared = SharedResources::new();
    let ipc = IpcManager::new();
    let running = Arc::new(AtomicBool::new(true));

    // Create fault injector
    let mut fault_injector = FaultInjector::new();
    fault_injector.set_probabilities(0.05, 0.03, 0.02, 0.05);

    // Create fault detector
    let mut fault_detector = FaultDetector::new(NUM_SENSOR_TYPES);

    // Create modules
    let mut sensor = SensorModule::new(
        ipc.get_sensor_sender(),
        ipc.get_feedback_receiver(),
        shared.clone(),
        Arc::clone(&running),
    );

    let mut actuator = ActuatorModule::new(
        ipc.get_sensor_receiver(),
        ipc.get_feedback_sender(),
        shared.clone(),
        Arc::clone(&running),
    );

    let mut faults_injected = 0;
    let mut faults_detected = 0;

    // Run cycles with fault injection
    for cycle in 0..num_cycles {
        // Generate and process sensor data
        if let Ok(processed_data) = sensor.run_cycle() {
            for data in processed_data {
                // Inject faults
                if let Some((faulty_data, record)) = fault_injector.apply_fault(data) {
                    if record.fault_type != FaultType::None {
                        faults_injected += 1;
                    }

                    // Detect faults
                    let issues = fault_detector.check_data(&faulty_data);
                    if !issues.is_empty() {
                        faults_detected += 1;
                    }
                }
            }
        }

        // Run actuator cycle
        let _ = actuator.run_cycle();

        thread::sleep(Duration::from_millis(1));
    }

    // Print fault injection results
    println!("\n--- Fault Injection Results ---");
    fault_injector.print_stats();
    println!("\nFault Detection:");
    println!("  Faults Injected: {}", faults_injected);
    println!("  Faults Detected: {}", fault_detector.get_fault_count());
    
    // Print fail-safe status
    actuator.get_failsafe().print_status();
}

// ----------------------------------------------------------------------------
// CPU Load Simulation
// ----------------------------------------------------------------------------

/// Run system under artificial CPU load
fn run_under_load(num_cycles: usize) {
    println!("Starting CPU load simulation...");

    // Test at different load levels
    let load_levels = [0.0, 0.3, 0.6, 0.8];
    let mut results: Vec<(f64, f64, usize)> = Vec::new();

    for &load in &load_levels {
        println!("\nTesting at {:.0}% CPU load...", load * 100.0);

        // Start load generators
        let num_load_threads = if load > 0.0 { 2 } else { 0 };
        let load_handles: Vec<_> = (0..num_load_threads)
            .map(|_| {
                let target_load = load;
                thread::spawn(move || {
                    let start = Instant::now();
                    let mut counter: u64 = 0;
                    while start.elapsed() < Duration::from_secs(2) {
                        // Busy work
                        for _ in 0..(target_load * 10000.0) as usize {
                            counter = counter.wrapping_add(1);
                            std::hint::black_box(counter);
                        }
                        // Small sleep
                        thread::sleep(Duration::from_micros(100));
                    }
                })
            })
            .collect();

        // Run system under load
        let shared = SharedResources::new();
        let ipc = IpcManager::new();
        let running = Arc::new(AtomicBool::new(true));

        let mut sensor = SensorModule::new(
            ipc.get_sensor_sender(),
            ipc.get_feedback_receiver(),
            shared.clone(),
            Arc::clone(&running),
        );

        let mut actuator = ActuatorModule::new(
            ipc.get_sensor_receiver(),
            ipc.get_feedback_sender(),
            shared.clone(),
            Arc::clone(&running),
        );

        let mut latencies: Vec<u64> = Vec::new();
        let mut missed = 0;

        for _ in 0..num_cycles {
            let cycle_start = Instant::now();
            
            let _ = sensor.run_cycle();
            let _ = actuator.run_cycle();
            
            let cycle_time = cycle_start.elapsed().as_nanos() as u64;
            latencies.push(cycle_time);
            
            if cycle_time > ACTUATOR_DEADLINE.as_nanos() as u64 {
                missed += 1;
            }

            thread::sleep(Duration::from_millis(1));
        }

        // Wait for load threads
        for handle in load_handles {
            let _ = handle.join();
        }

        // Calculate average latency
        let avg_latency = latencies.iter().sum::<u64>() as f64 / latencies.len() as f64;
        results.push((load, avg_latency, missed));
    }

    // Print load test results
    println!("\n--- CPU Load Test Results ---");
    println!("{:<10} {:<20} {:<15}", "Load %", "Avg Latency (µs)", "Missed Deadlines");
    println!("{}", "-".repeat(45));
    for (load, latency, missed) in &results {
        println!("{:<10.0} {:<20.3} {:<15}", 
            load * 100.0, 
            latency / 1000.0, 
            missed);
    }
}

// ----------------------------------------------------------------------------
// Performance Benchmarking
// ----------------------------------------------------------------------------

/// Run detailed performance benchmarks
fn run_benchmarks() {
    println!("Running performance benchmarks...");
    
    let mut benchmark = SystemBenchmark::new();
    let iterations = 1000;

    // Benchmark sensor generation
    println!("  Benchmarking sensor generation...");
    let mut sensor_sim = rts_manufacturing::sensor::SensorSimulator::new(0, "Test");
    for _ in 0..iterations {
        benchmark.sensor_generation.start();
        let _ = sensor_sim.generate_reading();
        benchmark.sensor_generation.stop();
    }

    // Benchmark data processing
    println!("  Benchmarking data processing...");
    let mut processor = rts_manufacturing::sensor::DataProcessor::new(NUM_SENSOR_TYPES);
    for i in 0..iterations {
        let reading = rts_manufacturing::types::SensorReading::new(
            0, "Test".to_string(), 50.0 + (i as f64 * 0.1), i as u64
        );
        benchmark.data_processing.start();
        let _ = processor.process(&reading);
        benchmark.data_processing.stop();
    }

    // Benchmark PID control
    println!("  Benchmarking PID control...");
    let mut pid = rts_manufacturing::pid_controller::PidController::with_defaults("Test");
    pid.set_setpoint(50.0);
    for i in 0..iterations {
        let measurement = 45.0 + (i as f64 * 0.01);
        benchmark.pid_control.start();
        let _ = pid.update(measurement);
        benchmark.pid_control.stop();
    }

    // Benchmark channel operations
    println!("  Benchmarking channel operations...");
    let (tx, rx) = crossbeam_channel::bounded::<u64>(100);
    for i in 0..iterations {
        benchmark.data_transmission.start();
        let _ = tx.send(i as u64);
        benchmark.data_transmission.stop();
        
        benchmark.actuator_reception.start();
        let _ = rx.recv();
        benchmark.actuator_reception.stop();
    }

    // Benchmark lock contention
    println!("  Benchmarking lock contention...");
    let mutex_data = parking_lot::Mutex::new(0u64);
    for _ in 0..iterations {
        benchmark.lock_contention.start();
        {
            let mut guard = mutex_data.lock();
            *guard += 1;
        }
        benchmark.lock_contention.stop();
    }

    // Print benchmark report
    benchmark.print_report();

    // Export to JSON
    let json = benchmark.export_json();
    println!("\nJSON Export (sample):");
    println!("{}", &json[..json.len().min(200)]);
}

// ----------------------------------------------------------------------------
// Async vs Threaded Comparison
// ----------------------------------------------------------------------------

/// Compare async and threaded implementations
fn run_async_comparison() {
    println!("Comparing async vs multi-threaded implementations...");
    
    let iterations = 500;
    
    // Threaded implementation benchmark
    println!("\n  Running threaded implementation...");
    let threaded_start = Instant::now();
    let mut threaded_latencies: Vec<u64> = Vec::new();
    
    let shared = SharedResources::new();
    let ipc = IpcManager::new();
    let running = Arc::new(AtomicBool::new(true));

    {
        let mut sensor = SensorModule::new(
            ipc.get_sensor_sender(),
            ipc.get_feedback_receiver(),
            shared.clone(),
            Arc::clone(&running),
        );

        let mut actuator = ActuatorModule::new(
            ipc.get_sensor_receiver(),
            ipc.get_feedback_sender(),
            shared.clone(),
            Arc::clone(&running),
        );

        for _ in 0..iterations {
            let cycle_start = Instant::now();
            let _ = sensor.run_cycle();
            let _ = actuator.run_cycle();
            threaded_latencies.push(cycle_start.elapsed().as_nanos() as u64);
            thread::sleep(Duration::from_micros(100));
        }
    }
    
    let threaded_time = threaded_start.elapsed();

    // Async implementation benchmark (using tokio runtime)
    println!("  Running async implementation...");
    let async_start = Instant::now();
    let async_latencies = run_async_benchmark(iterations);
    let async_time = async_start.elapsed();

    // Calculate statistics
    let threaded_avg = threaded_latencies.iter().sum::<u64>() as f64 / threaded_latencies.len() as f64;
    let async_avg = async_latencies.iter().sum::<u64>() as f64 / async_latencies.len() as f64;

    let threaded_max = *threaded_latencies.iter().max().unwrap_or(&0);
    let async_max = *async_latencies.iter().max().unwrap_or(&0);

    // Print comparison
    println!("\n--- Async vs Threaded Comparison ---");
    println!("{:<20} {:<20} {:<20}", "Metric", "Multi-Threaded", "Async");
    println!("{}", "-".repeat(60));
    println!("{:<20} {:<20.3} {:<20.3}", 
        "Avg Latency (µs)", 
        threaded_avg / 1000.0, 
        async_avg / 1000.0);
    println!("{:<20} {:<20.3} {:<20.3}", 
        "Max Latency (µs)", 
        threaded_max as f64 / 1000.0, 
        async_max as f64 / 1000.0);
    println!("{:<20} {:<20.3} {:<20.3}", 
        "Total Time (s)", 
        threaded_time.as_secs_f64(), 
        async_time.as_secs_f64());
    println!("{:<20} {:<20.2} {:<20.2}", 
        "Throughput (ops/s)", 
        iterations as f64 / threaded_time.as_secs_f64(),
        iterations as f64 / async_time.as_secs_f64());

    // Analysis
    let latency_diff = ((async_avg - threaded_avg) / threaded_avg) * 100.0;
    println!("\nAnalysis:");
    if latency_diff < 0.0 {
        println!("  Async implementation is {:.1}% faster in average latency", -latency_diff);
    } else {
        println!("  Threaded implementation is {:.1}% faster in average latency", latency_diff);
    }
}

/// Run async benchmark using Tokio
fn run_async_benchmark(iterations: usize) -> Vec<u64> {
    use tokio::runtime::Runtime;
    use tokio::sync::mpsc;

    let rt = Runtime::new().expect("Failed to create Tokio runtime");
    
    rt.block_on(async {
        let mut latencies = Vec::with_capacity(iterations);
        let (tx, mut rx) = mpsc::channel::<u64>(100);

        for i in 0..iterations {
            let cycle_start = Instant::now();
            
            // Simulate sensor data generation
            let sensor_data = i as u64;
            
            // Send through async channel
            tx.send(sensor_data).await.ok();
            
            // Receive and process
            if let Some(_data) = rx.recv().await {
                // Simulate processing
                tokio::task::yield_now().await;
            }
            
            latencies.push(cycle_start.elapsed().as_nanos() as u64);
            
            // Small delay
            tokio::time::sleep(Duration::from_micros(100)).await;
        }
        
        latencies
    })
}

// ----------------------------------------------------------------------------
// Unit Tests
// ----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_generation() {
        let mut sensor = rts_manufacturing::sensor::SensorSimulator::new(0, "Force");
        let reading = sensor.generate_reading();
        assert_eq!(reading.sensor_id, 0);
        assert!(reading.value > 0.0);
    }

    #[test]
    fn test_pid_controller() {
        let mut pid = rts_manufacturing::pid_controller::PidController::with_defaults("Test");
        pid.set_setpoint(100.0);
        let (output, error, _) = pid.update(50.0);
        assert!(output > 0.0, "PID should output positive value when below setpoint");
        assert!((error - 50.0).abs() < 0.001, "Error should be 50");
    }

    #[test]
    fn test_shared_resource_sync() {
        let shared = SharedResources::new();
        
        // Test atomic operations
        shared.status_memory.increment_cycles();
        assert_eq!(shared.status_memory.get_cycles(), 1);
        
        // Test logging
        shared.diagnostic_log.log(LogLevel::Info, "Test", "Test message");
        let recent = shared.diagnostic_log.get_recent(1);
        assert_eq!(recent.len(), 1);
    }

    #[test]
    fn test_failsafe_transitions() {
        let mut failsafe = FailSafeManager::new();
        
        assert!(failsafe.is_normal());
        
        // Trigger multiple deadline misses
        for _ in 0..5 {
            failsafe.report_missed_deadline();
        }
        
        assert!(failsafe.is_failsafe_active() || failsafe.get_state() == FailSafeState::Warning);
    }

    #[test]
    fn test_channel_communication() {
        let channel = SensorDataChannel::new(10);
        let sender = channel.get_sender();
        let receiver = channel.get_receiver();

        let data = ProcessedSensorData::new(
            0, "Test".to_string(), 50.0, 50.0, false, 1.0, 100, 1
        );

        sender.send(data.clone()).unwrap();
        let received = receiver.recv().unwrap();
        
        assert_eq!(received.sensor_id, 0);
        assert!((received.filtered_value - 50.0).abs() < 0.001);
    }
}
