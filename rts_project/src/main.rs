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
// - COMPARISON BENCHMARKS for assignment requirements
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

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use std::fs::File;
use std::io::Write;

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
    run_multithreaded_system(500);

    // 2. Run with fault injection enabled
    println!("\n=== Part 2: Fault Injection Testing ===");
    run_with_fault_injection(300);

    // 3. Run under CPU load simulation (COMPARISON: Different Load Levels)
    println!("\n=== Part 3: CPU Load Comparison ===");
    run_under_load(200);

    // 4. Run detailed benchmarks
    println!("\n=== Part 4: Performance Benchmarking ===");
    run_benchmarks();

    // =========================================================================
    // COMPARISON BENCHMARKS (Required for Assignment)
    // =========================================================================
    
    // 6. Lock Contention Comparison (High vs Low Contention)
    println!("\n=== Part 6: Lock Contention Comparison ===");
    run_lock_contention_comparison();
    
    // 7. Synchronization Primitives Comparison
    println!("\n=== Part 7: Synchronization Primitives Comparison ===");
    run_sync_primitives_comparison();
    
    // 8. Channel Types Comparison
    println!("\n=== Part 8: Channel Types Comparison ===");
    run_channel_comparison();

    // 9. Save all benchmark results to files
    println!("\n=== Part 9: Saving Results to Files ===");
    save_benchmark_results();

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

    let shared = SharedResources::new();
    let ipc = IpcManager::new();
    let running = Arc::new(AtomicBool::new(true));

    let sensor_sender = ipc.get_sensor_sender();
    let feedback_receiver = ipc.get_feedback_receiver();
    let sensor_shared = shared.clone();
    let sensor_running = Arc::clone(&running);

    let sensor_receiver = ipc.get_sensor_receiver();
    let feedback_sender = ipc.get_feedback_sender();
    let actuator_shared = shared.clone();
    let actuator_running = Arc::clone(&running);

    let start_time = Instant::now();

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

    let sensor_stats = sensor_handle.join().expect("Sensor thread panicked");
    let actuator_stats = actuator_handle.join().expect("Actuator thread panicked");
    let total_time = start_time.elapsed();

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

    shared.print_sync_stats();
}

// ----------------------------------------------------------------------------
// Fault Injection Testing
// ----------------------------------------------------------------------------

fn run_with_fault_injection(num_cycles: usize) {
    println!("Initializing fault injection test...");

    let shared = SharedResources::new();
    let ipc = IpcManager::new();
    let running = Arc::new(AtomicBool::new(true));

    let mut fault_injector = FaultInjector::new();
    fault_injector.set_probabilities(0.05, 0.03, 0.02, 0.05);

    let mut fault_detector = FaultDetector::new(NUM_SENSOR_TYPES);

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

    for _cycle in 0..num_cycles {
        if let Ok(processed_data) = sensor.run_cycle() {
            for data in processed_data {
                if let Some((faulty_data, record)) = fault_injector.apply_fault(data) {
                    if record.fault_type != FaultType::None {
                        faults_injected += 1;
                    }
                    let issues = fault_detector.check_data(&faulty_data);
                    if !issues.is_empty() {
                        faults_detected += 1;
                    }
                }
            }
        }
        let _ = actuator.run_cycle();
        thread::sleep(Duration::from_millis(1));
    }

    println!("\n--- Fault Injection Results ---");
    fault_injector.print_stats();
    println!("\nFault Detection:");
    println!("  Faults Injected: {}", faults_injected);
    println!("  Faults Detected: {}", fault_detector.get_fault_count());
    
    actuator.get_failsafe().print_status();
}

// ----------------------------------------------------------------------------
// CPU Load Simulation (COMPARISON: Normal vs High Load)
// ----------------------------------------------------------------------------

fn run_under_load(num_cycles: usize) {
    println!("Starting CPU load simulation...");
    println!("COMPARISON: System performance under varying CPU loads\n");

    let load_levels = [0.0, 0.3, 0.6, 0.8];
    let mut results: Vec<(f64, f64, f64, usize, f64)> = Vec::new();

    for &load in &load_levels {
        println!("Testing at {:.0}% CPU load...", load * 100.0);

        let num_load_threads = if load > 0.0 { 2 } else { 0 };
        let load_handles: Vec<_> = (0..num_load_threads)
            .map(|_| {
                let target_load = load;
                thread::spawn(move || {
                    let start = Instant::now();
                    let mut counter: u64 = 0;
                    while start.elapsed() < Duration::from_secs(2) {
                        for _ in 0..(target_load * 10000.0) as usize {
                            counter = counter.wrapping_add(1);
                            std::hint::black_box(counter);
                        }
                        thread::sleep(Duration::from_micros(100));
                    }
                })
            })
            .collect();

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

        for handle in load_handles {
            let _ = handle.join();
        }

        let avg_latency = latencies.iter().sum::<u64>() as f64 / latencies.len() as f64;
        let max_latency = *latencies.iter().max().unwrap_or(&0) as f64;
        let mut sorted = latencies.clone();
        sorted.sort_unstable();
        let p99_idx = (sorted.len() as f64 * 0.99) as usize;
        let p99_latency = sorted[p99_idx.min(sorted.len() - 1)] as f64;
        
        results.push((load, avg_latency, max_latency, missed, p99_latency));
    }

    println!("\n╔═══════════════════════════════════════════════════════════════════════════╗");
    println!("║                    CPU LOAD COMPARISON RESULTS                            ║");
    println!("╠═══════════════════════════════════════════════════════════════════════════╣");
    println!("║ {:^10} │ {:^15} │ {:^15} │ {:^15} │ {:^10} ║", 
        "Load %", "Avg Lat (µs)", "Max Lat (µs)", "P99 Lat (µs)", "Missed");
    println!("╠═══════════════════════════════════════════════════════════════════════════╣");
    for (load, avg, max, missed, p99) in &results {
        println!("║ {:^10.0} │ {:>15.3} │ {:>15.3} │ {:>15.3} │ {:^10} ║", 
            load * 100.0, avg / 1000.0, max / 1000.0, p99 / 1000.0, missed);
    }
    println!("╚═══════════════════════════════════════════════════════════════════════════╝");

    // Analysis
    let baseline = &results[0];
    let high_load = &results[3];
    let latency_increase = ((high_load.1 - baseline.1) / baseline.1) * 100.0;
    
    println!("\n--- Analysis ---");
    println!("  • At 80% CPU load, latency increased by {:.1}% compared to 0% load", latency_increase);
    println!("  • Missed deadlines increased from {} to {} under high load", baseline.3, high_load.3);
    println!("  • P99 latency shows worst-case behavior critical for real-time systems");
}

// ----------------------------------------------------------------------------
// Performance Benchmarking
// ----------------------------------------------------------------------------

fn run_benchmarks() {
    println!("Running performance benchmarks...");
    
    let mut benchmark = SystemBenchmark::new();
    let iterations = 1000;

    println!("  Benchmarking sensor generation...");
    let mut sensor_sim = rts_manufacturing::sensor::SensorSimulator::new(0, "Test");
    for _ in 0..iterations {
        benchmark.sensor_generation.start();
        let _ = sensor_sim.generate_reading();
        benchmark.sensor_generation.stop();
    }

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

    println!("  Benchmarking PID control...");
    let mut pid = rts_manufacturing::pid_controller::PidController::with_defaults("Test");
    pid.set_setpoint(50.0);
    for i in 0..iterations {
        let measurement = 45.0 + (i as f64 * 0.01);
        benchmark.pid_control.start();
        let _ = pid.update(measurement);
        benchmark.pid_control.stop();
    }

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

    benchmark.print_report();
}

// ============================================================================
// COMPARISON 1: FAIR ASYNC VS THREADED
// ============================================================================

/// Results structure for benchmark comparison
#[derive(Debug, Clone)]
struct ComparisonResults {
    avg_latency_us: f64,
    min_latency_us: f64,
    max_latency_us: f64,
    p99_latency_us: f64,
    jitter_us: f64,
    throughput: f64,
    total_time_ms: f64,
}

fn run_fair_async_comparison() {
    println!("Comparing async vs multi-threaded implementations...");
    println!("IMPORTANT: Both versions perform IDENTICAL work:\n");
    println!("  - Sensor data generation with noise");
    println!("  - Moving average filtering");
    println!("  - Anomaly detection (z-score)");
    println!("  - PID control calculation");
    println!("  - Channel communication\n");
    
    let iterations = 500;
    
    // THREADED VERSION
    println!("Running THREADED implementation...");
    let threaded_results = run_threaded_full_benchmark(iterations);
    
    // ASYNC VERSION (same work)
    println!("Running ASYNC implementation...");
    let async_results = run_async_full_benchmark(iterations);
    
    // Output
    println!("\n╔══════════════════════════════════════════════════════════════════╗");
    println!("║          ASYNC vs THREADED COMPARISON RESULTS                    ║");
    println!("╠══════════════════════════════════════════════════════════════════╣");
    println!("║ {:^20} │ {:^17} │ {:^17} ║", "Metric", "Threaded", "Async");
    println!("╠══════════════════════════════════════════════════════════════════╣");
    println!("║ {:^20} │ {:>14.3} µs │ {:>14.3} µs ║", 
        "Avg Latency", threaded_results.avg_latency_us, async_results.avg_latency_us);
    println!("║ {:^20} │ {:>14.3} µs │ {:>14.3} µs ║", 
        "Min Latency", threaded_results.min_latency_us, async_results.min_latency_us);
    println!("║ {:^20} │ {:>14.3} µs │ {:>14.3} µs ║", 
        "Max Latency", threaded_results.max_latency_us, async_results.max_latency_us);
    println!("║ {:^20} │ {:>14.3} µs │ {:>14.3} µs ║", 
        "P99 Latency", threaded_results.p99_latency_us, async_results.p99_latency_us);
    println!("║ {:^20} │ {:>14.3} µs │ {:>14.3} µs ║", 
        "Jitter (Max-Min)", threaded_results.jitter_us, async_results.jitter_us);
    println!("║ {:^20} │ {:>14.2} /s │ {:>14.2} /s ║", 
        "Throughput", threaded_results.throughput, async_results.throughput);
    println!("║ {:^20} │ {:>14.2} ms │ {:>14.2} ms ║", 
        "Total Time", threaded_results.total_time_ms, async_results.total_time_ms);
    println!("╚══════════════════════════════════════════════════════════════════╝");
    
    // Analysis
    let latency_diff = ((async_results.avg_latency_us - threaded_results.avg_latency_us) 
        / threaded_results.avg_latency_us) * 100.0;
    let throughput_diff = ((async_results.throughput - threaded_results.throughput) 
        / threaded_results.throughput) * 100.0;
    
    println!("\n--- Analysis ---");
    if latency_diff < 0.0 {
        println!("  ✓ Async is {:.1}% FASTER in average latency", -latency_diff);
    } else {
        println!("  ✓ Threaded is {:.1}% FASTER in average latency", latency_diff);
    }
    
    if throughput_diff > 0.0 {
        println!("  ✓ Async has {:.1}% HIGHER throughput", throughput_diff);
    } else {
        println!("  ✓ Threaded has {:.1}% HIGHER throughput", -throughput_diff);
    }
    
    if async_results.jitter_us < threaded_results.jitter_us {
        println!("  ✓ Async shows LOWER jitter - more predictable timing");
    } else {
        println!("  ✓ Threaded shows LOWER jitter - more predictable timing");
    }
}

fn run_threaded_full_benchmark(iterations: usize) -> ComparisonResults {
    use std::sync::mpsc;
    
    let start = Instant::now();
    let mut latencies: Vec<u64> = Vec::with_capacity(iterations);
    
    let (tx, rx) = mpsc::sync_channel::<ProcessedSensorData>(100);
    
    let mut pid = rts_manufacturing::pid_controller::PidController::with_defaults("Threaded");
    pid.set_setpoint(50.0);
    
    let mut processor = rts_manufacturing::sensor::DataProcessor::new(NUM_SENSOR_TYPES);
    let mut sensor_sim = rts_manufacturing::sensor::SensorSimulator::new(0, "Force");
    
    for _i in 0..iterations {
        let cycle_start = Instant::now();
        
        // SENSOR SIDE
        let reading = sensor_sim.generate_reading();
        let processed = processor.process(&reading);
        tx.send(processed).unwrap();
        
        // ACTUATOR SIDE
        let received = rx.recv().unwrap();
        let (_output, _error, _dt) = pid.update(received.filtered_value);
        
        latencies.push(cycle_start.elapsed().as_nanos() as u64);
    }
    
    let total_time = start.elapsed();
    calculate_comparison_results(&latencies, total_time)
}

fn run_async_full_benchmark(iterations: usize) -> ComparisonResults {
    use tokio::runtime::Runtime;
    use tokio::sync::mpsc;
    
    let rt = Runtime::new().expect("Failed to create Tokio runtime");
    
    rt.block_on(async {
        let start = Instant::now();
        let mut latencies: Vec<u64> = Vec::with_capacity(iterations);
        
        let (tx, mut rx) = mpsc::channel::<ProcessedSensorData>(100);
        
        let mut pid = rts_manufacturing::pid_controller::PidController::with_defaults("Async");
        pid.set_setpoint(50.0);
        
        let mut processor = rts_manufacturing::sensor::DataProcessor::new(NUM_SENSOR_TYPES);
        let mut sensor_sim = rts_manufacturing::sensor::SensorSimulator::new(0, "Force");
        
        for _i in 0..iterations {
            let cycle_start = Instant::now();
            
            // SENSOR SIDE (IDENTICAL to threaded)
            let reading = sensor_sim.generate_reading();
            let processed = processor.process(&reading);
            tx.send(processed).await.unwrap();
            
            // ACTUATOR SIDE (IDENTICAL to threaded)
            let received = rx.recv().await.unwrap();
            let (_output, _error, _dt) = pid.update(received.filtered_value);
            
            latencies.push(cycle_start.elapsed().as_nanos() as u64);
        }
        
        let total_time = start.elapsed();
        calculate_comparison_results(&latencies, total_time)
    })
}

fn calculate_comparison_results(latencies: &[u64], total_time: Duration) -> ComparisonResults {
    let mut sorted = latencies.to_vec();
    sorted.sort_unstable();
    
    let count = sorted.len();
    let sum: u64 = sorted.iter().sum();
    let avg = sum as f64 / count as f64;
    let min = sorted[0];
    let max = sorted[count - 1];
    let p99_idx = (count as f64 * 0.99) as usize;
    let p99 = sorted[p99_idx.min(count - 1)];
    
    ComparisonResults {
        avg_latency_us: avg / 1000.0,
        min_latency_us: min as f64 / 1000.0,
        max_latency_us: max as f64 / 1000.0,
        p99_latency_us: p99 as f64 / 1000.0,
        jitter_us: (max - min) as f64 / 1000.0,
        throughput: count as f64 / total_time.as_secs_f64(),
        total_time_ms: total_time.as_secs_f64() * 1000.0,
    }
}

// ============================================================================
// COMPARISON 2: LOCK CONTENTION (High vs Low)
// ============================================================================

#[derive(Debug, Clone)]
struct ContentionResults {
    avg_latency_us: f64,
    max_latency_us: f64,
    p99_latency_us: f64,
    throughput: f64,
    contention_count: usize,
}

fn run_lock_contention_comparison() {
    println!("Comparing HIGH vs LOW lock contention scenarios...\n");
    
    let iterations = 10000;
    let num_threads = 4;
    
    println!("Running HIGH CONTENTION (single Mutex, {} threads competing)...", num_threads);
    let high_results = run_high_contention_test(iterations, num_threads);
    
    println!("Running LOW CONTENTION (Atomics - lock-free)...");
    let low_results = run_low_contention_atomic_test(iterations, num_threads);
    
    println!("Running MEDIUM CONTENTION (RwLock, 90% reads)...");
    let rwlock_results = run_rwlock_contention_test(iterations, num_threads);
    
    println!("\n╔═══════════════════════════════════════════════════════════════════════════════╗");
    println!("║                    LOCK CONTENTION COMPARISON RESULTS                         ║");
    println!("║                    ({} threads, {} iterations each)                          ║", num_threads, iterations);
    println!("╠═══════════════════════════════════════════════════════════════════════════════╣");
    println!("║ {:^18} │ {:^15} │ {:^15} │ {:^15} ║", 
        "Metric", "High (Mutex)", "Low (Atomic)", "Medium (RwLock)");
    println!("╠═══════════════════════════════════════════════════════════════════════════════╣");
    println!("║ {:^18} │ {:>12.3} µs │ {:>12.3} µs │ {:>12.3} µs ║", 
        "Avg Latency", high_results.avg_latency_us, low_results.avg_latency_us, rwlock_results.avg_latency_us);
    println!("║ {:^18} │ {:>12.3} µs │ {:>12.3} µs │ {:>12.3} µs ║", 
        "Max Latency", high_results.max_latency_us, low_results.max_latency_us, rwlock_results.max_latency_us);
    println!("║ {:^18} │ {:>12.3} µs │ {:>12.3} µs │ {:>12.3} µs ║", 
        "P99 Latency", high_results.p99_latency_us, low_results.p99_latency_us, rwlock_results.p99_latency_us);
    println!("║ {:^18} │ {:>12.0} /s │ {:>12.0} /s │ {:>12.0} /s ║", 
        "Throughput", high_results.throughput, low_results.throughput, rwlock_results.throughput);
    println!("║ {:^18} │ {:>15} │ {:>15} │ {:>15} ║", 
        "Contentions", high_results.contention_count, low_results.contention_count, rwlock_results.contention_count);
    println!("╚═══════════════════════════════════════════════════════════════════════════════╝");
    
    let speedup = high_results.avg_latency_us / low_results.avg_latency_us;
    println!("\n--- Analysis ---");
    println!("  • Atomics are {:.1}x faster than Mutex under contention", speedup);
    println!("  • High contention caused {} blocking events", high_results.contention_count);
    println!("  • Max latency under contention: {:.3} µs (unpredictable!)", high_results.max_latency_us);
    println!("\n--- Impact on Real-Time Systems ---");
    println!("  • High contention causes UNPREDICTABLE latency spikes");
    println!("  • Use Atomics for status flags and counters");
    println!("  • Use RwLock for config (read often, write rarely)");
}

fn run_high_contention_test(iterations: usize, num_threads: usize) -> ContentionResults {
    use parking_lot::Mutex;
    
    let shared_data = Arc::new(Mutex::new(0u64));
    let shared_latencies = Arc::new(Mutex::new(Vec::with_capacity(iterations * num_threads)));
    let contention_counter = Arc::new(AtomicU64::new(0));
    
    let start = Instant::now();
    
    let handles: Vec<_> = (0..num_threads)
        .map(|_| {
            let data = Arc::clone(&shared_data);
            let latencies = Arc::clone(&shared_latencies);
            let contentions = Arc::clone(&contention_counter);
            
            thread::spawn(move || {
                for _ in 0..iterations {
                    let op_start = Instant::now();
                    {
                        let mut guard = data.lock();
                        *guard += 1;
                        std::hint::black_box(*guard);
                    }
                    let elapsed = op_start.elapsed().as_nanos() as u64;
                    
                    if elapsed > 1000 {
                        contentions.fetch_add(1, Ordering::Relaxed);
                    }
                    latencies.lock().push(elapsed);
                }
            })
        })
        .collect();
    
    for h in handles { h.join().unwrap(); }
    
    let total_time = start.elapsed();
    let latencies = shared_latencies.lock().clone();
    calculate_contention_results(&latencies, total_time, contention_counter.load(Ordering::Relaxed) as usize)
}

fn run_low_contention_atomic_test(iterations: usize, num_threads: usize) -> ContentionResults {
    use parking_lot::Mutex;
    
    let shared_data = Arc::new(AtomicU64::new(0));
    let shared_latencies = Arc::new(Mutex::new(Vec::with_capacity(iterations * num_threads)));
    
    let start = Instant::now();
    
    let handles: Vec<_> = (0..num_threads)
        .map(|_| {
            let data = Arc::clone(&shared_data);
            let latencies = Arc::clone(&shared_latencies);
            
            thread::spawn(move || {
                for _ in 0..iterations {
                    let op_start = Instant::now();
                    let val = data.fetch_add(1, Ordering::SeqCst);
                    std::hint::black_box(val);
                    latencies.lock().push(op_start.elapsed().as_nanos() as u64);
                }
            })
        })
        .collect();
    
    for h in handles { h.join().unwrap(); }
    
    let total_time = start.elapsed();
    let latencies = shared_latencies.lock().clone();
    calculate_contention_results(&latencies, total_time, 0)
}

fn run_rwlock_contention_test(iterations: usize, num_threads: usize) -> ContentionResults {
    use parking_lot::{RwLock, Mutex};
    
    let shared_data = Arc::new(RwLock::new(0u64));
    let shared_latencies = Arc::new(Mutex::new(Vec::with_capacity(iterations * num_threads)));
    let contention_counter = Arc::new(AtomicU64::new(0));
    
    let start = Instant::now();
    
    let handles: Vec<_> = (0..num_threads)
        .map(|thread_id| {
            let data = Arc::clone(&shared_data);
            let latencies = Arc::clone(&shared_latencies);
            let contentions = Arc::clone(&contention_counter);
            
            thread::spawn(move || {
                for i in 0..iterations {
                    let op_start = Instant::now();
                    
                    if (i + thread_id) % 10 == 0 {
                        let mut guard = data.write();
                        *guard += 1;
                        std::hint::black_box(*guard);
                    } else {
                        let guard = data.read();
                        std::hint::black_box(*guard);
                    }
                    
                    let elapsed = op_start.elapsed().as_nanos() as u64;
                    if elapsed > 1000 {
                        contentions.fetch_add(1, Ordering::Relaxed);
                    }
                    latencies.lock().push(elapsed);
                }
            })
        })
        .collect();
    
    for h in handles { h.join().unwrap(); }
    
    let total_time = start.elapsed();
    let latencies = shared_latencies.lock().clone();
    calculate_contention_results(&latencies, total_time, contention_counter.load(Ordering::Relaxed) as usize)
}

fn calculate_contention_results(latencies: &[u64], total_time: Duration, contentions: usize) -> ContentionResults {
    let mut sorted = latencies.to_vec();
    sorted.sort_unstable();
    
    let count = sorted.len();
    let sum: u64 = sorted.iter().sum();
    let p99_idx = (count as f64 * 0.99) as usize;
    
    ContentionResults {
        avg_latency_us: (sum as f64 / count as f64) / 1000.0,
        max_latency_us: sorted[count - 1] as f64 / 1000.0,
        p99_latency_us: sorted[p99_idx.min(count - 1)] as f64 / 1000.0,
        throughput: count as f64 / total_time.as_secs_f64(),
        contention_count: contentions,
    }
}

// ============================================================================
// COMPARISON 3: SYNCHRONIZATION PRIMITIVES
// ============================================================================

fn run_sync_primitives_comparison() {
    println!("Comparing synchronization primitives (single-threaded baseline)...\n");
    
    let iterations = 50000;
    
    println!("Running std::sync::Mutex benchmark...");
    let std_mutex = bench_std_mutex(iterations);
    
    println!("Running parking_lot::Mutex benchmark...");
    let pl_mutex = bench_parking_lot_mutex(iterations);
    
    println!("Running AtomicU64 benchmark...");
    let atomic = bench_atomic(iterations);
    
    println!("Running RwLock (read) benchmark...");
    let rwlock_read = bench_rwlock_read(iterations);
    
    println!("Running RwLock (write) benchmark...");
    let rwlock_write = bench_rwlock_write(iterations);
    
    println!("\n╔═══════════════════════════════════════════════════════════════════════════════════╗");
    println!("║                    SYNCHRONIZATION PRIMITIVES COMPARISON                          ║");
    println!("║                           ({} iterations, single-threaded)                       ║", iterations);
    println!("╠═══════════════════════════════════════════════════════════════════════════════════╣");
    println!("║ {:^20} │ {:^12} │ {:^12} │ {:^12} │ {:^12} ║", 
        "Primitive", "Avg (ns)", "Min (ns)", "Max (ns)", "Throughput");
    println!("╠═══════════════════════════════════════════════════════════════════════════════════╣");
    println!("║ {:^20} │ {:>12.1} │ {:>12} │ {:>12} │ {:>10.0}/s ║",
        "std::sync::Mutex", std_mutex.0, std_mutex.1, std_mutex.2, std_mutex.3);
    println!("║ {:^20} │ {:>12.1} │ {:>12} │ {:>12} │ {:>10.0}/s ║",
        "parking_lot::Mutex", pl_mutex.0, pl_mutex.1, pl_mutex.2, pl_mutex.3);
    println!("║ {:^20} │ {:>12.1} │ {:>12} │ {:>12} │ {:>10.0}/s ║",
        "AtomicU64", atomic.0, atomic.1, atomic.2, atomic.3);
    println!("║ {:^20} │ {:>12.1} │ {:>12} │ {:>12} │ {:>10.0}/s ║",
        "RwLock (read)", rwlock_read.0, rwlock_read.1, rwlock_read.2, rwlock_read.3);
    println!("║ {:^20} │ {:>12.1} │ {:>12} │ {:>12} │ {:>10.0}/s ║",
        "RwLock (write)", rwlock_write.0, rwlock_write.1, rwlock_write.2, rwlock_write.3);
    println!("╚═══════════════════════════════════════════════════════════════════════════════════╝");
    
    println!("\n--- Analysis ---");
    println!("  • parking_lot::Mutex is {:.1}x faster than std::sync::Mutex", std_mutex.0 / pl_mutex.0);
    println!("  • AtomicU64 is {:.1}x faster than parking_lot::Mutex", pl_mutex.0 / atomic.0);
    println!("  • RwLock reads are {:.1}x faster than writes", rwlock_write.0 / rwlock_read.0);
}

fn bench_std_mutex(iterations: usize) -> (f64, u64, u64, f64) {
    use std::sync::Mutex;
    let data = Mutex::new(0u64);
    let mut latencies = Vec::with_capacity(iterations);
    let start = Instant::now();
    
    for _ in 0..iterations {
        let op_start = Instant::now();
        { let mut g = data.lock().unwrap(); *g += 1; std::hint::black_box(*g); }
        latencies.push(op_start.elapsed().as_nanos() as u64);
    }
    
    let total = start.elapsed();
    let sum: u64 = latencies.iter().sum();
    (sum as f64 / iterations as f64, *latencies.iter().min().unwrap(), *latencies.iter().max().unwrap(), iterations as f64 / total.as_secs_f64())
}

fn bench_parking_lot_mutex(iterations: usize) -> (f64, u64, u64, f64) {
    use parking_lot::Mutex;
    let data = Mutex::new(0u64);
    let mut latencies = Vec::with_capacity(iterations);
    let start = Instant::now();
    
    for _ in 0..iterations {
        let op_start = Instant::now();
        { let mut g = data.lock(); *g += 1; std::hint::black_box(*g); }
        latencies.push(op_start.elapsed().as_nanos() as u64);
    }
    
    let total = start.elapsed();
    let sum: u64 = latencies.iter().sum();
    (sum as f64 / iterations as f64, *latencies.iter().min().unwrap(), *latencies.iter().max().unwrap(), iterations as f64 / total.as_secs_f64())
}

fn bench_atomic(iterations: usize) -> (f64, u64, u64, f64) {
    let data = AtomicU64::new(0);
    let mut latencies = Vec::with_capacity(iterations);
    let start = Instant::now();
    
    for _ in 0..iterations {
        let op_start = Instant::now();
        let val = data.fetch_add(1, Ordering::SeqCst);
        std::hint::black_box(val);
        latencies.push(op_start.elapsed().as_nanos() as u64);
    }
    
    let total = start.elapsed();
    let sum: u64 = latencies.iter().sum();
    (sum as f64 / iterations as f64, *latencies.iter().min().unwrap(), *latencies.iter().max().unwrap(), iterations as f64 / total.as_secs_f64())
}

fn bench_rwlock_read(iterations: usize) -> (f64, u64, u64, f64) {
    use parking_lot::RwLock;
    let data = RwLock::new(0u64);
    let mut latencies = Vec::with_capacity(iterations);
    let start = Instant::now();
    
    for _ in 0..iterations {
        let op_start = Instant::now();
        { let g = data.read(); std::hint::black_box(*g); }
        latencies.push(op_start.elapsed().as_nanos() as u64);
    }
    
    let total = start.elapsed();
    let sum: u64 = latencies.iter().sum();
    (sum as f64 / iterations as f64, *latencies.iter().min().unwrap(), *latencies.iter().max().unwrap(), iterations as f64 / total.as_secs_f64())
}

fn bench_rwlock_write(iterations: usize) -> (f64, u64, u64, f64) {
    use parking_lot::RwLock;
    let data = RwLock::new(0u64);
    let mut latencies = Vec::with_capacity(iterations);
    let start = Instant::now();
    
    for _ in 0..iterations {
        let op_start = Instant::now();
        { let mut g = data.write(); *g += 1; std::hint::black_box(*g); }
        latencies.push(op_start.elapsed().as_nanos() as u64);
    }
    
    let total = start.elapsed();
    let sum: u64 = latencies.iter().sum();
    (sum as f64 / iterations as f64, *latencies.iter().min().unwrap(), *latencies.iter().max().unwrap(), iterations as f64 / total.as_secs_f64())
}

// ============================================================================
// COMPARISON 4: CHANNEL TYPES
// ============================================================================

fn run_channel_comparison() {
    println!("Comparing channel implementations...\n");
    
    let iterations = 50000;
    
    println!("Running std::sync::mpsc (bounded) benchmark...");
    let std_bounded = bench_std_channel_bounded(iterations);
    
    println!("Running std::sync::mpsc (unbounded) benchmark...");
    let std_unbounded = bench_std_channel_unbounded(iterations);
    
    println!("Running crossbeam (bounded) benchmark...");
    let cb_bounded = bench_crossbeam_bounded(iterations);
    
    println!("Running crossbeam (unbounded) benchmark...");
    let cb_unbounded = bench_crossbeam_unbounded(iterations);
    
    println!("\n╔═══════════════════════════════════════════════════════════════════════════════════╗");
    println!("║                         CHANNEL TYPES COMPARISON                                  ║");
    println!("║                           ({} send+recv operations)                              ║", iterations);
    println!("╠═══════════════════════════════════════════════════════════════════════════════════╣");
    println!("║ {:^25} │ {:^12} │ {:^12} │ {:^12} │ {:^12} ║", 
        "Channel Type", "Avg (ns)", "Min (ns)", "Max (ns)", "Throughput");
    println!("╠═══════════════════════════════════════════════════════════════════════════════════╣");
    println!("║ {:^25} │ {:>12.1} │ {:>12} │ {:>12} │ {:>10.0}/s ║",
        "std::mpsc (bounded)", std_bounded.0, std_bounded.1, std_bounded.2, std_bounded.3);
    println!("║ {:^25} │ {:>12.1} │ {:>12} │ {:>12} │ {:>10.0}/s ║",
        "std::mpsc (unbounded)", std_unbounded.0, std_unbounded.1, std_unbounded.2, std_unbounded.3);
    println!("║ {:^25} │ {:>12.1} │ {:>12} │ {:>12} │ {:>10.0}/s ║",
        "crossbeam (bounded)", cb_bounded.0, cb_bounded.1, cb_bounded.2, cb_bounded.3);
    println!("║ {:^25} │ {:>12.1} │ {:>12} │ {:>12} │ {:>10.0}/s ║",
        "crossbeam (unbounded)", cb_unbounded.0, cb_unbounded.1, cb_unbounded.2, cb_unbounded.3);
    println!("╚═══════════════════════════════════════════════════════════════════════════════════╝");
    
    println!("\n--- Analysis ---");
    println!("  • crossbeam bounded is {:.1}x faster than std bounded", std_bounded.0 / cb_bounded.0);
    println!("  • Bounded channels have more predictable max latency (better for RT)");
    println!("\n--- Recommendations ---");
    println!("  • Use crossbeam::channel for best performance");
    println!("  • Prefer BOUNDED channels to prevent unbounded memory growth");
}

fn bench_std_channel_bounded(iterations: usize) -> (f64, u64, u64, f64) {
    use std::sync::mpsc;
    let (tx, rx) = mpsc::sync_channel::<u64>(100);
    let mut latencies = Vec::with_capacity(iterations);
    let start = Instant::now();
    
    for i in 0..iterations {
        let op_start = Instant::now();
        tx.send(i as u64).unwrap();
        let _ = rx.recv().unwrap();
        latencies.push(op_start.elapsed().as_nanos() as u64);
    }
    
    let total = start.elapsed();
    let sum: u64 = latencies.iter().sum();
    (sum as f64 / iterations as f64, *latencies.iter().min().unwrap(), *latencies.iter().max().unwrap(), iterations as f64 / total.as_secs_f64())
}

fn bench_std_channel_unbounded(iterations: usize) -> (f64, u64, u64, f64) {
    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<u64>();
    let mut latencies = Vec::with_capacity(iterations);
    let start = Instant::now();
    
    for i in 0..iterations {
        let op_start = Instant::now();
        tx.send(i as u64).unwrap();
        let _ = rx.recv().unwrap();
        latencies.push(op_start.elapsed().as_nanos() as u64);
    }
    
    let total = start.elapsed();
    let sum: u64 = latencies.iter().sum();
    (sum as f64 / iterations as f64, *latencies.iter().min().unwrap(), *latencies.iter().max().unwrap(), iterations as f64 / total.as_secs_f64())
}

fn bench_crossbeam_bounded(iterations: usize) -> (f64, u64, u64, f64) {
    let (tx, rx) = crossbeam_channel::bounded::<u64>(100);
    let mut latencies = Vec::with_capacity(iterations);
    let start = Instant::now();
    
    for i in 0..iterations {
        let op_start = Instant::now();
        tx.send(i as u64).unwrap();
        let _ = rx.recv().unwrap();
        latencies.push(op_start.elapsed().as_nanos() as u64);
    }
    
    let total = start.elapsed();
    let sum: u64 = latencies.iter().sum();
    (sum as f64 / iterations as f64, *latencies.iter().min().unwrap(), *latencies.iter().max().unwrap(), iterations as f64 / total.as_secs_f64())
}

fn bench_crossbeam_unbounded(iterations: usize) -> (f64, u64, u64, f64) {
    let (tx, rx) = crossbeam_channel::unbounded::<u64>();
    let mut latencies = Vec::with_capacity(iterations);
    let start = Instant::now();
    
    for i in 0..iterations {
        let op_start = Instant::now();
        tx.send(i as u64).unwrap();
        let _ = rx.recv().unwrap();
        latencies.push(op_start.elapsed().as_nanos() as u64);
    }
    
    let total = start.elapsed();
    let sum: u64 = latencies.iter().sum();
    (sum as f64 / iterations as f64, *latencies.iter().min().unwrap(), *latencies.iter().max().unwrap(), iterations as f64 / total.as_secs_f64())
}

// ----------------------------------------------------------------------------
// Save Benchmark Results
// ----------------------------------------------------------------------------

fn save_benchmark_results() {
    println!("Saving benchmark results to files...");
    
    let mut benchmark = SystemBenchmark::new();
    let iterations = 500;
    
    let mut sensor_sim = rts_manufacturing::sensor::SensorSimulator::new(0, "Test");
    for _ in 0..iterations {
        benchmark.sensor_generation.start();
        let _ = sensor_sim.generate_reading();
        benchmark.sensor_generation.stop();
    }
    
    let mut processor = rts_manufacturing::sensor::DataProcessor::new(NUM_SENSOR_TYPES);
    for i in 0..iterations {
        let reading = rts_manufacturing::types::SensorReading::new(
            0, "Test".to_string(), 50.0 + (i as f64 * 0.1), i as u64
        );
        benchmark.data_processing.start();
        let _ = processor.process(&reading);
        benchmark.data_processing.stop();
    }
    
    let mut pid = rts_manufacturing::pid_controller::PidController::with_defaults("Test");
    pid.set_setpoint(50.0);
    for i in 0..iterations {
        benchmark.pid_control.start();
        let _ = pid.update(45.0 + (i as f64 * 0.01));
        benchmark.pid_control.stop();
    }
    
    let (tx, rx) = crossbeam_channel::bounded::<u64>(100);
    for i in 0..iterations {
        benchmark.data_transmission.start();
        let _ = tx.send(i as u64);
        benchmark.data_transmission.stop();
        benchmark.actuator_reception.start();
        let _ = rx.recv();
        benchmark.actuator_reception.stop();
    }
    
    let json = benchmark.export_json();
    match File::create("benchmark_results.json") {
        Ok(mut file) => {
            if let Err(e) = writeln!(file, "{}", json) {
                eprintln!("Failed to write benchmark JSON: {}", e);
            } else {
                println!("  ✓ Saved benchmark_results.json");
            }
        }
        Err(e) => eprintln!("Failed to create benchmark JSON file: {}", e),
    }
    
    save_timing_log(&benchmark);
}

fn save_timing_log(benchmark: &SystemBenchmark) {
    let mut log = String::new();
    
    log.push_str("============================================================\n");
    log.push_str("  RTS Manufacturing System - Timing Log\n");
    log.push_str("============================================================\n");
    log.push_str(&format!("Generated at: {:?}\n\n", std::time::SystemTime::now()));
    
    let stats = benchmark.sensor_generation.get_stats();
    log.push_str("=== Sensor Generation ===\n");
    log.push_str(&format!("  Iterations: {}\n", stats.count));
    log.push_str(&format!("  Min: {:.3} µs, Max: {:.3} µs, Mean: {:.3} µs\n\n", 
        stats.min_ns as f64 / 1000.0, stats.max_ns as f64 / 1000.0, stats.mean_ns / 1000.0));
    
    let stats = benchmark.data_processing.get_stats();
    log.push_str("=== Data Processing ===\n");
    log.push_str(&format!("  Iterations: {}\n", stats.count));
    log.push_str(&format!("  Min: {:.3} µs, Max: {:.3} µs, Mean: {:.3} µs\n", 
        stats.min_ns as f64 / 1000.0, stats.max_ns as f64 / 1000.0, stats.mean_ns / 1000.0));
    log.push_str(&format!("  Deadline: {:.1} µs\n\n", PROCESSING_DEADLINE.as_nanos() as f64 / 1000.0));
    
    log.push_str("============================================================\n");
    
    match File::create("timing_log.txt") {
        Ok(mut file) => {
            if let Err(e) = write!(file, "{}", log) {
                eprintln!("Failed to write timing log: {}", e);
            } else {
                println!("  ✓ Saved timing_log.txt");
            }
        }
        Err(e) => eprintln!("Failed to create timing log file: {}", e),
    }
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
        assert!(output > 0.0);
        assert!((error - 50.0).abs() < 0.001);
    }

    #[test]
    fn test_shared_resource_sync() {
        let shared = SharedResources::new();
        shared.status_memory.increment_cycles();
        assert_eq!(shared.status_memory.get_cycles(), 1);
        
        shared.diagnostic_log.log(LogLevel::Info, "Test", "Test message");
        let recent = shared.diagnostic_log.get_recent(1);
        assert_eq!(recent.len(), 1);
    }

    #[test]
    fn test_failsafe_transitions() {
        let mut failsafe = FailSafeManager::new();
        assert!(failsafe.is_normal());
        
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