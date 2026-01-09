//! # Criterion Benchmarks for RTS V2 (Optimized)
//!
//! These benchmarks test the ACTUAL V2 system modules - the same code that
//! main.rs uses. This ensures benchmarks reflect real system performance.
//!
//! Run with: cargo bench
//! View reports in: target/criterion/report/index.html

use criterion::{criterion_group, criterion_main, Criterion, Throughput};
use std::hint::black_box;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

// Import ACTUAL V2 modules (same as main.rs)
use rts_manufacturing::sensor::{SensorModule, SensorSimulator, DataProcessor};
use rts_manufacturing::actuator::ActuatorModule;
use rts_manufacturing::pid_controller::PidController;
use rts_manufacturing::shared_resource::*;
use rts_manufacturing::ipc::*;
use rts_manufacturing::config::*;
use rts_manufacturing::types::*;

// ============================================================================
// ACTUAL SYSTEM BENCHMARKS
// ============================================================================
// These benchmark the EXACT same code paths that main.rs uses

/// Benchmark the ACTUAL SensorModule::run_cycle() - same as main.rs
fn bench_sensor_module_run_cycle(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_actual_system");

    group.bench_function("sensor_module_run_cycle", |b| {
        // Setup exactly like main.rs does
        let shared = SharedResources::new();
        let ipc = IpcManager::new();
        let running = Arc::new(AtomicBool::new(true));

        let sender = ipc.get_sensor_sender();
        let feedback_receiver = ipc.get_feedback_receiver();

        let mut sensor_module = SensorModule::new(
            sender,
            feedback_receiver,
            shared,
            running,
        );

        b.iter(|| {
            // Benchmark the ACTUAL run_cycle - same as main.rs calls
            black_box(sensor_module.run_cycle())
        })
    });

    group.finish();
}

/// Benchmark the ACTUAL ActuatorModule::run_cycle() - same as main.rs
fn bench_actuator_module_run_cycle(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_actual_system");

    group.bench_function("actuator_module_run_cycle", |b| {
        // Setup exactly like main.rs does
        let shared = SharedResources::new();
        let ipc = IpcManager::new();
        let running = Arc::new(AtomicBool::new(true));

        let sensor_receiver = ipc.get_sensor_receiver();
        let feedback_sender = ipc.get_feedback_sender();
        let sensor_sender = ipc.get_sensor_sender();

        // Pre-populate channel with test data so actuator has data to process
        for i in 0..10 {
            let data = ProcessedSensorData::new(
                0, "Force".to_string(), 50.0, 50.0 + (i as f64 * 0.1), 
                false, 1.0, 1000, i as u64
            );
            let _ = sensor_sender.try_send(data);
        }

        let mut actuator_module = ActuatorModule::new(
            sensor_receiver,
            feedback_sender,
            shared,
            running,
        );

        b.iter(|| {
            // Benchmark the ACTUAL run_cycle - same as main.rs calls
            black_box(actuator_module.run_cycle())
        })
    });

    group.finish();
}

/// Benchmark COMPLETE end-to-end cycle - sensor → channel → actuator
fn bench_complete_system_cycle(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_actual_system");
    group.throughput(Throughput::Elements(1));

    group.bench_function("complete_end_to_end_cycle", |b| {
        // Setup EXACTLY like main.rs
        let shared = SharedResources::new();
        let ipc = IpcManager::new();
        let running = Arc::new(AtomicBool::new(true));

        let sensor_sender = ipc.get_sensor_sender();
        let sensor_receiver = ipc.get_sensor_receiver();
        let feedback_sender = ipc.get_feedback_sender();
        let feedback_receiver = ipc.get_feedback_receiver();

        let mut sensor_module = SensorModule::new(
            sensor_sender,
            feedback_receiver,
            shared.clone(),
            Arc::clone(&running),
        );

        let mut actuator_module = ActuatorModule::new(
            sensor_receiver,
            feedback_sender,
            shared.clone(),
            Arc::clone(&running),
        );

        b.iter(|| {
            // 1. Sensor generates and sends data (ACTUAL run_cycle)
            let _ = sensor_module.run_cycle();

            // 2. Actuator receives and processes (ACTUAL run_cycle)
            let result = actuator_module.run_cycle();

            black_box(result)
        })
    });

    group.finish();
}

// ============================================================================
// COMPONENT TIMING BENCHMARKS
// ============================================================================
// Individual component timing to verify deadlines

/// Benchmark sensor generation (0.2ms deadline check)
fn bench_sensor_generation(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    group.bench_function("sensor_generation", |b| {
        let mut sensor = SensorSimulator::new(SENSOR_FORCE, "Force");

        b.iter(|| {
            black_box(sensor.generate_reading())
        })
    });

    group.finish();
}

/// Benchmark data processing with filtering (0.2ms deadline)
fn bench_data_processing(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    group.bench_function("data_processing_filter", |b| {
        let mut processor = DataProcessor::new(NUM_SENSOR_TYPES);
        let mut sensor = SensorSimulator::new(SENSOR_FORCE, "Force");

        b.iter(|| {
            let reading = sensor.generate_reading();
            black_box(processor.process(&reading))
        })
    });

    group.finish();
}

/// Benchmark PID controller update (part of actuator 1-2ms deadline)
fn bench_pid_update(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    group.bench_function("pid_update", |b| {
        let mut pid = PidController::with_defaults("Test");
        pid.set_setpoint(50.0);

        b.iter(|| {
            black_box(pid.update_with_dt(black_box(45.0), black_box(0.001)))
        })
    });

    group.finish();
}

/// Benchmark channel transmission (0.1ms deadline)
fn bench_channel_transmission(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    // Test actual SensorDataChannel (crossbeam bounded)
    group.bench_function("sensor_data_channel", |b| {
        let channel = SensorDataChannel::new(CHANNEL_BUFFER_SIZE);
        let sender = channel.get_sender();
        let receiver = channel.get_receiver();

        let data = ProcessedSensorData::new(
            0, "Force".to_string(), 50.0, 50.0, false, 1.0, 1000, 1
        );

        b.iter(|| {
            let _ = sender.try_send(data.clone());
            black_box(receiver.try_recv())
        })
    });

    // Test actual FeedbackChannel
    group.bench_function("feedback_channel", |b| {
        let channel = FeedbackChannel::new(CHANNEL_BUFFER_SIZE);
        let sender = channel.get_sender();
        let receiver = channel.get_receiver();

        let feedback = ActuatorFeedback::new(0, 1, ActuatorState::new(0));

        b.iter(|| {
            let _ = sender.try_send(feedback.clone());
            black_box(receiver.try_recv())
        })
    });

    group.finish();
}

/// Benchmark shared resource operations
fn bench_shared_resources(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_component_timing");

    // Atomic counter (what V2 uses for counters)
    group.bench_function("status_memory_increment", |b| {
        let status = StatusMemory::new();

        b.iter(|| {
            black_box(status.increment_cycles())
        })
    });

    // RwLock config read (what V2 uses for config)
    group.bench_function("config_buffer_read", |b| {
        let config = ConfigBuffer::new();

        b.iter(|| {
            black_box(config.read())
        })
    });

    // parking_lot::Mutex log write
    group.bench_function("diagnostic_log_write", |b| {
        let log = DiagnosticLog::new(1000);  // Use direct value

        b.iter(|| {
            log.try_log(LogLevel::Info, "Bench", "Test message")
        })
    });

    group.finish();
}

// ============================================================================
// SYNCHRONIZATION COMPARISON BENCHMARKS
// ============================================================================
// Compare V2's optimized primitives vs alternatives

fn bench_sync_primitives(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_sync_comparison");
    group.throughput(Throughput::Elements(10000));

    // What V2 uses: parking_lot::Mutex
    group.bench_function("parking_lot_mutex", |b| {
        let mutex = parking_lot::Mutex::new(0u64);

        b.iter(|| {
            for _ in 0..10000 {
                let mut guard = mutex.lock();
                *guard += 1;
                black_box(*guard);
            }
        })
    });

    // What V2 uses: AtomicU64 for counters
    group.bench_function("atomic_u64", |b| {
        let atomic = std::sync::atomic::AtomicU64::new(0);

        b.iter(|| {
            for _ in 0..10000 {
                let val = atomic.fetch_add(1, Ordering::SeqCst);
                black_box(val);
            }
        })
    });

    // What V2 uses: parking_lot::RwLock for config
    group.bench_function("rwlock_read", |b| {
        let rwlock = parking_lot::RwLock::new(0u64);

        b.iter(|| {
            for _ in 0..10000 {
                let guard = rwlock.read();
                black_box(*guard);
            }
        })
    });

    group.finish();
}

// ============================================================================
// LOCK CONTENTION BENCHMARKS
// ============================================================================

fn bench_lock_contention(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_lock_contention");
    group.throughput(Throughput::Elements(10000));

    // V2 uses parking_lot::Mutex
    group.bench_function("parking_lot_mutex_4threads", |b| {
        use std::thread;

        b.iter(|| {
            let counter = Arc::new(parking_lot::Mutex::new(0u64));
            let iterations_per_thread = 2500;

            let handles: Vec<_> = (0..4)
                .map(|_| {
                    let counter = Arc::clone(&counter);
                    thread::spawn(move || {
                        for _ in 0..iterations_per_thread {
                            let mut guard = counter.lock();
                            *guard += 1;
                            black_box(*guard);
                        }
                    })
                })
                .collect();

            for h in handles {
                h.join().unwrap();
            }

            let result = *counter.lock();
            black_box(result)
        })
    });

    // V2 uses AtomicU64 for counters (lock-free)
    group.bench_function("atomic_u64_4threads", |b| {
        use std::thread;
        use std::sync::atomic::AtomicU64;

        b.iter(|| {
            let counter = Arc::new(AtomicU64::new(0));
            let iterations_per_thread = 2500;

            let handles: Vec<_> = (0..4)
                .map(|_| {
                    let counter = Arc::clone(&counter);
                    thread::spawn(move || {
                        for _ in 0..iterations_per_thread {
                            let val = counter.fetch_add(1, Ordering::SeqCst);
                            black_box(val);
                        }
                    })
                })
                .collect();

            for h in handles {
                h.join().unwrap();
            }

            black_box(counter.load(Ordering::SeqCst))
        })
    });

    group.finish();
}

// ============================================================================
// CPU LOAD IMPACT BENCHMARKS
// ============================================================================

fn simulate_cpu_load(load_percent: f64, duration_us: u64) {
    if load_percent <= 0.0 {
        return;
    }

    let busy_time = (duration_us as f64 * load_percent) as u64;
    let start = Instant::now();

    let mut x: u64 = 0;
    while start.elapsed().as_micros() < busy_time as u128 {
        x = x.wrapping_add(1);
        black_box(x);
    }
}

fn bench_cpu_load_impact(c: &mut Criterion) {
    let mut group = c.benchmark_group("v2_cpu_load_impact");
    group.throughput(Throughput::Elements(100));

    // 0% load (baseline)
    group.bench_function("load_0_percent", |b| {
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
            shared,
            running,
        );

        b.iter(|| {
            for _ in 0..100 {
                simulate_cpu_load(0.0, 100);
                let _ = sensor.run_cycle();
                let _ = actuator.run_cycle();
            }
        })
    });

    // 30% load
    group.bench_function("load_30_percent", |b| {
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
            shared,
            running,
        );

        b.iter(|| {
            for _ in 0..100 {
                simulate_cpu_load(0.3, 100);
                let _ = sensor.run_cycle();
                let _ = actuator.run_cycle();
            }
        })
    });

    // 60% load
    group.bench_function("load_60_percent", |b| {
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
            shared,
            running,
        );

        b.iter(|| {
            for _ in 0..100 {
                simulate_cpu_load(0.6, 100);
                let _ = sensor.run_cycle();
                let _ = actuator.run_cycle();
            }
        })
    });

    // 80% load
    group.bench_function("load_80_percent", |b| {
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
            shared,
            running,
        );

        b.iter(|| {
            for _ in 0..100 {
                simulate_cpu_load(0.8, 100);
                let _ = sensor.run_cycle();
                let _ = actuator.run_cycle();
            }
        })
    });

    group.finish();
}

// ============================================================================
// ASYNC vs THREADED (Optional - Advanced 80%+)
// ============================================================================

fn bench_async_vs_threaded(c: &mut Criterion) {
    let mut group = c.benchmark_group("async_vs_threaded");
    group.throughput(Throughput::Elements(1000));

    // Multi-threaded (what V2 uses in main.rs)
    group.bench_function("threaded_std_thread", |b| {
        use std::thread;

        b.iter(|| {
            let (tx, rx) = crossbeam_channel::bounded::<f64>(100);
            let iterations = 1000u64;

            let sensor_handle = thread::spawn(move || {
                // Simple work without non-Send types
                for i in 0..iterations {
                    let value = 50.0 + (i as f64 * 0.1).sin() * 5.0;
                    let _ = tx.send(value);
                }
            });

            let actuator_handle = thread::spawn(move || {
                let mut pid = PidController::with_defaults("Test");
                pid.set_setpoint(50.0);
                let mut sum = 0.0;
                for _ in 0..iterations {
                    if let Ok(val) = rx.recv() {
                        let (output, _, _) = pid.update(val);
                        sum += output;
                    }
                }
                sum
            });

            sensor_handle.join().unwrap();
            black_box(actuator_handle.join().unwrap())
        })
    });

    // Async implementation
    group.bench_function("async_tokio", |b| {
        let rt = tokio::runtime::Runtime::new().unwrap();

        b.iter(|| {
            rt.block_on(async {
                let (tx, mut rx) = tokio::sync::mpsc::channel::<f64>(100);
                let iterations = 1000u64;

                let sensor_task = tokio::spawn(async move {
                    // Simple work without non-Send types
                    for i in 0..iterations {
                        let value = 50.0 + (i as f64 * 0.1).sin() * 5.0;
                        let _ = tx.send(value).await;
                    }
                });

                let actuator_task = tokio::spawn(async move {
                    let mut pid = PidController::with_defaults("Test");
                    pid.set_setpoint(50.0);
                    let mut sum = 0.0;
                    while let Some(val) = rx.recv().await {
                        let (output, _, _) = pid.update(val);
                        sum += output;
                    }
                    sum
                });

                let _ = sensor_task.await;
                black_box(actuator_task.await.unwrap())
            })
        })
    });

    group.finish();
}

// ============================================================================
// CRITERION GROUPS
// ============================================================================

// ACTUAL SYSTEM benchmarks (tests real code from main.rs)
criterion_group!(
    actual_system_benches,
    bench_sensor_module_run_cycle,
    bench_actuator_module_run_cycle,
    bench_complete_system_cycle
);

// Component timing benchmarks
criterion_group!(
    component_benches,
    bench_sensor_generation,
    bench_data_processing,
    bench_pid_update,
    bench_channel_transmission,
    bench_shared_resources
);

// Sync comparison benchmarks
criterion_group!(
    sync_benches,
    bench_sync_primitives,
    bench_lock_contention
);

// Load impact benchmarks
criterion_group!(
    load_benches,
    bench_cpu_load_impact
);

// Optional async comparison
criterion_group!(
    optional_benches,
    bench_async_vs_threaded
);

criterion_main!(
    actual_system_benches,
    component_benches,
    sync_benches,
    load_benches,
    optional_benches
);
