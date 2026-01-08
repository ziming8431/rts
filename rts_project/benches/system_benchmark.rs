// ============================================================================
// System Benchmarks using Criterion
// ============================================================================
// Run with: cargo bench
// ============================================================================

use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use rts_manufacturing::config::*;
use rts_manufacturing::pid_controller::*;
use rts_manufacturing::sensor::*;
use rts_manufacturing::types::*;
use rts_manufacturing::shared_resource::*;
use rts_manufacturing::ipc::*;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

/// Benchmark sensor data generation
fn bench_sensor_generation(c: &mut Criterion) {
    let mut sensor = SensorSimulator::new(0, "Force");
    
    c.bench_function("sensor_generation", |b| {
        b.iter(|| {
            black_box(sensor.generate_reading())
        })
    });
}

/// Benchmark data processing with moving average filter
fn bench_data_processing(c: &mut Criterion) {
    let mut processor = DataProcessor::new(NUM_SENSOR_TYPES);
    let reading = SensorReading::new(0, "Force".to_string(), 50.0, 1);
    
    c.bench_function("data_processing", |b| {
        b.iter(|| {
            black_box(processor.process(&reading))
        })
    });
}

/// Benchmark PID controller update
fn bench_pid_control(c: &mut Criterion) {
    let mut pid = PidController::with_defaults("Test");
    pid.set_setpoint(100.0);
    
    c.bench_function("pid_control_update", |b| {
        b.iter(|| {
            black_box(pid.update_with_dt(black_box(50.0), black_box(0.001)))
        })
    });
}

/// Benchmark channel send/receive operations
fn bench_channel_operations(c: &mut Criterion) {
    let channel = SensorDataChannel::new(100);
    let sender = channel.get_sender();
    let receiver = channel.get_receiver();
    
    let data = ProcessedSensorData::new(
        0, "Test".to_string(), 50.0, 50.0, false, 1.0, 100, 1
    );
    
    c.bench_function("channel_send", |b| {
        b.iter(|| {
            let _ = sender.try_send(black_box(data.clone()));
            let _ = receiver.try_recv();
        })
    });
}

/// Benchmark mutex lock acquisition
fn bench_mutex_lock(c: &mut Criterion) {
    let log = DiagnosticLog::new(100);
    
    c.bench_function("mutex_lock_log", |b| {
        b.iter(|| {
            log.try_log(
                black_box(LogLevel::Info),
                black_box("Bench"),
                black_box("Test message"),
            )
        })
    });
}

/// Benchmark RwLock read operations
fn bench_rwlock_read(c: &mut Criterion) {
    let config = ConfigBuffer::new();
    
    c.bench_function("rwlock_read", |b| {
        b.iter(|| {
            black_box(config.read())
        })
    });
}

/// Benchmark atomic operations
fn bench_atomic_operations(c: &mut Criterion) {
    let status = StatusMemory::new();
    
    c.bench_function("atomic_increment", |b| {
        b.iter(|| {
            black_box(status.increment_cycles())
        })
    });
}

/// Benchmark complete sensor cycle
fn bench_sensor_cycle(c: &mut Criterion) {
    let shared = SharedResources::new();
    let channel = SensorDataChannel::new(100);
    let feedback_channel = FeedbackChannel::new(100);
    let running = Arc::new(AtomicBool::new(true));
    
    let mut sensor = SensorModule::new(
        channel.get_sender(),
        feedback_channel.get_receiver(),
        shared,
        running,
    );
    
    c.bench_function("complete_sensor_cycle", |b| {
        b.iter(|| {
            let _ = black_box(sensor.run_cycle());
            // Drain channel
            while channel.get_receiver().try_recv().is_ok() {}
        })
    });
}

/// Benchmark with varying data sizes
fn bench_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("scaling");
    
    for size in [10, 100, 1000].iter() {
        group.bench_with_input(
            BenchmarkId::new("moving_average_buffer", size),
            size,
            |b, &size| {
                let mut processor = DataProcessor::new(1);
                let reading = SensorReading::new(0, "Test".to_string(), 50.0, 1);
                
                // Fill buffer
                for _ in 0..size {
                    processor.process(&reading);
                }
                
                b.iter(|| {
                    black_box(processor.process(&reading))
                })
            },
        );
    }
    
    group.finish();
}

/// Benchmark predictive controller
fn bench_predictive_control(c: &mut Criterion) {
    let mut controller = PredictiveController::new("Test", PID_KP, PID_KI, PID_KD, 3);
    controller.set_setpoint(100.0);
    
    // Fill history
    for i in 0..10 {
        controller.update(50.0 + i as f64);
    }
    
    c.bench_function("predictive_control", |b| {
        b.iter(|| {
            black_box(controller.update(black_box(75.0)))
        })
    });
}

/// Benchmark sync primitive comparison (Mutex vs RwLock vs Atomic)
/// This is a CORE requirement for benchmarking lock contention
fn bench_sync_primitive_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("sync_primitives");
    
    // Mutex benchmark
    let mutex_data = parking_lot::Mutex::new(0u64);
    group.bench_function("mutex_lock_unlock", |b| {
        b.iter(|| {
            let mut guard = mutex_data.lock();
            *guard += 1;
            black_box(*guard)
        })
    });
    
    // RwLock read benchmark
    let rwlock_data = parking_lot::RwLock::new(0u64);
    group.bench_function("rwlock_read", |b| {
        b.iter(|| {
            let guard = rwlock_data.read();
            black_box(*guard)
        })
    });
    
    // RwLock write benchmark
    group.bench_function("rwlock_write", |b| {
        b.iter(|| {
            let mut guard = rwlock_data.write();
            *guard += 1;
            black_box(*guard)
        })
    });
    
    // Atomic benchmark
    let atomic_data = std::sync::atomic::AtomicU64::new(0);
    group.bench_function("atomic_fetch_add", |b| {
        b.iter(|| {
            black_box(atomic_data.fetch_add(1, std::sync::atomic::Ordering::SeqCst))
        })
    });
    
    group.finish();
}

/// Benchmark sensor count scaling (3 vs 6 vs 10 sensors)
/// This is a CORE requirement for normal vs high-load comparison
fn bench_sensor_count_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("sensor_scaling");
    
    for sensor_count in [3, 6, 10].iter() {
        group.bench_with_input(
            BenchmarkId::new("sensor_count", sensor_count),
            sensor_count,
            |b, &count| {
                // Create multiple sensors
                let mut sensors: Vec<SensorSimulator> = (0..count)
                    .map(|i| SensorSimulator::new(i, &format!("Sensor{}", i)))
                    .collect();
                
                let mut processor = DataProcessor::new(count);
                
                b.iter(|| {
                    // Generate and process data from all sensors
                    for sensor in sensors.iter_mut() {
                        let reading = sensor.generate_reading();
                        let _ = processor.process(&reading);
                        black_box(&reading);
                    }
                })
            },
        );
    }
    
    group.finish();
}

/// Benchmark IPC channel buffer size comparison
/// This is a CORE requirement for comparing different IPC mechanisms
fn bench_ipc_channel_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("ipc_channel_size");
    
    // Test different buffer sizes
    for buffer_size in [10, 50, 100, 500].iter() {
        group.bench_with_input(
            BenchmarkId::new("bounded_channel", buffer_size),
            buffer_size,
            |b, &size| {
                let (sender, receiver) = crossbeam_channel::bounded::<u64>(size);
                
                b.iter(|| {
                    // Send and receive to measure round-trip
                    let _ = sender.try_send(black_box(42u64));
                    let _ = receiver.try_recv();
                })
            },
        );
    }
    
    // Compare bounded vs unbounded channel
    let (bounded_tx, bounded_rx) = crossbeam_channel::bounded::<u64>(100);
    group.bench_function("bounded_100", |b| {
        b.iter(|| {
            let _ = bounded_tx.try_send(black_box(42u64));
            let _ = bounded_rx.try_recv();
        })
    });
    
    let (unbounded_tx, unbounded_rx) = crossbeam_channel::unbounded::<u64>();
    group.bench_function("unbounded", |b| {
        b.iter(|| {
            let _ = unbounded_tx.send(black_box(42u64));
            let _ = unbounded_rx.try_recv();
        })
    });
    
    group.finish();
}

criterion_group!(
    benches,
    bench_sensor_generation,
    bench_data_processing,
    bench_pid_control,
    bench_channel_operations,
    bench_mutex_lock,
    bench_rwlock_read,
    bench_atomic_operations,
    bench_sensor_cycle,
    bench_scaling,
    bench_predictive_control,
    bench_sync_primitive_comparison,
    bench_sensor_count_scaling,
    bench_ipc_channel_comparison,
);

criterion_main!(benches);
