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
);

criterion_main!(benches);
