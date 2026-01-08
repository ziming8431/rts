// ============================================================================
// CRITERION BENCHMARKS FOR RTS MANUFACTURING SYSTEM
// ============================================================================
// This file contains the REQUIRED comparative benchmarks based on assignment:
//
// REQUIRED:
// 1. Lock Contention Comparison (High vs Low) - "Benchmark lock contention"
// 2. Sync Primitives Comparison - "Compare different concurrency constructs"
// 3. CPU Load Comparison - "Measure performance under varying load conditions"
// 4. Version Comparison (V1 vs V2) - "Compare performance under different designs"
//
// OPTIONAL (Advanced 80%+):
// 5. Async vs Threaded - "Async vs Multi-Threaded Comparison"
//
// Run with: cargo bench
// View reports in: target/criterion/report/index.html
// ============================================================================

use criterion::{criterion_group, criterion_main, Criterion, BenchmarkId, Throughput};
use std::sync::{Arc, Mutex as StdMutex};
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::Instant;
use std::collections::VecDeque;
use std::hint::black_box;

// ============================================================================
// 1. LOCK CONTENTION COMPARISON (REQUIRED)
// ============================================================================
// Requirement: "Benchmark and discuss lock contention and priority inversion effects"
// ============================================================================

fn bench_lock_contention(c: &mut Criterion) {
    let mut group = c.benchmark_group("lock_contention");
    group.throughput(Throughput::Elements(10000));
    
    // HIGH CONTENTION: 4 threads competing for single Mutex
    group.bench_function("high_contention_mutex_4threads", |b| {
        b.iter(|| {
            let counter = Arc::new(parking_lot::Mutex::new(0u64));
            let iterations_per_thread = 2500;
            
            let handles: Vec<_> = (0..4).map(|_| {
                let counter = Arc::clone(&counter);
                thread::spawn(move || {
                    for _ in 0..iterations_per_thread {
                        let mut guard = counter.lock();
                        *guard += 1;
                        black_box(*guard);
                    }
                })
            }).collect();
            
            for h in handles {
                h.join().unwrap();
            }
            
            let result = *counter.lock();
            black_box(result)
        })
    });
    
    // LOW CONTENTION: 4 threads using Atomics (lock-free)
    group.bench_function("low_contention_atomic_4threads", |b| {
        b.iter(|| {
            let counter = Arc::new(AtomicU64::new(0));
            let iterations_per_thread = 2500;
            
            let handles: Vec<_> = (0..4).map(|_| {
                let counter = Arc::clone(&counter);
                thread::spawn(move || {
                    for _ in 0..iterations_per_thread {
                        let val = counter.fetch_add(1, Ordering::SeqCst);
                        black_box(val);
                    }
                })
            }).collect();
            
            for h in handles {
                h.join().unwrap();
            }
            
            black_box(counter.load(Ordering::SeqCst))
        })
    });
    
    // MEDIUM CONTENTION: RwLock with 90% reads, 10% writes
    group.bench_function("medium_contention_rwlock_4threads", |b| {
        b.iter(|| {
            let data = Arc::new(parking_lot::RwLock::new(0u64));
            let iterations_per_thread = 2500;
            
            let handles: Vec<_> = (0..4).map(|tid| {
                let data = Arc::clone(&data);
                thread::spawn(move || {
                    for i in 0..iterations_per_thread {
                        if (i + tid) % 10 == 0 {
                            let mut guard = data.write();
                            *guard += 1;
                            black_box(*guard);
                        } else {
                            let guard = data.read();
                            black_box(*guard);
                        }
                    }
                })
            }).collect();
            
            for h in handles {
                h.join().unwrap();
            }
            
            let result = *data.read();
            black_box(result)
        })
    });
    
    group.finish();
}

// ============================================================================
// 2. SYNCHRONIZATION PRIMITIVES COMPARISON (REQUIRED)
// ============================================================================
// Requirement: "Compare different concurrency constructs where appropriate"
// ============================================================================

fn bench_sync_primitives(c: &mut Criterion) {
    let mut group = c.benchmark_group("sync_primitives");
    group.throughput(Throughput::Elements(10000));
    
    // std::sync::Mutex
    group.bench_function("std_sync_mutex", |b| {
        let mutex = StdMutex::new(0u64);
        b.iter(|| {
            for _ in 0..10000 {
                let mut guard = mutex.lock().unwrap();
                *guard += 1;
                black_box(*guard);
            }
        })
    });
    
    // parking_lot::Mutex
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
    
    // AtomicU64 (lock-free)
    group.bench_function("atomic_u64", |b| {
        let atomic = AtomicU64::new(0);
        b.iter(|| {
            for _ in 0..10000 {
                let val = atomic.fetch_add(1, Ordering::SeqCst);
                black_box(val);
            }
        })
    });
    
    // parking_lot::RwLock (read operations)
    group.bench_function("rwlock_read", |b| {
        let rwlock = parking_lot::RwLock::new(0u64);
        b.iter(|| {
            for _ in 0..10000 {
                let guard = rwlock.read();
                black_box(*guard);
            }
        })
    });
    
    // parking_lot::RwLock (write operations)
    group.bench_function("rwlock_write", |b| {
        let rwlock = parking_lot::RwLock::new(0u64);
        b.iter(|| {
            for _ in 0..10000 {
                let mut guard = rwlock.write();
                *guard += 1;
                black_box(*guard);
            }
        })
    });
    
    group.finish();
}

// ============================================================================
// 3. CPU LOAD COMPARISON (REQUIRED)
// ============================================================================
// Requirement: "Measure and discuss system performance under varying load conditions"
// ============================================================================

/// Simulated sensor-actuator workload
fn do_sensor_actuator_work(iteration: u64) -> f64 {
    let base_value = 50.0;
    let noise = ((iteration as f64 * 0.1).sin() * 5.0) + 
                ((iteration as f64 * 0.7).cos() * 2.0);
    let sensor_value = base_value + noise;
    let filtered = sensor_value * 0.2 + base_value * 0.8;
    
    // PID calculation
    let setpoint = 50.0;
    let error = setpoint - filtered;
    let output = 1.0 * error + 0.1 * error * 0.001 + 0.05 * (error - noise) / 0.001;
    output.max(-100.0).min(100.0)
}

/// Simulate CPU load by doing busy work
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
    let mut group = c.benchmark_group("cpu_load_impact");
    group.throughput(Throughput::Elements(100));
    
    // 0% CPU Load (baseline - normal conditions)
    group.bench_function("load_0_percent", |b| {
        b.iter(|| {
            let mut sum = 0.0;
            for i in 0..100 {
                simulate_cpu_load(0.0, 100);
                sum += do_sensor_actuator_work(i);
            }
            black_box(sum)
        })
    });
    
    // 30% CPU Load
    group.bench_function("load_30_percent", |b| {
        b.iter(|| {
            let mut sum = 0.0;
            for i in 0..100 {
                simulate_cpu_load(0.3, 100);
                sum += do_sensor_actuator_work(i);
            }
            black_box(sum)
        })
    });
    
    // 60% CPU Load
    group.bench_function("load_60_percent", |b| {
        b.iter(|| {
            let mut sum = 0.0;
            for i in 0..100 {
                simulate_cpu_load(0.6, 100);
                sum += do_sensor_actuator_work(i);
            }
            black_box(sum)
        })
    });
    
    // 80% CPU Load (high load conditions)
    group.bench_function("load_80_percent", |b| {
        b.iter(|| {
            let mut sum = 0.0;
            for i in 0..100 {
                simulate_cpu_load(0.8, 100);
                sum += do_sensor_actuator_work(i);
            }
            black_box(sum)
        })
    });
    
    group.finish();
}

// ============================================================================
// 4. VERSION COMPARISON - V1 vs V2 (REQUIRED)
// ============================================================================
// Requirement: "Measure and compare actual performance under different designs"
// ============================================================================

/// V1: Unoptimized design (std::sync::Mutex, no filtering)
mod v1_unoptimized {
    use std::sync::Mutex;
    use std::collections::VecDeque;
    
    pub struct V1Sensor {
        pub value: f64,
    }
    
    impl V1Sensor {
        pub fn new() -> Self {
            Self { value: 50.0 }
        }
        
        pub fn read(&mut self, cycle: u64) -> f64 {
            // V1: Raw value with noise, NO filtering
            let noise = (cycle as f64 * 0.1).sin() * 5.0;
            self.value = 50.0 + noise;
            self.value
        }
    }
    
    pub struct V1SharedState {
        pub counter: Mutex<u64>,
        pub readings: Mutex<VecDeque<f64>>,
    }
    
    impl V1SharedState {
        pub fn new() -> Self {
            Self {
                counter: Mutex::new(0),
                readings: Mutex::new(VecDeque::with_capacity(100)),
            }
        }
        
        pub fn increment(&self) {
            let mut c = self.counter.lock().unwrap();
            *c += 1;
        }
        
        pub fn add_reading(&self, val: f64) {
            let mut r = self.readings.lock().unwrap();
            r.push_back(val);
            if r.len() > 100 {
                r.pop_front();
            }
        }
    }
    
    pub struct V1Pid {
        pub setpoint: f64,
        pub integral: f64,
        pub last_error: f64,
    }
    
    impl V1Pid {
        pub fn new(setpoint: f64) -> Self {
            Self {
                setpoint,
                integral: 0.0,
                last_error: 0.0,
            }
        }
        
        pub fn update(&mut self, measurement: f64) -> f64 {
            let error = self.setpoint - measurement;
            self.integral += error * 0.001;  // No anti-windup limit
            let derivative = (error - self.last_error) / 0.001;
            self.last_error = error;
            1.0 * error + 0.1 * self.integral + 0.05 * derivative  // No output clamping
        }
    }
}

/// V2: Optimized design (parking_lot, filtering, anti-windup)
mod v2_optimized {
    use parking_lot::Mutex;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::collections::VecDeque;
    
    pub struct V2Sensor {
        pub value: f64,
        pub filter_buffer: VecDeque<f64>,
    }
    
    impl V2Sensor {
        pub fn new() -> Self {
            Self {
                value: 50.0,
                filter_buffer: VecDeque::with_capacity(5),
            }
        }
        
        pub fn read(&mut self, cycle: u64) -> f64 {
            // V2: With moving average filter
            let noise = (cycle as f64 * 0.1).sin() * 5.0;
            let raw = 50.0 + noise;
            
            self.filter_buffer.push_back(raw);
            if self.filter_buffer.len() > 5 {
                self.filter_buffer.pop_front();
            }
            
            let sum: f64 = self.filter_buffer.iter().sum();
            self.value = sum / self.filter_buffer.len() as f64;
            self.value
        }
    }
    
    pub struct V2SharedState {
        pub counter: AtomicU64,  // Lock-free
        pub readings: Mutex<VecDeque<f64>>,
    }
    
    impl V2SharedState {
        pub fn new() -> Self {
            Self {
                counter: AtomicU64::new(0),
                readings: Mutex::new(VecDeque::with_capacity(100)),
            }
        }
        
        pub fn increment(&self) {
            self.counter.fetch_add(1, Ordering::Relaxed);
        }
        
        pub fn add_reading(&self, val: f64) {
            let mut r = self.readings.lock();
            r.push_back(val);
            if r.len() > 100 {
                r.pop_front();
            }
        }
    }
    
    pub struct V2Pid {
        pub setpoint: f64,
        pub integral: f64,
        pub integral_limit: f64,
        pub last_error: f64,
        pub output_min: f64,
        pub output_max: f64,
    }
    
    impl V2Pid {
        pub fn new(setpoint: f64) -> Self {
            Self {
                setpoint,
                integral: 0.0,
                integral_limit: 100.0,
                last_error: 0.0,
                output_min: -100.0,
                output_max: 100.0,
            }
        }
        
        pub fn update(&mut self, measurement: f64) -> f64 {
            let error = self.setpoint - measurement;
            
            // Anti-windup
            self.integral += error * 0.001;
            self.integral = self.integral.max(-self.integral_limit).min(self.integral_limit);
            
            let derivative = (error - self.last_error) / 0.001;
            self.last_error = error;
            
            let output = 1.0 * error + 0.1 * self.integral + 0.05 * derivative;
            output.max(self.output_min).min(self.output_max)  // Clamped output
        }
    }
}

fn bench_version_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("version_comparison");
    group.throughput(Throughput::Elements(1000));
    
    // V1: Unoptimized (std::sync::Mutex, no filtering, basic PID)
    group.bench_function("v1_unoptimized", |b| {
        b.iter(|| {
            let state = v1_unoptimized::V1SharedState::new();
            let mut sensor = v1_unoptimized::V1Sensor::new();
            let mut pid = v1_unoptimized::V1Pid::new(50.0);
            
            for i in 0..1000 {
                let reading = sensor.read(i);
                state.add_reading(reading);
                state.increment();
                let output = pid.update(reading);
                black_box(output);
            }
        })
    });
    
    // V2: Optimized (parking_lot, filtering, anti-windup PID)
    group.bench_function("v2_optimized", |b| {
        b.iter(|| {
            let state = v2_optimized::V2SharedState::new();
            let mut sensor = v2_optimized::V2Sensor::new();
            let mut pid = v2_optimized::V2Pid::new(50.0);
            
            for i in 0..1000 {
                let reading = sensor.read(i);
                state.add_reading(reading);
                state.increment();
                let output = pid.update(reading);
                black_box(output);
            }
        })
    });
    
    group.finish();
}

// ============================================================================
// 5. ASYNC vs THREADED COMPARISON (OPTIONAL - Advanced 80%+)
// ============================================================================
// Requirement: "Implement one version using async/await and another using multi-threading"
// ============================================================================

fn bench_async_vs_threaded(c: &mut Criterion) {
    let mut group = c.benchmark_group("async_vs_threaded");
    group.throughput(Throughput::Elements(1000));
    
    // Multi-threaded implementation
    group.bench_function("threaded_std_thread", |b| {
        b.iter(|| {
            let (tx, rx) = std::sync::mpsc::sync_channel::<f64>(100);
            let iterations = 1000u64;
            
            let sensor_handle = thread::spawn(move || {
                for i in 0..iterations {
                    let output = do_sensor_actuator_work(i);
                    let _ = tx.send(output);
                }
            });
            
            let actuator_handle = thread::spawn(move || {
                let mut sum = 0.0;
                for _ in 0..iterations {
                    if let Ok(val) = rx.recv() {
                        sum += val;
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
                    for i in 0..iterations {
                        let output = do_sensor_actuator_work(i);
                        let _ = tx.send(output).await;
                    }
                });
                
                let actuator_task = tokio::spawn(async move {
                    let mut sum = 0.0;
                    while let Some(val) = rx.recv().await {
                        sum += val;
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

// REQUIRED benchmarks
criterion_group!(
    name = required_benches;
    config = Criterion::default().sample_size(50);
    targets = 
        bench_lock_contention,      // 1. Lock contention comparison
        bench_sync_primitives,      // 2. Sync primitives comparison
        bench_cpu_load_impact,      // 3. CPU load comparison
        bench_version_comparison    // 4. V1 vs V2 comparison
);

// OPTIONAL benchmarks (Advanced 80%+)
criterion_group!(
    name = optional_benches;
    config = Criterion::default().sample_size(50);
    targets = 
        bench_async_vs_threaded     // 5. Async vs Threaded (optional)
);

criterion_main!(required_benches, optional_benches);
