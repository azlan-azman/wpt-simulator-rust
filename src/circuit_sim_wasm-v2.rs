use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[derive(Clone, Copy, PartialEq)]
pub enum TopologyMode {
    LCCLS,
    SS,
}

#[wasm_bindgen]
pub struct ResonantCircuitSim {
    mode: TopologyMode,

    // Circuit components
    v_dc: f64,
    f_sw: f64,
    dead_time: f64,
    l1: f64,
    c1: f64,
    cp: f64,
    lp: f64,
    ls: f64,
    cs: f64,
    rl: f64,
    k: f64,
    r_esr: f64,

    // Circuit state variables
    i1: f64,
    v_c1: f64,
    v_cp: f64,
    ip: f64,
    is_: f64,
    v_cs: f64,

    time: f64,

    // Pre-allocated ring buffers for high-performance rendering
    history_ip: Vec<f64>,
    history_is: Vec<f64>,
    history_vin: Vec<f64>,
    max_history: usize,
    sample_counter: usize,
    sample_stride: usize,
}

#[wasm_bindgen]
impl ResonantCircuitSim {
    #[wasm_bindgen(constructor)]
    pub fn new() -> ResonantCircuitSim {
        ResonantCircuitSim {
            mode: TopologyMode::LCCLS,
            v_dc: 48.0,
            f_sw: 100_000.0,
            dead_time: 200e-9,
            l1: 2.75e-6,
            c1: 918e-9,
            cp: 47e-9,
            lp: 56.37e-6,
            ls: 160e-6,
            cs: 15.67e-9,
            rl: 12.0,
            k: 0.2,
            r_esr: 0.05,

            i1: 0.0,
            v_c1: 0.0,
            v_cp: 0.0,
            ip: 0.0,
            is_: 0.0,
            v_cs: 0.0,
            time: 0.0,

            history_ip: Vec::with_capacity(800),
            history_is: Vec::with_capacity(800),
            history_vin: Vec::with_capacity(800),
            max_history: 800,
            sample_counter: 0,
            sample_stride: 2, // Sample every 2 substeps for ultra-fast performance
        }
    }

    pub fn set_topology(&mut self, is_ss_mode: bool) {
        self.mode = if is_ss_mode { TopologyMode::SS } else { TopologyMode::LCCLS };
        self.reset_state();
    }

    pub fn reset_state(&mut self) {
        self.i1 = 0.0;
        self.v_c1 = 0.0;
        self.v_cp = 0.0;
        self.ip = 0.0;
        self.is_ = 0.0;
        self.v_cs = 0.0;
        self.time = 0.0;
        self.sample_counter = 0;
        self.history_ip.clear();
        self.history_is.clear();
        self.history_vin.clear();
    }

    pub fn set_param(&mut self, param: &str, val: f64) {
        match param {
            "v_dc" => self.v_dc = val,
            "f_sw" => self.f_sw = val,
            "dead_time" => self.dead_time = val,
            "l1" => self.l1 = val,
            "c1" => self.c1 = val,
            "cp" => self.cp = val,
            "lp" => self.lp = val,
            "ls" => self.ls = val,
            "cs" => self.cs = val,
            "rl" => self.rl = val,
            "k" => self.k = val,
            "r_esr" => self.r_esr = val,
            _ => (),
        }
    }

    fn step_physics(&mut self, dt: f64) {
        let period = 1.0 / self.f_sw;
        let t_mod = self.time % period;

        // Inverter square wave with dead time
        let v_in = if t_mod < (period / 2.0 - self.dead_time) {
            self.v_dc
        } else if t_mod < (period / 2.0) {
            0.0
        } else if t_mod < (period - self.dead_time) {
            -self.v_dc
        } else {
            0.0
        };

        let m = self.k * (self.lp * self.ls).sqrt();
        let det = self.lp * self.ls - m * m;

        if det.abs() > 1e-18 {
            match self.mode {
                TopologyMode::LCCLS => {
                    let rhs_p = (self.v_c1 - self.v_cp) - self.ip * self.r_esr;
                    let rhs_s = -self.v_cs - self.is_ * (self.rl + self.r_esr);

                    let dip_dt = (self.ls * rhs_p - m * rhs_s) / det;
                    let dis_dt = (-m * rhs_p + self.lp * rhs_s) / det;
                    let di1_dt = if self.l1 > 0.0 {
                        (v_in - self.v_c1 - self.i1 * self.r_esr) / self.l1
                    } else {
                        0.0
                    };

                    self.i1 += di1_dt * dt;
                    self.ip += dip_dt * dt;
                    self.is_ += dis_dt * dt;

                    if self.c1 > 0.0 {
                        self.v_c1 += ((self.i1 - self.ip) / self.c1) * dt;
                    }
                    if self.cp > 0.0 {
                        self.v_cp += (self.ip / self.cp) * dt;
                    }
                    if self.cs > 0.0 {
                        self.v_cs += (self.is_ / self.cs) * dt;
                    }
                }
                TopologyMode::SS => {
                    self.v_c1 = v_in;
                    let rhs_p = (v_in - self.v_cp) - self.ip * self.r_esr;
                    let rhs_s = -self.v_cs - self.is_ * (self.rl + self.r_esr);

                    let dip_dt = (self.ls * rhs_p - m * rhs_s) / det;
                    let dis_dt = (-m * rhs_p + self.lp * rhs_s) / det;

                    self.ip += dip_dt * dt;
                    self.is_ += dis_dt * dt;
                    self.i1 = self.ip;

                    if self.cp > 0.0 {
                        self.v_cp += (self.ip / self.cp) * dt;
                    }
                    if self.cs > 0.0 {
                        self.v_cs += (self.is_ / self.cs) * dt;
                    }
                }
            }
        }

        self.time += dt;

        // Downsample & record into ring buffer efficiently
        self.sample_counter += 1;
        if self.sample_counter % self.sample_stride == 0 {
            if self.history_ip.len() >= self.max_history {
                self.history_ip.remove(0);
                self.history_is.remove(0);
                self.history_vin.remove(0);
            }
            self.history_ip.push(self.ip);
            self.history_is.push(self.is_);
            self.history_vin.push(v_in);
        }
    }

    pub fn update(&mut self, total_duration: f64, substeps: usize) {
        let dt = total_duration / (substeps as f64);
        for _ in 0..substeps {
            self.step_physics(dt);
        }
    }

    pub fn get_ip(&self) -> f64 { self.ip }
    pub fn get_is(&self) -> f64 { self.is_ }
    pub fn get_i1(&self) -> f64 { self.i1 }
    pub fn get_v_c1(&self) -> f64 { self.v_c1 }
    pub fn get_time(&self) -> f64 { self.time }

    pub fn get_history_ip(&self) -> Vec<f64> { self.history_ip.clone() }
    pub fn get_history_is(&self) -> Vec<f64> { self.history_is.clone() }
    pub fn get_history_vin(&self) -> Vec<f64> { self.history_vin.clone() }
}
