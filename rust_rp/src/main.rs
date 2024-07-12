// =============
// Read me first
// =============
// This RUST implementation has support for both Steepest Gradient Descent (SGD) and Reinforcement Learning (RL) approaches for dynamic resource provisioning
// RP_MODE should be set to 2 for RL and 3 for SGD
// UDP_WAIT_TIME and REPEAT_REQUEST variables must be correctly set (can be further tuned) for the approach chosen
// rp-sim-ul.json should be run on Agora and sender side before running this RUST implementation
// Build RUST code:
// <agora_workspace>/rust_rp cargo build (debug build)
// Run RUST code:
// <agora_workspace>/rust_rp/target/debug ./rust_rp (on a separate third terminal)
// For SGD, an example output will be "SGD: The optimal number of CPU cores is 8" for the below Agora configuration
// For RL, model has to be trained first and then the trained model should be used to determine optimal cores as per the reward function defined
// In RL approach (RP_MODE == 2), is_training_enabled to be set true and is_testing_enabled to be set false for training
// In RL approach (RP_MODE == 2), is_training_enabled to be set false and is_testing_enabled to be set true for testing
// num_episodes and terminate_count can be further changed as needed. Their current values work proper for the below Agora configuration
//
// Sample Size for moving averaging is computed as n = (100*z*s/r*m)^2
// where, 
// z -> normal variate od the desired confidence level = 1.96 for 95% confidence level
// r -> accuracy required
// s -> sample standard deviation
// m -> sample mean
// For r = 5%, s = 53, m = 535 (Agora configuration: 64x8 MIMO, 25 MCS UL, 5 UL LDPC Iterations and 10 workers), n = 15
// Reference for Statistical Analysis: "The art of Computer Systems Perforamnce Analysis - Raj Jain"

use tokio::net::UdpSocket;
use tokio::time::{delay_for, Duration};
use async_trait::async_trait;
use std::cmp::Ordering;
use serde_json;
use std::fs::{self, File, OpenOptions};
use std::io::{self, Write, BufReader};
use std::arch::asm;

extern crate rand;
use rand::Rng;

// Define a trait for the environment
#[async_trait]
trait AgoraEnv {
    fn new() -> Self;
    fn rdtsc() -> u64;
    fn set_initial_values(&mut self, num_max_cores: u16, num_min_cores: u16, num_max_users: u16, num_latency_levels: u16, num_actions: u16, max_latency_limit: f32, ma_window_size: u16, data_outlier_percentage: u16, rewards: Vec<i16>, state: u64, done: bool);
    fn reset_ma_filter(&mut self, num_max_cores: u16, num_max_users: u16, ma_window_size: u16);
    fn reset(&mut self) -> u64;
    fn set(&mut self, state: u64);
    fn compute_state(&mut self, curr_cores: u16, curr_users: u16, curr_latency: u16) -> u64;
    fn get_num_states(&mut self) -> u64;
    fn get_num_actions(&mut self) -> u16;
    fn get_curr_users(&mut self, state: u64) -> u16;
    fn agora_to_rl_index_mapping(&mut self, index_param: u16) -> u16;
    fn rl_to_agora_index_mapping(&mut self, index_param: u16) -> u16;
    fn compute_absolute_latency(&mut self, curr_cores: u16, curr_users: u16) -> f32;
    fn compute_curr_latency_reward_done_real(&mut self, absolute_latency: f32) -> (u16, i16, bool);
    fn compute_curr_latency_reward_done_emulated(&mut self, absolute_latency: f32) -> (u16, i16, bool);
    fn moving_average(&mut self, curr_cores: u16, curr_users: u16) -> f32;
    async fn step_real(&mut self, socket: &mut UdpSocket, action: u16) -> io::Result<(f32, u16, u16, u64, i16, bool)>;
    fn step_emulated(&mut self, action: u16) -> (u64, i16, bool);
    async fn print_state(&mut self, state: u64, delay: u64);

    fn set_initial_values_sgd(&mut self, num_max_cores: u16, num_min_cores: u16, num_max_users: u16, ma_window_size: u16, max_latency_limit: f32, data_outlier_percentage: u16, learn_rate: f32, tolerance: f32, cores_latency_window: u16);
    fn reset_ma_filter_sgd(&mut self, num_max_cores: u16, num_max_users: u16, ma_window_size: u16);
    fn moving_average_sgd(&mut self, curr_cores: u16, curr_users: u16) -> f32;
    async fn agora_latency_sgd(&mut self, socket: &mut UdpSocket, current_cores: u16) -> io::Result<f32>;
    async fn measure_latency_sgd(&mut self, socket: &mut UdpSocket, current_cores: u16) -> io::Result<f32>;
    fn detect_oscillations_sgd(&mut self, current_cores: u16, delta: u8, latency1: f32, latency2: f32) -> (f32, f32, f32, f32, u16);
    async fn optimization_sgd(&mut self, socket: &mut UdpSocket, start_cores: u16, n_iter: u16) -> io::Result<u16>;
}

// Implement the trait for Agora environment
struct MyAgoraEnv {
    num_max_cores: u16,
    num_min_cores: u16,
    num_max_users: u16,
    num_latency_levels: u16,
    num_states: u64,
    num_actions: u16,
    max_latency_limit: f32,
    rewards: Vec<i16>,
    ma_active_window_sizes: Vec<Vec<u16>>,
    ma_window_size: u16,
    ma_window: Vec<Vec<Vec<f32>>>,
    ma_no_change_count: Vec<Vec<u16>>,
    data_outlier_percentage: u16,
    previous_frame_id: u64,
    state: u64,
    done: bool,

    num_max_cores_sgd: u16,
    num_min_cores_sgd: u16,
    num_max_users_sgd: u16,
    ma_active_window_sizes_sgd: Vec<Vec<u16>>,
    ma_window_size_sgd: u16,
    ma_window_sgd: Vec<Vec<Vec<f32>>>,
    max_latency_limit_sgd: f32,
    data_outlier_percentage_sgd: u16,
    learn_rate_sgd: f32,
    tolerance_sgd: f32,
    cores_latency_sgd : Vec<f32>,
    cores_latency_window_sgd: u16
}

#[async_trait]
impl AgoraEnv for MyAgoraEnv {
    fn new() -> Self {
        // Initialize the environment with custom values
        MyAgoraEnv {
            num_max_cores: 10,
            num_min_cores: 1,
            num_max_users: 16,
            num_latency_levels: 10 + 1,
            num_states: 10 * 16 * (10 + 1),
            num_actions: 3,
            max_latency_limit: 1.0,
            rewards: vec![-12, -10, -8, -6, -4, -2, -1, 20, -12, -16, -20],
            ma_active_window_sizes: vec![vec![0; 16]; 25],
            ma_window_size: 10,
            ma_window: vec![vec![vec![0.0; 10]; 16]; 25],
            ma_no_change_count: vec![vec![0; 16]; 25],
            data_outlier_percentage: 10,
            previous_frame_id: 0,
            state: 0,
            done: false,

            num_max_cores_sgd: 25,
            num_min_cores_sgd: 3,
            num_max_users_sgd: 16,
            ma_active_window_sizes_sgd: vec![vec![0; 16]; 25],
            ma_window_size_sgd: 15,
            ma_window_sgd: vec![vec![vec![0.0; 10]; 16]; 25],
            max_latency_limit_sgd: 1000.0,
            data_outlier_percentage_sgd: 10,
            learn_rate_sgd: 0.01,
            tolerance_sgd: 0.01,
            cores_latency_sgd: vec![0.0; 25],
            cores_latency_window_sgd: 3
        }
    }

    fn rdtsc() -> u64 {
        let mut rax: u64;
        let mut rdx: u64;
        unsafe {
            asm!(
                "rdtsc",
                out("rax") rax,
                out("rdx") rdx,
                options(pure, nomem, nostack, preserves_flags)
            );
        }
        (rdx << 32) | rax
    }

    fn set_initial_values(&mut self, num_max_cores: u16, num_min_cores: u16, num_max_users: u16, num_latency_levels: u16, num_actions: u16, max_latency_limit: f32, ma_window_size: u16, data_outlier_percentage: u16, rewards: Vec<i16>, state: u64, done: bool) {
        // Set initial values
        self.num_max_cores = num_max_cores;
        self.num_min_cores = num_min_cores;
        self.num_max_users = num_max_users;
        self.num_latency_levels = num_latency_levels;
        self.num_states = num_max_cores as u64 * num_max_users as u64 * num_latency_levels as u64;
        self.num_actions = num_actions;
        self.max_latency_limit = max_latency_limit;
        self.ma_window_size = ma_window_size;
        self.data_outlier_percentage = data_outlier_percentage;
        self.rewards = rewards;
        self.state = state;
        self.done = done;
    }

    fn reset_ma_filter(&mut self, num_max_cores: u16, num_max_users: u16, ma_window_size: u16) {
        // Reset ma filter contents
        self.ma_active_window_sizes = vec![vec![0; num_max_users as usize]; num_max_cores as usize];
        self.ma_window = vec![vec![vec![0.0; ma_window_size as usize]; num_max_users as usize]; num_max_cores as usize];
        self.ma_no_change_count = vec![vec![0; num_max_users as usize]; num_max_cores as usize];
    }

    fn set_initial_values_sgd(&mut self, num_max_cores: u16, num_min_cores: u16, num_max_users: u16, ma_window_size: u16, max_latency_limit:f32, data_outlier_percentage: u16, learn_rate: f32, tolerance: f32, cores_latency_window: u16){
        // Set initial values
        self.num_max_cores_sgd = num_max_cores;
        self.num_min_cores_sgd = num_min_cores;
        self.num_max_users_sgd = num_max_users;
        self.ma_window_size_sgd = ma_window_size;
        self.max_latency_limit_sgd = max_latency_limit;
        self.data_outlier_percentage_sgd = data_outlier_percentage;
        self.learn_rate_sgd = learn_rate;
        self.tolerance_sgd = tolerance;
        self.cores_latency_sgd = vec![0.0; num_max_cores as usize];
        self.cores_latency_window_sgd = cores_latency_window;
    }

    fn reset_ma_filter_sgd(&mut self, num_max_cores: u16, num_max_users: u16, ma_window_size: u16) {
        // Reset ma filter contents
        self.ma_active_window_sizes_sgd = vec![vec![0; num_max_users as usize]; num_max_cores as usize];
        self.ma_window_sgd = vec![vec![vec![0.0; ma_window_size as usize]; num_max_users as usize]; num_max_cores as usize];
    }

    fn reset(&mut self) -> u64 {
        // Reset the state with a random value within the range of num_states
        let mut rng = rand::thread_rng();
        self.state = rng.gen_range(0..self.num_states);

        self.state
    }

    fn set(&mut self, state: u64) {
        self.state = state;
    }

    fn compute_state(&mut self, curr_cores: u16, curr_users: u16, curr_latency: u16) -> u64 {
        let curr_state = (curr_cores as u64 + curr_users as u64 * self.num_max_cores as u64 + curr_latency as u64 * self.num_max_cores as u64 * self.num_max_users as u64) % self.num_states;

        curr_state
    }

    fn get_num_states(&mut self) -> u64 {

        self.num_states
    }

    fn get_num_actions(&mut self) -> u16 {

        self.num_actions
    }

    fn get_curr_users(&mut self, state: u64) -> u16 {
        let curr_users = (state / self.num_max_cores as u64) % self.num_max_users as u64;

        curr_users as u16
    }

    fn agora_to_rl_index_mapping(&mut self, index_param: u16) -> u16 {

        index_param - 1
    }

    fn rl_to_agora_index_mapping(&mut self, index_param: u16) -> u16 {

        index_param + 1
    }

    fn compute_absolute_latency(&mut self, curr_cores: u16, curr_users: u16) -> f32 {
        let min_latency = 0.05 * self.max_latency_limit;
        let per_user_latency = 0.1 * self.max_latency_limit;
        let absolute_latency = (min_latency + (curr_users + 1) as f32 * per_user_latency) / (curr_cores + 1) as f32; // Using actual number of users and cores

        absolute_latency
    }

    fn compute_curr_latency_reward_done_real(&mut self, absolute_latency: f32) -> (u16, i16, bool) {
        let next_latency;
        let reward;
        let mut done = false;
        if 0.0 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.1 * self.max_latency_limit {
            next_latency = 0;
            reward = self.rewards[0];
        } else if 0.1 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.2 * self.max_latency_limit {
            next_latency = 1;
            reward = self.rewards[1];
        } else if 0.2 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.3 * self.max_latency_limit {
            next_latency = 2;
            reward = self.rewards[2];
        } else if 0.3 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.4 * self.max_latency_limit {
            next_latency = 3;
            reward = self.rewards[3];
        } else if 0.4 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.5 * self.max_latency_limit {
            next_latency = 4;
            reward = self.rewards[4];
        } else if 0.5 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.6 * self.max_latency_limit {
            next_latency = 5;
            reward = self.rewards[5];
        } else if 0.6 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.7 * self.max_latency_limit {
            next_latency = 6;
            reward = self.rewards[6];
        } else if 0.7 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.8 * self.max_latency_limit {
            next_latency = 7;
            reward = self.rewards[7];
            done = true;
        } else if 0.8 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.9 * self.max_latency_limit {
            next_latency = 8;
            reward = self.rewards[8];
        } else if 0.9 * self.max_latency_limit <= absolute_latency && absolute_latency < 1.0 * self.max_latency_limit {
            next_latency = 9;
            reward = self.rewards[9];
        } else {
            next_latency = 10;
            reward = self.rewards[10];
        }

        (next_latency, reward, done)
    }

    fn compute_curr_latency_reward_done_emulated(&mut self, absolute_latency: f32) -> (u16, i16, bool) {
        let next_latency;
        let reward;
        let mut done = false;
        if 0.0 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.1 * self.max_latency_limit {
            next_latency = 0;
            reward = self.rewards[0];
        } else if 0.1 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.2 * self.max_latency_limit {
            next_latency = 1;
            reward = self.rewards[1];
        } else if 0.2 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.3 * self.max_latency_limit {
            next_latency = 2;
            reward = self.rewards[2];
        } else if 0.3 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.4 * self.max_latency_limit {
            next_latency = 3;
            reward = self.rewards[3];
        } else if 0.4 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.5 * self.max_latency_limit {
            next_latency = 4;
            reward = self.rewards[4];
        } else if 0.5 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.6 * self.max_latency_limit {
            next_latency = 5;
            reward = self.rewards[5];
        } else if 0.6 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.7 * self.max_latency_limit {
            next_latency = 6;
            reward = self.rewards[6];
        } else if 0.7 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.8 * self.max_latency_limit {
            next_latency = 7;
            reward = self.rewards[7];
            done = true;
        } else if 0.8 * self.max_latency_limit <= absolute_latency && absolute_latency < 0.9 * self.max_latency_limit {
            next_latency = 8;
            reward = self.rewards[8];
        } else if 0.9 * self.max_latency_limit <= absolute_latency && absolute_latency < 1.0 * self.max_latency_limit {
            next_latency = 9;
            reward = self.rewards[9];
        } else {
            next_latency = 10;
            reward = self.rewards[10];
        }

        (next_latency, reward, done)
    }

    fn moving_average(&mut self, curr_cores: u16, curr_users: u16) -> f32 {
        // let curr_cores_ag = MyAgoraEnv::rl_to_agora_index_mapping(self, curr_cores);
        // let ma_window_row_elements = &self.ma_window[curr_cores as usize][curr_users as usize][count as usize];
        // println!("agora cores {}, ma active window size {}, ma window size {}\n", curr_cores_ag, self.ma_active_window_sizes[curr_cores as usize][curr_users as usize], self.ma_window_size);
        // println!("ma window {:?}\n", ma_window_row_elements);
        let mut average_value = 0.0;
        for count in (self.ma_window_size - self.ma_active_window_sizes[curr_cores as usize][curr_users as usize])..= (self.ma_window_size - 1) {
            average_value = average_value + self.ma_window[curr_cores as usize][curr_users as usize][count as usize];
        }
        average_value = average_value/self.ma_active_window_sizes[curr_cores as usize][curr_users as usize] as f32;

        average_value
    }

    async fn step_real(&mut self, socket: &mut UdpSocket, action: u16) -> io::Result<(f32, u16, u16, u64, i16, bool)> {
        // Take a step in the environment based on the given action
        let curr_cores = (self.state % self.num_max_cores as u64) as u16;
        // let curr_users = ((self.state / self.num_max_cores as u64) % self.num_max_users as u64) as u16;
        let next_cores;

        let curr_cores_ag = MyAgoraEnv::rl_to_agora_index_mapping(self, curr_cores);
        if action == 1 { // Add One Core
            if curr_cores_ag < self.num_max_cores {
                next_cores = (curr_cores + 1) % self.num_max_cores;
                send_cores_control_message(socket, 1, 0, curr_cores_ag as u8).await?;
            } else {
                println!("Maximum cores limit reached ...");
                next_cores = curr_cores;
            }
        } else if action == 2 { // Remove One Core
            if curr_cores_ag <= self.num_min_cores {
                println!("Minimum cores limit reached ...");
                next_cores = curr_cores;
            } else {
                next_cores = (curr_cores - 1) % self.num_max_cores;
                send_cores_control_message(socket, 0, 1, curr_cores_ag as u8).await?;
            }
        } else {
            // No Change (curr_cores remain as it is)
            next_cores = curr_cores;
        }

        delay_for(Duration::from_millis(REPEAT_REQUEST)).await;
        let mut retrieved_msg = retrieve_agora_traffic(socket).await?;
        let mut absolute_latency = retrieved_msg[0];
        let mut agora_cores = retrieved_msg[1];
        let mut agora_users = retrieved_msg[2];
        let mut frame_id = retrieved_msg[3];
        println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", absolute_latency, agora_cores, agora_users, frame_id);    
        let mut is_valid_latency = false;
        while is_valid_latency == false {
            if absolute_latency <= 2 * self.max_latency_limit as u64 {
                is_valid_latency = true;
            } else {
                println!("Out of bound latency, requesting Agora again ...\n");
                delay_for(Duration::from_millis(REPEAT_REQUEST)).await;
                retrieved_msg = retrieve_agora_traffic(socket).await?;
                absolute_latency = retrieved_msg[0];
                agora_cores = retrieved_msg[1];
                agora_users = retrieved_msg[2];
                frame_id = retrieved_msg[3];
                println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", absolute_latency, agora_cores, agora_users, frame_id);
            }
        }

        let mut is_same_frame_id = true;
        while is_same_frame_id == true {
            if frame_id != self.previous_frame_id {
                is_same_frame_id = false;
            } else {
                println!("Latency received for same frame_id, requesting Agora again ...\n");
                delay_for(Duration::from_millis(REPEAT_REQUEST)).await;
                retrieved_msg = retrieve_agora_traffic(socket).await?;
                absolute_latency = retrieved_msg[0];
                agora_cores = retrieved_msg[1];
                agora_users = retrieved_msg[2];
                frame_id = retrieved_msg[3];
                println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", absolute_latency, agora_cores, agora_users, frame_id);
            }
        }
        self.previous_frame_id = frame_id;

        // Outlier is ignored for moving average filtering
        let agora_cores_rl = MyAgoraEnv::agora_to_rl_index_mapping(self, agora_cores as u16);
        let agora_users_rl = MyAgoraEnv::agora_to_rl_index_mapping(self, agora_users as u16);
        let ma_absolute_latency;
        if self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize] < self.ma_window_size {
            // let ma_window_row_elements = &self.ma_window[agora_cores_rl as usize][agora_users_rl as usize][count as usize];
            // println!("agora cores {}, ma active window size {}, ma window size {}\n", agora_cores, self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize], self.ma_window_size);
            // println!("ma window (before update) {:?}\n", ma_window_row_elements);

            for count in (self.ma_window_size - self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize])..= (self.ma_window_size  - 1) {
                // println!("agora cores {}, ma active window size {}, count {}\n", agora_cores, self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize], count);
                self.ma_window[agora_cores_rl as usize][agora_users_rl as usize][count as usize - 1] = self.ma_window[agora_cores_rl as usize][agora_users_rl as usize][count as usize];
            }
            self.ma_window[agora_cores_rl as usize][agora_users_rl as usize][self.ma_window_size as usize - 1] = absolute_latency as f32;
            self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize] = self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize] + 1;

            // let ma_window_row_elements = &self.ma_window[agora_cores_rl as usize][agora_users_rl as usize];
            // println!("agora cores {}, ma active window size {}, ma window size {}\n", agora_cores, self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize], self.ma_window_size);
            // println!("ma window (after update) {:?}\n", ma_window_row_elements);

            ma_absolute_latency = MyAgoraEnv::moving_average(self, agora_cores_rl as u16, agora_users_rl as u16);
            println!("agora cores {}, agora users {}, ma active window size {}, ma absolute latency {}\n", agora_cores, agora_users, self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize], ma_absolute_latency);
        } else {
            let ma_absolute_latency_before_update = MyAgoraEnv::moving_average(self, agora_cores_rl as u16, agora_users_rl as u16);
            let outlier_lower_limit = ((100.0 - self.data_outlier_percentage as f32)/100.0) * ma_absolute_latency_before_update;
            let outlier_higher_limit = ((100.0 + self.data_outlier_percentage as f32)/100.0) * ma_absolute_latency_before_update;
            println!("agora cores {}, agora users {}, ma active window size {}, ma absolute latency before update {}, outlier percentage {}, outlier lower limit {}, outlier higher limit {}, absolute latency {}\n",
                agora_cores, agora_users, self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize], ma_absolute_latency_before_update, self.data_outlier_percentage, outlier_lower_limit, outlier_higher_limit, absolute_latency);
            if absolute_latency as f32 >= outlier_lower_limit as f32 && absolute_latency as f32 <= outlier_higher_limit as f32 {
                // let ma_window_row_elements = &self.ma_window[agora_cores_rl as usize][agora_users_rl as usize];
                // println!("agora cores {}, ma active window size {}, ma window size {}\n", agora_cores, self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize], self.ma_window_size);
                // println!("ma window (before update) {:?}\n", ma_window_row_elements);
    
                for count in 1..= (self.ma_window_size - 1) {
                    self.ma_window[agora_cores_rl as usize][agora_users_rl as usize][count as usize - 1] = self.ma_window[agora_cores_rl as usize][agora_users_rl as usize][count as usize];
                }
                self.ma_window[agora_cores_rl as usize][agora_users_rl as usize][self.ma_window_size as usize - 1] = absolute_latency as f32;
                if self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize] < self.ma_window_size {
                    self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize] = self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize] + 1;
                }

                // let ma_window_row_elements = &self.ma_window[agora_cores_rl as usize][agora_users_rl as usize];
                // println!("agora cores {}, ma active window size {}, ma window size {}\n", agora_cores, self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize], self.ma_window_size);
                // println!("ma window (after update) {:?}\n", ma_window_row_elements);
        
                ma_absolute_latency = MyAgoraEnv::moving_average(self, agora_cores_rl as u16, agora_users_rl as u16);
                println!("absolute_latency {} is not an outlier and taken in for moving average filtering; ma absolute latency {}\n", absolute_latency, ma_absolute_latency);
            } else {
                ma_absolute_latency = ma_absolute_latency_before_update;
                println!("absolute latency {} is an outlier and ignored for moving average filtering; ma absolute latency {}\n", absolute_latency, ma_absolute_latency);
            }

            if ma_absolute_latency_before_update == ma_absolute_latency {
                self.ma_no_change_count[agora_cores_rl as usize][agora_users_rl as usize] = self.ma_no_change_count[agora_cores_rl as usize][agora_users_rl as usize] + 1;
                if self.ma_no_change_count[agora_cores_rl as usize][agora_users_rl as usize] == self.ma_window_size {
                    println!("No change in ma absolute latency for last {} updates for core {}", self.ma_window_size, agora_cores);
                    println!("Resetting ma window, ma active window sizes and ma no change count for core {}", agora_cores);
                    self.ma_window[agora_cores_rl as usize][agora_users_rl as usize] = vec![0.0; self.ma_window_size as usize];
                    self.ma_active_window_sizes[agora_cores_rl as usize][agora_users_rl as usize] = 0;
                    self.ma_no_change_count[agora_cores_rl as usize][agora_users_rl as usize] = 0;
                }
            }
        }

        let curr_latency_reward_done_real = MyAgoraEnv::compute_curr_latency_reward_done_real(self, ma_absolute_latency as f32);
        let curr_latency = curr_latency_reward_done_real.0;
        let reward = curr_latency_reward_done_real.1;
        let done = curr_latency_reward_done_real.2;
        let agora_users_rl = MyAgoraEnv::agora_to_rl_index_mapping(self, agora_users as u16);
        let next_state = MyAgoraEnv::compute_state(self, next_cores, agora_users_rl, curr_latency);

        // println!("curr_cores: {}", curr_cores);
        // println!("next_cores: {}", next_cores);
        // println!("agora_users: {}", agora_users_rl);
        // println!("absolute_latency: {}", absolute_latency);
        // println!("curr_latency: {}", curr_latency);
        // println!("reward: {}", reward);
        // println!("done: {}", done);
        // println!("next_state: {}", next_state);

        // Return the next state, reward and done flag
        Ok((ma_absolute_latency, agora_cores as u16, agora_users as u16, next_state, reward, done))
    }

    fn step_emulated(&mut self, action: u16) -> (u64, i16, bool) {
        // Take a step in the environment based on the given action
        let curr_cores = (self.state % self.num_max_cores as u64) as u16;
        let curr_users = ((self.state / self.num_max_cores as u64) % self.num_max_users as u64) as u16;
        let next_cores;

        if action == 1 { // Add One Core
            if (curr_cores + 1) % self.num_max_cores != 0 {
                next_cores = (curr_cores + 1) % self.num_max_cores;
            } else {
                // print("Maximum cores limit reached ...")
                next_cores = curr_cores;
            }
        } else if action == 2 { // Remove One Core
            if (curr_cores % self.num_max_cores) == 0 {
                // print("Minimum cores limit reached ...")
                next_cores = curr_cores;
            } else {
                next_cores = (curr_cores - 1) % self.num_max_cores;
            }
        } else {
            // No Change (curr_cores remain as it is)
            next_cores = curr_cores;
        }

        let absolute_latency = MyAgoraEnv::compute_absolute_latency(self, next_cores, curr_users);
        let curr_latency_reward_done = MyAgoraEnv::compute_curr_latency_reward_done_emulated(self, absolute_latency);
        let curr_latency = curr_latency_reward_done.0;
        let reward = curr_latency_reward_done.1;
        let done = curr_latency_reward_done.2;
        let next_state = MyAgoraEnv::compute_state(self, next_cores, curr_users, curr_latency);

        // println!("curr_cores: {}", curr_cores);
        // println!("next_cores: {}", next_cores);
        // println!("curr_users: {}", curr_users);
        // println!("absolute_latency: {}", absolute_latency);
        // println!("curr_latency: {}", curr_latency);
        // println!("reward: {}", reward);
        // println!("done: {}", done);
        // println!("next_state: {}", next_state);

        // Return the next state, reward and done flag
        (next_state, reward, done)
    }

    async fn print_state(&mut self, state: u64, delay: u64) {
        let curr_cores = state % self.num_max_cores as u64;
        let curr_users = (state / self.num_max_cores as u64) % self.num_max_users as u64;
        let curr_latency = (state / (self.num_max_cores as u64 * self.num_max_users as u64)) % self.num_latency_levels as u64;
        println!("\nState {} is decoded as: ", state);
        println!("curr_cores: {}", curr_cores);
        println!("curr_users: {}", curr_users);
        println!("curr_latency: {}", curr_latency);
        println!("delay: {}", delay);
        // delay_for(Duration::from_millis(delay)).await;
    }

    // SGD Functions
    // MA filter
    fn moving_average_sgd(&mut self, curr_cores: u16, curr_users: u16) -> f32 {
        // let curr_cores_ag = MyAgoraEnv::rl_to_agora_index_mapping(self, curr_cores);
        // let ma_window_row_elements = &self.ma_window_sgd[curr_cores as usize][curr_users as usize][count as usize];
        // println!("agora cores {}, ma active window size {}, ma window size {}\n", curr_cores_ag, self.ma_active_window_sizes_sgd[curr_cores as usize][curr_users as usize], self.ma_window_size_sgd);
        // println!("ma window {:?}\n", ma_window_row_elements);
        let mut average_value = 0.0;
        for count in (self.ma_window_size_sgd - self.ma_active_window_sizes_sgd[curr_cores as usize][curr_users as usize])..= (self.ma_window_size_sgd - 1) {
            average_value = average_value + self.ma_window_sgd[curr_cores as usize][curr_users as usize][count as usize];
        }
        average_value = average_value/self.ma_active_window_sizes_sgd[curr_cores as usize][curr_users as usize] as f32;

        average_value
    }

    // Function to measure Agora latency
    async fn agora_latency_sgd(&mut self, socket: &mut UdpSocket, current_cores: u16) -> io::Result<f32> {
        // println!("current cores {:?}\n", current_cores);
        let mut absolute_latency;
        let mut ma_absolute_latency = 0.0;
        let mut retrieved_msg;

        retrieved_msg = retrieve_agora_traffic(socket).await?;
        if (retrieved_msg[1] as u16) < current_cores {
            println!("Current cores {:?}; Agora cores {:?}; Adding {:?} core(s)\n", current_cores, retrieved_msg[1], (current_cores - retrieved_msg[1] as u16));
            let cores_to_be_added = current_cores - retrieved_msg[1] as u16;
            for _ in 1..=cores_to_be_added {
                send_cores_control_message(socket, 1 as u8, 0, retrieved_msg[1] as u8).await?;
            }
        } else if (retrieved_msg[1] as u16) > current_cores {
            println!("Current cores {:?}; Agora cores {:?}; Removing {:?} core(s)\n", current_cores, retrieved_msg[1], (retrieved_msg[1] as u16 - current_cores));
            let cores_to_be_removed = retrieved_msg[1] as u16 - current_cores;
            for _ in 1..=cores_to_be_removed {
                send_cores_control_message(socket, 0, 1 as u8, current_cores as u8).await?;
            }
        } else {

        }

        for rep in 1..=self.ma_window_size_sgd {
            delay_for(Duration::from_millis(REPEAT_REQUEST)).await;
            retrieved_msg = retrieve_agora_traffic(socket).await?;
            println!("Agora status: repeat: {}, absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", rep, retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
            absolute_latency = retrieved_msg[0];

            let mut is_valid_latency = false;
            while is_valid_latency == false {
                if absolute_latency <= 2 * self.max_latency_limit_sgd as u64 {
                    is_valid_latency = true;
                } else {
                    println!("Out of bound latency, requesting Agora again ...\n");
                    delay_for(Duration::from_millis(REPEAT_REQUEST)).await;
                    retrieved_msg = retrieve_agora_traffic(socket).await?;
                    absolute_latency = retrieved_msg[0];
                    println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                }
            }

            let agora_cores = retrieved_msg[1];
            let agora_users = self.num_max_users_sgd;
            // Outlier is ignored for moving average filtering
            let agora_cores_rl = MyAgoraEnv::agora_to_rl_index_mapping(self, agora_cores as u16);
            let agora_users_rl = MyAgoraEnv::agora_to_rl_index_mapping(self, agora_users as u16);
            // println!("agora_latency_sgd: agora_cores: {}, agora_cores_rl: {}, agora_users: {}, agora_users_rl: {}\n", agora_cores, agora_cores_rl, agora_users, agora_users_rl);
            // println!("ma_window_size_sgd: {}, ma_active_window_sizes_sgd: {}\n", self.ma_window_size_sgd, self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize]);
            if self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize] < self.ma_window_size_sgd {
                // let ma_window_row_elements = &self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize];
                // println!("agora cores {}, ma active window size {}, ma window size {}\n", agora_cores, self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize], self.ma_window_size_sgd);
                // println!("ma window (before update) {:?}\n", ma_window_row_elements);
    
                for count in (self.ma_window_size_sgd - self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize])..= (self.ma_window_size_sgd  - 1) {
                    // println!("agora cores {}, ma active window size {}, count {}\n", agora_cores, self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize], count);
                    self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize][count as usize - 1] = self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize][count as usize];
                }
                self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize][self.ma_window_size_sgd as usize - 1] = absolute_latency as f32;
                self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize] = self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize] + 1;
    
                let ma_window_row_elements = &self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize];
                // println!("agora cores {}, ma active window size {}, ma window size {}\n", agora_cores, self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize], self.ma_window_size_sgd);
                println!("ma window (after update) {:?}\n", ma_window_row_elements);
    
                ma_absolute_latency = MyAgoraEnv::moving_average_sgd(self, agora_cores_rl as u16, agora_users_rl as u16);
                println!("agora cores {}, agora users {}, ma active window size {}, ma absolute latency {}\n", agora_cores, agora_users, self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize], ma_absolute_latency);
            } else {
                let ma_absolute_latency_before_update = MyAgoraEnv::moving_average_sgd(self, agora_cores_rl as u16, agora_users_rl as u16);
                let outlier_lower_limit = ((100.0 - self.data_outlier_percentage_sgd as f32)/100.0) * ma_absolute_latency_before_update;
                let outlier_higher_limit = ((100.0 + self.data_outlier_percentage_sgd as f32)/100.0) * ma_absolute_latency_before_update;
                println!("agora cores {}, agora users {}, ma active window size {}, ma absolute latency before update {}, outlier percentage {}, outlier lower limit {}, outlier higher limit {}, absolute latency {}\n",
                    agora_cores, agora_users, self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize], ma_absolute_latency_before_update, self.data_outlier_percentage_sgd, outlier_lower_limit, outlier_higher_limit, absolute_latency);
                if absolute_latency as f32 >= outlier_lower_limit as f32 && absolute_latency as f32 <= outlier_higher_limit as f32 {
                    // let ma_window_row_elements = &self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize];
                    // println!("agora cores {}, ma active window size {}, ma window size {}\n", agora_cores, self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize], self.ma_window_size_sgd);
                    // println!("ma window (before update) {:?}\n", ma_window_row_elements);
        
                    for count in 1..= (self.ma_window_size_sgd - 1) {
                        self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize][count as usize - 1] = self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize][count as usize];
                    }
                    self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize][self.ma_window_size_sgd as usize - 1] = absolute_latency as f32;
                    if self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize] < self.ma_window_size_sgd {
                        self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize] = self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize] + 1;
                    }
    
                    let ma_window_row_elements = &self.ma_window_sgd[agora_cores_rl as usize][agora_users_rl as usize];
                    // println!("agora cores {}, ma active window size {}, ma window size {}\n", agora_cores, self.ma_active_window_sizes_sgd[agora_cores_rl as usize][agora_users_rl as usize], self.ma_window_size_sgd);
                    println!("ma window (after update) {:?}\n", ma_window_row_elements);
            
                    ma_absolute_latency = MyAgoraEnv::moving_average_sgd(self, agora_cores_rl as u16, agora_users_rl as u16);
                    println!("absolute_latency {} is not an outlier and taken in for moving average filtering; ma absolute latency {}\n", absolute_latency, ma_absolute_latency);
                } else {
                    ma_absolute_latency = ma_absolute_latency_before_update;
                    println!("absolute latency {} is an outlier and ignored for moving average filtering; ma absolute latency {}\n", absolute_latency, ma_absolute_latency);
                }
            }
        }

        Ok(ma_absolute_latency)
    }

    // Measure latency
    async fn measure_latency_sgd(&mut self, socket: &mut UdpSocket, current_cores: u16) -> io::Result<f32> {
        let latency = MyAgoraEnv::agora_latency_sgd(self, socket, current_cores).await?;
        println!("current_cores: {}, latency: {}\n", current_cores, latency);

        Ok(latency)
    }

    // Detect oscillations in latency
    fn detect_oscillations_sgd(&mut self, current_cores: u16, delta: u8, latency1: f32, latency2: f32) -> (f32, f32, f32, f32, u16) {
        println!("current_cores: {}, delta: {}, latency1: {}, latency2: {}\n", current_cores, delta, latency1, latency2);
        let current_cores_rl = MyAgoraEnv::agora_to_rl_index_mapping(self, current_cores as u16);
        if self.cores_latency_sgd[current_cores_rl as usize] != 0.0 {
            self.cores_latency_sgd[current_cores_rl as usize] = (self.cores_latency_sgd[current_cores_rl as usize] + latency1) / 2.0;
        } else {
            self.cores_latency_sgd[current_cores_rl as usize] = latency1;
        }
        if self.cores_latency_sgd[current_cores_rl as usize + delta as usize] != 0.0 {
            self.cores_latency_sgd[current_cores_rl as usize + delta as usize] = (self.cores_latency_sgd[current_cores_rl as usize + delta as usize] + latency2) / 2.0;
        } else {
            self.cores_latency_sgd[current_cores_rl as usize + delta as usize] = latency2;
        }
        let cores_latency_sgd_elements = &self.cores_latency_sgd;
        println!("cores_latency_sgd (after update) {:?}\n", cores_latency_sgd_elements);

        let mut increasing_latency;
        let mut decreasing_latency;
        let mut latency_pattern = 0; // 0 - No Pattern; 1 - Descending; 2 - Oscillating
        increasing_latency = vec![false; self.cores_latency_window_sgd as usize - 1];
        decreasing_latency = vec![false; self.cores_latency_window_sgd as usize - 1];
        if latency_pattern == 0 {
            for count in 0..(self.cores_latency_window_sgd - 1) {
                if self.cores_latency_sgd[current_cores_rl as usize + delta as usize - count as usize] != 0.0 && self.cores_latency_sgd[current_cores_rl as usize - count as usize] != 0.0 {
                    // println!("current_cores: {}, delta: {}, count: {}, cores_latency_sgd[current_cores_rl + delta - count]: {}, cores_latency_sgd[current_cores_rl - count]: {}\n",
                    // MyAgoraEnv::rl_to_agora_index_mapping(self, current_cores_rl as u16), delta, count, self.cores_latency_sgd[current_cores_rl as usize + delta as usize - count as usize], self.cores_latency_sgd[current_cores_rl as usize - count as usize]);
                    if self.cores_latency_sgd[current_cores_rl as usize + delta as usize - count as usize] > self.cores_latency_sgd[current_cores_rl as usize - count as usize] {
                        increasing_latency[count as usize] = true;
                    } else if self.cores_latency_sgd[current_cores_rl as usize + delta as usize - count as usize] < self.cores_latency_sgd[current_cores_rl as usize - count as usize] {
                        decreasing_latency[count as usize] = true;
                    }
                }
            }

            for count in 0..(self.cores_latency_window_sgd - 1) {
                if decreasing_latency[count as usize] == true {
                    latency_pattern = 1; // Descending
                } else {
                    latency_pattern = 0; // No Pattern
                }
            }
                
            if increasing_latency[0 as usize] == true && decreasing_latency[1 as usize] == true {
                latency_pattern = 2; // Oscillating
            }

            let increasing_latency_elements = &increasing_latency;
            println!("increasing_latency (printing reverse): {:?}\n", increasing_latency_elements);
            let decreasing_latency_elements = &decreasing_latency;
            println!("decreasing_latency (printing reverse): {:?}\n", decreasing_latency_elements);
        }
        
        (self.cores_latency_sgd[current_cores_rl as usize - 2 * delta as usize], self.cores_latency_sgd[current_cores_rl as usize - delta as usize], self.cores_latency_sgd[current_cores_rl as usize], self.cores_latency_sgd[current_cores_rl as usize + delta as usize], latency_pattern)
    }

    // SGD optimization
    async fn optimization_sgd(&mut self, socket: &mut UdpSocket, start_cores: u16, num_iter: u16) -> io::Result<u16> {
        let cores = start_cores;
        let delta = 1;
        let mut update_cores = 1;
        let mut back_cores = self.num_max_cores_sgd;
        let optimal_cores;
        let mut oscillation_started = false;
        let mut latency1 = MyAgoraEnv::measure_latency_sgd(self, socket, cores).await?;
        for iter_count in 0..num_iter {
            let latency2 = MyAgoraEnv::measure_latency_sgd(self, socket, cores + update_cores as u16).await?;

            let detect_oscillations_out = MyAgoraEnv::detect_oscillations_sgd(self, cores + update_cores as u16 - 1 as u16, 1 as u8, latency1, latency2);
            let osc_latency0 = detect_oscillations_out.0;
            let osc_latency1 = detect_oscillations_out.1;
            let osc_latency2 = detect_oscillations_out.2;
            let osc_latency3 = detect_oscillations_out.3;
            let latency_pattern = detect_oscillations_out.4;
            let gradient = (latency2 - latency1) / (delta as f32);
            println!("iter_count: {}, current_cores: {}, osc_latency0: {}, osc_latency1: {}, osc_latency2: {}, osc_latency3: {}, latency_pattern: {}, gradient: {}\n",
                iter_count, cores + iter_count as u16 + 1 as u16, osc_latency0, osc_latency1, osc_latency2, osc_latency3, latency_pattern, gradient);

            println!("Before update: oscillation_started: {}, update_cores: {}, back_cores: {}\n", oscillation_started, update_cores, back_cores);
            if latency_pattern == 2 { // Oscillating
                oscillation_started = true;
                if update_cores >= self.cores_latency_window_sgd - 1 as u16 {
                    back_cores = self.cores_latency_window_sgd - 1 as u16;
                } else {
                    back_cores = update_cores;
                }
                update_cores = update_cores - back_cores; // Go back in cores
                latency1 = MyAgoraEnv::measure_latency_sgd(self, socket, cores + update_cores as u16).await?;
                update_cores = update_cores + 1;
                back_cores = back_cores - 2;
            } else if latency_pattern == 1 { // Descending
                if oscillation_started && back_cores == 0 {
                    optimal_cores = cores + update_cores as u16;
                    return Ok(optimal_cores); // Terminate the loop by returning previous cores    
                } else {
                    // Update cores
                    update_cores = update_cores + 1;
                    back_cores = back_cores - 1;
                    // Copying latency value
                    latency1 = latency2;
                }
            } else { // No Pattern
                // Update cores
                update_cores = update_cores + 1;
                // back_cores = back_cores - 1;
                // Copying latency value
                latency1 = latency2;
            }
            println!("After udpate: oscillation_started: {}, update_cores: {}, back_cores: {}\n", oscillation_started, update_cores, back_cores);

            // A simple gradient check based on sign change below
            // if gradient > 0.0 {
            //     return Ok(cores - 1); // Terminate the loop by returning previous cores
            // }
            // cores = (cores as f32 + diff).round() as u16;
            // println!("cores: {}\n", cores);
            // if cores < self.num_min_cores_sgd {
            //     cores = self.num_min_cores_sgd; // Ensure at least the minimum number of CPU cores
            // } else if cores > self.num_max_cores_sgd {
            //     cores = self.num_max_cores_sgd; // Ensure no more than the maximum number of CPU cores
            // }
            // println!("cores: {}\n", cores);
        }

        Ok(cores)
    }
}

fn argmax_row<T: PartialOrd + Copy>(matrix: &[Vec<T>], row_index: usize) -> usize {
    matrix[row_index].iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(Ordering::Equal))
        .map(|(index, _)| index)
        .unwrap_or_default()
}

// Communication parameters
static TX_ADDR: &str = "127.0.0.1:4000";
static RX_ADDR: &str = "127.0.0.1:3000";

// Resource parameters
static CORES_STEP: u8 = 1;

// CAUTION: Change UDP_WAIT_TIME and REPEAT_REQUEST based on RP_MODE
// static RP_MODE: u8 = 2; // RL approach
// static UDP_WAIT_TIME: u64 = 15; // for RL
// static REPEAT_REQUEST: u64 = 30; // for RL

static RP_MODE: u8 = 3; // SGD approach
static UDP_WAIT_TIME: u64 = 50; // Trying to set double of radio frame duration for SGD
static REPEAT_REQUEST: u64 = 15; // Matching to radio frame duration for SGD

static PERIODICITY: u64 = 500; // sends request every PERIODICITY ms
static LATENCY_UPPER_TH: u64 = 1000; // 1 ms is the maximum latency limit per frame for Agora
static LATENCY_LOWER_TH: u64 = 750; // 0.75 ms is the minimum latency limit per frame for Agora

#[tokio::main]
async fn main() -> io::Result<()> {
    let mut socket = create_udp_socket(RX_ADDR).await?;
    let mut one_time_cores_update = true;
    delay_for(Duration::from_secs(1)).await; // wait for 5s to start
    let retrieved_msg = retrieve_agora_config(&mut socket).await?;
    let cores_for_rest = retrieved_msg[0]; // retrieved_msg[0] has the cores allocated for rest of the processing in Agora
    let max_cores_for_workers = retrieved_msg[1] - retrieved_msg[0]; // retrieved_msg[1] has the max cores available for workers
    let min_cores_for_workers = retrieved_msg[2];  // retrieved_msg[2] has the min cores available for workers
    let max_bs_ants = retrieved_msg[3]; // retrieved_msg[3] has the max bs ants configured
    let max_users = retrieved_msg[4]; // retrieved_msg[4] has the max users configured
    println!("Agora cores details: cores for rest - {}, max cores for workers - {}, min core(s) for workers - {}, max bs ant(s) - {}, max user(s) - {}\n",
        cores_for_rest, max_cores_for_workers, min_cores_for_workers, max_bs_ants, max_users);
    loop {
        if RP_MODE == 0 {  // Retrieve Agora status periodically
            delay_for(Duration::from_millis(PERIODICITY)).await;
            let retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
            println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
        } else if RP_MODE == 1 { // Update cores dynamically
            delay_for(Duration::from_secs(1)).await; // sends request every 5s
            let retrieved_msg = retrieve_agora_traffic(&mut socket).await?; // TODO: implement timeout!
            println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
            if one_time_cores_update == true {
                if retrieved_msg[0] >= LATENCY_UPPER_TH {
                    let mut add_cores = CORES_STEP;
                    if retrieved_msg[1] as u8 + add_cores > max_cores_for_workers as u8 {
                        add_cores = max_cores_for_workers as u8 - retrieved_msg[1] as u8;
                    }
                    if add_cores > 0 {
                        println!("Too much load in Agora: add {:?} cores\n", add_cores);
                        send_cores_control_message(&mut socket, add_cores, 0, retrieved_msg[1] as u8).await?;
                    }
                    if add_cores <= 0 {
                        println!("Reached maximum cores limit of {:?}\n", max_cores_for_workers);
                    }
                } else if retrieved_msg[0] < LATENCY_LOWER_TH {
                    let mut remove_cores = CORES_STEP;
                    if retrieved_msg[1] as u8 - remove_cores < min_cores_for_workers as u8 {
                        remove_cores = retrieved_msg[1] as u8 - min_cores_for_workers as u8;
                    }
                    if remove_cores > 0 {
                        println!("Too relaxed load in Agora: remove {:?} cores\n", remove_cores);
                        send_cores_control_message(&mut socket, 0, remove_cores, retrieved_msg[1] as u8).await?;
                    }
                    if remove_cores <= 0 {
                        println!("Reached minimum cores limit of {:?}\n", min_cores_for_workers);
                    }
                } else {
                }
                one_time_cores_update = !one_time_cores_update;
            }
        } else if RP_MODE == 2 { // RL based Resource Provisioning
            if one_time_cores_update == true {
                let is_real_agora = true;
                let is_training_enabled = false;
                let is_testing_enabled = true;
                let num_max_cores;
                let num_min_cores;
                let num_max_users;
                let num_latency_levels;
                let num_actions;
                let max_latency_limit;
                let ma_window_size = 10;
                let data_outlier_percentage = 10;
                let rewards;

                if is_real_agora == true {
                    num_max_cores = 10;
                    num_min_cores = 3;
                    num_max_users = 16;
                    num_latency_levels = 10 + 1; // + 1 --> To account for max_latency_limit and beyond latency values
                    num_actions = 3;
                    max_latency_limit = 1000.0;
                    rewards = vec![-35, -30, -25, -20, -15, -10, -5, 40, -30, -35, -40];
                } else {
                    num_max_cores = 10;
                    num_min_cores = 1;
                    num_max_users = 16;
                    num_latency_levels = 10 + 1; // + 1 --> To account for max_latency_limit and beyond latency values
                    num_actions = 3;
                    max_latency_limit = 0.001;
                    rewards = vec![-12, -10, -8, -6, -4, -2, -1, 20, -12, -16, -20];
                }
        
                // Q-learning parameters
                let alpha = 0.1;  // learning rate
                let gamma = 0.9;  // discount factor
                let epsilon = 0.1;  // exploration-exploitation trade-off

                // num_episodes and terminate_count can be modified as needed based on state space size and training accuracy 
                let num_episodes = 1000; // Episodes count for training
                let terminate_count = 1000; // Epochs count to terminate training when not converging

                let consecutive_epochs_exit_count = 5; // Number of consecutive epochs with same state and reward to treat the training converged. Applicable when the target reward is the least negative.
                let alternative_epochs_exit_count = 5; // Number of alternative epochs with same state and reward to treat the training converged. Applicable when the target reward is the least negative.
                let record_count = 5;
                let alternating_epochs1_count = 3;
                let alternating_epochs2_count = alternative_epochs_exit_count - alternating_epochs1_count;

                // Create an instance of the agora environment using the 'new' function
                let mut agora_env = MyAgoraEnv::new();
                agora_env.set_initial_values(num_max_cores, num_min_cores, num_max_users, num_latency_levels, num_actions, max_latency_limit, ma_window_size as u16, data_outlier_percentage, rewards, 0, false); // Set initial state to 2 and done to false

                let num_states = agora_env.get_num_states();
                println!("num_states {:?}\n", num_states);

                // Try to open existing cores_users_absolute_latency file
                let file_result = OpenOptions::new().read(true).open("cores_users_absolute_latency.json");
                let mut cores_users_absolute_latency: Vec<Vec<Vec<f32>>> = if let Ok(file) = file_result {
                    // Read existing cores_users_absolute_latency content and parse JSON
                    let reader = BufReader::new(file);
                    let content: Vec<Vec<Vec<f32>>> = serde_json::from_reader(reader).unwrap_or_else(|_| vec![vec![vec![0.0; num_latency_levels as usize]; num_max_users as usize]; num_max_cores as usize]);
                    println!("cores_users_absolute_latency loaded from cores_users_absolute_latency.json");
                    content
                } else {
                    println!("cores_users_absolute_latency.json does not exist. Creating cores_users_absolute_latency.json and initializing with zeros");
                    vec![vec![vec![0.0; num_latency_levels as usize]; num_max_users as usize]; num_max_cores as usize]
                };
            
                // Initialize q_table with zeros only if q_table.json doesn't exist
                let mut q_table = if let Ok(file_content) = fs::read_to_string("q_table.json") {
                    // Deserialize q_table from JSON content if q_table.json exists
                    if let Ok(deserialized) = serde_json::from_str::<Vec<Vec<f32>>>(&file_content) {
                        println!("q_table loaded from q_table.json");
                        deserialized
                    } else {
                        println!("Failed to deserialize q_table from q_table.json. Initializing with zeros");
                        vec![vec![0.0; num_actions as usize]; num_states as usize]
                    }
                } else {
                    println!("q_table.json does not exist. Creating q_table.json and initializing with zeros");
                    vec![vec![0.0; num_actions as usize]; num_states as usize]
                };

                if is_training_enabled == true {
                    // Training
                    println!("Training started - Running for {} Episodes ...", num_episodes);

                    println!("Printing current Q Table contents before Training ...");
                    println!("{:?}", q_table);

                    // Reset ma filter
                    println!("Resetting moving average filter contents ...");
                    agora_env.reset_ma_filter(num_max_cores, num_max_users, ma_window_size as u16);

                    for episode in 1..= num_episodes {
                        println!("Episode: {:?}\n", episode);

                        // Reset the state with a random value
                        let mut state;
                        // let mut state = agora_env.reset();
                        let mut rng = rand::thread_rng();
                        let reset_cores = rng.gen_range((num_min_cores as u16)..(num_max_cores as u16));
                        let mut epochs = 0;
                        let mut record_rewards = vec![0; record_count as usize];
                        let mut record_states = vec![0; record_count as usize];
                        let mut agora_cores;
                        let mut next_state;
                        let mut reward = -40;
                        let mut action;
                        let mut done = false;
    
                        println!("Training: Re-initializing cores in Agora as per reset state - START\n");
                        // delay_for(Duration::from_millis(UDP_WAIT_TIME)).await;
                        let mut retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                        println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                        let agora_start_cores = retrieved_msg[1];

                        let reset_cores_ag = agora_env.rl_to_agora_index_mapping(reset_cores);
                        println!("Agora to be reset with {} cores ...\n", reset_cores_ag);
                        let mut curr_cores;
                        if reset_cores_ag as u8 > agora_start_cores as u8 {
                            let mut add_cores = CORES_STEP;
                            curr_cores = agora_start_cores as u8;
                            for _step_cores in (0..(reset_cores_ag as u8 - agora_start_cores as u8)).step_by(CORES_STEP as usize) {
                                if curr_cores as u8 + add_cores > max_cores_for_workers as u8 {
                                    add_cores = max_cores_for_workers as u8 - curr_cores as u8;
                                }
                                if add_cores > 0 {
                                    println!("Start cores in Agora {:?}; Re-initializing cores in Agora to {:?}; Target reset cores in Agora {:?}\n", agora_start_cores, curr_cores as u8 + add_cores, reset_cores_ag);
                                    send_cores_control_message(&mut socket, add_cores, 0, curr_cores as u8).await?;
                                    retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                                    println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                                }
                                if add_cores <= 0 {
                                    println!("Reached maximum cores limit of {:?}\n", max_cores_for_workers);
                                }
                                curr_cores = curr_cores as u8 + add_cores;
                            }
                        } else {
                            let mut remove_cores = CORES_STEP;
                            curr_cores = agora_start_cores as u8;
                            for _step_cores in (0..(agora_start_cores as u8 - reset_cores_ag as u8)).step_by(CORES_STEP as usize) {
                                if curr_cores as u8 - remove_cores < min_cores_for_workers as u8 {
                                    remove_cores = curr_cores as u8 - min_cores_for_workers as u8;
                                }
                                if remove_cores > 0 {
                                    println!("Start cores in Agora {:?}; Re-initializing cores in Agora to {:?}; Target cores in Agora {:?}\n", agora_start_cores, curr_cores as u8 - remove_cores, reset_cores_ag);
                                    send_cores_control_message(&mut socket, 0, remove_cores, curr_cores as u8).await?;
                                    retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                                    println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                                }
                                if remove_cores <= 0 {
                                    println!("Reached maximum cores limit of {:?}\n", min_cores_for_workers);
                                }
                                curr_cores = curr_cores as u8 - remove_cores;
                            }
                        }
                        println!("Training: Re-initializing cores in Agora as per reset state - END\n");

                        // delay_for(Duration::from_millis(UDP_WAIT_TIME)).await;
                        let retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                        let reset_absolute_latency = retrieved_msg[0];
                        let reset_cores = retrieved_msg[1];
                        let reset_users = retrieved_msg[2];
                        let curr_latency_reward_done_real = agora_env.compute_curr_latency_reward_done_real(reset_absolute_latency as f32);
                        let reset_latency = curr_latency_reward_done_real.0;
                        println!("Agora status: reset absolute latency - {}, reset cores - {}, reset users - {}, frame id - {}\n",
                            reset_absolute_latency, reset_cores, reset_users, retrieved_msg[3]);
                        let reset_cores_rl = agora_env.agora_to_rl_index_mapping(reset_cores as u16);
                        let reset_users_rl = agora_env.agora_to_rl_index_mapping(reset_users as u16);
                        state = agora_env.compute_state(reset_cores_rl as u16, reset_users_rl as u16, reset_latency as u16);

                        while done == false {
                            // Count epochs
                            epochs += 1;

                            let current_tsc = MyAgoraEnv::rdtsc();
                            println!("TSC: {:?}, Episode {:?}, Epoch {:?}\n", current_tsc, episode, epochs);
                            agora_env.set(state);
                            let users_before_action = agora_env.get_curr_users(state);
                            let users_before_action_ag = agora_env.rl_to_agora_index_mapping(users_before_action as u16);
                            // agora_env.print_state(state, UDP_WAIT_TIME).await;
                            println!("elements of state (before update): {}", state);
                            for &row_element in &q_table[state as usize] {
                                println!("{}", row_element);
                            }

                            let explore_exploit_rand = rng.gen_range(0.0..1.0);
                            if explore_exploit_rand < epsilon { // Explore action space
                                action = rng.gen_range(0..num_actions as usize - 1);
                                println!("Explore: action for state {}: {} (0 - No change in cores, 1 - Add one core, 2 - Remove one core)", state, action);
                            } else { // Exploit learned values
                                action = argmax_row(&q_table, state as usize);
                                println!("Exploit: action for state {}: {} (0 - No change in cores, 1 - Add one core, 2 - Remove one core)", state, action);
                            }

                            if is_real_agora == true {
                                let ma_absolute_latency;
                                let agora_users;
                                (ma_absolute_latency, agora_cores, agora_users, next_state, reward, done) = agora_env.step_real(&mut socket, action as u16).await?;
                                println!("step_output: ma_absolute_latency {}, current_cores {}, current_users {}, next_state {}, reward {}, done {}\n",
                                    ma_absolute_latency, agora_cores, agora_users, next_state, reward, done);
                                let agora_cores_rl = agora_env.agora_to_rl_index_mapping(agora_cores as u16);
                                let agora_users_rl = agora_env.agora_to_rl_index_mapping(agora_users as u16);
                                cores_users_absolute_latency[agora_cores_rl as usize][agora_users_rl as usize].push(ma_absolute_latency);
                            } else {
                                let step_output = agora_env.step_emulated(action as u16);
                                next_state = step_output.0;
                                reward = step_output.1;
                                done = step_output.2;
                                println!("step_output: next_state {}, reward {}, done {}\n", next_state, reward, done);
                            }

                            // Record current state and reward
                            for count in 1..=(record_count - 1) {
                                record_states[count as usize - 1] = record_states[count as usize];
                                record_rewards[count as usize - 1] = record_rewards[count as usize];
                            }
                            record_states[record_count as usize - 1] = state;
                            record_rewards[record_count as usize - 1] = reward;

                            // Initialize count and mask for repetition checking
                            let mut repetition_count = vec![1; record_count as usize];
                            let mut repetition_mask = vec![1; record_count as usize];
                            for count in 0..record_count {
                                if record_states[count as usize] == 0 {
                                    repetition_count[count as usize] = 0;
                                    repetition_mask[count as usize] = 0;
                                }
                            }

                            // Counting state and reward occurences
                            for count1 in 1..=record_count {
                                for count2 in 1..=(record_count - count1) {
                                    if repetition_mask[record_count as usize - count1 as usize] == 1 {
                                        if record_states[record_count as usize - count1] == record_states[record_count as usize - count1 as usize - count2 as usize] && record_rewards[record_count as usize - count1] == record_rewards[record_count as usize - count1 as usize - count2 as usize] {
                                            repetition_count[record_count as usize - count1] = repetition_count[record_count as usize - count1] + 1;
                                            repetition_count[record_count as usize - count1 as usize - count2 as usize] = 0;
                                            repetition_mask[record_count as usize - count1 as usize - count2 as usize] = 0;
                                        }
                                    }
                                }
                            }

                            // Sorting the elements of repetition_count in descending order
                            repetition_count.sort_by(|a, b| b.cmp(a));

                            // Check continuous epochs, states and rewards condition to exit training
                            if repetition_count[0 as usize] == consecutive_epochs_exit_count {
                                done = true;
                                for count in 0..=(record_count - 1) {
                                    println!("record_states[count] {}, record_rewards[count] {}, count {}\n", record_states[count as usize], record_rewards[count as usize], count);
                                }
                                println!("Terminating training for Episode {} based on continous count of {} Epochs with same state and reward ...\n",
                                    episode, consecutive_epochs_exit_count);
                            }

                            // Check alternating epochs, states and rewards condition to exit training
                            if repetition_count[0 as usize] == alternating_epochs1_count && repetition_count[1 as usize] == alternating_epochs2_count {
                                done = true;
                                for count in 0..=(record_count - 1) {
                                    println!("record_states[count] {}, record_rewards[count] {}, count {}\n", record_states[count as usize], record_rewards[count as usize], count);
                                }
                                println!("Terminating training for Episode {} based on alternating count of {} Epochs with alternate state and reward (need not be ordered) ...\n",
                                    episode, alternative_epochs_exit_count);
                            }
    
                            let users_after_action = agora_env.get_curr_users(next_state);
                            let users_after_action_ag = agora_env.rl_to_agora_index_mapping(users_after_action as u16);

                            if users_before_action_ag == users_after_action_ag {
                                // Q-value update using the Q-learning update rule
                                let old_value = q_table[state as usize][action];
                                let next_max = q_table[next_state as usize].iter().cloned().fold(f32::NEG_INFINITY, f32::max);
                                let new_value = old_value + alpha * (reward as f32 + gamma * next_max - old_value);
                                    q_table[state as usize][action] = new_value;
                                println!("elements of state (after update): {}", state);
                                for &row_element in &q_table[state as usize] {
                                    println!("{}", row_element);
                                }
                            } else {
                                println!("curr_users before action: {}; curr_users before action: {}", users_before_action_ag, users_after_action_ag);
                                println!("curr_users changed during action. Ignoring Q table update for state: {}", state);
                            }
    
                            // Update state
                            state = next_state;

                            if epochs == terminate_count {
                                println!("Terminating training for Episode {} based on terminal count of {} Epochs ...\n",
                                    episode, terminate_count);
                                done = true;
                            }
                        }
                        println!("episode {:?} state {:?} reward {:?} Epochs {:?}\n", episode, state, reward, epochs);

                        // println!("{:?}", q_table);

                        // Write updated cores_users_absolute_latency content to the file
                        let mut file = File::create("cores_users_absolute_latency.json")?;
                        let json_content = serde_json::to_string(&cores_users_absolute_latency)?;
                        file.write_all(json_content.as_bytes())?;

                        // Serialize q_table to JSON
                        let serialized = serde_json::to_string_pretty(&q_table)?;
                        // Write q_table JSON content to file
                        let mut file = File::create("q_table.json")?;
                        file.write_all(serialized.as_bytes())?;
                        println!("Updating q_table.json after Episode {}\n", episode);
                    }
                    println!("Training completed ...");
                    // println!("Printing current Q Table contents after Training ...");
                    // println!("{:?}", q_table);
                }

                if is_testing_enabled == true {
                    // Testing
                    println!("Testing started ...");

                    println!("Printing current Q Table contents before Testing ...");
                    println!("{:?}", q_table);
    
                    // Reset ma filter
                    println!("Resetting moving average filter contents ...");
                    agora_env.reset_ma_filter(num_max_cores, num_max_users, ma_window_size as u16);

                    // Reset the state with a random value
                    let mut state;
                    // let mut state = agora_env.reset();
                    let mut rng = rand::thread_rng();
                    let reset_cores = rng.gen_range((num_min_cores as u16)..(num_max_cores as u16));
                    let mut epochs = 0;
                    let mut record_rewards = vec![0; record_count as usize];
                    let mut record_states = vec![0; record_count as usize];
                    let mut agora_cores;
                    let mut next_state;
                    let mut reward = -40;
                    let mut action;
                    let mut done = false;

                    println!("Testing: Re-initializing cores in Agora as per reset state - START\n");
                    // delay_for(Duration::from_millis(UDP_WAIT_TIME)).await;
                    let mut retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                    println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                    let agora_start_cores = retrieved_msg[1];

                    let reset_cores_ag = agora_env.rl_to_agora_index_mapping(reset_cores);
                    println!("Agora to be reset with {} cores ...\n", reset_cores_ag);
                    let mut curr_cores;
                    if reset_cores_ag as u8 > agora_start_cores as u8 {
                        let mut add_cores = CORES_STEP;
                        curr_cores = agora_start_cores as u8;
                        for _step_cores in (0..(reset_cores_ag as u8 - agora_start_cores as u8)).step_by(CORES_STEP as usize) {
                            if curr_cores as u8 + add_cores > max_cores_for_workers as u8 {
                                add_cores = max_cores_for_workers as u8 - curr_cores as u8;
                            }
                            if add_cores > 0 {
                                println!("Start cores in Agora {:?}; Re-initializing cores in Agora to {:?}; Target reset cores in Agora {:?}\n", agora_start_cores, curr_cores as u8 + add_cores, reset_cores_ag);
                                send_cores_control_message(&mut socket, add_cores, 0, curr_cores as u8).await?;
                                retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                                println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                            }
                            if add_cores <= 0 {
                                println!("Reached maximum cores limit of {:?}\n", max_cores_for_workers);
                            }
                            curr_cores = curr_cores as u8 + add_cores;
                        }
                    } else {
                        let mut remove_cores = CORES_STEP;
                        curr_cores = agora_start_cores as u8;
                        for _step_cores in (0..(agora_start_cores as u8 - reset_cores_ag as u8)).step_by(CORES_STEP as usize) {
                            if curr_cores as u8 - remove_cores < min_cores_for_workers as u8 {
                                remove_cores = curr_cores as u8 - min_cores_for_workers as u8;
                            }
                            if remove_cores > 0 {
                                println!("Start cores in Agora {:?}; Re-initializing cores in Agora to {:?}; Target cores in Agora {:?}\n", agora_start_cores, curr_cores as u8 - remove_cores, reset_cores_ag);
                                send_cores_control_message(&mut socket, 0, remove_cores, curr_cores as u8).await?;
                                retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                                println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                            }
                            if remove_cores <= 0 {
                                println!("Reached maximum cores limit of {:?}\n", min_cores_for_workers);
                            }
                            curr_cores = curr_cores as u8 - remove_cores;
                        }
                    }
                    println!("Testing: Re-initializing cores in Agora as per reset state - END\n");

                    // delay_for(Duration::from_millis(UDP_WAIT_TIME)).await;
                    let retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                    let reset_absolute_latency = retrieved_msg[0];
                    let reset_cores = retrieved_msg[1];
                    let reset_users = retrieved_msg[2];
                    let curr_latency_reward_done_real = agora_env.compute_curr_latency_reward_done_real(reset_absolute_latency as f32);
                    let reset_latency = curr_latency_reward_done_real.0;
                    println!("Agora status: reset absolute latency - {}, reset cores - {}, reset users - {}, frame id - {}\n",
                        reset_absolute_latency, reset_cores, reset_users, retrieved_msg[3]);
                    let reset_cores_rl = agora_env.agora_to_rl_index_mapping(reset_cores as u16);
                    let reset_users_rl = agora_env.agora_to_rl_index_mapping(reset_users as u16);
                    state = agora_env.compute_state(reset_cores_rl as u16, reset_users_rl as u16, reset_latency as u16);

                    while done == false {
                        // Count epochs
                        epochs += 1;

                        println!("Epoch {:?}\n", epochs);
                        agora_env.set(state);
                        println!("elements of state (before update): {}", state);
                        for &row_element in &q_table[state as usize] {
                            println!("{}", row_element);
                        }

                        let explore_exploit_rand = rng.gen_range(0.0..1.0);
                        if explore_exploit_rand < epsilon { // Explore action space
                            action = rng.gen_range(0..num_actions as usize - 1);
                            println!("Explore: action for state {}: {} (0 - No change in cores, 1 - Add one core, 2 - Remove one core)", state, action);
                        } else { // Exploit learned values
                            action = argmax_row(&q_table, state as usize);
                            println!("Exploit: action for state {}: {} (0 - No change in cores, 1 - Add one core, 2 - Remove one core)", state, action);
                        }

                        if is_real_agora == true {
                            let ma_absolute_latency;
                            let agora_users;
                            (ma_absolute_latency, agora_cores, agora_users, next_state, reward, done) = agora_env.step_real(&mut socket, action as u16).await?;
                            println!("step_output: ma_absolute_latency {}, current_cores {}, current_users {}, next_state {}, reward {}, done {}\n",
                                ma_absolute_latency, agora_cores, agora_users, next_state, reward, done);    
                            let agora_cores_rl = agora_env.agora_to_rl_index_mapping(agora_cores as u16);
                            let agora_users_rl = agora_env.agora_to_rl_index_mapping(agora_users as u16);
                            cores_users_absolute_latency[agora_cores_rl as usize][agora_users_rl as usize].push(ma_absolute_latency);
                        } else {
                            let step_output = agora_env.step_emulated(action as u16);
                            next_state = step_output.0;
                            reward = step_output.1;
                            done = step_output.2;
                            println!("step_output: next_state {}, reward {}, done {}\n", next_state, reward, done);    
                        }

                        // Record current state and reward
                        for count in 1..=(record_count - 1) {
                            record_states[count as usize - 1] = record_states[count as usize];
                            record_rewards[count as usize - 1] = record_rewards[count as usize];
                        }
                        record_states[record_count as usize - 1] = state;
                        record_rewards[record_count as usize - 1] = reward;

                        // Initialize count and mask for repetition checking
                        let mut repetition_count = vec![1; record_count as usize];
                        let mut repetition_mask = vec![1; record_count as usize];
                        for count in 0..record_count {
                            if record_states[count as usize] == 0 {
                                repetition_count[count as usize] = 0;
                                repetition_mask[count as usize] = 0;
                            }
                        }

                        // Counting state and reward occurences
                        for count1 in 1..=record_count {
                            for count2 in 1..=(record_count - count1) {
                                if repetition_mask[record_count as usize - count1 as usize] == 1 {
                                    if record_states[record_count as usize - count1] == record_states[record_count as usize - count1 as usize - count2 as usize] && record_rewards[record_count as usize - count1] == record_rewards[record_count as usize - count1 as usize - count2 as usize] {
                                        repetition_count[record_count as usize - count1] = repetition_count[record_count as usize - count1] + 1;
                                        repetition_count[record_count as usize - count1 as usize - count2 as usize] = 0;
                                        repetition_mask[record_count as usize - count1 as usize - count2 as usize] = 0;
                                    }
                                }
                            }
                        }

                        // Sorting the elements of repetition_count in descending order
                        repetition_count.sort_by(|a, b| b.cmp(a));

                        // Check continuous epochs, states and rewards condition to exit training
                        if repetition_count[0 as usize] == consecutive_epochs_exit_count {
                            done = true;
                            for count in 0..=(record_count - 1) {
                                println!("record_states[count] {}, record_rewards[count] {}, count {}\n", record_states[count as usize], record_rewards[count as usize], count);
                            }
                            println!("Terminating training based on continous count of {} Epochs with same state and reward ...\n",
                                consecutive_epochs_exit_count);
                        }

                        // Check alternating epochs, states and rewards condition to exit training
                        if repetition_count[0 as usize] == alternating_epochs1_count && repetition_count[1 as usize] == alternating_epochs2_count {
                            done = true;
                            for count in 0..=(record_count - 1) {
                                println!("record_states[count] {}, record_rewards[count] {}, count {}\n", record_states[count as usize], record_rewards[count as usize], count);
                            }
                            println!("Terminating training based on alternating count of {} Epochs with alternate state and reward (need not be ordered) ...\n",
                                alternative_epochs_exit_count);
                        }

                        // Update state
                        state = next_state;

                        if epochs == terminate_count {
                            println!("Terminating testing based on terminal count of {} Epochs ...\n", terminate_count);
                            done = true;
                        }
                    }
                    println!("state {:?} reward {:?} Epochs {:?}\n", state, reward, epochs);
                    println!("Testing completed ...");
                }
                one_time_cores_update = !one_time_cores_update;
            }
        } else if RP_MODE == 3 { // Steepest Gradient Descent Approach for Dynamic Resource Provisioing
            if one_time_cores_update == true {
                let num_max_cores = 20;
                let num_min_cores = 3;
                let num_max_users = 16;
                let ma_window_size = 15;
                let max_latency_limit = 1000.0;
                let data_outlier_percentage = 10;
                let learn_rate = 0.01;
                let num_iter = 50;
                let tolerance = 0.001;
                let cores_latency_window = 4;

                // Create an instance of the agora environment using the 'new' function
                let mut agora_env = MyAgoraEnv::new();
                agora_env.set_initial_values_sgd(num_max_cores, num_min_cores, num_max_users, ma_window_size as u16, max_latency_limit, data_outlier_percentage, learn_rate, tolerance, cores_latency_window); // Set initial values

                // Reset ma filter
                println!("Resetting moving average filter contents ...");
                agora_env.reset_ma_filter_sgd(num_max_cores, num_max_users, ma_window_size as u16);

                // let mut rng = rand::thread_rng();
                // let mut reset_cores = rng.gen_range((num_min_cores as u16)..(num_max_cores as u16));
                let reset_cores = 3;

                println!("SGD: Re-initializing cores in Agora - START\n");
                // delay_for(Duration::from_millis(UDP_WAIT_TIME)).await;
                let mut retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                let agora_start_cores = retrieved_msg[1] as u16;

                println!("Agora to be reset with {} cores ...\n", reset_cores);
                let mut curr_cores;
                if reset_cores as u8 > agora_start_cores as u8 {
                    let mut add_cores = CORES_STEP;
                    curr_cores = agora_start_cores as u8;
                    for _step_cores in (0..(reset_cores as u8 - agora_start_cores as u8)).step_by(CORES_STEP as usize) {
                        if curr_cores as u8 + add_cores > max_cores_for_workers as u8 {
                            add_cores = max_cores_for_workers as u8 - curr_cores as u8;
                        }
                        if add_cores > 0 {
                            println!("Start cores in Agora {:?}; Re-initializing cores in Agora to {:?}; Target reset cores in Agora {:?}\n", agora_start_cores, curr_cores as u8 + add_cores, reset_cores);
                            send_cores_control_message(&mut socket, add_cores, 0, curr_cores as u8).await?;
                            retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                            println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                        }
                        if add_cores <= 0 {
                            println!("Reached maximum cores limit of {:?}\n", max_cores_for_workers);
                        }
                        curr_cores = curr_cores as u8 + add_cores;
                    }
                } else {
                    let mut remove_cores = CORES_STEP;
                    curr_cores = agora_start_cores as u8;
                    for _step_cores in (0..(agora_start_cores as u8 - reset_cores as u8)).step_by(CORES_STEP as usize) {
                        if curr_cores as u8 - remove_cores < min_cores_for_workers as u8 {
                            remove_cores = curr_cores as u8 - min_cores_for_workers as u8;
                        }
                        if remove_cores > 0 {
                            println!("Start cores in Agora {:?}; Re-initializing cores in Agora to {:?}; Target cores in Agora {:?}\n", agora_start_cores, curr_cores as u8 - remove_cores, reset_cores);
                            send_cores_control_message(&mut socket, 0, remove_cores, curr_cores as u8).await?;
                            retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                            println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
                        }
                        if remove_cores <= 0 {
                            println!("Reached maximum cores limit of {:?}\n", min_cores_for_workers);
                        }
                        curr_cores = curr_cores as u8 - remove_cores;
                    }
                }
                println!("SGD: Re-initializing cores in Agora - END\n");
                let optimal_cores = agora_env.optimization_sgd(&mut socket, reset_cores, num_iter).await?;
                println!("SGD: The optimal number of CPU cores is {optimal_cores}");
                one_time_cores_update = !one_time_cores_update;
            }
        } else if RP_MODE == 4 { // Update cores - legacy implementation
            delay_for(Duration::from_secs(5)).await; // sends request every 5s
            let retrieved_msg = retrieve_agora_traffic(&mut socket).await?; // TODO: implement timeout!
            println!("Agora status: absolute latency - {}, current cores - {}, current users - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2], retrieved_msg[3]);
            if retrieved_msg[0] > 100000 {
                let mut add_cores = CORES_STEP;
                if retrieved_msg[1] as u8 + add_cores > max_cores_for_workers as u8 {
                    add_cores = max_cores_for_workers as u8 - retrieved_msg[1] as u8;
                }
                if add_cores > 0 {
                    println!("Too much load in Agora: add {:?} cores\n", add_cores);
                    send_cores_control_message(&mut socket, add_cores, 0, retrieved_msg[1] as u8).await?;
                }
                if add_cores <= 0 {
                    println!("Reached maximum cores limit of {:?}\n", max_cores_for_workers);
                }
            } else if retrieved_msg[0] < 50000 {
                let mut remove_cores = CORES_STEP;
                if retrieved_msg[1] as u8 - remove_cores < min_cores_for_workers as u8 {
                    remove_cores = retrieved_msg[1] as u8 - min_cores_for_workers as u8;
                }
                if remove_cores > 0 {
                    println!("Too relaxed load in Agora: remove {:?} cores\n", remove_cores);
                    send_cores_control_message(&mut socket, 0, remove_cores, retrieved_msg[1] as u8).await?;
                }
                if remove_cores <= 0 {
                    println!("Reached minimum cores limit of {:?}\n", min_cores_for_workers);
                }
            } else {
            }
        }
    }
}

async fn create_udp_socket(host: &str) -> io::Result<UdpSocket> {
    println!("Creating socket on {} (RX_ADDR)", host);
    let socket = UdpSocket::bind(&host).await?;

    return Ok(socket);
}

/* UDP Functions */

// Send message through UDP socket
async fn send_message(socket: &mut UdpSocket, message: &[u8; 24]) -> io::Result<()> {
    let len = socket.send_to(message, TX_ADDR).await?;
    let message_temp = [message[0], message[8], message[16]];

    if message[0] == 0 {
        println!("{:?} bytes sent to {:?} (TX_ADDR) requesting Agora cores and users details", len, TX_ADDR);
    } else if message[0] == 1 {
        if message[8] == 0 && message[16] == 0 {
            println!("{:?} bytes sent to {:?} (TX_ADDR) requesting Agora cores and latency details", len, TX_ADDR);
        } else {
            println!("{:?} bytes sent to {:?} (TX_ADDR) to update cores - content [msg type, add core, remove core]: {:?}", len, TX_ADDR, &message_temp);
        }
    } else {
        println!("Wrong message type to Agora");
    }

    return Ok(());
}

// Listen message on UDP socket
async fn listen_message(socket: &mut UdpSocket) -> io::Result<[u64; 5]> {
    let mut buf = [0u8; 40];
    let (len, addr) = socket.recv_from(&mut buf).await?;
    if len != 40 {
        // Handle invalid message length
        return Err(io::Error::new(io::ErrorKind::InvalidData, "Invalid message length"));
    }
    let a: u64 = u64::from_ne_bytes(buf[..8].try_into().unwrap());
    let b: u64 = u64::from_ne_bytes(buf[8..16].try_into().unwrap());
    let c: u64 = u64::from_ne_bytes(buf[16..24].try_into().unwrap());
    let d: u64 = u64::from_ne_bytes(buf[24..32].try_into().unwrap());
    let e: u64 = u64::from_ne_bytes(buf[32..].try_into().unwrap());
    let message = [a, b, c, d, e];
    println!("{:?} bytes received from {:?} (RX_ADDR) - content [msg_0, msg_1, msg_2, msg_3, msg_4]: {:?}", len, addr, &message);

    return Ok(message);
}

/* Resource Provisioner Functions */

// Retrieve Agora status
async fn retrieve_agora_config(socket: &mut UdpSocket) -> io::Result<[u64; 5]> {
    let message = [0; 24]; // message type 0 - retrieve Agora config details
    send_message(socket, &message).await?;
    return listen_message(socket).await;
}

// Retrieve Agora status
async fn retrieve_agora_traffic(socket: &mut UdpSocket) -> io::Result<[u64; 5]> {
    let mut message = [0; 24];
    message[0] = 1; // message type 1 - retrieve Agora core, user and latency details
    send_message(socket, &message).await?;
    // Fifth byte is only a place holder for now, can be used in future
    return listen_message(socket).await;
}

// Send cores update to Agora
async fn send_cores_control_message(socket: &mut UdpSocket, add_cores: u8, remove_cores: u8, _curr_cores: u8) -> io::Result<()> {
    let mut message = [0; 24];
    message[0] = 1; // message type - update cores
    message[8] = add_cores;
    message[16] = remove_cores;

    if add_cores > remove_cores {
        // ask agora to add cores
        send_message(socket, &message).await?;
        // // println!("{:?} core(s) added in Agora", add_cores);
    } else {
        // ask agora to remove cores
        send_message(socket, &message).await?;
        // // println!("{:?} core(s) removed in Agora", remove_cores);
        // loop { // wait until worker has been removed
        //     let retrieved_msg = retrieve_agora_traffic(socket).await?;
        //     if retrieved_msg[1] <= (curr_cores + add_cores - remove_cores) as u64 {
        //         println!("{:?} core(s) removed in Agora", remove_cores);
        //         break;
        //     }
        // }
    }
    delay_for(Duration::from_millis(UDP_WAIT_TIME)).await; // give time for Agora to process request TODO: more intelligent way?
    return Ok(());
}

/* Reinforcement Learning Functions */

// Agora Init
