use tokio::net::UdpSocket;
use tokio::time::{delay_for, Duration};
use async_trait::async_trait;
use std::io;
use std::cmp::Ordering;

extern crate rand;
use rand::Rng;

// Define a trait for the environment
#[async_trait]
trait AgoraEnv {
    fn new() -> Self;
    fn set_initial_values(&mut self, num_max_cores: u16, num_min_cores: u16, num_users: u16, num_latency_levels: u16, num_actions: u16, max_latency_limit: f32, ma_window_size: u16, outlier_percentage: u16, rewards: Vec<i16>, state: u64, done: bool);
    fn reset(&mut self) -> u64;
    fn set(&mut self, state: u64);
    fn compute_state(&mut self, curr_cores: u16, curr_users: u16, curr_latency: u16) -> u64;
    fn get_num_states(&mut self) -> u64;
    fn get_num_actions(&mut self) -> u16;
    fn agora_to_rl_index_mapping(&mut self, index_param: u16) -> u16;
    fn rl_to_agora_index_mapping(&mut self, index_param: u16) -> u16;
    fn compute_absolute_latency(&mut self, curr_cores: u16, curr_users: u16) -> f32;
    fn compute_curr_latency_reward_done_real(&mut self, absolute_latency: f32) -> (u16, i16, bool);
    fn compute_curr_latency_reward_done_emulated(&mut self, absolute_latency: f32) -> (u16, i16, bool);
    fn moving_average(&mut self, curr_cores: u16,) -> f32;
    async fn step_real(&mut self, socket: &mut UdpSocket, action: u16) -> io::Result<(u64, i16, bool)>;
    fn step_emulated(&mut self, action: u16) -> (u64, i16, bool);
    async fn print_state(&mut self, state: u64, delay: u64);
}

// Implement the trait for Agora environment
struct MyAgoraEnv {
    num_max_cores: u16,
    num_min_cores: u16,
    num_users: u16,
    num_latency_levels: u16,
    num_states: u64,
    num_actions: u16,
    max_latency_limit: f32,
    rewards: Vec<i16>,
    ma_active_window_sizes: Vec<u16>,
    ma_window_size: u16,
    ma_window: Vec<Vec<f32>>,
    outlier_percentage: u16,
    state: u64,
    done: bool,
}

#[async_trait]
impl AgoraEnv for MyAgoraEnv {
    fn new() -> Self {
        // Initialize the environment with custom values
        MyAgoraEnv {
            num_max_cores: 10,
            num_min_cores: 1,
            num_users: 16,
            num_latency_levels: 10 + 1,
            num_states: 10 * 16 * (10 + 1),
            num_actions: 3,
            max_latency_limit: 1.0,
            rewards: vec![-12, -10, -8, -6, -4, -2, -1, 20, -12, -16, -20],
            ma_active_window_sizes: vec![0; 25],
            ma_window_size: 10,
            ma_window: vec![vec![0.0; 10]; 25],
            outlier_percentage: 10,
            state: 0,
            done: false }
    }

    fn set_initial_values(&mut self, num_max_cores: u16, num_min_cores: u16, num_users: u16, num_latency_levels: u16, num_actions: u16, max_latency_limit: f32, ma_window_size: u16, outlier_percentage: u16, rewards: Vec<i16>, state: u64, done: bool) {
        // Set initial values
        self.num_max_cores = num_max_cores;
        self.num_min_cores = num_min_cores;
        self.num_users = num_users;
        self.num_latency_levels = num_latency_levels;
        self.num_states = num_max_cores as u64 * num_users as u64 * num_latency_levels as u64;
        self.num_actions = num_actions;
        self.max_latency_limit = max_latency_limit;
        self.ma_active_window_sizes = vec![0; num_max_cores as usize];
        self.ma_window_size = ma_window_size;
        self.ma_window = vec![vec![0.0; ma_window_size as usize]; num_max_cores as usize];
        self.outlier_percentage = outlier_percentage;
        self.rewards = rewards;
        self.state = state;
        self.done = done;
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
        let curr_state = (curr_cores as u64 + curr_users as u64 * self.num_max_cores as u64 + curr_latency as u64 * self.num_max_cores as u64 * self.num_users as u64) % self.num_states;

        curr_state
    }

    fn get_num_states(&mut self) -> u64 {

        self.num_states
    }

    fn get_num_actions(&mut self) -> u16 {

        self.num_actions
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

    fn moving_average(&mut self) -> f32 {
        let moving_window_elements = &self.moving_window;
        println!("running_window_size {}, window_size {}\n", self.running_window_size, self.window_size);
        println!("moving_window_elements {:?}\n", moving_window_elements);
        let mut average_value = 0.0;
        for count in (self.ma_window_size - self.ma_active_window_sizes[curr_cores as usize])..= (self.ma_window_size - 1) {
            average_value = average_value + self.ma_window[curr_cores as usize][count as usize];
        }
        average_value = average_value/self.ma_active_window_sizes[curr_cores as usize] as f32;

        average_value
    }

    async fn step_real(&mut self, socket: &mut UdpSocket, action: u16) -> io::Result<(u64, i16, bool)> {
        // Take a step in the environment based on the given action
        let curr_cores = (self.state % self.num_max_cores as u64) as u16;
        let curr_users = ((self.state / self.num_max_cores as u64) % self.num_users as u64) as u16;
        let next_cores;

        let curr_cores_mapped = curr_cores + 1;
        if action == 1 { // Add One Core
            if curr_cores_mapped < self.num_max_cores {
                next_cores = (curr_cores + 1) % self.num_max_cores;
                send_cores_control_message(socket, 1, 0, curr_cores_mapped as u8).await?;
            } else {
                println!("Maximum cores limit reached...");
                next_cores = curr_cores;
            }
        } else if action == 2 { // Remove One Core
            if curr_cores_mapped <= self.num_min_cores {
                println!("Minimum cores limit reached...");
                next_cores = curr_cores;
            } else {
                next_cores = (curr_cores - 1) % self.num_max_cores;
                send_cores_control_message(socket, 0, 1, curr_cores_mapped as u8).await?;
            }
        } else {
            // No Change (curr_cores remain as it is)
            next_cores = curr_cores;
        }

        // delay_for(Duration::from_millis(PERIODICITY)).await;
        let mut retrieved_msg = retrieve_agora_traffic(socket).await?;
        let mut absolute_latency = retrieved_msg[0];
        let mut agora_cores = retrieved_msg[1];
        let mut frame_id = retrieved_msg[2];
        println!("Agora status: absolute latency - {}, current cores - {}, frame id - {}\n", absolute_latency, agora_cores, frame_id);    
        let mut is_valid_latency = false;
        while is_valid_latency == false {
            if absolute_latency <= 2 * self.max_latency_limit as u64 {
                is_valid_latency = true;
            } else {
                println!("Out of bound latency, requesting Agora again...\n");
                // delay_for(Duration::from_millis(PERIODICITY)).await;
                retrieved_msg = retrieve_agora_traffic(socket).await?;
                absolute_latency = retrieved_msg[0];
                agora_cores = retrieved_msg[1];
                frame_id = retrieved_msg[2];
                println!("Agora status: absolute latency - {}, current cores - {}, frame id - {}\n", absolute_latency, agora_cores, frame_id);            
            }
        }

        for count in 1..= (self.window_size - 1) {
            self.moving_window[count as usize - 1] = self.moving_window[count as usize];
        }
        self.moving_window[self.window_size as usize - 1] = absolute_latency as f32;
        if self.running_window_size < self.window_size {
            self.running_window_size = self.running_window_size + 1;
        }
        let mv_absolute_latency = MyAgoraEnv::moving_average(self);
        println!("mv_absolute_latency - {}\n", mv_absolute_latency);

        let curr_latency_reward_done_real = MyAgoraEnv::compute_curr_latency_reward_done_real(self, mv_absolute_latency as f32);
        let curr_latency = curr_latency_reward_done_real.0;
        let reward = curr_latency_reward_done_real.1;
        let done = curr_latency_reward_done_real.2;
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
        Ok((next_state, reward, done))
    }

    fn step_emulated(&mut self, action: u16) -> (u64, i16, bool) {
        // Take a step in the environment based on the given action
        let curr_cores = (self.state % self.num_max_cores as u64) as u16;
        let curr_users = ((self.state / self.num_max_cores as u64) % self.num_users as u64) as u16;
        let next_cores;

        if action == 1 { // Add One Core
            if (curr_cores + 1) % self.num_max_cores != 0 {
                next_cores = (curr_cores + 1) % self.num_max_cores;
            } else {
                // print("Maximum cores limit reached...")
                next_cores = curr_cores;
            }
        } else if action == 2 { // Remove One Core
            if (curr_cores % self.num_max_cores) == 0 {
                // print("Minimum cores limit reached...")
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
        let curr_users = (state / self.num_max_cores as u64) % self.num_users as u64;
        let curr_latency = (state / (self.num_max_cores as u64 * self.num_users as u64)) % self.num_latency_levels as u64;
        println!("\nState {} is decoded as: ", state);
        println!("curr_cores: {}", curr_cores);
        println!("curr_users: {}", curr_users);
        println!("curr_latency: {}", curr_latency);
        println!("delay: {}", delay);
        // delay_for(Duration::from_millis(delay)).await;
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

static RP_MODE: u8 = 2;
static UDP_WAIT_TIME: u64 = 100;
static PERIODICITY: u64 = 500; // sends request every PERIODICITY ms
static LATENCY_UPPER_TH: u64 = 1000; // 1 ms is the maximum latency limit per frame for Agora
static LATENCY_LOWER_TH: u64 = 750; // 0.75 ms is the minimum latency limit per frame for Agora

#[tokio::main]
async fn main() -> io::Result<()> {
    let mut socket = create_udp_socket(RX_ADDR).await?;
    let mut one_time_cores_update = true;
    delay_for(Duration::from_secs(1)).await; // wait for 5s to start
    let retrieved_msg = retrieve_agora_cores(&mut socket).await?;
    let cores_for_rest = retrieved_msg[0]; // retrieved_msg[0] has the cores allocated for rest of the processing in Agora
    let max_cores_for_workers = retrieved_msg[1] - retrieved_msg[0]; // retrieved_msg[1] has the max cores available for workers
    let min_cores_for_workers = retrieved_msg[2];  // retrieved_msg[2] has the min cores available for workers
    println!("Agora cores details: cores for rest - {}, max cores for workers - {}, min core(s) for workers - {}\n", cores_for_rest, max_cores_for_workers, min_cores_for_workers);
    loop {
        if RP_MODE == 0 {  // Retrieve Agora status periodically
            delay_for(Duration::from_millis(PERIODICITY)).await;
            let retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
            println!("Agora status: absolute latency - {}, current cores - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2]);
        } else if RP_MODE == 1 { // Update cores dynamically
            delay_for(Duration::from_secs(1)).await; // sends request every 5s
            let retrieved_msg = retrieve_agora_traffic(&mut socket).await?; // TODO: implement timeout!
            println!("Agora status: absolute latency - {}, current cores - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2]);
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
            let is_real_agora = true;
            let num_max_cores;
            let num_min_cores;
            let num_users;
            let num_latency_levels;
            let num_actions;
            let max_latency_limit;
            let running_window_size = 0;
            let window_size = 10;
            let moving_window = vec![0.0; window_size];
            let rewards;
            if is_real_agora == true {
                num_max_cores = 20;
                num_min_cores = 8;
                num_users = 8;
                num_latency_levels = 10 + 1; // + 1 --> To account for max_latency_limit and beyond latency values
                num_actions = 3;
                max_latency_limit = 1000.0;
                rewards = vec![-12, -10, -8, -6, -4, -2, -1, 20, -12, -16, -20];
            } else {
                num_max_cores = 10;
                num_min_cores = 1;
                num_users = 16;
                num_latency_levels = 10 + 1; // + 1 --> To account for max_latency_limit and beyond latency values
                num_actions = 3;
                max_latency_limit = 0.001;
                rewards = vec![-12, -10, -8, -6, -4, -2, -1, 20, -12, -16, -20];
            }
        
            // Q-learning parameters
            let alpha = 0.1;  // learning rate
            let gamma = 0.9;  // discount factor
            // let epsilon = 0.1;  // exploration-exploitation trade-off
        
            let num_episodes = 10;
            let terminate_count = 100; // Epochs count to terminate learning when not converging
            let consecutive_epochs = 10; // Number of consecutive epochs with same state and reward to treat the learning converged. Applicable when the target reward is the least negative.

            // Create an instance of the agora environment using the 'new' function
            let mut agora_env = MyAgoraEnv::new();
            agora_env.set_initial_values(num_max_cores, num_min_cores, num_users, num_latency_levels, num_actions, max_latency_limit, running_window_size as u16, window_size as u16, moving_window, rewards, 0, false); // Set initial state to 2 and done to false
        
            let num_states = agora_env.get_num_states();
            // println!("num_states {:?}\n", num_states);
            let mut q_table: Vec<Vec<f32>> = vec![vec![0.0; num_actions as usize]; num_states as usize];
            // println!("{:?}", q_table);

            if one_time_cores_update == true {
                // Training
                println!("Training started ...");
                for episode in 0..= (num_episodes - 1) {
                    println!("Episode: {:?}\n", episode);
                    // Reset the state with a random value
                    let mut state;
                    // let mut state = agora_env.reset();
                    // state = 269;
                    let mut rng = rand::thread_rng();
                    let reset_cores = rng.gen_range((num_min_cores as u16 - 1)..(num_max_cores as u16 - 1));
                    let reset_users = 7;
                    let mut epochs = 0;
                    let mut previous_reward = 0;
                    let mut target_reward_count = 1;
                    let mut next_state;
                    let mut reward = -20;
                    let mut action;
                    let mut done = false;
    
                    println!("Re-initializing cores in Agora as per reset state - START\n");
                    // delay_for(Duration::from_millis(UDP_WAIT_TIME)).await;
                    let mut retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                    println!("Agora status: absolute latency - {}, current cores - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2]);
                    let agora_start_cores = retrieved_msg[1];

                    let reset_cores_mapped = reset_cores as u8 + 1;
                    let mut curr_cores;
                    if reset_cores_mapped as u8 > agora_start_cores as u8 {
                        let mut add_cores = CORES_STEP;
                        curr_cores = agora_start_cores as u8;
                        for _step_cores in (0..(reset_cores_mapped as u8 - agora_start_cores as u8)).step_by(CORES_STEP as usize) {
                            if curr_cores as u8 + add_cores > max_cores_for_workers as u8 {
                                add_cores = max_cores_for_workers as u8 - curr_cores as u8;
                            }
                            if add_cores > 0 {
                                println!("Start cores in Agora {:?}; Re-initializing cores in Agora to {:?}; Target reset cores in Agora {:?}\n", agora_start_cores, curr_cores as u8 + add_cores, reset_cores_mapped);
                                send_cores_control_message(&mut socket, add_cores, 0, curr_cores as u8).await?;
                                retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                                println!("Agora status: absolute latency - {}, current cores - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2]);
                            }
                            if add_cores <= 0 {
                                println!("Reached maximum cores limit of {:?}\n", max_cores_for_workers);
                            }
                            curr_cores = curr_cores as u8 + add_cores;
                        }
                    } else {
                        let mut remove_cores = CORES_STEP;
                        curr_cores = agora_start_cores as u8;
                        for _step_cores in (0..(agora_start_cores as u8 - reset_cores_mapped as u8)).step_by(CORES_STEP as usize) {
                            if curr_cores as u8 - remove_cores < min_cores_for_workers as u8 {
                                remove_cores = curr_cores as u8 - min_cores_for_workers as u8;
                            }
                            if remove_cores > 0 {
                                println!("Start cores in Agora {:?}; Re-initializing cores in Agora to {:?}; Target cores in Agora {:?}\n", agora_start_cores, curr_cores as u8 - remove_cores, reset_cores_mapped);
                                send_cores_control_message(&mut socket, 0, remove_cores, curr_cores as u8).await?;
                                retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                                println!("Agora status: absolute latency - {}, current cores - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2]);
                            }
                            if remove_cores <= 0 {
                                println!("Reached maximum cores limit of {:?}\n", min_cores_for_workers);
                            }
                            curr_cores = curr_cores as u8 - remove_cores;
                        }
                    }
                    println!("Re-initializing cores in Agora as per reset state - END\n");

                    // delay_for(Duration::from_millis(PERIODICITY)).await;
                    let retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
                    let reset_absolute_latency = retrieved_msg[0];
                    let reset_cores = retrieved_msg[1];
                    let curr_latency_reward_done_real = agora_env.compute_curr_latency_reward_done_real(reset_absolute_latency as f32);
                    let reset_latency = curr_latency_reward_done_real.0;
                    println!("Agora status: reset absolute latency - {}, reset cores - {}, frame id - {}\n",
                        reset_absolute_latency, reset_cores, retrieved_msg[2]);
                    let reset_cores_unmapped = reset_cores - 1;
                    state = agora_env.compute_state(reset_cores_unmapped as u16, reset_users as u16, reset_latency as u16);
            
                    while done == false {
                        println!("Episode {:?}, Epoch {:?}\n", episode, epochs);
                        agora_env.set(state);
                        // agora_env.print_state(state, UDP_WAIT_TIME).await;
    
                        if 0 != 0 {
                            action = 0;
                        } else {
                            // println!("elements of state: {}", state);
                            // for &row_element in &q_table[state as usize] {
                            //     println!("{}", row_element);
                            // }                    
                            action = argmax_row(&q_table, state as usize);
                            println!("action for state {}: {}", state, action);
                        }

                        if is_real_agora == true {
                            (next_state, reward, done) = agora_env.step_real(&mut socket, action as u16).await?;
                            println!("step_output: next_state {}, reward {}, done {}\n", next_state, reward, done);
                        } else {
                            let step_output = agora_env.step_emulated(action as u16);
                            next_state = step_output.0;
                            reward = step_output.1;
                            done = step_output.2;
                            // println!("step_output: next_state {}, reward {}, done {}\n", next_state, reward, done);
                        }

                        // Check consecutive epochs, rewards and statea condition to exit learning
                        if reward == previous_reward && state == next_state {
                            target_reward_count += 1;
                            if target_reward_count == consecutive_epochs {
                                done = true;
                                println!("previous_reward: {}", previous_reward);
                                println!("reward: {}", reward);
                                println!("state: {}", state);
                                println!("next_state: {}", next_state);
                                println!("target_reward_count: {}", target_reward_count);
                            }
                        } else {
                            target_reward_count = 1;
                        }
    
                        // Q-value update using the Q-learning update rule
                        let old_value = q_table[state as usize][action];
                        let next_max = q_table[next_state as usize].iter().cloned().fold(f32::NEG_INFINITY, f32::max);
                        let new_value = old_value + alpha * (reward as f32 + gamma * next_max - old_value);
                        q_table[state as usize][action] = new_value;
                        // println!("elements of state: {}", state);
                        // for &row_element in &q_table[state as usize] {
                        //     println!("{}", row_element);
                        // }
    
                        // Update state
                        state = next_state;

                        // Update reward
                        previous_reward = reward;
    
                        // Count epochs
                        epochs += 1;

                        if epochs == terminate_count {
                            println!("Terminating training for Episode {} based on terminal count of {} Epochs ...\n", episode, terminate_count);
                            done = true;
                        }
                    }
                    println!("episode {:?} state {:?} reward {:?} epochs {:?}\n", episode, state, reward, epochs);
                    // agora_env.print_state(next_state, UDP_WAIT_TIME).await;
                }
                println!("Training completed ...");

                // Testing
                println!("Testing started ...");
                let mut state = agora_env.reset();
                // agora_env.print_state(state, UDP_WAIT_TIME).await;
                let mut epochs = 0;
                let mut previous_reward = 0;
                let mut target_reward_count = 1;
                let mut next_state;
                let mut reward = -20;
                let mut action;
                let mut done = false;

                while done == true {
                    agora_env.set(state);

                    if 0 != 0 {
                        action = 0;
                    } else {
                        // println!("elements of state: {}", state);
                        // for &row_element in &q_table[state as usize] {
                        //     println!("{}", row_element);
                        // }                    
                        action = argmax_row(&q_table, state as usize);
                        println!("action for state {}: {}", state, action);
                    }

                    if is_real_agora == true {
                        (next_state, reward, done) = agora_env.step_real(&mut socket, action as u16).await?;
                        println!("step_output: next_state {}, reward {}, done {}\n", next_state, reward, done);    
                    } else {
                        let step_output = agora_env.step_emulated(action as u16);

                        next_state = step_output.0;
                        reward = step_output.1;
                        done = step_output.2;
                        // println!("step_output: next_state {}, reward {}, done {}\n", next_state, reward, done);    
                    }

                    // Check consecutive epochs, rewards and statea condition to exit learning
                    if reward == previous_reward && state == next_state {
                        target_reward_count += 1;
                        if target_reward_count == consecutive_epochs {
                            done = true;
                            // println!("previous_reward: {}", previous_reward);
                            // println!("reward: {}", reward);
                            // println!("state: {}", state);
                            // println!("next_state: {}", next_state);
                            // println!("target_reward_count: {}", target_reward_count);
                        }
                    } else {
                        target_reward_count = 1;
                    }

                    // Update state
                    state = next_state;

                    // Update reward
                    previous_reward = reward;
    
                    // Count epochs
                    epochs += 1;

                    if epochs == terminate_count {
                        done = true;
                    }
                }
                println!("state {:?} reward {:?} epochs {:?}\n", state, reward, epochs);
                // agora_env.print_state(state, UDP_WAIT_TIME).await;
                one_time_cores_update = !one_time_cores_update;
            }
        } else if RP_MODE == 3 { // Update cores - legacy implementation
            delay_for(Duration::from_secs(5)).await; // sends request every 5s
            let retrieved_msg = retrieve_agora_traffic(&mut socket).await?; // TODO: implement timeout!
            println!("Agora status: absolute latency - {}, current cores - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2]);
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
        println!("{:?} bytes sent to {:?} (TX_ADDR) requesting Agora cores details", len, TX_ADDR);
    } else if message[0] == 1 {
        if message[8] == 0 && message[16] == 0 {
            println!("{:?} bytes sent to {:?} (TX_ADDR) requesting Agora cores and latency details", len, TX_ADDR);
        } else {
            println!("{:?} bytes sent to {:?} (TX_ADDR) to update cores - content [msg type, add core, remove core]: {:?}", len, TX_ADDR, &message_temp);
        }
    } else if message[0] == 2 {
        if message[8] == 0 && message[16] == 0 {
            println!("{:?} bytes sent to {:?} (TX_ADDR) requesting Agora users details", len, TX_ADDR);
        } else {
            println!("{:?} bytes sent to {:?} (TX_ADDR) to update users - content [msg type, add user, remove user]: {:?}", len, TX_ADDR, &message_temp);
        }
    }

    return Ok(());
}

// Listen message on UDP socket
async fn listen_message(socket: &mut UdpSocket) -> io::Result<[u64; 3]> {
    let mut buf = [0u8; 24];
    let (len, addr) = socket.recv_from(&mut buf).await?;
    if len != 24 {
        // Handle invalid message length
        return Err(io::Error::new(io::ErrorKind::InvalidData, "Invalid message length"));
    }
    let a: u64 = u64::from_ne_bytes(buf[..8].try_into().unwrap());
    let b: u64 = u64::from_ne_bytes(buf[8..16].try_into().unwrap());
    let c: u64 = u64::from_ne_bytes(buf[16..].try_into().unwrap());
    let message = [a, b, c];
    println!("{:?} bytes received from {:?} (RX_ADDR) - content [msg_0, msg_1, msg_2]: {:?}", len, addr, &message);

    return Ok(message);
}

/* Resource Provisioner Functions */

// Retrieve Agora status
async fn retrieve_agora_cores(socket: &mut UdpSocket) -> io::Result<[u64; 3]> {
    let message = [0; 24]; // message type - retrieve Agora core details
    send_message(socket, &message).await?;
    return listen_message(socket).await;
}

// Retrieve Agora status
async fn retrieve_agora_traffic(socket: &mut UdpSocket) -> io::Result<[u64; 3]> {
    let mut message = [0; 24];
    message[0] = 1; // message type - retrieve Agora core and latency details
    send_message(socket, &message).await?;
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
        println!("{:?} core(s) added in Agora", add_cores);
    } else {
        // ask agora to remove cores
        send_message(socket, &message).await?;
        println!("{:?} core(s) removed in Agora", remove_cores);
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
