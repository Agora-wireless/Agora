use tokio::net::UdpSocket;
use tokio::time::{delay_for, Duration};
use std::io;

// Communication parameters
static TX_ADDR: &str = "127.0.0.1:4000";
static RX_ADDR: &str = "127.0.0.1:3000";

// Resource parameters
static CORES_STEP: u8 = 5;

static RP_MODE: u8 = 1;
static PERIODICITY: u64 = 500; // sends request every PERIODICITY ms
static LATENCY_UPPER_TH: u64 = 1000; // 1 ms is the maximum latency limit per frame for Agora
static LATENCY_LOWER_TH: u64 = 750; // 0.75 ms is the minimum latency limit per frame for Agora

#[tokio::main]
async fn main() -> io::Result<()> {
    let mut socket = create_udp_socket(RX_ADDR).await?;
    let mut one_time_cores_update = true;
    delay_for(Duration::from_secs(5)).await; // wait for 5s to start
    let retrieved_msg = retrieve_agora_cores(&mut socket).await?;
    let cores_for_rest = retrieved_msg[0]; // retrieved_msg[0] has the cores allocated for rest of the threads in Agora
    let max_cores_for_workers = retrieved_msg[1] - retrieved_msg[0]; // retrieved_msg[1] has the max cores available for workers
    let min_cores_for_workers = retrieved_msg[2];  // retrieved_msg[2] has the min cores available for workers
    println!("Agora cores details: cores for rest - {}, max cores for workers - {}, min cores for workers - {}\n", cores_for_rest, max_cores_for_workers, min_cores_for_workers);
    loop {
        if RP_MODE == 0 {  // Retrieve Agora status periodically
            delay_for(Duration::from_millis(PERIODICITY)).await;
            let retrieved_msg = retrieve_agora_traffic(&mut socket).await?;
            println!("Agora status: latency - {}, current cores - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2]);
        } else if RP_MODE == 1 { // Update cores dynamically
            delay_for(Duration::from_secs(1)).await; // sends request every 5s
            let retrieved_msg = retrieve_agora_traffic(&mut socket).await?; // TODO: implement timeout!
            println!("Agora status: latency - {}, current cores - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2]);
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
        } else if RP_MODE == 2 { // Update cores - legacy implementation
            delay_for(Duration::from_secs(5)).await; // sends request every 5s
            let retrieved_msg = retrieve_agora_traffic(&mut socket).await?; // TODO: implement timeout!
            println!("Agora status: latency - {}, current cores - {}, frame id - {}\n", retrieved_msg[0], retrieved_msg[1], retrieved_msg[2]);
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
async fn send_cores_control_message(socket: &mut UdpSocket, add_cores: u8, remove_cores: u8, curr_cores: u8) -> io::Result<()> {
    let mut message = [0; 24];
    message[0] = 1; // message type - update cores
    message[8] = add_cores;
    message[16] = remove_cores;

    if add_cores > remove_cores {
        // ask agora to add cores
        send_message(socket, &message).await?;
        println!("{:?} cores added in Agora", add_cores);
    } else {
        // ask agora to remove cores
        send_message(socket, &message).await?;
        loop { // wait until worker has been removed
            let retrieved_msg = retrieve_agora_traffic(socket).await?;
            if retrieved_msg[1] <= (curr_cores + add_cores - remove_cores) as u64 {
                println!("{:?} cores removed in Agora", remove_cores);
                break;
            }
        }
    }

    delay_for(Duration::from_secs(1)).await; // give time for Agora to process request TODO: more intelligent way?
    return Ok(());
}
