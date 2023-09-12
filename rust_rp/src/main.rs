use tokio::net::UdpSocket;
use tokio::time::{delay_for, Duration};
use std::io;
// use std::cmp;
// use std::process::Command;

// Communication parameters
static TX_ADDR: &str  = "127.0.0.1:4000";
static RX_ADDR: &str = "127.0.0.1:3000";

// Resource parameters
static MAX_CORE: u8 = 64;
static MIN_CORE: u8 = 10;
static STEPS: u8 = 5;

#[tokio::main]
async fn main() -> io::Result<()> {
    let mut socket = create_udp_socket(RX_ADDR).await?;
    loop {
        delay_for(Duration::from_secs(5)).await; // sends request every 5s
        let traffic = retrieve_agora_traffic(&mut socket).await?; // TODO: implement timeout!
        println!("Agora status: latency - {}, current cores - {}\n", traffic[0], traffic[1]);

        if traffic[0] > 100000 {
            let mut add_cores = STEPS;
            if traffic[1] as u8 + add_cores > MAX_CORE {
                add_cores = MAX_CORE - traffic[1] as u8;
            }

            if add_cores > 0 {
                println!("Too much load in Agora: add {:?} cores\n", add_cores);
                send_control_message(&mut socket, add_cores, 0, traffic[1] as u8).await?;
            }

            if add_cores <= 0 {
                println!("Reached maximum cores limit of {:?}\n", MAX_CORE);
            }
        } else if traffic[0] < 50000 {
            let mut remove_cores = STEPS;
            if traffic[1] as u8 - remove_cores < MIN_CORE {
                remove_cores = traffic[1] as u8 - MIN_CORE;
            }

            if remove_cores > 0 {
                println!("Too relaxed load in Agora: remove {:?} cores\n", remove_cores);
                send_control_message(&mut socket, 0, remove_cores, traffic[1] as u8).await?;
            }

            if remove_cores <= 0 {
                println!("Reached minimum cores limit of {:?}\n", MIN_CORE);
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
async fn send_message(socket: &mut UdpSocket, message: &[u8; 16]) -> io::Result<()> {
    let len = socket.send_to(message, TX_ADDR).await?;
    let message_temp = [message[0], message[8]];
    if message[0] == 0 && message[8] == 0 {
        println!("{:?} bytes sent to {:?} (TX_ADDR) requesting Agora status", len, TX_ADDR);
    } else {
        println!("{:?} bytes sent to {:?} (TX_ADDR) to update cores - content [add, remove]: {:?}", len, TX_ADDR, &message_temp);
    }
    return Ok(());
}

// Listen message on UDP socket
async fn listen_message(socket: &mut UdpSocket) -> io::Result<[u64; 2]> {
    let mut buf = [0; 100];
    let (len, addr) = socket.recv_from(&mut buf).await?;
    let a: u64 = u64::from_ne_bytes(buf[..8].try_into().unwrap());
    let b: u64 = u64::from_ne_bytes(buf[8..len].try_into().unwrap());
    let message = [a, b];
    println!("{:?} bytes received from {:?} (RX_ADDR) - content [latency, cores]: {:?}", len, addr, &message);
    return Ok(message);
}

/* Resource Provisioner Logic */

// Retrieve Agora status
async fn retrieve_agora_traffic(socket: &mut UdpSocket) -> io::Result<[u64; 2]> {
    let message = [0; 16];
    send_message(socket, &message).await?;
    return listen_message(socket).await;
}

// Send cores update to Agora
async fn send_control_message(socket: &mut UdpSocket, add_cores: u8, remove_cores: u8, curr_cores: u8) -> io::Result<()> {
    let mut message = [0; 16];
    message[0] = add_cores;
    message[8] = remove_cores;

    if add_cores > remove_cores {
        // ask agora to add cores
        send_message(socket, &message).await?;
        println!("{:?} cores added in Agora", add_cores);
    } else {
        // ask agora to remove cores
        send_message(socket, &message).await?;
        loop { // wait until worker has been removed
            let traffic = retrieve_agora_traffic(socket).await?;
            if traffic[1] <= (curr_cores + add_cores - remove_cores) as u64 {
                println!("{:?} cores removed in Agora", remove_cores);
                break;
            }
        }
    }

    delay_for(Duration::from_secs(1)).await; // give time for Agora to process request TODO: more intelligent way?
    return Ok(());
}
