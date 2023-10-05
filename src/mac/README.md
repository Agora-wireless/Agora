# Mac Video Demo
  Agora currently only builds and runs on Linux, and has been tested on Ubuntu 16.04, 18.04, 20.04 (Recommended), and 22.04 LTS. 
  Agora requires CMake 2.8+ and works with both GNU and Intel compilers with C++17 support. 
## Over the Air  32x2 uplink video streaming
  * ![Image of Video Demo Interactions](/assets/10745791/272974286-1a32ed49-08b6-468b-b671-97a6f57bdc13.png)
  * For Faros RRU and Iris UEs, pass `-DRADIO_TYPE=SOAPY_IRIS -DENABLE_MAC=True` to cmake
  * Configure the gains, and verify the topology file (outdoor may need up to 85/100 rx/tx gains)
  * Terminal 1:
    <pre>
      $ ./build/data_generator --conf_file files/config/examples/mac-video-ul.json
    </pre>
      to generate data files.
    <pre>
      $ ./build/user --conf_file files/config/examples/mac-video-ul.json
    </pre>
      to start users.
    * Terminal 2:
    <pre>
      $ ./build/macbs --enable_slow_start 0 --conf_file files/experiment/ul-vulture.json --core_offset 16 --fwd_udp_port 1360 --fwd_udp_address "127.0.0.1" -num_receiver_threads 2
    </pre>
      to run the macbs (receives mac packets from Agora phy)
    * Terminal 3:
    <pre>
      $ ./build/agora --conf_file files/config/examples/mac-video-ul.json
    </pre>
    run agora before running macuser.  user -> macbs -> agora -> macuser (after the UE detects the BS beacon). 
    * Terminal 4:
    <pre>
      $ ./build/macuser --enable_slow_start 0 --conf_file files/experiment/ul-vulture.json --core_offset 19 --num_sender_update_threads 2
    </pre>
    to run to base user app (Sends Uplink Mac packets to the user mac thread)
  **Note**: Insure agora / user / macuser / macbs are using different set of cores, otherwise there will be performance slow down.

  * Stream the video in udp mode to the mac (Using VLC media player)
    Stream (Tx)
    <pre>
      vlc media/Tour.mp4 --sout="#duplicate{dst=display, dst=std{access=udp, mux=ts, dst=127.0.0.1:1350}" --ttl 2 --no-sout-all --sout-keep --network-caching=1000 --mtu=188
    </pre>
    Receive (Rx)
    <pre>
      vlc udp://@127.0.0.1:1360
    </pre>
    Increment the port number by 1 for each UE (1351 / 1361, etc)
  * To date, vlc will pack UDP traffic in packets that have a data size in multiples of 188.  