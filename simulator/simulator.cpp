#include "simulator.hpp"

Simulator::Simulator(Config* cfg, size_t in_task_thread_num,
    size_t in_core_offset, size_t sender_delay)
    : TASK_THREAD_NUM(in_task_thread_num)
    , SOCKET_RX_THREAD_NUM(in_task_thread_num)
    , SOCKET_TX_THREAD_NUM(in_task_thread_num)
    , CORE_OFFSET(in_core_offset)
{
    std::string directory = TOSTRING(PROJECT_DIRECTORY);
    printf("PROJECT_DIRECTORY: %s\n", directory.c_str());
    printf("Main thread: on core %d\n", sched_getcpu());
    // setenv("MKL_THREADING_LAYER", "sequential", true /* overwrite */);
    // std::cout << "MKL_THREADING_LAYER =  " << getenv("MKL_THREADING_LAYER")
    //           << std::endl;
    printf("enter constructor\n");

    this->config_ = cfg;

    initialize_vars_from_cfg(cfg);
    pin_to_core_with_offset(ThreadType::kMaster, CORE_OFFSET, 0);

    initialize_queues();

    printf("initialize buffers\n");
    initialize_uplink_buffers();

#ifdef USE_DPDK
    DpdkTransport::dpdk_init(in_core_offset, 1 + SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM);
    rte_mempool* mbuf_pool = DpdkTransport::create_mempool();

    uint16_t portid = 0; // For now, hard-code to port zero
    if (DpdkTransport::nic_init(portid, mbuf_pool, in_task_thread_num, in_task_thread_num) != 0)
        rte_exit(EXIT_FAILURE, "Cannot init port %u\n", portid);

    printf("new Sender\n");
    sender_ = new Sender(
        config_, 1, SOCKET_TX_THREAD_NUM, CORE_OFFSET + 1, sender_delay, true, "ff:ff:ff:ff:ff:ff", true, mbuf_pool);

    printf("new Receiver\n");
    receiver_ = new Receiver(config_, SOCKET_RX_THREAD_NUM, CORE_OFFSET + 2 + SOCKET_RX_THREAD_NUM,
        &message_queue_, rx_ptoks_ptr);
#else
    printf("new Sender\n");
    sender_ = new Sender(
        config_, 1, SOCKET_TX_THREAD_NUM, CORE_OFFSET + 1, sender_delay, true, "ff:ff:ff:ff:ff:ff", true);

    printf("new Receiver\n");
    receiver_ = new Receiver(config_, SOCKET_RX_THREAD_NUM, CORE_OFFSET,
        &message_queue_, rx_ptoks_ptr);
#endif
}

Simulator::~Simulator() { free_uplink_buffers(); }

void Simulator::stop()
{
    std::cout << "stopping threads " << std::endl;
    config_->running = false;
    sender_->join_thread();
    receiver_->join_thread();
    usleep(1000);
    delete receiver_;
    receiver_ = NULL;
    delete sender_;
    sender_ = NULL;
}

void Simulator::start()
{
    config_->running = true;
    /* start receiver */
    // std::vector<pthread_t> rx_threads
    //     = receiver_->startRecv(socket_buffer_, socket_buffer_status_,
    //         socket_buffer_status_size_, socket_buffer_size_, frame_start);
    receiver_->startRecv(socket_buffer_, socket_buffer_status_,
        socket_buffer_status_size_, socket_buffer_size_, frame_start);

    /* tokens used for dequeue */
    moodycamel::ConsumerToken ctok(message_queue_);
    moodycamel::ConsumerToken ctok_complete(complete_task_queue_);

    buffer_frame_num
        = symbol_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    max_packet_num_per_frame = BS_ANT_NUM * dl_data_symbol_num_perframe;

    /* counters for printing summary */
    int frame_count_rx = 0;

    int ret = 0;
    EventData events_list[kDequeueBulkSize];

    /* start transmitter */
    sender_->startTXfromMain(frame_start_tx, frame_end_tx);
    // while (config_->running && !SignalHandler::gotExitSignal()) {
    //     /* get a bulk of events */
    //     ret = 0;
    //     ret = message_queue_.try_dequeue_bulk(
    //         ctok, events_list, kDequeueBulkSizeSingle);

    //     if (ret == 0)
    //         continue;

    //     /* handle each event */
    //     for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
    //         EventData& event = events_list[bulk_count];
    //         switch (event.event_type_) {
    //         case EventType::kPacketRX: {
    //             int socket_thread_id = rx_tag_t(event.tags_[0]).tid;
    //             int buf_offset = rx_tag_t(event.tags_[0]).offset;

    //             char* socket_buffer_ptr = socket_buffer_[socket_thread_id]
    //                 + (long long)buf_offset * packet_length;
    //             struct Packet* pkt = (struct Packet*)socket_buffer_ptr;
    //             int frame_id = pkt->frame_id % 10000;
    //             int symbol_id = pkt->symbol_id;
    //             int ant_id = pkt->ant_id;
    //             int frame_id_in_buffer = (frame_id % TASK_BUFFER_FRAME_NUM);
    //             socket_buffer_status_[socket_thread_id][buf_offset] = 0;

    //             // printf(
    //             //     "In main: received from frame %d %d, symbol %d, ant
    //             //     %d\n", frame_id, frame_id_in_buffer, symbol_id,
    //             //     ant_id);

    //             update_rx_counters(
    //                 frame_id, frame_id_in_buffer, symbol_id, ant_id);
    //         } break;

    //         default:
    //             printf("Wrong event type in message queue!");
    //             exit(0);
    //         } /* end of switch */
    //     } /* end of for */
    // } /* end of while */
    while (sender_->running_ || receiver_->completion_num_ < SOCKET_RX_THREAD_NUM) {
    // while (sender_->running_) {
    // while (receiver_->completion_num_ < SOCKET_RX_THREAD_NUM) {
        // printf("Test sender %u receiver %u\n", sender_->running_, receiver_->completion_num_.load());
        // printf("Test receiver %u\n", receiver_->completion_num_.load());
        sleep(1);
    }
    printf("Simulator: sender finishes\n");
    this->stop();
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/timeresult_simulator.txt";
    FILE* fp = fopen(filename.c_str(), "w");
    if (fp == NULL) {
        printf("open file faild\n");
        std::cerr << "Error: " << strerror(errno) << std::endl;
        exit(0);
    }

    printf("Printing results to file......\n");
    for (int ii = 0; ii < frame_count_rx; ii++) {
        fprintf(fp, "%.3f %.3f %.3f %.3f\n", frame_start[0][ii],
            frame_start[1][ii], frame_start_receive[ii], frame_end_receive[ii]);
    }
    exit(0);
}

inline void Simulator::update_frame_count(int* frame_count)
{
    *frame_count = *frame_count + 1;
    if (*frame_count == 1e9)
        *frame_count = 0;
}

void Simulator::update_rx_counters(
    size_t frame_id, size_t frame_id_in_buffer, size_t symbol_id, size_t ant_id)
{
    rx_counter_packets_[frame_id_in_buffer]++;
    if (rx_counter_packets_[frame_id_in_buffer] == 1) {
        frame_start_receive[frame_id] = get_time();
        if (kDebugPrintPerFrameStart) {
            printf("Main thread: data received from frame %zu, symbol %zu, ant "
                   "%zu, in %.2f since tx, in %.2f us since last frame\n",
                frame_id, symbol_id, ant_id,
                frame_start_receive[frame_id] - frame_start_tx[frame_id],
                frame_start_receive[frame_id]
                    - frame_start_receive[frame_id - 1]);
        }
    } else if (rx_counter_packets_[frame_id_in_buffer]
        == max_packet_num_per_frame) {
        frame_end_receive[frame_id] = get_time();
        print_per_frame_done(PrintType::kPacketRX, frame_id);
        rx_counter_packets_[frame_id_in_buffer] = 0;
    }
}

void Simulator::print_per_frame_done(PrintType print_type, size_t frame_id)
{
    if (!kDebugPrintPerFrameDone)
        return;
    switch (print_type) {
    case (PrintType::kPacketRX): {
        printf(
            "Main thread: received all packets in frame: %zu, in %.2f us since "
            "tx, in %.2f us since rx, tx duration: %.2f us\n",
            frame_id, frame_end_receive[frame_id] - frame_start_tx[frame_id],
            frame_end_receive[frame_id] - frame_start_receive[frame_id],
            frame_end_tx[frame_id] - frame_start_tx[frame_id]);
    } break;
    default:
        printf("Wrong task type in frame done print!");
    }
}

void Simulator::initialize_vars_from_cfg(Config* cfg)
{
    BS_ANT_NUM = cfg->BS_ANT_NUM;
    UE_NUM = cfg->UE_NUM;
    OFDM_CA_NUM = cfg->OFDM_CA_NUM;
    OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    symbol_num_perframe = cfg->symbol_num_perframe;
    data_symbol_num_perframe = cfg->data_symbol_num_perframe;
    ul_data_symbol_num_perframe = cfg->ul_data_symbol_num_perframe;
    dl_data_symbol_num_perframe = cfg->dl_data_symbol_num_perframe;
    dl_data_symbol_start = cfg->dl_data_symbol_start;
    dl_data_symbol_end = cfg->dl_data_symbol_end;
    packet_length = cfg->packet_length;

    demul_block_size = cfg->demul_block_size;
    demul_block_num = OFDM_DATA_NUM / demul_block_size
        + (OFDM_DATA_NUM % demul_block_size == 0 ? 0 : 1);
}

void Simulator::initialize_queues()
{
    message_queue_ = moodycamel::ConcurrentQueue<EventData>(
        512 * data_symbol_num_perframe);
    complete_task_queue_ = moodycamel::ConcurrentQueue<EventData>(
        512 * data_symbol_num_perframe * 4);

    rx_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(
        64, SOCKET_RX_THREAD_NUM * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < SOCKET_RX_THREAD_NUM; i++)
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);

    task_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(
        64, TASK_THREAD_NUM * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < TASK_THREAD_NUM; i++)
        task_ptoks_ptr[i] = new moodycamel::ProducerToken(complete_task_queue_);
}

void Simulator::initialize_uplink_buffers()
{
    alloc_buffer_1d(&task_threads, TASK_THREAD_NUM, 64, 0);
    alloc_buffer_1d(&context, TASK_THREAD_NUM, 64, 0);

    socket_buffer_size_ = (long long)packet_length * symbol_num_perframe
        * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    socket_buffer_status_size_
        = symbol_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    socket_buffer_.malloc(SOCKET_RX_THREAD_NUM, socket_buffer_size_, 64);
    socket_buffer_status_.calloc(
        SOCKET_RX_THREAD_NUM, socket_buffer_status_size_, 64);

    /* initilize all uplink status checkers */
    alloc_buffer_1d(&rx_counter_packets_, TASK_BUFFER_FRAME_NUM, 64, 1);

    frame_start.calloc(SOCKET_RX_THREAD_NUM, kNumStatsFrames, 4096);
    alloc_buffer_1d(&frame_start_receive, kNumStatsFrames, 4096, 1);
    alloc_buffer_1d(&frame_end_receive, kNumStatsFrames, 4096, 1);
    alloc_buffer_1d(&frame_start_tx, kNumStatsFrames, 4096, 1);
    alloc_buffer_1d(&frame_end_tx, kNumStatsFrames, 4096, 1);
}

void Simulator::free_uplink_buffers()
{
    // free_buffer_1d(&pilots_);

    socket_buffer_.free();
    socket_buffer_status_.free();

    free_buffer_1d(&rx_counter_packets_);

    frame_start.free();
    free_buffer_1d(&frame_start_receive);
    free_buffer_1d(&frame_end_receive);
    free_buffer_1d(&frame_start_tx);
    free_buffer_1d(&frame_end_tx);
}
