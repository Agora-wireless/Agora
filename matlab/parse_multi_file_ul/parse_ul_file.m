function [avg_proc_duration, std_proc_duration, avg_proc_start, avg_fft_duration, avg_zf_duration, avg_decode_duration] = parse_ul_file(filename)
    fid = fopen(filename);

    % 1. Pilot RX by socket threads (= reference time), 
    % 2. kPilotRX, 3. kProcessingStarted, 4. kPilotAllRX, 5. kFFTDone, 6. kZFDone, 
    % 7. kDemulDone, 8. kDecodeDone, 9. kRXDone, 10. time in CSI, 11. time in FFT, "
    % 12. time in ZF, 13. time in Demul, 14. time in Decode
    temp = textscan(fid,"%f%f%f%f%f%f%f%f%f%f%f%f%f", 'HeaderLines',2);
    frame_start = temp{1};   
    pilot_recv=temp{2};
    process_start = temp{3};
    all_pilot_received = temp{4};
    fft_processed = temp{5};
    zf_processed = temp{6};
    demul_processed = temp{7};
    decode_processed = temp{8};
    rx_processed = temp{9};


    demul_duration = demul_processed-zf_processed;
    zf_duration = zf_processed-fft_processed;
    decode_duration = decode_processed-zf_processed;
    fft_duration = fft_processed;
    frame_duration_rx = rx_processed;
    frame_duration_fft = fft_processed;
    frame_duration_demul = demul_processed;
    frame_duration_decode = decode_processed;
    frame_duration_zf = zf_processed;
    frame_duration = diff(frame_start);
    frame_count = length(demul_processed);
    frame_count = frame_count - 1;

    pilot_rx_duration = all_pilot_received;

    frame_duration_processing = max([frame_duration,frame_duration_decode(1:end-1)],[],2);

    csi_time_in_function = temp{10};
    fft_time_in_function = temp{11};
    zf_time_in_function = temp{12};
    demul_time_in_function = temp{13};


    index = [1:frame_count];
    % index(1:512:frame_count) = [];
    subset = index(1200:frame_count-2);


    fprintf("Average delay between rx thread and main thread: %.2f\n", ...
        mean(pilot_recv(subset)));

    fprintf("In functions: CSI: %.2f, ZF: %.2f, FFT + Demul: %.2f+%.2f=%.2f, total: %.2f\n",...
        mean(csi_time_in_function(subset)),...
        mean(zf_time_in_function(subset)),mean(fft_time_in_function(subset)),...
        mean(demul_time_in_function(1200:frame_count)),mean(fft_time_in_function(subset))+mean(demul_time_in_function(1200:frame_count)),...
        mean(fft_time_in_function(subset))+mean(zf_time_in_function(subset))+mean(demul_time_in_function(1200:frame_count)));

    avg_frame_duration = mean(frame_duration(subset));
    avg_rx_duration = mean(frame_duration_rx(subset));
    avg_demul_duration = mean(frame_duration_demul(subset));
    avg_proc_duration = mean(frame_duration_decode(subset));
    std_proc_duration = std(frame_duration_decode(subset));
%     [f_proc_delay,x_proc_delay] = ecdf(frame_duration_decode(subset));
%     x_set = x_proc_delay(f_proc_delay>0.99);
%     std_proc_duration = x_set(1);

    avg_proc_start = mean(process_start(subset));
    avg_zf_duration = mean(zf_duration(subset));
    avg_fft_duration = mean(fft_duration(subset));
    avg_decode_duration = mean(decode_duration(subset));
    fprintf("In main thread: queueing delay: %.2f, FFT of pilots done: %.2f, ZF done: %.2f, Decode done: %.2f, total: %.2f\n",...
        avg_proc_start,avg_fft_duration,avg_zf_duration,...
        avg_decode_duration,avg_proc_duration);
    fprintf("Frame duration: %.2f, RX duration: %.5f, Processing delay: %.5f\n", avg_frame_duration,avg_rx_duration, avg_proc_duration-avg_rx_duration);
    fprintf("Pilot RX duration: %.2f\n", mean(pilot_rx_duration(subset)));

    % process_delay = demul_processed-rx_processed;
    process_delay = frame_duration_processing - frame_duration;
    avg_proc_delay = mean(process_delay(subset));
    std_proc_delay = std(process_delay(subset));

    [f_proc_delay,x_proc_delay] = ecdf(process_delay(subset));
    x_set = x_proc_delay(f_proc_delay>0.99);
    proc_delay_99th = x_set(1);

    [f_zf_delay,x_zf_delay] = ecdf(zf_duration(subset));
    x_set = x_zf_delay(f_zf_delay>0.99);
    zf_delay_99th = x_set(1);

    fprintf("Process delay 99th: %.2f, ZF delay 99th: %.3f\n", proc_delay_99th, zf_delay_99th);
    fprintf("Process delay mean: %.2f (std: %.2f), ZF delay mean: %.2f\n", avg_proc_delay, std_proc_delay, avg_zf_duration);
    fprintf("UL processing duration: %.2f, std: %.2f\n", avg_proc_duration, std_proc_duration);
end