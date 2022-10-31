function [avg_proc_duration, std_proc_duration] = parse_dl_file(filename)
    fid = fopen(filename);

    PLOT = 0;
    temp = textscan(fid,"%f%f%f%f%f%f%f%f%f%f", 'HeaderLines',2);
    frame_start = temp{1};   
    pilot_recv=temp{2};
    process_start = temp{3};
    all_pilot_received = temp{4};
    fft_processed = temp{5};
    zf_processed = temp{6};
    precode_processed = temp{7};
    encode_processed = temp{9};
    rx_processed = temp{10};
    ifft_processed = temp{8};


    zf_duration = zf_processed-fft_processed;
    fft_duration = fft_processed;
    ifft_duration = ifft_processed - zf_duration;
    frame_duration_rx = rx_processed;
    frame_duration_fft = fft_processed;
    frame_duration_precode = precode_processed;
    frame_duration_encode = encode_processed;
    frame_duration_zf = zf_processed;
    frame_duration_ifft = ifft_processed;
    frame_duration = diff(frame_start);
    frame_count = length(precode_processed);
    frame_count = frame_count - 1;

    pilot_rx_duration = all_pilot_received;

    queueing_delay = process_start;
    frame_duration_processing = max([frame_duration,frame_duration_encode(1:end-1)],[],2);

 
    index = [1:frame_count];
    % index(1:512:frame_count) = [];
    subset = index(1200:frame_count-2);


    fprintf("Average delay between rx thread and main thread: %.2f\n", ...
        mean(pilot_recv(subset)));

    avg_frame_duration = mean(frame_duration(subset));
    avg_rx_duration = mean(frame_duration_rx(subset));
    avg_precode_duration = mean(frame_duration_precode(subset));
    avg_proc_duration = mean(frame_duration_ifft(subset));
    std_proc_duration = std(frame_duration_ifft(subset));
    avg_proc_start = mean(process_start(subset));
    avg_zf_duration = mean(zf_duration(subset));
    avg_fft_duration = mean(fft_duration(subset));
    avg_ifft_duration = mean(ifft_duration(subset));

    fprintf("In main thread: queueing delay: %.2f, FFT of pilots done: %.2f, ZF done: %.2f, IFFT done: %.2f, total: %.2f\n",...
        avg_proc_start,avg_fft_duration,avg_zf_duration,...
        avg_ifft_duration,avg_proc_duration);
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
    fprintf("DL processing duration: %.2f, std: %.2f\n", avg_proc_duration, std_proc_duration);

    if PLOT
        figure(1);clf;
        % plot(fft_duration(1:frame_count),'LineWidth',2);
        plot(fft_time_in_function(subset), 'LineWidth',2);
        hold on;
        plot(zf_time_in_function(subset), 'LineWidth',2);
        hold on;
        plot(demul_time_in_function(subset), 'LineWidth',2);
        hold on;
        plot(zf_duration(subset),'LineWidth',2);
        set(gca,'FontSize',18);
        grid on;
        xlabel('Frame id');
        ylabel('Duration (us)');
        legend({'FFT in function','ZF in function','Demodulation in function','ZF'});

        figure(2);clf;
        plot(frame_duration_fft(subset),'LineWidth',2);
        hold on;
        plot(frame_duration_zf(subset),'LineWidth',2);
        hold on;
        plot(frame_duration_demul(subset),'LineWidth',2);
        hold on;
        plot(frame_duration_rx(subset),'LineWidth',2);
        hold on;
        plot(frame_duration_encode(subset),'LineWidth',2);
        set(gca,'FontSize',18);
        grid on;
        xlabel('Frame id');
        ylabel('Duration (us)');
        legend({'FFT of pilots','ZF','Demodulation','RX', "Deocde"});



        figure(4);clf;
        plot(process_delay(1:frame_count),'LineWidth',2);
        set(gca,'FontSize',18);
        grid on;
        xlabel('Frame id');
        ylabel('Delay (us)');
        title('Processing delay')

        figure(5);clf;
        h1 = cdfplot(process_delay(subset));
        set(h1,'LineWidth',2);
        set(gca,'FontSize',18);
        grid on;
        % xlabel('Frame id');
        xlabel('Delay (us)');
        title('Processing delay')

        % figure(7);clf;
        % plot(pilot_recv(1:end)-frame_start(1:end),'LineWidth',2);
        % set(gca,'FontSize',18);
        % grid on;
        % xlabel('Frame id');
        % ylabel('Time (us)');
        % title('Delay between main thread and rx thread')

        % figure(8);clf;
        % h1 = cdfplot(pilot_recv(subset)-frame_start(subset));
        % set(h1,'LineWidth',2);
        % set(gca,'FontSize',18);
        % grid on;
        % % xlabel('Frame id');
        % xlabel('Time (us)');
        % title('Delay between main thread and rx thread')

        %%
        % fid = fopen('tx_result.txt');
        % temp = textscan(fid,"%f");
        % frame_ends=temp{1};
        % frame_duration_tx = diff(frame_ends);
        % avg_tx_duration = mean(frame_duration_tx(1200:length(frame_duration_tx)-1));
        % fprintf("In emulator: TX duration %.5f\n", avg_tx_duration);
        % figure(6);clf;
        % plot(frame_duration_tx(subset),'LineWidth',24);
        % set(gca,'FontSize', 24);
        % grid on;
        % xlabel('Frame id');
        % ylabel('Time (us)');
        % title('TX duration');

        frame_duration_rx1 = diff(frame_start);
        figure(3);clf;
        h1=cdfplot(frame_duration_rx(subset));
        hold on;
        h2=cdfplot(frame_duration_precode(subset));
        hold on;
        % h3=cdfplot(frame_duration_tx(1200:length(frame_duration_tx)-1));
        h3=cdfplot(ifft_processed(subset));
        set(h1,'LineWidth',2);
        set(h2,'LineWidth',2);
        set(h3,'LineWidth',2,'LineStyle', '--', 'Color', 'k');
        grid on;
        set(gca,'FontSize',18);
        xlabel('Duration (us)');
        % legend({sprintf('RX duration (avg: %.2f us)',avg_rx_duration),...
        %     sprintf('Data processing duration (avg: %.2f us)', avg_proc_duration)},'Location','southeast');
        % legend({sprintf('RX duration (avg: %.2f us)',avg_rx_duration),...
        %     sprintf('Data processing duration (avg: %.2f us)', avg_proc_duration),...
        %     sprintf('TX duration in emulator (avg: %.2f us)', avg_tx_duration)},'Location','southeast');
        legend({sprintf('RX duration (avg: %.2f us)',avg_rx_duration),...
            sprintf('Precode done (avg: %.2f us)', avg_precode_duration),...
            sprintf('IFFT done (avg: %.2f us)', avg_proc_duration)},'Location','southeast');
        title('CDF of frame duration and processing duration')
    end
end

