fid = fopen('../data/timeresult.txt');

% 1. Pilot RX by socket threads (= reference time), 
% 2. kPilotRX, 3. kProcessingStarted, 4. kPilotAllRX, 5. kFFTDone, 6. kZFDone, 
% 7. kDemulDone, 8. kDecodeDone, 9. kRXDone, 10. time in CSI, 11. time in FFT, "
% 12. time in ZF, 13. time in Demul, 14. time in Decode
temp = textscan(fid,"%f%f%f%f%f%f%f%f%f%f%f%f%f%f", 'HeaderLines',2);
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
fft_duration = fft_processed;
decode_duration = decode_processed - zf_duration;
frame_duration_rx = rx_processed;
frame_duration_fft = fft_processed;
frame_duration_demul = demul_processed;
frame_duration_decode = decode_processed;
frame_duration_zf = zf_processed;
frame_duration = diff(frame_start);
frame_count = length(demul_processed);
frame_count = frame_count - 1;

pilot_rx_duration = all_pilot_received;

queueing_delay = process_start;
frame_duration_processing = max([frame_duration,frame_duration_decode(1:end-1)],[],2);

csi_time_in_function = temp{10};
fft_time_in_function = temp{11};
zf_time_in_function = temp{12};
demul_time_in_function = temp{13};
decode_time_in_function = temp{14};

index = [1:frame_count];
subset = index(1200:frame_count-2);


fprintf("Average delay between rx thread and main thread: %.2f\n", ...
    mean(pilot_recv(subset)));

fprintf("In functions: CSI: %.2f, ZF: %.2f, FFT + Demul + Decode: %.2f+%.2f+%.2f=%.2f, total: %.2f\n",...
    mean(csi_time_in_function(subset)),...
    mean(zf_time_in_function(subset)),mean(fft_time_in_function(subset)),...
    mean(demul_time_in_function(subset)), mean(decode_time_in_function(subset)), ...
    mean(fft_time_in_function(subset))+mean(demul_time_in_function(subset))+mean(decode_time_in_function(subset)),...
    mean(fft_time_in_function(subset))+mean(zf_time_in_function(subset))+mean(demul_time_in_function(subset))+mean(decode_time_in_function(subset)));

avg_frame_duration = mean(frame_duration(subset));
avg_rx_duration = mean(frame_duration_rx(subset));
avg_demul_duration = mean(frame_duration_demul(subset));
avg_proc_duration = mean(frame_duration_decode(subset));
std_proc_duration = std(frame_duration_decode(subset));
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
fprintf("UL processing duration: %.2f, std: %.2f\n", mean(frame_duration_decode(subset)), std(frame_duration_decode(subset)));

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
plot(frame_duration_decode(subset),'LineWidth',2);
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
h2=cdfplot(frame_duration_demul(subset));
hold on;
% h3=cdfplot(frame_duration_tx(1200:length(frame_duration_tx)-1));
h3=cdfplot(decode_processed(subset));
set(h1,'LineWidth',2);
set(h2,'LineWidth',2);
set(h3,'LineWidth',2,'LineStyle', '--', 'Color', 'k');
grid on;
set(gca,'FontSize',18);
xlabel('Duration (us)');

[f_proc_delay,x_proc_delay] = ecdf(frame_duration_decode(subset));
x_set = x_proc_delay(f_proc_delay>0.99);
tail_proc_duration = x_set(1);
legend({sprintf('RX duration (avg: %.2f us)',avg_rx_duration),...
    sprintf('Demul done (avg: %.2f us)', avg_demul_duration),...
    sprintf('Decode done (avg: %.2f us)', ...
    avg_proc_duration)},'Location','southeast');
title('CDF of frame duration and processing duration')

fprintf('99th: %.2f, max: %.2f\n',  tail_proc_duration, max(frame_duration_decode(subset)));



