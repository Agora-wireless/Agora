clearvars; clc; close all;

PLOT_PROFILING = 1;

%%
% Read Agora configuration parameters
fid = fopen('../../files/config/experiment/agora_config.txt');
tline_split = textscan(fid, "%f %f %d %d %n %d %d %d %n %d %d %d %n %d %d", 'HeaderLines', 1);

freq_ghz = tline_split{1}; freq_mhz  = freq_ghz * 1000;
rate = tline_split{2};
fft_size = tline_split{3};
ofdm_data_num = tline_split{4};
samps_per_symbol = tline_split{5};
demul_block_size = tline_split{6};
bs_radio_num = tline_split{7};
ue_radio_num = tline_split{8};
worker_thread_num = tline_split{9};
pilot_symbol_num_perframe = tline_split{10};
ul_symbol_num_perframe = tline_split{11};
dl_symbol_num_perframe = tline_split{12};
total_symbol_num_perframe = tline_split{13};
max_frame = tline_split{14};
profiling_frame = tline_split{15};
fclose(fid);

linewidth = 4;
max_events_count = 500;
x_axis_start = 0;
x_axis_end = 1800;
y_axis_start = 0;
y_axis_end = worker_thread_num + 5;

%%
keySet = {'PacketRX', 'FFT', 'Beam', 'Demul', 'IFFT', 'Precode', 'PacketTX', 'Decode', 'Encode', 'PacketToMac', 'SNRReport', 'Broadcast'};
valueSet = [1 2 3 4 5 6 7 8 9 10 11 12];
M = containers.Map(keySet,valueSet);
num_worker_types = length(valueSet);

master_eq_start = zeros(num_worker_types, total_symbol_num_perframe, max_events_count);
master_eq_end = zeros(num_worker_types, total_symbol_num_perframe, max_events_count);
master_eq_duration = zeros(num_worker_types, total_symbol_num_perframe, max_events_count);
master_eq_id = zeros(num_worker_types, total_symbol_num_perframe);
master_dq_start = zeros(num_worker_types, max_events_count);
master_dq_end = zeros(num_worker_types, max_events_count);
master_dq_duration = zeros(num_worker_types, max_events_count);
master_dq_id = zeros(num_worker_types, 1);

% Read master timestamps
fid = fopen('../../files/config/experiment/timestamps_master.txt');

line_count_master = 0;
while ~feof(fid)
    line_count_master = line_count_master + 1;
    tline = fgetl(fid);
    tline_split = split(tline, [" ", ":", "[", "]", "-"]);
    if (length(tline_split) ~= 16 && length(tline_split) ~= 14)
        disp(tline);
        if (length(tline_split) == 10)
            ref_tsc = str2num(tline_split{10});
        end
    elseif  length(tline_split) == 16
        type = M(tline_split{9});
        symbol_id = str2num(tline_split{5}) + 1;
        task_start = str2num(tline_split{13});
        task_end = str2num(tline_split{12});
        duration = str2num(tline_split{16});
        if (tline_split(7) == "enqueue")
            master_eq_id(type, symbol_id) = master_eq_id(type, symbol_id) + 1;
            master_eq_start(type, symbol_id, master_eq_id(type, symbol_id)) = task_start;
            master_eq_end(type, symbol_id, master_eq_id(type, symbol_id)) = task_end;
            master_eq_duration(type, symbol_id, master_eq_id(type, symbol_id)) = duration;
        end
    elseif length(tline_split) == 14
        type = M(tline_split{7});
        task_start = str2num(tline_split{11});
        task_end = str2num(tline_split{10});
        duration = str2num(tline_split{14});
        master_dq_id(type) = master_dq_id(M(tline_split{7})) + 1;
        master_dq_start(type, master_dq_id(type)) = task_start;
        master_dq_end(type, master_dq_id(type)) = task_end;
        master_dq_duration(type, master_dq_id(type)) = duration;
    end
end
master_dq_id(3) = 0; % Ignoring PacketTX Dq events since PacketTX Eq events are not logged
fclose(fid);


%%
% Read workers timestamps
fid = fopen('../../files/config/experiment/timestamps_workers.txt');
worker_eq_start = zeros(worker_thread_num, num_worker_types, total_symbol_num_perframe, max_events_count);
worker_eq_end = zeros(worker_thread_num, num_worker_types, total_symbol_num_perframe, max_events_count);
worker_eq_duration = zeros(worker_thread_num, num_worker_types, total_symbol_num_perframe, max_events_count);
worker_eq_id = zeros(worker_thread_num, num_worker_types, total_symbol_num_perframe);
worker_dq_start = zeros(worker_thread_num, num_worker_types, total_symbol_num_perframe, max_events_count);
worker_dq_end = zeros(worker_thread_num, num_worker_types, total_symbol_num_perframe, max_events_count);
worker_dq_duration = zeros(worker_thread_num, num_worker_types, total_symbol_num_perframe, max_events_count);
worker_dq_id = zeros(worker_thread_num, num_worker_types, total_symbol_num_perframe);
enqueue_time = zeros(worker_thread_num, 1);
dequeue_time = zeros(worker_thread_num, 1);

line_count_worker = 0;
while ~feof(fid)
    line_count_worker = line_count_worker + 1;
    tline = fgetl(fid);
    tline_split = split(tline, [" ", ":", "[", "]", "-", "(", ")"]);
    if (length(tline_split) ~= 17)
        disp(tline);
        if (length(tline_split) == 20)
            worker_id = str2num(tline_split{2}) + 1;
            enqueue_time(worker_id) = str2num(tline_split{9});
            dequeue_time(worker_id) = str2num(tline_split{19});
        end
    else
        worker_id = str2num(tline_split{2}) + 1;
        symbol_id = str2num(tline_split{6}) + 1;
        type = M(tline_split{10});
        task_start = str2num(tline_split{14});
        task_end = str2num(tline_split{13});
        duration = str2num(tline_split{17});
        if (tline_split(8) == "enqueue")
            worker_eq_id(worker_id, type, symbol_id) = worker_eq_id(worker_id, type, symbol_id) + 1;
            id = worker_eq_id(worker_id, type, symbol_id);
            worker_eq_start(worker_id, type, symbol_id, id) = task_start;
            worker_eq_end(worker_id, type, symbol_id, id) = task_end;
            worker_eq_duration(worker_id, type, symbol_id, id) = duration;
        else
            worker_dq_id(worker_id, type, symbol_id) = worker_dq_id(worker_id, type, symbol_id) + 1;
            id = worker_dq_id(worker_id, type, symbol_id);
            worker_dq_start(worker_id, type, symbol_id, id) = task_start;
            worker_dq_end(worker_id, type, symbol_id, id) = task_end;
            worker_dq_duration(worker_id, type, symbol_id, id) = duration;
        end
    end
end
fclose(fid);
fprintf("Worker enqueue %.2f us per thread, dequeue %.2f us per thread\n", mean(enqueue_time), mean(dequeue_time));

%%
worker_min_tsc = min(worker_dq_start(find(worker_dq_start(:) > 0)));
if worker_min_tsc < ref_tsc
    ref_tsc = worker_min_tsc;
end
master_eq_start_2 = (master_eq_start - ref_tsc)/freq_mhz;
master_eq_end_2 = (master_eq_end - ref_tsc)/freq_mhz;
master_dq_start_2 = (master_dq_start - ref_tsc)/freq_mhz;
master_dq_end_2 = (master_dq_end - ref_tsc)/freq_mhz;

worker_eq_start_2 = (worker_eq_start - ref_tsc)/freq_mhz;
worker_eq_end_2 = (worker_eq_end - ref_tsc)/freq_mhz;
worker_dq_start_2 = (worker_dq_start - ref_tsc)/freq_mhz;
worker_dq_end_2 = (worker_dq_end - ref_tsc)/freq_mhz;

worker_avg_task_duration = [];
worker_total_num_tasks = [];
sum_task_duration = zeros(num_worker_types, 1);
sum_task_duration_per_symbol = zeros(num_worker_types, total_symbol_num_perframe);
sum_task_num = zeros(num_worker_types, 1);
sum_task_num_per_symbol = zeros(num_worker_types, total_symbol_num_perframe);
for i = 1:worker_thread_num
    task_duration = [];
    eq_duration = [];
    dq_duration = [];
    num_tasks = [];
    for j = 1:num_worker_types
        task_duration(j) = 0;
        eq_duration(j) = 0;
        dq_duration(j) = 0;
        num_tasks(j) = 0;
        for k = 1: total_symbol_num_perframe
            task_duration_this_symbol = sum((worker_eq_start_2(i, j, k, 1:worker_dq_id(i, j, k)) - worker_dq_end_2(i, j, k, 1:worker_dq_id(i, j, k))));
            sum_task_duration_per_symbol(j, k) = sum_task_duration_per_symbol(j, k) + task_duration_this_symbol;
            sum_task_num_per_symbol(j, k) = sum_task_num_per_symbol(j, k) + worker_dq_id(i, j, k);
            num_tasks(j) = num_tasks(j) + worker_dq_id(i, j, k);
            task_duration(j) = task_duration(j) + task_duration_this_symbol;
            eq_duration(j) = eq_duration(j) + sum((worker_eq_end_2(i, j, k, 1:worker_dq_id(i, j, k)) - worker_eq_start_2(i, j, k, 1:worker_dq_id(i, j, k))));
            dq_duration(j) = dq_duration(j) + sum((worker_dq_end_2(i, j, k, 1:worker_dq_id(i, j, k)) - worker_dq_start_2(i, j, k, 1:worker_dq_id(i, j, k))));
        end
        sum_task_duration(j) = sum_task_duration(j) + task_duration(j);
        sum_task_num(j) = sum_task_num(j) + num_tasks(j);
        task_duration(j) = task_duration(j) / num_tasks(j);
        dq_duration(j) = dq_duration(j) / num_tasks(j);
        eq_duration(j) = eq_duration(j) / num_tasks(j);
        worker_avg_task_duration(i, j) = task_duration(j);
        worker_total_num_tasks(i, j) = num_tasks(j);
    end
    fprintf("Worker %d: %d FFTs: %.2f us, %d Beams: %.2f us, %d Demuls: %.2f us, %d Decodes: %.2f us, %d Encodes: %.2f us, %d Precodes: %.2f us, %d IFFTs: %.2f us\n", ...
        i, num_tasks(1), task_duration(1), num_tasks(4), task_duration(4), num_tasks(5), task_duration(5), num_tasks(6), task_duration(6), num_tasks(7), task_duration(7), num_tasks(8), task_duration(8), num_tasks(9), task_duration(9));
end

fprintf("Total time: %d FFTs: %.2f us, %d Beams: %.2f us, %d Demuls: %.2f us, %d Decodes: %.2f us, %d Encodes: %.2f us, %d Precodes: %.2f us, %d IFFTs: %.2f us\n", ...
    sum_task_num(1), sum_task_duration(1), sum_task_num(4), sum_task_duration(4), sum_task_num(5), sum_task_duration(5), sum_task_num(6), sum_task_duration(6), sum_task_num(7), sum_task_duration(7), sum_task_num(8), sum_task_duration(8), sum_task_num(9), sum_task_duration(9));

%%
if PLOT_PROFILING
    colors = [0                     0.4470              0.7410;
        0.8500                0.3250              0.0980;
        0.9290                0.6940              0.1250;
        0.4940                0.1840              0.5560;
        0.4660                0.6740              0.1880;
        0.3010                0.7450              0.9330;
        0.6350                0.0780              0.1840;
        0.2136                0.8934              0.4327;
        0.9873                0.3091              0.7263];
    offset = 0.15;
    colors_2 = [0                   0.4470              0.7410 - offset;
        0.8500              0.3250              0.0980 - offset;
        0.9290              0.6940 - offset     0.1250;
        0.4940              0.1840              0.5560 - offset;
        0.4660              0.6740 - offset     0.1880;
        0.3010              0.7450              0.9330 - offset;
        0.6350 - offset     0.0780              0.1840;
        0.2136              0.8934 - offset     0.4327;
        0.5873              0.2591              0.7263 - offset];

    figure(100);
    % Plot worker timings
    h_all = [];
    h_dq = [];
    h_eq = [];
    for worker = 1 : worker_thread_num
        for type = 1 : num_worker_types
            for symbol = 1 : total_symbol_num_perframe
                for j = 1 : worker_dq_id(worker, type, symbol)
                    h_all(type) = plot([worker_dq_start_2(worker, type, symbol, j), worker_eq_end_2(worker, type, symbol, j)], worker * ones(1, 2), 'Color', [colors(type, :)], 'LineWidth', linewidth);
                    hold on;
                end
            end
        end
    end

    % Plot master timings
    h_master = [[]];
    h_dq = [[]];
    h_eq = [[]];
    for i = 1 : num_worker_types
        for k = 1 : total_symbol_num_perframe
            for j = 1 : master_eq_id(i, k)
                h_eq(i) = plot([master_eq_start_2(i, k, j), master_eq_end_2(i, k, j)], (worker_thread_num + 2) * ones(1, 2), 'Color', colors(i, :), 'LineWidth', linewidth);
                h_master = plot([master_eq_start_2(i, k, j), master_eq_end_2(i, k, j)], (worker_thread_num + 3) * ones(1, 2), 'Color', 'k', 'LineWidth', linewidth);
                hold on;
            end
        end
    end

    for i = 1 : num_worker_types
        for j = 1 : master_dq_id(i)
            h_dq(i, :) = plot([master_dq_start_2(i, j), master_dq_end_2(i, j)], (worker_thread_num + 2) * ones(1, 2), 'Color', colors(i, :), 'LineWidth', linewidth);
            h_master = plot([master_dq_start_2(i, j), master_dq_end_2(i, j)], (worker_thread_num + 3) * ones(1, 2), 'Color', 'k', 'LineWidth', linewidth);
            hold on;
        end
    end

    % h_symbol = plot([0, (samps_per_symbol * total_symbol_num_perframe * 1e6)/rate], (worker_thread_num + 4) * ones(1, 2), 'Color', [0.8 0.8 0.8], 'LineWidth', linewidth);

    set(gca,'FontSize', 12);
    xlabel('Time (us)');
    ylabel('Thread Id');
    title(['Profiling Frame Number - ', num2str(profiling_frame)]);
    grid on;
    % xlim([x_axis_start x_axis_end]);
    ylim([y_axis_start y_axis_end]);
    if dl_symbol_num_perframe == 0 % UL only frame configuration
        l = legend([h_all(2), h_all(3), h_all(4), h_all(8), h_master],...
            'FFT', 'Beam', 'Demul', 'Decode', 'Master', 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'NumColumns', 4);
    elseif ul_symbol_num_perframe == 0 % DL only frame confguration
        l = legend([h_all(2), h_all(3), h_all(9), h_all(6), h_all(5), h_master],...
            'FFT', 'Beam', 'Encode', 'Precode', 'IFFT', 'Master', 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'NumColumns', 4);
    else % Both UL and DL frame configuration
        l = legend([h_all(2), h_all(3), h_all(4), h_all(5), h_all(6), h_all(8), h_all(9), h_master],...
            'FFT', 'Beam', 'Demul', 'IFFT', 'Precode', 'Decode', 'Encode', 'Master', 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'NumColumns', 4);
    end
end
