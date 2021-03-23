frame_length_range = [1:5];
ue_num = 16;
ant_num = 64;


avg_proc_duration_agora = [];
std_proc_duration_agora = [];
avg_proc_duration_bigstation = [];
std_proc_duration_bigstation = [];

avg_fft_duration_agora = [];
avg_zf_duration_agora = [];
avg_decode_duration_agora = [];
avg_proc_start_agora = [];

avg_fft_duration_bigstation = [];
avg_zf_duration_bigstation = [];
avg_decode_duration_bigstation = [];
avg_proc_start_bigstation = [];

index = 0;
for i = frame_length_range
    index = index+1;
    filename = sprintf('uplink/timeresult_agora_%dx%d_%dms.txt',ant_num, ue_num, frame_length_range(index)); 
    [avg_proc_duration_agora(index), std_proc_duration_agora(index), ...
        avg_proc_start_agora(index), avg_fft_duration_agora(index), ...
        avg_zf_duration_agora(index), avg_decode_duration_agora(index)] = parse_ul_file(filename);
end

index = 0;
for i = frame_length_range
    index = index+1;
    filename = sprintf('uplink/timeresult_bigstation_%dx%d_%dms.txt',ant_num, ue_num, frame_length_range(index)); 
    [avg_proc_duration_bigstation(index), std_proc_duration_bigstation(index), ...
        avg_proc_start_bigstation(index), avg_fft_duration_bigstation(index), ...
        avg_zf_duration_bigstation(index), avg_decode_duration_bigstation(index)] = parse_ul_file(filename);
end






%%
% load("results.mat")
colors = [0    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
    0.3010    0.7450    0.9330;
    0.6350    0.0780    0.1840;];

figure(6);
clf;
errorbar(frame_length_range*1000,avg_proc_duration_bigstation,std_proc_duration_bigstation,'Color', colors(1,:), 'LineWidth',2);
hold on;
h1 = plot(frame_length_range*1000,avg_proc_duration_bigstation,'s-','Color', colors(1,:), 'LineWidth',2,'MarkerSize',20);
hold on;
errorbar(frame_length_range*1000,avg_proc_duration_agora,std_proc_duration_agora,'Color', colors(2,:), 'LineWidth',2);
hold on;
h2 = plot(frame_length_range*1000,avg_proc_duration_agora, '^-', 'Color', colors(2,:), 'LineWidth',2, 'MarkerSize',20);
hold on;
h3 = plot(frame_length_range*1000, frame_length_range*1000, 'k--', 'LineWidth',2);
set(gca,'FontSize',24);
xlabel('Frame length (us)');
ylabel('Processing time (us)');
grid on;
% title('Uplink frame Processing latency');
% grid on;
% l1 = legend([h_fft_big,h_fft_big_5core,h_fft_comp,h_pilot,h_theory],{'BigStation (n_{FFT}=min)','BigStation (n_{FFT}=5)', 'agora', 'Pilot RX', 'Theoretical pilot RX'},'Location','best','Orientation','vertical');
% l1.NumColumns = 2; 
% set(gca, 'YScale', 'log')
ylim([0 7000]);
legend([h1,h2,h3],'Pipeline parallel','Agora','Frame length', 'location','best');
%xticks([0:8:32]);
%xlim([5,35]);
%set(gcf,'Position',[100 100 350 350])

%%
avg_proc_start_duration = [avg_proc_start_agora(1), avg_proc_start_bigstation(1)];
avg_zf_duration = [avg_zf_duration_agora(1), avg_zf_duration_bigstation(1)];
avg_fft_duration = [avg_fft_duration_agora(1) - avg_proc_start_agora(1), avg_fft_duration_bigstation(1) - avg_proc_start_bigstation(1)];
avg_decode_duration = [avg_decode_duration_agora(1), avg_decode_duration_bigstation(1)];
avg_all = [avg_proc_start_duration; avg_fft_duration; avg_zf_duration; avg_decode_duration];
figure(7);
clf;
h1=bar(avg_all);
set(h1(1),'facecolor',[0.9,0.9,0.9]);
set(h1(2),'facecolor',[0.4,0.4,0.4]);
set(gca,'FontSize',24);
names = {'Queueing'; 'Pilot'; 'ZF'; 'DataProc';};
ylabel('Processing time (us)');
set(gca,'xtick',[1:length(names)],'xticklabel',names);
legend("agora", "Pipeline parallel", "location", "northwest");


%%
figure(8);
clf;
h1 = plot(frame_length_range*1000,std_proc_duration_bigstation,'s--','Color', colors(1,:), 'LineWidth',2,'MarkerSize',15);
hold on;
h2 = plot(frame_length_range*1000,avg_proc_duration_bigstation,'s-','Color', colors(1,:), 'LineWidth',2,'MarkerSize',15);
hold on;
h3 = plot(frame_length_range*1000,std_proc_duration_agora,'^--','Color', colors(2,:), 'LineWidth',2,'MarkerSize',15);
hold on;
h4 = plot(frame_length_range*1000,avg_proc_duration_agora, '^-', 'Color', colors(2,:), 'LineWidth',2, 'MarkerSize',15);
hold on;
h5 = plot(frame_length_range*1000, frame_length_range*1000, 'k--', 'LineWidth',2);
set(gca,'FontSize',24);
xlabel('Frame length (us)');
ylabel('Processing time (us)');
grid on;
% title('Uplink frame Processing latency');
% grid on;
% l1 = legend([h_fft_big,h_fft_big_5core,h_fft_comp,h_pilot,h_theory],{'BigStation (n_{FFT}=min)','BigStation (n_{FFT}=5)', 'agora', 'Pilot RX', 'Theoretical pilot RX'},'Location','best','Orientation','vertical');
% l1.NumColumns = 2; 
% set(gca, 'YScale', 'log')
ylim([0 7000]);
l=legend([h1,h2,h3,h4,h5],'BigStation (99th tail)','BigStation','Agora (99th tail)','Agora','Frame length', 'location','best');
set(l,'FontSize',18);
