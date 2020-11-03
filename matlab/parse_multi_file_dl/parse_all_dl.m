
ant_num_range = [16, 32, 48, 64];
num_cores = [11, 15,18, 30];


avg_proc_millipede = [];
std_proc_millipede = [];
avg_proc_bigstation = [];
std_proc_bigstation = [];

index = 0;
for ant_num = ant_num_range
    index = index+1;
    filename = sprintf('timeresult_millipede_%dx%d_1ms.txt',ant_num, 16); 
    [avg_proc_millipede(index), std_proc_millipede(index)] = parse_dl_file(filename);
end

index = 0;
for ant_num = ant_num_range
    index = index+1;
    filename = sprintf('timeresult_bigstation_%dx%d_1ms.txt',ant_num, 16); 
    [avg_proc_bigstation(index), std_proc_bigstation(index)] = parse_dl_file(filename);
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
errorbar(ant_num_range,avg_proc_bigstation,std_proc_bigstation,'Color', colors(1,:), 'LineWidth',2);
hold on;
h1 = plot(ant_num_range,avg_proc_bigstation, '^', 'Color', colors(1,:), 'LineWidth',2, 'MarkerSize',20);
hold on;
errorbar(ant_num_range,avg_proc_millipede,std_proc_millipede,'Color', colors(2,:), 'LineWidth',2);
hold on;
h2 = plot(ant_num_range,avg_proc_millipede,'s', 'Color', colors(2,:), 'LineWidth',2,'MarkerSize',20);

set(gca,'FontSize',24);
xlim([10,70]);
xlabel('Number of antennas');
ylabel('Time (us)');
xticks(ant_num_range);
grid on;
% title('Uplink frame Processing latency');
% grid on;
% l1 = legend([h_fft_big,h_fft_big_5core,h_fft_comp,h_pilot,h_theory],{'BigStation (n_{FFT}=min)','BigStation (n_{FFT}=5)', 'Millipede', 'Pilot RX', 'Theoretical pilot RX'},'Location','best','Orientation','vertical');
% l1.NumColumns = 2; 
% set(gca, 'YScale', 'log')
% ylim([0 7000]);

%xticks([0:8:32]);
%xlim([5,35]);
%set(gcf,'Position',[100 100 350 350])
yyaxis right;
h3 = plot(ant_num_range, num_cores, 'o--', 'Color', [0.8,0.8,0.8], 'LineWidth',2, 'MarkerSize',15);
set(gca,'YColor',[0.8,0.8,0.8]);
ylabel('Number of cores');
% ylim([13, 50]);
l = legend([h1,h2],'Pipeline parallel','Cheetah', 'location','west');
set(l,'FontSize',20);
% set(gcf,'Position',[100 100 350 350]);

%%
ue_num_range = [4, 8, 16];
num_cores = [15, 16, 30];
avg_proc_millipede = [];
std_proc_millipede = [];
avg_proc_bigstation = [];
std_proc_bigstation = [];

index = 0;
for ue_num = ue_num_range
    index = index+1;
    filename = sprintf('timeresult_millipede_64x%d_1ms.txt',ue_num); 
    [avg_proc_millipede(index), std_proc_millipede(index)] = parse_dl_file(filename);
end

index = 0;
for ue_num = ue_num_range(2:3)
    index = index+1;
    filename = sprintf('timeresult_bigstation_64x%d_1ms.txt',ue_num); 
    [avg_proc_bigstation(index), std_proc_bigstation(index)] = parse_dl_file(filename);
end


figure(7);
clf;
errorbar(ue_num_range(2:3),avg_proc_bigstation,std_proc_bigstation,'Color', colors(1,:), 'LineWidth',2);
hold on;
h1 = plot(ue_num_range(2:3),avg_proc_bigstation, '^', 'Color', colors(1,:), 'LineWidth',2, 'MarkerSize',20);
hold on;
errorbar(ue_num_range,avg_proc_millipede,std_proc_millipede,'Color', colors(2,:), 'LineWidth',2);
hold on;
h2 = plot(ue_num_range,avg_proc_millipede,'s', 'Color', colors(2,:), 'LineWidth',2,'MarkerSize',20);

set(gca,'FontSize',24);
% xlim([10,70]);
xlabel('Number of users');
ylabel('Time (us)');
xticks(ant_num_range);
grid on;
% title('Uplink frame Processing latency');
% grid on;
% l1 = legend([h_fft_big,h_fft_big_5core,h_fft_comp,h_pilot,h_theory],{'BigStation (n_{FFT}=min)','BigStation (n_{FFT}=5)', 'Millipede', 'Pilot RX', 'Theoretical pilot RX'},'Location','best','Orientation','vertical');
% l1.NumColumns = 2; 
% set(gca, 'YScale', 'log')
% ylim([0 7000]);

%xticks([0:8:32]);
%xlim([5,35]);
%set(gcf,'Position',[100 100 350 350])
yyaxis right;
h3 = plot(ue_num_range, num_cores, 'o--', 'Color', [0.8,0.8,0.8], 'LineWidth',2, 'MarkerSize',15);
set(gca,'YColor',[0.8,0.8,0.8]);
ylabel('Number of cores');
% ylim([13, 50]);
l = legend([h1,h2],'Pipeline parallel','Cheetah', 'location','west');
set(l,'FontSize',18);
set(gcf,'Position',[100 100 350 350]);






