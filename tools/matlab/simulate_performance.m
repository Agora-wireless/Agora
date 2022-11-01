n = 25;
M = 8;
s = 70;
T = 5;
sp = 8;
if M == 8
    T_zf = 29.661;
    T_fft = 2.904;
    T_csi = 0.39;
    T_demod = 23.162;
elseif M == 16
    T_zf = 41.425;
    T_fft = 5.685;
    T_csi = 0.767;
    T_demod = 25.507;
    
elseif M == 24
    T_zf = 43;%42.271;
    T_fft = 8.511;
    T_csi = 1.191;
    T_demod = 31.286;
elseif M == 32
    T_zf = 44;%42.267;
    T_fft = 11;%11.309;
    T_csi = 1.54;%1.564;
    T_demod = 36;%35.734;
end

%% BigStation
x1 = 9:(n-1-5);
x2 = 9:(n-2-5);
x3 = 9:(n-3-5);
y1 = sp/s*T + T_zf./x1+T_demod./(n-1-x1);
y1_1 = T+1/s*T_demod./(n-1-x1);
y2 = sp/s*T + T_zf./x2+T_demod./(n-2-x2);
y2_1 = T+1/s*T_demod./(n-2-x2);
y3 = sp/s*T + T_zf./x3+T_demod./(n-3-x3);
y3_1 = T+1/s*T_demod./(n-3-x3);
figure(6);clf;
plot(x1,max(y1,y1_1),'LineWidth',2);
hold on;
plot(x2,max(y2,y2_1),'LineWidth',2);
hold on;
plot(x3,max(y3,y3_1),'LineWidth',2);
set(gca,'FontSize',18);
xlabel('Number of ZF cores');
ylabel('Frame processing time (ms)');
legend({'n_{fft}=1','n_{fft}=2','n_{fft}=3'});
title(sprintf('Total number of cores: %d',n));
grid on;
% ylim([5,10]);

min_y = [];
min_x = [];
core_usage_fft = [];
core_usage_zf = [];
core_usage_demod = [];
core_usage_bigstation = [];
index = 0;
n_min = (ceil(T_zf/T)+ceil(T_fft/T)+ceil(T_demod/T));
n_demul_min = ceil(T_demod/T);
n_zf_min = ceil(T_zf/T);
fprintf("min number of cores: %d\n", n_min);
n_bigstation = n_min:30;
n_fft = ceil(T_fft/T);
disp(n_fft);
for n=n_bigstation
   index = index+1;
   x = n_zf_min:(n-n_fft-n_demul_min); 
%    disp(x)
   y1 = sp/s*T+T_zf./x+T_demod./(n-n_fft-x);%+T_fft/n_fft;
%    disp(y1)
   y1_1 = T+1/s*T_demod./(n-n_fft-x);
   y = y1;%max(y1,y1_1);
   [min_val,min_loc] = min(y);
   min_y(index)=min_val;
   min_x(index)=x(min_loc);
   core_usage_fft(index) = T_fft/n_fft/T;
   core_usage_zf(index) = (T_zf/min_x(index))/T;
   core_usage_demod(index) = (T_demod/(n-n_fft-min_x(index)))/T;
   core_usage_bigstation(index) = (core_usage_fft(index)*n_fft+min_x(index)*core_usage_zf(index)+(n-1-min_x(index))*core_usage_demod(index))/n;
end


figure(7);clf;
plot(n_bigstation,min_y,'LineWidth',2);
set(gca,'FontSize',18);
xlabel('Total number of cores');
ylabel('Frame processing time (ms)');
title('Minimum frame processing time');
grid on;
% ylim([5,11]);

figure(38);clf;
plot(n_bigstation,min_x,'LineWidth',2);
hold on;
plot(n_bigstation,n_bigstation-n_fft-min_x,'LineWidth',2);
hold on;
plot(n_bigstation,n_fft * ones(length(n_bigstation),1),'LineWidth',2);
set(gca,'FontSize',18);
xlabel('Total number of cores');
ylabel('Number of cores');
legend({'n_{zf}','n_{demod}','n_{fft}'});
title('Best core assignment policy');
grid on;

disp(n_bigstation);
disp(n_bigstation-n_fft-min_x);
disp(min_x);

%% CoMP
n = 11:34;
% y1 = sp/s*T+((s-sp)/s*T_fft+T_zf+T_demod)./n;
y1 = sp/s*T+(T_fft+T_zf+T_demod)./n;
y1_1 = T*(s-1)/s + 0.04 + 1/s*T_demod./n;
y = max(y1,y1_1);
core_usage_comp = (T_fft+T_csi+T_zf+T_demod)./n/T;

figure(9);clf;
plot(n,y-T,'LineWidth',2);
set(gca,'FontSize',18);
xlabel('Total number of cores');
ylabel('Processing delay (ms)');
title('Frame Processing delay');
grid on;
% xlim([14,34]);

figure(19);clf;
plot(n,y,'LineWidth',2);
set(gca,'FontSize',18);
xlabel('Total number of cores');
ylabel('Processing time (ms)');
title('Frame Processing time');
grid on;
% xlim([14,34]);



figure(10);clf;
plot(n_bigstation,min_y,'LineWidth',2);
hold on;
plot(n,y,'LineWidth',2)
set(gca,'FontSize',18);
xlabel('Total number of cores');
ylabel('Frame processing time (ms)');
legend({'BigStation','CoMP'});
title('Frame processing time comparison');
grid on;
% xlim([14,34]);

figure(11);clf;
plot(n_bigstation,min_y-T,'LineWidth',2);
hold on;
plot(n,y-T,'LineWidth',2)
set(gca,'FontSize',18);
xlabel('Total number of cores');
ylabel('Latency (ms)');
legend({'BigStation','AGORA'});
title('Frame processing latency comparison');
grid on;
% xlim([14,34]);

figure(12);clf;
plot(n_bigstation,core_usage_fft*100,'LineWidth',2,'LineStyle','--');
hold on;
plot(n_bigstation,core_usage_zf*100,'LineWidth',2,'LineStyle','--');
hold on;
plot(n_bigstation,core_usage_demod*100,'LineWidth',2,'LineStyle','--');
hold on;
plot(n_bigstation,core_usage_bigstation*100,'LineWidth',2);
hold on;
plot(n,core_usage_comp*100,'LineWidth',2);
set(gca,'FontSize',18);
xlabel('Total number of cores');
ylabel('Core utilization (%)');
legend({'BigStation: FFT cores','BigStation: ZF cores','BigStation: Demodulation cores','Bigstation average','CoMP cores'});
title('Core utilization');
grid on;
% xlim([14,34]);

%%
n=13:30;
figure(29);clf;
plot(n,min_y(1:length(n))-T,'LineWidth',2);
hold on;
plot(n,(demul_done_mean-rx_done_mean)/1000,'LineWidth',2)
set(gca,'FontSize',18);
xlabel('Total number of cores');
ylabel('Processing delay (ms)');
title('Frame Processing delay');
grid on;
legend({'Simulation','Measurement'})
% xlim([14,29]);

figure(39);clf;
plot(n,min_y(1:length(n)),'LineWidth',2);
hold on;
plot(n,demul_done_mean/1000,'LineWidth',2);
hold on;
plot(n,ones(length(n),1)*T,'LineWidth',2,'LineStyle','--');
hold on;
plot(n,rx_done_mean/1000,'LineWidth',2,'LineStyle','--');
set(gca,'FontSize',18);
xlabel('Total number of cores');
ylabel('Processing time (ms)');
title('Frame Processing time');
legend({'Simulation: data processing time','Measurement: data processing time','Simulation: frame duration', 'Meausurement: frame duration'});
grid on;
xlim([14,29]);

%%
n=25;
x1 = 9:(n-1-5);
x2 = 9:(n-2-5);
x3 = 9:(n-3-5);
y1 = sp/s*T+T_zf./x1+T_demod./(n-1-x1);
y1_1 = T+1/s*T_demod./(n-1-x1);
n=9:19;
figure(29);clf;
plot(x1,max(y1,y1_1)-T,'LineWidth',2);
hold on;
plot(n,(demul_done_mean-rx_done_mean)/1000,'LineWidth',2)
set(gca,'FontSize',18);
xlabel('Number of ZF cores');
ylabel('Processing delay (ms)');
title('Frame Processing delay');
grid on;
legend({'Simulation','Measurement'})
% xlim([14,29]);
