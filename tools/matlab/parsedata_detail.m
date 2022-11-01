fid = fopen('../files/experiment/timeresult_detail.txt');
% fid = fopen('result/timeresult_bigstation_newsender_smp.txt');
% fid = fopen('result/timeresult_bigstation3.txt');
%  fid = fopen('result_newsender/timeresult_25core.txt');
% fid = fopen('result/timeresult_25core_cmp_bigstation_newsender_smp.txt');
% fid = fopen('result_bench_overhead/timeresult_30core_30task_2RX.txt');
% fid = fopen('result_compare/timeresult_23core_bigstation.txt');
% fid = fopen('result_scale_ant/timeresult_ant16_21core.txt');
temp = textscan(fid,"%f%f%f%f%f%f%f%f%f%f%f%f",'HeaderLines',2);
M = 8;
n_core = 30;
n_fft_task = M * 70;
n_zf_task = 1200;
n_demul_task = 1200 * 62;
fft_1 = temp{1};
fft_2 = temp{2};
fft_3 = temp{3};
zf_1 = temp{4};
zf_2 = temp{5};
zf_3 = temp{6};
demul_1 = temp{7};
demul_2 = temp{8};
demul_3 = temp{9};
decode_1 = temp{10};
decode_2 = temp{11};
decode_3 = temp{12};


frame_count = length(fft_1);
frame_count = frame_count - 1;
index = [1:frame_count];
% index(1:512:frame_count) = [];
subset = index(1200:end);

avg_fft_1 = mean(fft_1(subset));
avg_fft_2 = mean(fft_2(subset));
avg_fft_3 = mean(fft_3(subset));
avg_fft = avg_fft_1 + avg_fft_2 + avg_fft_3;


avg_zf_1 = mean(zf_1(subset));
avg_zf_2 = mean(zf_2(subset));
avg_zf_3 = mean(zf_3(subset));
avg_zf = avg_zf_1 + avg_zf_2+avg_zf_3;

avg_demul_1 = mean(demul_1(subset));
avg_demul_2 = mean(demul_2(subset));
avg_demul_3 = mean(demul_3(subset));
avg_demul = avg_demul_1 + avg_demul_2 + avg_demul_3;

avg_decode_1 = mean(decode_1(subset));
avg_decode_2 = mean(decode_2(subset));
avg_decode_3 = mean(decode_3(subset));
avg_decode = avg_decode_1 + avg_decode_2 + avg_decode_3;


fprintf("FFT: %.2f(%.2f,%.2f,%.2f), ZF: %.2f(%.2f, %.2f,%.2f), Demul: %.2f(%.2f,%.2f,%.2f), Decode: %.2f(%.2f,%.2f,%.2f)\n", ...
    avg_fft, avg_fft_1, avg_fft_2, avg_fft_3, avg_zf, avg_zf_1, avg_zf_2, avg_zf_3, avg_demul, avg_demul_1, avg_demul_2, avg_demul_3, avg_decode, avg_decode_1, avg_decode_2 ,avg_decode_3);


fprintf("FFT: %.2f(%.2f,%.2f), ZF: %.2f(%.2f, %.2f), Demul: %.2f(%.2f,%.2f)\n", ...
    avg_fft, avg_fft_1 + avg_fft_3, avg_fft_2, avg_zf, avg_zf_1, avg_zf_2, avg_demul, avg_demul_1+ avg_demul_3, avg_demul_2);
fprintf("Per task: FFT: %.2f(%.3f,%.3f,%.3f), ZF: %.3f(%.3f,%.3f,%.3f), Demul: %.2f(%.3f,%.3f,%.3f)\n", ...
    [avg_fft, avg_fft_1, avg_fft_2, avg_fft_3]/n_fft_task*n_core, [avg_zf, avg_zf_1, avg_zf_2, avg_zf_3]/n_zf_task*n_core, [avg_demul, avg_demul_1, avg_demul_2, avg_demul_3]/n_demul_task*n_core);


avg_fft_compute = mean(fft_2(subset))/896*26;
std_fft_compute = std(fft_2(subset))/896*26;

avg_zf_compute = mean(zf_2(subset))/75*26;
std_zf_compute = std(zf_2(subset))/75*26;

avg_demul_compute = mean(demul_2(subset)+demul_3(subset))/15600*26;
std_demul_compute = std(demul_2(subset)+demul_3(subset))/15600*26;

avg_decode_compute = mean(decode_2(subset))/208*26;
std_decode_compute = std(decode_2(subset))/208*26;

fprintf("Time per task: fft: %.1f+%.2f, zf: %.1f+%.2f, demul:%.2f+%.3f, decode:%.1f+%.2f\n",...
    avg_fft_compute, std_fft_compute, avg_zf_compute, std_zf_compute, avg_demul_compute, std_demul_compute, avg_decode_compute, std_decode_compute);

avg_fft_compute = mean(fft_2(subset))*26;
std_fft_compute = std(fft_2(subset))*26;

avg_zf_compute = mean(zf_2(subset))*26;
std_zf_compute = std(zf_2(subset))*26;

avg_demul_compute = mean(demul_2(subset)+demul_3(subset))*26;
std_demul_compute = std(demul_2(subset)+demul_3(subset))*26;

avg_decode_compute = mean(decode_2(subset))*26;
std_decode_compute = std(decode_2(subset))*26;

fprintf("Time all cores: fft: %.1f+%.2f, zf: %.1f+%.2f, demul:%.2f+%.3f, decode:%.1f+%.2f\n",...
    avg_fft_compute, std_fft_compute, avg_zf_compute, std_zf_compute, avg_demul_compute, std_demul_compute, avg_decode_compute, std_decode_compute);


avg_fft_data = mean(fft_1(subset)+fft_3(subset))/1000;
avg_zf_data = mean(zf_1(subset))/1000;
avg_demul_data = mean(demul_1(subset))/1000;
avg_decode_data = mean(decode_1(subset))/1000;

fprintf("Data Time all cores: fft: %.4f, zf: %.4f, demul:%.4f, decode:%.4f\n",...
    avg_fft_data, avg_zf_data,avg_demul_data, avg_decode_data);