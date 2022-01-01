set terminal pdfcairo
set output 'fig_fft_latency.pdf'
set style line 1 \
	linecolor rgb '#a00000' \
	linetype 1 linewidth 2
set style line 2 \
	linecolor rgb '#00a000' \
	linetype 2 linewidth 2
set style line 3 \
	linecolor rgb '#0000a0' \
	linetype 3 linewidth 2
set style line 4 \
	linecolor rgb '#a0a000' \
	linetype 4 linewidth 2
set style line 5 \
	linecolor rgb '#a000a0' \
	linetype 5 linewidth 2
set style line 6 \
	linecolor rgb '#00a0a0' \
	linetype 6 linewidth 2
# set xr [-0.2:3]
# set yr [-4:110]

set style line 501 lt 1 lc rgb "#000000" lw 2
set border 3 ls 501

red = "#FF0000"; green = "#00FF00"; blue = "#0000FF"; skyblue = "#87CEEB";
c1 = "#0570b0"; c2 = "#74a9cf"; c3 = "#bdc9e1"; c4 = "#f1eef6";
set yrange [1:2.2]
set style data histogram
set style histogram rowstacked
# set style histogram cluster gap 1
set style fill solid
set boxwidth 0.9
set xtics format ""
set grid ytics

set xtics font ', 20'
set ytics font ', 20'
# set xlabel 'Settings' font 'Times-Roman,20'
set ylabel '99.99-th latency (ms)' font 'Times-Roman,20'
set key top right font 'Times-Roman,20'
# set key off
# set grid xtics ytics mxtics mytics
# set mxtics 1
# set mytics 1
set style fill solid border -1
plot 'fig_fft_latency.txt' using ($4+1):xtic(1) title "FFT" linecolor rgb c2 lw 1.5, \
    'fig_fft_latency.txt' using ($5) title "Subcarrier-parallel" linecolor rgb c3 lw 1.5, \
    'fig_fft_latency.txt' using ($6) title "User-parallel" linecolor rgb c4 lw 1.5
	# plot 'fig_fft_latency.txt' using ($3+1):xtic(1) title "IQ receiving" linecolor rgb c1 lw 1.5, \