set terminal pdfcairo
set output 'fig_bandwidth_setting.pdf'
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

red = "#FF0000"; green = "#00FF00"; blue = "#0000FF"; skyblue = "#87CEEB";
set yrange [0:150]
set style data histogram
set style histogram cluster gap 1
set style fill solid
set boxwidth 0.9
set xtics format ""
set grid ytics

set xtics font ', 20'
set ytics font ', 20'
set xlabel 'Setting' font 'Times-Roman,20'
set ylabel 'Bandwidth (Gbps)' font 'Times-Roman,20'
set key top right font 'Times-Roman,20'
# set key off
# set grid xtics ytics mxtics mytics
# set mxtics 1
# set mytics 1
plot 'fig_bandwidth_setting.txt' using 2:xtic(1) title "Hydra" linecolor rgb red, \
	'fig_bandwidth_setting.txt' using 3 title "Include FFT" linecolor rgb green, \
	'fig_bandwidth_setting.txt' using 4 title "Agora-placement" linecolor rgb blue, \
	'fig_bandwidth_setting.txt' using 5 title "Central coordination" linecolor rgb skyblue