set terminal pdfcairo
set output 'fig_central_core.pdf'
set size ratio 0.4
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

set style line 501 lt 1 lc rgb "#000000" lw 2
set border 3 ls 501
# set xr [-0.2:3]
# set yr [-4:110]

red = "#FF0000"; green = "#00FF00"; blue = "#0000FF"; skyblue = "#87CEEB";
c1 = "#2B8CBE"; c2 = "#A6BDDB"; c3 = "#ECE7F2";
set yrange [0:140]
set style data histogram
set style histogram cluster gap 1
set style fill solid
set boxwidth 0.9
set xtics format ""
set grid ytics
set offset -1,0,0,0

set xtics font ', 20'
set ytics font ', 20'
# set xlabel 'Settings' font 'Times-Roman,20'
set ylabel 'Cores required' font 'Times-Roman,20'
set key top right font 'Times-Roman,18' 
# set key off
# set grid xtics ytics mxtics mytics
# set mxtics 1
# set mytics 1
set style fill solid border -1
plot 'fig_central_core.txt' using 2:xtic(1) title "Hydra" linecolor rgb c1 lw 1.5, '' using 0:2:2 with labels notitle offset -3,1 font 'Times-Roman,16', \
	'fig_central_core.txt' using ($3>=8?$3/1.:1/0) title "Central-coordination" linecolor rgb c2 lw 1.5, '' using 0:($3>=8?$3/1.:1/0):($3>=8?$3/1.:1/0) with labels notitle offset 0,1 font 'Times-Roman,16', \
	'fig_central_core.txt' using ($4>=8?$4/1.:1/0) title "Agora-placement" linecolor rgb c3 lw 1.5, '' using 0:($4>=8?$4/1.:1/0):($4>=8?$4/1.:1/0) with labels notitle offset 3,1 font 'Times-Roman,16'