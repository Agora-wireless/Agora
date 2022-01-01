set terminal pdfcairo
set output 'fig_core_time.pdf'
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
# set xr [0:]
set yr [0:]
set style line 501 lt 1 lc rgb "#000000" lw 2
set border 3 ls 501
set xtics font ', 20'
set ytics 0,25,125 font ', 20'
set xlabel 'Time (second)' font 'Times-Roman,20'
set ylabel 'CPU usage (%)' font 'Times-Roman,20'
# set key bottom right font 'Times-Roman,20'
set key off
set grid xtics ytics mxtics mytics
set mxtics 1
set mytics 1
plot 'fig_core_time.txt' using ($1/1.):($2/1.)  with lines linestyle 1 lw 3