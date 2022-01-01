set terminal pdfcairo
set output 'fig_9999_latency_slot_size.pdf'
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
set xr [0:2.5]
set yr [0:2.5]
set xtics font ', 20'
set ytics font ', 20'
set xlabel 'Slot size (ms)' font 'Times-Roman,20'
set ylabel '99.99-th latency' font 'Times-Roman,20'
set key bottom right font 'Times-Roman,20'
set grid xtics ytics mxtics mytics
set mxtics 1
set mytics 1
plot 'fig_9999_latency_slot_size.txt' using ($1/1.):($2/1.) title '64*16' with linespoints linestyle 1, \
    'fig_9999_latency_slot_size.txt' using ($1/1.):($3/1.) title '64*32' with linespoints linestyle 2, \
	'fig_9999_latency_slot_size.txt' using ($1/1.):($4/1.) title '128*16' with linespoints linestyle 3, \
	'fig_9999_latency_slot_size.txt' using ($1/1.):($5/1.) title '128*32' with linespoints linestyle 4