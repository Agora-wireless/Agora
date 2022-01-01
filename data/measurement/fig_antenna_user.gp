set terminal pdfcairo
set output 'fig_antenna_user.pdf'
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
set xr [0:80]
set yr [0:170]
set xtics font ', 20'
set ytics font ', 20'
set xlabel '# cores' font 'Times-Roman,20'
set ylabel '# antennas' font 'Times-Roman,20'
set key bottom right font 'Times-Roman,20'
set grid xtics ytics mxtics mytics
set mxtics 1
set mytics 1
set label "maximum # antennas" at 25, 160 font 'Times-Roman,20'
plot 'fig_antenna_user.txt' using ($1/1.):($2/1.) title '16 users' with linespoint linestyle 1, \
	'fig_antenna_user.txt' using ($1/1.):($3>=32?$3/1.:1/0) title '32 users' with linespoint linestyle 2, \
	150 notitle dashtype 17 lw 4 lc rgb "red"