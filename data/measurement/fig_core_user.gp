set terminal pdfcairo
set output 'fig_core_user.pdf'
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
set xr [0:48]
set yr [0:]
set xtics 0,8,56 font ', 20'
set ytics font ', 20'
set xlabel '# users' font 'Times-Roman,20'
set ylabel '# cores' font 'Times-Roman,20'
set key bottom right font 'Times-Roman,20'
set grid xtics ytics mxtics mytics
set mxtics 1
set mytics 1
set style line 501 lt 1 lc rgb "#000000" lw 2
set border 3 ls 501
# set label "maximum # antennas" at 25, 160 font 'Times-Roman,20'
plot 'fig_core_user.txt' using ($1/1.):($2/1.) title '64 antennas' with linespoint linestyle 1 lw 3 pt 7 lc rgb "#e41a1c", \
	'fig_core_user.txt' using ($1/1.):($3>=8?$3/1.:1/0) title '128 antennas' with linespoint linestyle 2 lw 3 pt 5 lc rgb "#377eb8", \
	'fig_core_user.txt' using ($1/1.):($4>=8?$4/1.:1/0) title '240 antennas' with linespoint linestyle 3 lw 3 pt 3 lc rgb "#008000"
	# 150 notitle dashtype 17 lw 4 lc rgb "red"