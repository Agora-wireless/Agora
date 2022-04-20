set terminal pdfcairo size 5, 3
# set size 1, 1
set lmargin 10
set output 'fig_cdf_latency.pdf'
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
set xr [0: 2.5]
set yr [-1:1.1]
set xtics font ', 20'
set ytics font ', 20'
set xlabel 'Latency (ms)' font 'Times-Roman,20'
set ylabel 'Complementary CDF' font 'Times-Roman,20' offset -3, 0, 0
set key top left font 'Times-Roman,20'
set grid xtics ytics mxtics mytics
set mxtics 1
set mytics 1
set logscale y
set format y "10^{%T}"
set yrange [ 0.00001 : 1.0 ]
set ytics nolog
set style line 501 lt 1 lc rgb "#000000" lw 2
set border 3 ls 501
set label "99.99th latency" at 0.1, 0.00005 font 'Times-Roman,20'
# plot 'fig_cdf_latency.txt' using ($1/1.):($2/1.) title '64×8' with lines linestyle 1 dt 1, \
# 	'fig_cdf_latency.txt' using ($1/1.):($4/1.) title '64×16' with lines linestyle 2 dt 2, \
#     'fig_cdf_latency.txt' using ($1/1.):($6/1.) title '64×32' with lines linestyle 3 dt 3 lw 7, \
# 	'fig_cdf_latency.txt' using ($1/1.):($8/1.) title '128×16' with lines linestyle 4 dt 4, \
# 	'fig_cdf_latency.txt' using ($1/1.):($10/1.) title '128×32' with lines linestyle 5 dt 5, \
# 	'fig_cdf_latency.txt' using ($1/1.):($12/1.) title '150×32' with lines linestyle 6 dashtype '- ._. -', \
# 	0.0001 notitle dashtype 17 lw 5
plot 'fig_cdf_latency.txt' using ($1/1.):($12/1.) title 'Uplink' with lines linestyle 1 dt 1, \
	'fig_cdf_latency.txt' using ($1/1.):($14/1.) title 'Downlink' with lines linestyle 3 dt 3 lw 7, \
	0.0001 notitle dashtype 17 lw 5 linecolor "red"