set terminal pdfcairo
set output 'fig_9999_latency_core.pdf'
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
set xr [0:100]
set yr [:20]
set xtics font ', 20'
set ytics font ', 20'
set xlabel 'Number of CPU cores' font 'Times-Roman,20'
set ylabel '99.99-th latency (ms)' font 'Times-Roman,20'
set key top right font 'Times-Roman,20'
set grid xtics ytics mxtics mytics
set logscale y
set ytics 1,2,16
set style line 501 lt 1 lc rgb "#000000" lw 2
set border 3 ls 501
# set mxtics 1
# set mytics 1
# plot 'fig_9999_latency_core.txt' using ($1/1.):($2/1.) title '64×8' with lines linestyle 1 lw 3, \
# 	'fig_9999_latency_core.txt' using ($1/1.):($4/1.) title '64×16' with lines linestyle 2 lw 3, \
#     'fig_9999_latency_core.txt' using ($1/1.):($6/1.) title '64×32' with lines linestyle 3 lw 3, \
# 	'fig_9999_latency_core.txt' using ($1/1.):($8/1.) title '128×16' with lines linestyle 4 lw 3, \
# 	'fig_9999_latency_core.txt' using ($1/1.):($10/1.) title '128×32' with lines linestyle 5 lw 3, \
# 	'fig_9999_latency_core.txt' using ($1/1.):($12/1.) title '150×32' with lines linestyle 6 lw 3, \
# 	'fig_9999_latency_core.txt' using ($1/1.):($3==1?$2/1.:1/0) title "Supported" pt 7 ps 0.5 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($5==1?$4/1.:1/0) notitle pt 7 ps 0.5 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($7==1?$6/1.:1/0) notitle pt 7 ps 0.5 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($9==1?$8/1.:1/0) notitle pt 7 ps 0.5 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($11==1?$10/1.:1/0) notitle pt 7 ps 0.5 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($13==1?$12/1.:1/0) notitle pt 7 ps 0.5 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($3==0?$2/1.:1/0) title "Not supported" pt 2 ps 1 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($5==0?$4/1.:1/0) notitle pt 2 ps 1 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($7==0?$6/1.:1/0) notitle pt 2 ps 1 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($9==0?$8/1.:1/0) notitle pt 2 ps 1 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($11==0?$10/1.:1/0) notitle pt 2 ps 1 lc rgb "red", \
# 	'fig_9999_latency_core.txt' using ($1/1.):($13==0?$12/1.:1/0) notitle pt 2 ps 1 lc rgb "red"
plot 'fig_9999_latency_core.txt' using ($1/1.):($4/1.) title '64×16' with lines linestyle 2 lw 3 lc rgb "#e41a1c" dt 1, \
	'fig_9999_latency_core.txt' using ($1/1.):($8/1.) title '128×16' with lines linestyle 4 lw 5 lc rgb "#377eb8" dt 2, \
	'fig_9999_latency_core.txt' using ($1/1.):($12/1.) title '150×32' with lines linestyle 6 lw 5 lc rgb "#4daf4a" dt 4, \
	'fig_9999_latency_core.txt' using ($1/1.):($5==1?$4/1.:1/0) title "Supported" pt 7 ps 1 lc rgb "red", \
	'fig_9999_latency_core.txt' using ($1/1.):($9==1?$8/1.:1/0) notitle pt 7 ps 1 lc rgb "red", \
	'fig_9999_latency_core.txt' using ($1/1.):($13==1?$12/1.:1/0) notitle pt 7 ps 1 lc rgb "red", \
	'fig_9999_latency_core.txt' using ($1/1.):($5==0?$4/1.:1/0) title "Not supported" pt 2 ps 2 lc rgb "red", \
	'fig_9999_latency_core.txt' using ($1/1.):($9==0?$8/1.:1/0) notitle pt 2 ps 2 lc rgb "red", \
	'fig_9999_latency_core.txt' using ($1/1.):($13==0?$12/1.:1/0) notitle pt 2 ps 2 lc rgb "red"