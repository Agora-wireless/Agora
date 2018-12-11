for irq_num in {147..210}
do
	cpu_id=$(($irq_num%7+2))
	echo $cpu_id $irq_num
	sudo sh -c "echo $cpu_id > /proc/irq/$irq_num/smp_affinity_list"
	cat /proc/irq/$irq_num/smp_affinity_list
	cat /proc/irq/$irq_num/smp_affinity
done
