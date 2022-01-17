#!/bin/bash
for irq_num in `ls /proc/irq | grep -v default`
do
	cpu_id=0
	echo "Setting IRQ $irq_num to CPU $cpu_id..."
	sudo sh -c "echo $cpu_id > /proc/irq/$irq_num/smp_affinity_list"
	echo "IRQ $irq_num is not attached to CPU"
	cat /proc/irq/$irq_num/smp_affinity_list
	#cat /proc/irq/$irq_num/smp_affinity
done
