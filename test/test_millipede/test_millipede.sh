echo "generate data for correctness test......"
./data_generator data/tddconfig-correctness-test-ul.json
sleep 2
echo -e "-------------------------------------------------------\n\n\n"
echo "==================================="
echo "run uplink correctness test......"
echo -e "===================================\n"
./millipede data/tddconfig-correctness-test-ul.json &
./sender 4 10 5000 data/tddconfig-correctness-test-ul.json

sleep 2
echo -e "-------------------------------------------------------\n\n\n"
echo "==================================="
echo "run downlink correctness test......"
echo -e "===================================\n"
./millipede data/tddconfig-correctness-test-dl.json &
./sender 4 10 5000 data/tddconfig-correctness-test-dl.json