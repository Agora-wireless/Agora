./millipede data/tddconfig-correctness-test-ul.json &
./sender 4 10 5000 data/tddconfig-correctness-test-ul.json

sleep 2

./millipede data/tddconfig-correctness-test-dl.json &
./sender 4 10 5000 data/tddconfig-correctness-test-dl.json