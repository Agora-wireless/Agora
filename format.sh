FORMAT_EXE=clang-format-11
$FORMAT_EXE -i src/agora/txrx/*.cc
$FORMAT_EXE -i src/agora/txrx/*.h

$FORMAT_EXE -i src/agora/txrx/workers/*.cc
$FORMAT_EXE -i src/agora/txrx/workers/*.h

$FORMAT_EXE -i src/radio/*.cc
$FORMAT_EXE -i src/radio/*.h

$FORMAT_EXE -i src/agora/*.cc
$FORMAT_EXE -i src/agora/*.h

$FORMAT_EXE -i src/recorder/*.cc
$FORMAT_EXE -i src/recorder/*.h

$FORMAT_EXE -i src/client/*.cc
$FORMAT_EXE -i src/client/*.h

$FORMAT_EXE -i src/client/txrx/*.cc
$FORMAT_EXE -i src/client/txrx/*.h

$FORMAT_EXE -i src/client/txrx/workers/*.cc
$FORMAT_EXE -i src/client/txrx/workers/*.h

$FORMAT_EXE -i src/common/*.cc
$FORMAT_EXE -i src/common/*.h

$FORMAT_EXE -i src/common/ipc/*.cc
$FORMAT_EXE -i src/common/ipc/*.h

$FORMAT_EXE -i src/common/loggers/*.cc
$FORMAT_EXE -i src/common/loggers/*.h

$FORMAT_EXE -i src/encoder/*.cc
$FORMAT_EXE -i src/encoder/*.h

$FORMAT_EXE -i src/mac/*.cc
$FORMAT_EXE -i src/mac/*.h

$FORMAT_EXE -i src/mac/schedulers/*.cc
$FORMAT_EXE -i src/mac/schedulers/*.h

$FORMAT_EXE -i src/resource_provisioner/*.cc
$FORMAT_EXE -i src/resource_provisioner/*.h

$FORMAT_EXE -i simulator/*.cc
$FORMAT_EXE -i simulator/*.h
$FORMAT_EXE -i simulator/channel_models/*.cc
$FORMAT_EXE -i simulator/channel_models/*.h

$FORMAT_EXE -i src/data_generator/*.cc
$FORMAT_EXE -i src/data_generator/*.h

$FORMAT_EXE -i test/test_agora/*.cc
$FORMAT_EXE -i test/unit_tests/*.cc
$FORMAT_EXE -i test/compute_kernels/*.cc
$FORMAT_EXE -i test/compute_kernels/*.h
$FORMAT_EXE -i test/compute_kernels/ldpc/*.cc
