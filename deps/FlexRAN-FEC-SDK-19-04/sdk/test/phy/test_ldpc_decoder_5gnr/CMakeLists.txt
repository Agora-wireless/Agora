#######################################################################
#
# <COPYRIGHT_TAG>
#
#######################################################################

# Options sepcific to unitttests
# Call add_compile_options' for EACH option
if(WIN32)
  # windows options
  add_compile_options("")
else()
  # linux options
  add_compile_options("")
endif()

# Define kernel being tested, eg lib_modulation
set(kernel
    lib_ldpc_decoder_5gnr
)

# Define all test files
set(test_files
    ldpc_decoder_functional.cc
    ldpc_decoder_performance.cc
)

# Call macro to create test binary
ADD_TEST_SUITE("${kernel}" "${test_files}" "unittests")


