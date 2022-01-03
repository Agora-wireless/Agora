/**********************************************************************
*
* <COPYRIGHT_TAG>
*
**********************************************************************/

#include "common.hpp"

#include "phy_ldpc_decoder_5gnr.h"

const std::string module_name = "ldpc_decoder_5gnr";

class LDPCDecoder5GNRCheck : public KernelTests {
protected:
    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request{};
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response{};
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_reference{};
    int numBlocksToCheck;
    void SetUp() override {
        init_test("functional");

        const long int buffer_len = 1024 * 1024;
        ldpc_decoder_5gnr_request.Zc = get_input_parameter<uint16_t>("Zc");
        ldpc_decoder_5gnr_request.baseGraph = get_input_parameter<int32_t>("baseGraph");
        ldpc_decoder_5gnr_request.nRows = get_input_parameter<int32_t>("nRows");
        ldpc_decoder_5gnr_request.numChannelLlrs = get_input_parameter<int16_t>("numChannelLlrs");
        ldpc_decoder_5gnr_request.numFillerBits = get_input_parameter<int16_t>("numFillerBits");
        ldpc_decoder_5gnr_request.maxIterations = get_input_parameter<int16_t>("maxIterations");
        ldpc_decoder_5gnr_request.enableEarlyTermination = get_input_parameter<bool>("enableEarlyTermination");
        int numMsgBits = ldpc_decoder_5gnr_request.Zc * (
                        ldpc_decoder_5gnr_request.baseGraph == 1 ? 22 : 10) -
                        ldpc_decoder_5gnr_request.numFillerBits;
        int numMsgBytes = (numMsgBits + 7) / 8;

        ldpc_decoder_5gnr_request.varNodes = get_input_parameter<int8_t*>("input");
        ldpc_decoder_5gnr_response.varNodes = aligned_malloc<int16_t>(buffer_len, 64);
        ldpc_decoder_5gnr_response.compactedMessageBytes = aligned_malloc<uint8_t>(numMsgBytes, 64);
        memset(ldpc_decoder_5gnr_response.varNodes, 0, numMsgBytes);
        memset(ldpc_decoder_5gnr_response.compactedMessageBytes, 0, numMsgBytes);
        ldpc_decoder_5gnr_reference.compactedMessageBytes = get_reference_parameter<uint8_t*>("output");
    }

    void TearDown() override {
        aligned_free(ldpc_decoder_5gnr_request.varNodes);
        aligned_free(ldpc_decoder_5gnr_response.varNodes);
        aligned_free(ldpc_decoder_5gnr_response.compactedMessageBytes);
        aligned_free(ldpc_decoder_5gnr_reference.compactedMessageBytes);
    }

    template <typename F, typename ... Args>
    void functional(F function, const std::string isa, Args ... args)
    {
        function(args ...);
        int numMsgBits = ldpc_decoder_5gnr_request.Zc * (
                        ldpc_decoder_5gnr_request.baseGraph == 1 ? 22 : 10) -
                        ldpc_decoder_5gnr_request.numFillerBits;
        /*printf("Iteration passing %d %d\n",
                ldpc_decoder_5gnr_response.parityPassedAtTermination,
                ldpc_decoder_5gnr_response.iterationAtTermination);*/
        ASSERT_ARRAY_EQ(ldpc_decoder_5gnr_reference.compactedMessageBytes,
                        ldpc_decoder_5gnr_response.compactedMessageBytes,
                        (numMsgBits + 7) / 8);
        ASSERT_EQ(ldpc_decoder_5gnr_response.numMsgBits, numMsgBits);
        ASSERT_TRUE(ldpc_decoder_5gnr_response.parityPassedAtTermination);
        print_test_description(isa, module_name);
    }
};

#ifdef _BBLIB_AVX512_
TEST_P(LDPCDecoder5GNRCheck, AVX512_Check)
{
    functional(bblib_ldpc_decoder_5gnr_avx512, "AVX512", &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
}
#endif

#ifdef _BBLIB_AVX2_
TEST_P(LDPCDecoder5GNRCheck, AVX2_Check)
{
    functional(bblib_ldpc_decoder_5gnr_avx2, "AVX2", &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
}
#endif

TEST_P(LDPCDecoder5GNRCheck, Default_Check)
{
    functional(bblib_ldpc_decoder_5gnr, "Default", &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
}

INSTANTIATE_TEST_CASE_P(UnitTest, LDPCDecoder5GNRCheck,
                        testing::ValuesIn(get_sequence(LDPCDecoder5GNRCheck::get_number_of_cases("functional"))));
