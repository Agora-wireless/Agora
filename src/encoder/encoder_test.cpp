#include "cyclic_shift.hpp"
#include "encoder.hpp"
#include "iobuffer.hpp"
#include <fstream>

std::string input_filename = "test_vectors/input_BG1_Zc192.bin";
std::string reference_filename = "test_vectors/output_BG1_Zc192.bin";

char* read_binfile(std::string filename, int buffer_size)
{
    std::ifstream infile;
    infile.open(filename, std::ios::binary | std::ios::in);

    char* x = (char*)malloc(buffer_size * sizeof(char));

    infile.read((char*)x, buffer_size * sizeof(char));
    infile.close();

    return x;
}

int main()
{
    // __declspec (align(PROC_BYTES)) int8_t internalBuffer0[BG1_ROW_TOTAL * PROC_BYTES] = {0};
    // __declspec (align(PROC_BYTES)) int8_t internalBuffer1[BG1_ROW_TOTAL * PROC_BYTES] = {0};
    // __declspec (align(PROC_BYTES)) int8_t internalBuffer2[BG1_COL_TOTAL * PROC_BYTES] = {0};
    ALIGNED_(PROC_BYTES)
    int8_t internalBuffer0[BG1_ROW_TOTAL * PROC_BYTES] = { 0 };
    ALIGNED_(PROC_BYTES)
    int8_t internalBuffer1[BG1_ROW_TOTAL * PROC_BYTES] = { 0 };
    ALIGNED_(PROC_BYTES)
    int8_t internalBuffer2[BG1_COL_TOTAL * PROC_BYTES] = { 0 };

    // input -----------------------------------------------------------
    // these values depend on the application
    uint16_t Zc = 192;
    int numberCodeblocks = 1;
    uint16_t Bg = 1;

    int nRows = (Bg == 1) ? 46 : 42;
    uint32_t cbEncLen = nRows * Zc;
    uint32_t cbLen = (Bg == 1) ? Zc * 22 : Zc * 10;
    uint32_t cbCodewLen = (Bg == 1) ? Zc * 66 : Zc * 50;
    const int16_t* pShiftMatrix;
    const int16_t* pMatrixNumPerCol;
    const int16_t* pAddr;

    int8_t* input[numberCodeblocks];
    int8_t* parity[numberCodeblocks];
    int8_t* output[numberCodeblocks];
    int8_t* reference[numberCodeblocks];

    for (int i = 0; i < numberCodeblocks; i++) {
        input[i] = (int8_t*)read_binfile(input_filename, (cbLen + 7) >> 3);
        parity[i] = (int8_t*)malloc(BG1_ROW_TOTAL * PROC_BYTES * sizeof(int8_t));
        output[i] = (int8_t*)malloc(BG1_COL_TOTAL * PROC_BYTES * sizeof(int8_t));
        reference[i] = (int8_t*)read_binfile(reference_filename, (cbEncLen + 7) >> 3);
    }

    // i_Ls decides the base matrix entries
    uint8_t i_LS;
    if ((Zc % 15) == 0)
        i_LS = 7;
    else if ((Zc % 13) == 0)
        i_LS = 6;
    else if ((Zc % 11) == 0)
        i_LS = 5;
    else if ((Zc % 9) == 0)
        i_LS = 4;
    else if ((Zc % 7) == 0)
        i_LS = 3;
    else if ((Zc % 5) == 0)
        i_LS = 2;
    else if ((Zc % 3) == 0)
        i_LS = 1;
    else
        i_LS = 0;

    if (Bg == 1) {
        pShiftMatrix = Bg1HShiftMatrix + i_LS * BG1_NONZERO_NUM;
        pMatrixNumPerCol = Bg1MatrixNumPerCol;
        pAddr = Bg1Address;
    } else {
        pShiftMatrix = Bg2HShiftMatrix + i_LS * BG2_NONZERO_NUM;
        pMatrixNumPerCol = Bg2MatrixNumPerCol;
        pAddr = Bg2Address;
    }

    // encoding --------------------------------------------------------------------
    clock_t t_start, t_end;
    double t_encoder[numberCodeblocks];
    double t_total = 0;

    LDPC_ADAPTER_P ldpc_adapter_func = ldpc_select_adapter_func(Zc);

    LDPC_ENCODER ldpc_encoder_func = ldpc_select_encoder_func(Bg);

    for (int n = 0; n < numberCodeblocks; n++) {
        t_start = clock();
        // read input into z-bit segments
        ldpc_adapter_func(input[n], internalBuffer0, Zc, cbLen, 1);
        // encode
        ldpc_encoder_func(internalBuffer0, internalBuffer1, pMatrixNumPerCol, pAddr, pShiftMatrix, (int16_t)Zc, i_LS);
        // scatter the output back to compacted
        // combine the input sequence and the parity bits into codeword outputs
        memcpy(internalBuffer2, internalBuffer0 + 2 * PROC_BYTES, (cbLen / Zc - 2) * PROC_BYTES);
        memcpy(internalBuffer2 + (cbLen / Zc - 2) * PROC_BYTES, internalBuffer1, cbEncLen / Zc * PROC_BYTES);

        // printf("internalBuffer1:\n");
        // for (int i = 0; i < BG1_ROW_TOTAL * PROC_BYTES; i++) {
        //     printf("%i ", internalBuffer1[i]);
        // }
        // printf("\n");

        ldpc_adapter_func(output[n], internalBuffer2, Zc, cbCodewLen, 0);
        t_end = clock();

        t_encoder[n] = (t_start - t_end) / CLOCKS_PER_SEC * (1e9);
        t_total = t_total + t_encoder[n];

        // this is for testing only because intel's reference files only provide the parity bits
        ldpc_adapter_func(parity[n], internalBuffer1, Zc, cbEncLen, 0);
    }

    // performance -----------------------------------------------------------------
    printf("--------5gnr ldpc encoder test--------\n");
    printf("there are %d code blocks in total\n", numberCodeblocks);

    int err_cnt = 0;
    for (int n = 0; n < numberCodeblocks; n++) {
        printf("the encoding for the %dth code block took %f nano seconds\n", n, t_encoder[n]);
        int8_t* parity_buffer = parity[n];
        int8_t* reference_buffer = reference[n];
        for (int i = 0; i < (cbEncLen >> 3); i++) {
            uint8_t error = parity_buffer[i] ^ reference_buffer[i];
            // printf("%d: (%i %i), error: %i\n", i, reference_buffer[i], parity_buffer[i], error);
            for (int j = 0; j < 8; j++) {
                err_cnt += error & 1;
                error >>= 1;
            }
        }
    }

    double ber = (double)err_cnt / cbEncLen / numberCodeblocks;
    printf("the bit error rate is %f (%d/%d)\n", ber, err_cnt, (cbEncLen * numberCodeblocks));

    double thruput = (double)cbLen * numberCodeblocks / t_total * 125;
    printf("the encoder's speed is %f MB/sec\n", thruput);

    for (int n = 0; n < numberCodeblocks; n++) {
        free(input[n]);
        free(reference[n]);
        free(parity[n]);
        free(output[n]);
    }

    return 0;
}