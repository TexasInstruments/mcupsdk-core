#ifndef INPUT_CONFIG_H_
#define INPUT_CONFIG_H_

#define SKIP_NORMALIZE
#define OUTPUT_INT
#define FE_FFT
#define FE_DC_REM
#define FE_BIN
#define FE_LOG
#define FE_CONCAT
#define FE_VARIABLES 3
#define FE_FRAME_SIZE 256
#define FE_HL 1
#define FE_FEATURE_SIZE_PER_FRAME 16
#define FE_STACKING_CHANNELS 3
#define FE_STACKING_FRAME_WIDTH 128
#define FE_NN_OUT_SIZE 2
#define FE_OFFSET 0
#define FE_SCALE 1
#define FE_FFT_STAGES 8
#define FE_MIN_FFT_BIN 1
#define FE_FFT_BIN_SIZE 8
#define FE_BIN_NORMALIZE 1
#define FE_LOG_MUL 20
#define FE_LOG_BASE 10
#define FE_LOG_TOL 1e-100
#define FE_NUM_FRAME_CONCAT 8

#endif /* INPUT_CONFIG_H_ */
