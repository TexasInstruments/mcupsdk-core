/* Defines */
#define MCU_M4F_MAX_MEM_SECTIONS   (1u)

#define MCAN1_MCANSS_MSGMEM_ADD   (0x020718000u)


/* Function prototypes */
void ECC_Test_copyResetVector(void);
int32_t ECC_Test_EsmInitHandlerInit(SDL_ESM_Inst esmInstType);
int32_t ECC_Test_init (void);

static SDL_ECC_MemSubType ECC_Test_subMemTypeList[MCU_M4F_MAX_MEM_SECTIONS] = //Deon
{
  SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IIRAM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_ECCInitConfig =
{
    .numRams = MCU_M4F_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_M4FCoresubMemTypeList[0]),
    /**< Sub type list  */
};

#define AGGR1_MEM_SECTIONS (2u)
static SDL_ECC_MemSubType ECC_Test_AGGR1_A0subMemTypeList[AGGR1_MEM_SECTIONS] =
{
    SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
    SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MAINMSMCA0ECCInitConfig =
{
    .numRams = AGGR1_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_AGGR1_A0subMemTypeList[0]),
    /**< Sub type list  */
};

#endif
