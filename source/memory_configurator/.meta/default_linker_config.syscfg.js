function def_memory_regions(regionInst, ind, device, core){

    if(device == "AM273x")
    {
        if(core.includes("r5fss0-0")){
            if(ind == 0){
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].size               = 0x40;
                regionInst.memory_region[ind].auto               = false;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x3FC0;
            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].size               = 0x4000;
                regionInst.memory_region[ind].$name              = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].type               = "MSS_L2_R5F";
                regionInst.memory_region[ind].$name              = "MSS_L2";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x10260000;
                regionInst.memory_region[ind].size               = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].type               = "DSS_L3_ALL";
                regionInst.memory_region[ind].$name              = "DSS_L3";
                regionInst.memory_region[ind].size               = 0x390000;
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["c66ss0","r5fss0-1"];
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "MSS_L2_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x102E8000;
                regionInst.memory_region[ind].size               = 0x4000;
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["r5fss0-1"];
            }
            else if(ind == 6){
                regionInst.memory_region[ind].type               = "MSS_L2_R5F";
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
                regionInst.memory_region[ind].size               = 0x4000;
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["r5fss0-1"];
            }
            else if(ind == 7){
                regionInst.memory_region[ind].type               = "CUSTOM_ALL";
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0xC5000000;
                regionInst.memory_region[ind].size               = 0x1F40;
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["c66ss0","r5fss0-1"];
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type               = "CUSTOM_ALL";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].$name              = "MAILBOX_HSM";
                regionInst.memory_region[ind].manualStartAddress = 0x44000000;
                regionInst.memory_region[ind].size               = 0x3CE;
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["c66ss0","r5fss0-1"];
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type               = "CUSTOM_ALL";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["c66ss0","r5fss0-1"];
            }
        }
        else if(core.includes("r5fss0-1")){
            if(ind == 0){
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].size               = 0x40;
                regionInst.memory_region[ind].auto               = false;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x3FC0;
            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].size               = 0x4000;
                regionInst.memory_region[ind].$name              = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].type               = "MSS_L2_R5F";
                regionInst.memory_region[ind].$name              = "MSS_L2";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x102A0000;
                regionInst.memory_region[ind].size               = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].type               = "DSS_L3_ALL";
                regionInst.memory_region[ind].$name              = "DSS_L3";
                regionInst.memory_region[ind].size               = 0x390000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "MSS_L2_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x102E8000;
                regionInst.memory_region[ind].size               = 0x4000;
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
            }
            else if(ind == 6){
                regionInst.memory_region[ind].type               = "MSS_L2_R5F";
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
                regionInst.memory_region[ind].size               = 0x4000;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].type               = "CUSTOM_ALL";
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0xC5000000;
                regionInst.memory_region[ind].size               = 0x1F40;
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type               = "CUSTOM_ALL";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].$name              = "MAILBOX_HSM";
                regionInst.memory_region[ind].manualStartAddress = 0x44000000;
                regionInst.memory_region[ind].size               = 0x3CE;
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type               = "CUSTOM_ALL";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
            }
        }
        else if(core.includes("c66")){

            if(ind == 0){
                regionInst.memory_region[ind].type               = "DSP_L2_C66";
                regionInst.memory_region[ind].$name              = "DSS_L2";
                regionInst.memory_region[ind].size               = 0x60000;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "DSS_L3_ALL";
                regionInst.memory_region[ind].size               = 0x390000;
                regionInst.memory_region[ind].$name              = "DSS_L3";
            }
            else if(ind == 2){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0xC02E8000;
                regionInst.memory_region[ind].size               = 0x4000;
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
                regionInst.memory_region[ind].size               = 0x4000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].type               = "CUSTOM_ALL";
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0xC5000000;
                regionInst.memory_region[ind].size               = 0x1F40;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "CUSTOM_ALL";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].$name              = "MAILBOX_HSM";
                regionInst.memory_region[ind].manualStartAddress = 0x44000000;
                regionInst.memory_region[ind].size               = 0x3CE;
            }
            else if(ind == 6){
                regionInst.memory_region[ind].type               = "CUSTOM_ALL";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
            }
        }
    }
    else if(device == "AM263x_beta")
    {
        if(core.includes("r5fss0-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].size               = 0x40;
                regionInst.memory_region[ind].auto               = false;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;
            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB";
                regionInst.memory_region[ind].size               = 0x8000;
                regionInst.memory_region[ind].$name              = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name                 = "SBL";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress    = 0x70000000;
                regionInst.memory_region[ind].size                  = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name               = "OCRAM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x70040000;
                regionInst.memory_region[ind].size                = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type                = "FLASH";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x60100000;
                regionInst.memory_region[ind].size                = 0x80000;
                regionInst.memory_region[ind].$name               = "FLASH";
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name               = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D0000;
                regionInst.memory_region[ind].size                = 0x4000;
                regionInst.memory_region[ind].isShared            = true;
                regionInst.memory_region[ind].shared_cores        = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
            else if(ind == 7){
                regionInst.memory_region[ind].$name               = "LOG_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D4000;
                regionInst.memory_region[ind].size                = 0x4000;
                regionInst.memory_region[ind].isShared            = true;
                regionInst.memory_region[ind].shared_cores        = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x72000000;
                regionInst.memory_region[ind].size                = 0x3E80;
                regionInst.memory_region[ind].isShared            = true;
                regionInst.memory_region[ind].shared_cores        = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "MAILBOX_HSM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x44000000;
                regionInst.memory_region[ind].size                = 0x3CE;
                regionInst.memory_region[ind].isShared            = true;
                regionInst.memory_region[ind].shared_cores        = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
            else if(ind == 10){
                regionInst.memory_region[ind].type               = "CUSTOM";
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
        }
        else if(core.includes("r5fss0-1")){
            if(ind == 0){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_VECS";
                regionInst.memory_region[ind].auto  = false;
                regionInst.memory_region[ind].size  = 0x40;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_TCMA";
                regionInst.memory_region[ind].size  = 0x7FC0;
                            }
            else if(ind == 2){
                regionInst.memory_region[ind].type  = "TCMB";
                regionInst.memory_region[ind].size  = 0x8000;
                regionInst.memory_region[ind].$name = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name                = "SBL";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress   = 0x70000000;
                regionInst.memory_region[ind].size                 = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name                 = "OCRAM";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress    = 0x70080000;
                regionInst.memory_region[ind].size                  = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type                = "FLASH";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x60180000;
                regionInst.memory_region[ind].size                = 0x80000;
                regionInst.memory_region[ind].$name               = "FLASH";
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name               = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D0000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].$name               = "LOG_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D4000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x72000000;
                regionInst.memory_region[ind].size                = 0x3E80;
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "MAILBOX_HSM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x44000000;
                regionInst.memory_region[ind].size                = 0x3CE;
            }
            else if(ind == 10){
                regionInst.memory_region[ind].type               = "CUSTOM";
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
            }
        }
        else if(core.includes("r5fss1-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_VECS";
                regionInst.memory_region[ind].auto  = false;
                regionInst.memory_region[ind].size  = 0x40;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_TCMA";
                regionInst.memory_region[ind].size  = 0x7FC0;
                            }
            else if(ind == 2){
                regionInst.memory_region[ind].type  = "TCMB";
                regionInst.memory_region[ind].size  = 0x8000;
                regionInst.memory_region[ind].$name = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name                = "SBL";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress   = 0x70000000;
                regionInst.memory_region[ind].size                 = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name                 = "OCRAM";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress    = 0x700C0000;
                regionInst.memory_region[ind].size                  = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type                = "FLASH";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x60200000;
                regionInst.memory_region[ind].size                = 0x80000;
                regionInst.memory_region[ind].$name               = "FLASH";
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name               = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D0000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].$name               = "LOG_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D4000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x72000000;
                regionInst.memory_region[ind].size                = 0x3E80;
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "MAILBOX_HSM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x44000000;
                regionInst.memory_region[ind].size                = 0x3CE;
            }
            else if(ind == 10){
                regionInst.memory_region[ind].type               = "CUSTOM";
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
            }
        }
        else if(core.includes("r5fss1-1")){
            if(ind == 0){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_VECS";
                regionInst.memory_region[ind].auto  = false;
                regionInst.memory_region[ind].size  = 0x40;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_TCMA";
                regionInst.memory_region[ind].size  = 0x7FC0;
                            }
            else if(ind == 2){
                regionInst.memory_region[ind].type  = "TCMB";
                regionInst.memory_region[ind].size  = 0x8000;
                regionInst.memory_region[ind].$name = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name                = "SBL";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress   = 0x70000000;
                regionInst.memory_region[ind].size                 = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name                 = "OCRAM";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress    = 0x70100000;
                regionInst.memory_region[ind].size                  = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type                = "FLASH";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x60280000;
                regionInst.memory_region[ind].size                = 0x80000;
                regionInst.memory_region[ind].$name               = "FLASH";
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name               = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D0000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].$name               = "LOG_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D4000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x72000000;
                regionInst.memory_region[ind].size                = 0x3E80;
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "MAILBOX_HSM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x44000000;
                regionInst.memory_region[ind].size                = 0x3CE;
            }
            else if(ind == 10){
                regionInst.memory_region[ind].type               = "CUSTOM";
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
            }
        }
    }
    else if(device == "AM263Px")
    {
        if(core.includes("r5fss0-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].size               = 0x40;
                regionInst.memory_region[ind].auto               = false;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;
            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB";
                regionInst.memory_region[ind].size               = 0x8000;
                regionInst.memory_region[ind].$name              = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name                 = "SBL";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress    = 0x70000000;
                regionInst.memory_region[ind].size                  = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name               = "OCRAM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x70040000;
                regionInst.memory_region[ind].size                = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type                = "FLASH";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x60100000;
                regionInst.memory_region[ind].size                = 0x80000;
                regionInst.memory_region[ind].$name               = "FLASH";
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name               = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D0000;
                regionInst.memory_region[ind].size                = 0x4000;
                regionInst.memory_region[ind].isShared            = true;
                regionInst.memory_region[ind].shared_cores        = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
            else if(ind == 7){
                regionInst.memory_region[ind].$name               = "LOG_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D4000;
                regionInst.memory_region[ind].size                = 0x4000;
                regionInst.memory_region[ind].isShared            = true;
                regionInst.memory_region[ind].shared_cores        = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x72000000;
                regionInst.memory_region[ind].size                = 0x3E80;
                regionInst.memory_region[ind].isShared            = true;
                regionInst.memory_region[ind].shared_cores        = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "MAILBOX_HSM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x44000000;
                regionInst.memory_region[ind].size                = 0x3CE;
                regionInst.memory_region[ind].isShared            = true;
                regionInst.memory_region[ind].shared_cores        = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
            else if(ind == 10){
                regionInst.memory_region[ind].type               = "CUSTOM";
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["r5fss0-1","r5fss1-0","r5fss1-1"];
            }
        }
        else if(core.includes("r5fss0-1")){
            if(ind == 0){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_VECS";
                regionInst.memory_region[ind].auto  = false;
                regionInst.memory_region[ind].size  = 0x40;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_TCMA";
                regionInst.memory_region[ind].size  = 0x7FC0;
                            }
            else if(ind == 2){
                regionInst.memory_region[ind].type  = "TCMB";
                regionInst.memory_region[ind].size  = 0x8000;
                regionInst.memory_region[ind].$name = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name                = "SBL";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress   = 0x70000000;
                regionInst.memory_region[ind].size                 = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name                 = "OCRAM";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress    = 0x70080000;
                regionInst.memory_region[ind].size                  = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type                = "FLASH";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x60180000;
                regionInst.memory_region[ind].size                = 0x80000;
                regionInst.memory_region[ind].$name               = "FLASH";
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name               = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D0000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].$name               = "LOG_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D4000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x72000000;
                regionInst.memory_region[ind].size                = 0x3E80;
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "MAILBOX_HSM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x44000000;
                regionInst.memory_region[ind].size                = 0x3CE;
            }
            else if(ind == 10){
                regionInst.memory_region[ind].type               = "CUSTOM";
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
            }
        }
        else if(core.includes("r5fss1-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_VECS";
                regionInst.memory_region[ind].auto  = false;
                regionInst.memory_region[ind].size  = 0x40;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_TCMA";
                regionInst.memory_region[ind].size  = 0x7FC0;
                            }
            else if(ind == 2){
                regionInst.memory_region[ind].type  = "TCMB";
                regionInst.memory_region[ind].size  = 0x8000;
                regionInst.memory_region[ind].$name = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name                = "SBL";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress   = 0x70000000;
                regionInst.memory_region[ind].size                 = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name                 = "OCRAM";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress    = 0x700C0000;
                regionInst.memory_region[ind].size                  = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type                = "FLASH";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x60200000;
                regionInst.memory_region[ind].size                = 0x80000;
                regionInst.memory_region[ind].$name               = "FLASH";
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name               = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D0000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].$name               = "LOG_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D4000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x72000000;
                regionInst.memory_region[ind].size                = 0x3E80;
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "MAILBOX_HSM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x44000000;
                regionInst.memory_region[ind].size                = 0x3CE;
            }
            else if(ind == 10){
                regionInst.memory_region[ind].type               = "CUSTOM";
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
            }
        }
        else if(core.includes("r5fss1-1")){
            if(ind == 0){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_VECS";
                regionInst.memory_region[ind].auto  = false;
                regionInst.memory_region[ind].size  = 0x40;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type  = "TCMA";
                regionInst.memory_region[ind].$name = "R5F_TCMA";
                regionInst.memory_region[ind].size  = 0x7FC0;
                            }
            else if(ind == 2){
                regionInst.memory_region[ind].type  = "TCMB";
                regionInst.memory_region[ind].size  = 0x8000;
                regionInst.memory_region[ind].$name = "R5F_TCMB";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name                = "SBL";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress   = 0x70000000;
                regionInst.memory_region[ind].size                 = 0x40000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name                 = "OCRAM";
                regionInst.memory_region[ind].auto                  = false;
                regionInst.memory_region[ind].manualStartAddress    = 0x70100000;
                regionInst.memory_region[ind].size                  = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type                = "FLASH";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x60280000;
                regionInst.memory_region[ind].size                = 0x80000;
                regionInst.memory_region[ind].$name               = "FLASH";
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name               = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D0000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].$name               = "LOG_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x701D4000;
                regionInst.memory_region[ind].size                = 0x4000;
            }
            else if(ind == 8){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x72000000;
                regionInst.memory_region[ind].size                = 0x3E80;
            }
            else if(ind == 9){
                regionInst.memory_region[ind].type                = "CUSTOM";
                regionInst.memory_region[ind].$name               = "MAILBOX_HSM";
                regionInst.memory_region[ind].auto                = false;
                regionInst.memory_region[ind].manualStartAddress  = 0x44000000;
                regionInst.memory_region[ind].size                = 0x3CE;
            }
            else if(ind == 10){
                regionInst.memory_region[ind].type               = "CUSTOM";
                regionInst.memory_region[ind].$name              = "MAILBOX_R5F";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x44000400;
                regionInst.memory_region[ind].size               = 0x3CE;
            }
        }
    }
    else if(device == "AM64x")
    {
        if(core.includes("r5fss0-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].size               = 0x40;
                regionInst.memory_region[ind].auto               = false;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;
            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMB0";
                regionInst.memory_region[ind].size               = 0x8000;
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "NON_CACHE_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70060000;
                regionInst.memory_region[ind].size               = 0x8000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name              = "MSRAM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70080000;
                regionInst.memory_region[ind].size               = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "FLASH";
                regionInst.memory_region[ind].$name              = "FLASH";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x60100000;
                regionInst.memory_region[ind].size               = 0x80000;
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x80;
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["a53ss0-0","a53ss0-1","m4fss0-0","r5fss0-1","r5fss1-0","r5fss1-1"];

            }
            else if(ind == 7){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0080;
                regionInst.memory_region[ind].size               = 0x3F80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["a53ss0-0","a53ss0-1","m4fss0-0","r5fss0-1","r5fss1-0","r5fss1-1"];

            }
            else if(ind == 8){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["a53ss0-0","a53ss0-1","m4fss0-0","r5fss0-1","r5fss1-0","r5fss1-1"];
            }
        }
        else if(core.includes("r5fss0-1")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].size               = 0x40;

            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;

            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMB0";
                regionInst.memory_region[ind].size               = 0x8000;

            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "NON_CACHE_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70068000;
                regionInst.memory_region[ind].size               = 0x8000;

            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name              = "MSRAM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x700C0000;
                regionInst.memory_region[ind].size               = 0x40000;

            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "FLASH";
                regionInst.memory_region[ind].$name              = "FLASH";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x60180000;
                regionInst.memory_region[ind].size               = 0x80000;
                            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x80;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0080;
                regionInst.memory_region[ind].size               = 0x3F80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
            }
            else if(ind == 8){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
            }
        }
        else if(core.includes("r5fss1-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].size               = 0x40;

            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;

            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMB0";
                regionInst.memory_region[ind].size               = 0x8000;

            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "NON_CACHE_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70070000;
                regionInst.memory_region[ind].size               = 0x8000;

            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name              = "MSRAM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70100000;
                regionInst.memory_region[ind].size               = 0x40000;

            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "FLASH";
                regionInst.memory_region[ind].$name              = "FLASH";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x60200000;
                regionInst.memory_region[ind].size               = 0x80000;
                            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x80;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0080;
                regionInst.memory_region[ind].size               = 0x3F80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
            }
            else if(ind == 8){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
            }
        }
        else if(core.includes("r5fss1-1")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].size               = 0x40;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;
            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMB0";
                regionInst.memory_region[ind].size               = 0x8000;
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "NON_CACHE_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70078000;
                regionInst.memory_region[ind].size               = 0x8000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name              = "MSRAM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70140000;
                regionInst.memory_region[ind].size               = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "FLASH";
                regionInst.memory_region[ind].$name              = "FLASH";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x60280000;
                regionInst.memory_region[ind].size               = 0x80000;
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x80;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0080;
                regionInst.memory_region[ind].size               = 0x3F80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
            }
            else if(ind == 8){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
            }
        }
        else if(core.includes("m4fss0-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "RAM0_M4F";
                regionInst.memory_region[ind].$name              = "M4F_VECS";
                regionInst.memory_region[ind].size               = 0x200;

            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "RAM0_M4F";
                regionInst.memory_region[ind].$name              = "M4F_IRAM";
                regionInst.memory_region[ind].size               = 0x2FE00;
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x200;

            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "DRAM_M4F";
                regionInst.memory_region[ind].$name              = "M4F_DRAM";
                regionInst.memory_region[ind].size               = 0x10000;
                regionInst.memory_region[ind].auto               = false;

            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x80;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0080;
                regionInst.memory_region[ind].size               = 0x3F80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
            }
            else if(ind == 5){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "IPC_VRING_MEM";
            }
        }
        else if(core.includes("a53s0-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type  = "DDR_ALL";
                regionInst.memory_region[ind].size  = 0x2000000;
                regionInst.memory_region[ind].auto  = false;
                regionInst.memory_region[ind].$name = "DDR";

            }
            else if(ind == 1){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x80;
            }
            else if(ind == 2){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0080;
                regionInst.memory_region[ind].size               = 0x3F80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
            }
            else if(ind == 3){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
            }
        }
    }
    else if( device == "AM243x_ALX_beta" || device == "AM243x_ALV_beta")
    {
        if(core.includes("r5fss0-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].size               = 0x40;
                regionInst.memory_region[ind].auto               = false;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;
            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMB0";
                regionInst.memory_region[ind].size               = 0x8000;
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "NON_CACHE_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70060000;
                regionInst.memory_region[ind].size               = 0x8000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name              = "MSRAM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70080000;
                regionInst.memory_region[ind].size               = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "FLASH";
                regionInst.memory_region[ind].$name              = "FLASH";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x60100000;
                regionInst.memory_region[ind].size               = 0x80000;
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x180;
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["m4fss0-0","r5fss0-1","r5fss1-0","r5fss1-1"];

            }
            else if(ind == 7){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0180;
                regionInst.memory_region[ind].size               = 0x3E80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["m4fss0-0","r5fss0-1","r5fss1-0","r5fss1-1"];

            }
            else if(ind == 8){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
                regionInst.memory_region[ind].isShared           = true;
                regionInst.memory_region[ind].shared_cores       = ["m4fss0-0","r5fss0-1","r5fss1-0","r5fss1-1"];
            }
        }
        else if(core.includes("r5fss0-1")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].size               = 0x40;

            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;

            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMB0";
                regionInst.memory_region[ind].size               = 0x8000;

            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "NON_CACHE_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70068000;
                regionInst.memory_region[ind].size               = 0x8000;

            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name              = "MSRAM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x700C0000;
                regionInst.memory_region[ind].size               = 0x40000;

            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "FLASH";
                regionInst.memory_region[ind].$name              = "FLASH";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x60180000;
                regionInst.memory_region[ind].size               = 0x80000;
                            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x180;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0180;
                regionInst.memory_region[ind].size               = 0x3E80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
            }
            else if(ind == 8){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
            }
        }
        else if(core.includes("r5fss1-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].size               = 0x40;

            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;

            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMB0";
                regionInst.memory_region[ind].size               = 0x8000;

            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "NON_CACHE_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70070000;
                regionInst.memory_region[ind].size               = 0x8000;

            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name              = "MSRAM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70100000;
                regionInst.memory_region[ind].size               = 0x40000;

            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "FLASH";
                regionInst.memory_region[ind].$name              = "FLASH";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x60200000;
                regionInst.memory_region[ind].size               = 0x80000;
                            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x180;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0180;
                regionInst.memory_region[ind].size               = 0x3E80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
            }
            else if(ind == 8){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
            }
        }
        else if(core.includes("r5fss1-1")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_VECS";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].size               = 0x40;
            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "TCMA_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMA";
                regionInst.memory_region[ind].size               = 0x7FC0;
            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "TCMB_R5F";
                regionInst.memory_region[ind].$name              = "R5F_TCMB0";
                regionInst.memory_region[ind].size               = 0x8000;
            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "NON_CACHE_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70078000;
                regionInst.memory_region[ind].size               = 0x8000;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].$name              = "MSRAM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x70140000;
                regionInst.memory_region[ind].size               = 0x40000;
            }
            else if(ind == 5){
                regionInst.memory_region[ind].type               = "FLASH";
                regionInst.memory_region[ind].$name              = "FLASH";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x60280000;
                regionInst.memory_region[ind].size               = 0x80000;
            }
            else if(ind == 6){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x180;
            }
            else if(ind == 7){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0180;
                regionInst.memory_region[ind].size               = 0x3E80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
            }
            else if(ind == 8){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "RTOS_NORTOS_IPC_SHM_MEM";
            }
        }
        else if(core.includes("m4fss0-0")){
            if(ind == 0){
                regionInst.memory_region[ind].type               = "RAM0_M4F";
                regionInst.memory_region[ind].$name              = "M4F_VECS";
                regionInst.memory_region[ind].size               = 0x200;

            }
            else if(ind == 1){
                regionInst.memory_region[ind].type               = "RAM0_M4F";
                regionInst.memory_region[ind].$name              = "M4F_IRAM";
                regionInst.memory_region[ind].size               = 0x2FE00;
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x200;

            }
            else if(ind == 2){
                regionInst.memory_region[ind].type               = "DRAM_M4F";
                regionInst.memory_region[ind].$name              = "M4F_DRAM";
                regionInst.memory_region[ind].size               = 0x10000;
                regionInst.memory_region[ind].auto               = false;

            }
            else if(ind == 3){
                regionInst.memory_region[ind].$name              = "USER_SHM_MEM";
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0000;
                regionInst.memory_region[ind].size               = 0x180;
            }
            else if(ind == 4){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D0180;
                regionInst.memory_region[ind].size               = 0x3E80;
                regionInst.memory_region[ind].$name              = "LOG_SHM_MEM";
            }
            else if(ind == 5){
                regionInst.memory_region[ind].auto               = false;
                regionInst.memory_region[ind].manualStartAddress = 0x701D4000;
                regionInst.memory_region[ind].size               = 0xC000;
                regionInst.memory_region[ind].$name              = "IPC_VRING_MEM";
            }
        }
    }
}

function def_shared_region_name_change(sharedRegionNameChangeInst, device, core){

    if( (device == "AM64x" || device == "AM243x_ALV_beta" || device == "AM243x_ALX_beta") && core.includes("m4f") ){
            sharedRegionNameChangeInst.$name                     = "CONFIG_SHARED0";
            sharedRegionNameChangeInst.shared_region             = "RTOS_NORTOS_IPC_SHM_MEM";
            sharedRegionNameChangeInst.shared_region_name_change = "IPC_VRING_MEM";
    }
}

function def_sections(sectionInst, ind, device, core){

    if( device == "AM273x" ){
        if( core.includes("r5f") ) {
            if(ind == 1) {
                sectionInst.$name                   = "Vector Table";
                sectionInst.group                   = false;
                sectionInst.load_memory             = "R5F_VECS";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name = ".vectors";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 2){

                sectionInst.$name                   = "Code and Read-Only Data";
                sectionInst.load_memory             = "MSS_L2";
                sectionInst.output_section.create(2);
                sectionInst.output_section[0].$name = ".text";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name = ".rodata";
                sectionInst.output_section[1].palignment = true;
            }
            else if(ind ==3){
                sectionInst.$name                   = "Data Segment";
                sectionInst.load_memory             = "MSS_L2";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name = ".data";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 4){
                sectionInst.$name                                   = "Memory Segments";
                sectionInst.load_memory                             = "MSS_L2";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name                 = ".bss";
                sectionInst.output_section[0].palignment               = true;
                sectionInst.output_section[0].output_sections_start = "__BSS_START";
                sectionInst.output_section[0].output_sections_end   = "__BSS_END";
                sectionInst.output_section[1].$name                 = ".sysmem";
                sectionInst.output_section[1].palignment                = true;
                sectionInst.output_section[2].$name                 = ".stack";
                sectionInst.output_section[2].palignment               = true;
            }
            else if(ind == 5){
                sectionInst.$name                                    = "Stack Segments";
                sectionInst.load_memory                              = "MSS_L2";
                sectionInst.output_section.create(5);
                sectionInst.output_section[0].$name                  = ".irqstack";
                sectionInst.output_section[0].output_sections_start  = "__IRQ_STACK_START";
                sectionInst.output_section[0].output_sections_end    = "__IRQ_STACK_END";
                sectionInst.output_section[0].input_section.create(1);
                sectionInst.output_section[0].input_section[0].$name = ". = . + __IRQ_STACK_SIZE;";
                sectionInst.output_section[1].$name                  = ".fiqstack";
                sectionInst.output_section[1].output_sections_start  = "__FIQ_STACK_START";
                sectionInst.output_section[1].output_sections_end    = "__FIQ_STACK_END";
                sectionInst.output_section[1].input_section.create(1);
                sectionInst.output_section[1].input_section[0].$name = ". = . + __FIQ_STACK_SIZE;";
                sectionInst.output_section[2].$name                  = ".svcstack";
                sectionInst.output_section[2].output_sections_start  = "__SVC_STACK_START";
                sectionInst.output_section[2].output_sections_end    = "__SVC_STACK_END";
                sectionInst.output_section[2].input_section.create(1);
                sectionInst.output_section[2].input_section[0].$name = ". = . + __SVC_STACK_SIZE;";
                sectionInst.output_section[3].$name                  = ".abortstack";
                sectionInst.output_section[3].output_sections_start  = "__ABORT_STACK_START";
                sectionInst.output_section[3].output_sections_end    = "__ABORT_STACK_END";
                sectionInst.output_section[3].input_section.create(1);
                sectionInst.output_section[3].input_section[0].$name = ". = . + __ABORT_STACK_SIZE;";
                sectionInst.output_section[4].$name                  = ".undefinedstack";
                sectionInst.output_section[4].output_sections_start  = "__UNDEFINED_STACK_START";
                sectionInst.output_section[4].output_sections_end    = "__UNDEFINED_STACK_END";
                sectionInst.output_section[4].input_section.create(1);
                sectionInst.output_section[4].input_section[0].$name = ". = . + __UNDEFINED_STACK_SIZE;";
            }
            else if(ind == 6){
                sectionInst.$name                   = "Initialization and Exception Handling";
                sectionInst.load_memory             = "MSS_L2";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name = ".ARM.exidx";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name = ".init_array";
                sectionInst.output_section[1].palignment = true;
                sectionInst.output_section[2].$name = ".fini_array";
                sectionInst.output_section[2].palignment = true;
            }
            else if(ind == 7){
                sectionInst.$name                       = "BSS DSS L3";
                sectionInst.load_memory                 = "DSS_L3";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.dss_l3";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 8){
                sectionInst.$name                       = "User Shared Memory";
                sectionInst.load_memory                 = "USER_SHM_MEM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.user_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 9){
                sectionInst.$name                       = "Log Shared Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.load_memory                 = "LOG_SHM_MEM";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.log_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 10){
                sectionInst.$name                       = "IPC Shared Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.load_memory                 = "RTOS_NORTOS_IPC_SHM_MEM";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].alignment = 0;
                sectionInst.output_section[0].$name     = ".bss.ipc_vring_mem";
            }
            else if(ind == 11){
                sectionInst.$name                       = "SIPC HSM Queue Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.load_memory                 = "MAILBOX_HSM";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].alignment = 0;
                sectionInst.output_section[0].$name     = ".bss.sipc_hsm_queue_mem";
            }
            else if(ind == 12){
                sectionInst.$name                       = "SIPC R5F Queue Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.group                       = false;
                sectionInst.load_memory                 = "MAILBOX_R5F";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].alignment = 0;
                sectionInst.output_section[0].$name     = ".bss.sipc_r5f_queue_mem";
            }
        }
        else if( core.includes("c66")){
            if(ind == 1) {
                sectionInst.load_to_memory                           = "Address";
                sectionInst.group                                    = false;
                sectionInst.load_to_address                          = "0x00800000";
                sectionInst.$name                                    = "Vector Table";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name                  = ".text:vectors";
                sectionInst.output_section[0].alignment              = 1024;
            }
            else if(ind == 2){
                sectionInst.load_memory                 = "DSS_L2";
                sectionInst.group                       = false;
                sectionInst.$name                       = "Memory Segments";
                sectionInst.output_section.create(10);
                sectionInst.output_section[0].$name     = ".text";
                sectionInst.output_section[0].alignment = 0;
                sectionInst.output_section[1].$name     = ".const";
                sectionInst.output_section[1].alignment = 0;
                sectionInst.output_section[2].$name     = ".cinit";
                sectionInst.output_section[2].alignment = 0;
                sectionInst.output_section[3].$name     = ".data";
                sectionInst.output_section[3].alignment = 0;
                sectionInst.output_section[4].$name     = ".stack";
                sectionInst.output_section[4].alignment = 0;
                sectionInst.output_section[5].$name     = ".switch";
                sectionInst.output_section[5].alignment = 0;
                sectionInst.output_section[6].$name     = ".cio";
                sectionInst.output_section[6].alignment = 0;
                sectionInst.output_section[7].$name     = ".sysmem";
                sectionInst.output_section[7].alignment = 0;
                sectionInst.output_section[8].$name     = ".fardata";
                sectionInst.output_section[8].alignment = 0;
                sectionInst.output_section[9].$name     = ".far";
                sectionInst.output_section[9].alignment = 0;
            }
            else if(ind ==3){
                sectionInst.load_memory                 = "DSS_L2";
                sectionInst.$name                       = "Code and Read-Only Data";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name     = ".rodata";
                sectionInst.output_section[0].alignment = 0;
                sectionInst.output_section[1].$name     = ".bss";
                sectionInst.output_section[1].alignment = 0;
                sectionInst.output_section[2].$name     = ".neardata";
                sectionInst.output_section[2].alignment = 0;
            }
            else if(ind == 4){
                sectionInst.load_memory             = "DSS_L2";
                sectionInst.$name                   = "Initialization and Exception Handling";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name = ".c6xabi.exidx";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name = ".init_array";
                sectionInst.output_section[1].palignment = true;
                sectionInst.output_section[2].$name = ".fini_array";
                sectionInst.output_section[2].$name      = ".fini_array";
                sectionInst.output_section[2].palignment = true;
            }
            else if(ind == 5){
                sectionInst.$name                       = "BSS DSS L3";
                sectionInst.load_memory                 = "DSS_L3";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.dss_l3";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 6){
                sectionInst.$name                       = "User Shared Memory";
                sectionInst.load_memory                 = "USER_SHM_MEM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.user_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 7){
                sectionInst.$name                       = "Log Shared Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.load_memory                 = "LOG_SHM_MEM";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.log_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 8){
                sectionInst.$name                       = "IPC Shared Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.load_memory                 = "RTOS_NORTOS_IPC_SHM_MEM";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].alignment = 0;
                sectionInst.output_section[0].$name     = ".bss.ipc_vring_mem";
            }
            else if(ind == 9){
                sectionInst.$name                       = "SIPC HSM Queue Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.load_memory                 = "MAILBOX_HSM";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].alignment = 0;
                sectionInst.output_section[0].$name     = ".bss.sipc_hsm_queue_mem";
            }
            else if(ind == 10){
                sectionInst.$name                       = "SIPC R5F Queue Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.group                       = false;
                sectionInst.load_memory                 = "MAILBOX_R5F";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].alignment = 0;
                sectionInst.output_section[0].$name     = ".bss.sipc_r5f_queue_mem";
            }
        }
    }
    else if( device == "AM263x_beta" ){
        if( core.includes("r5f") ) {
            if(ind == 1) {
                sectionInst.load_memory                  = "R5F_VECS";
                sectionInst.group                        = false;
                sectionInst.$name                        = "Vector Table";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name      = ".vectors";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 2){
                sectionInst.load_memory                  = "OCRAM";
                sectionInst.$name                        = "Text Segments";
                sectionInst.output_section.create(5);
                sectionInst.output_section[0].$name      = ".text.hwi";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".text.cache";
                sectionInst.output_section[1].palignment = true;
                sectionInst.output_section[2].$name      = ".text.mpu";
                sectionInst.output_section[2].palignment = true;
                sectionInst.output_section[3].$name      = ".text.boot";
                sectionInst.output_section[3].palignment = true;
                sectionInst.output_section[4].$name      = ".text:abort";
                sectionInst.output_section[4].palignment = true;
            }
            else if(ind == 3){
                sectionInst.load_memory                  = "OCRAM";
                sectionInst.$name                        = "Code and Read-Only Data";
                sectionInst.output_section.create(2);
                sectionInst.output_section[0].$name      = ".text";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".rodata";
                sectionInst.output_section[1].palignment = true;
            }
            else if(ind == 4){
                sectionInst.load_memory                  = "OCRAM";
                sectionInst.$name                        = "Data Segment";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name      = ".data";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 5){
                sectionInst.load_memory                             = "OCRAM";
                sectionInst.$name                                   = "Memory Segments";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name                 = ".bss";
                sectionInst.output_section[0].output_sections_start = "__BSS_START";
                sectionInst.output_section[0].output_sections_end   = "__BSS_END";
                sectionInst.output_section[0].palignment            = true;
                sectionInst.output_section[1].$name                 = ".sysmem";
                sectionInst.output_section[1].palignment            = true;
                sectionInst.output_section[2].$name                 = ".stack";
                sectionInst.output_section[2].palignment            = true;
            }
            else if(ind == 6){
                sectionInst.load_memory                              = "OCRAM";
                sectionInst.$name                                    = "Stack Segments";
                sectionInst.output_section.create(5);
                sectionInst.output_section[0].$name                  = ".irqstack";
                sectionInst.output_section[0].output_sections_start  = "__IRQ_STACK_START";
                sectionInst.output_section[0].output_sections_end    = "__IRQ_STACK_END";
                sectionInst.output_section[0].input_section.create(1);
                sectionInst.output_section[0].input_section[0].$name = ". = . + __IRQ_STACK_SIZE;";
                sectionInst.output_section[1].$name                  = ".fiqstack";
                sectionInst.output_section[1].output_sections_start  = "__FIQ_STACK_START";
                sectionInst.output_section[1].output_sections_end    = "__FIQ_STACK_END";
                sectionInst.output_section[1].input_section.create(1);
                sectionInst.output_section[1].input_section[0].$name = ". = . + __FIQ_STACK_SIZE;";
                sectionInst.output_section[2].$name                  = ".svcstack";
                sectionInst.output_section[2].output_sections_start  = "__SVC_STACK_START";
                sectionInst.output_section[2].output_sections_end    = "__SVC_STACK_END";
                sectionInst.output_section[2].input_section.create(1);
                sectionInst.output_section[2].input_section[0].$name = ". = . + __SVC_STACK_SIZE;";
                sectionInst.output_section[3].$name                  = ".abortstack";
                sectionInst.output_section[3].output_sections_start  = "__ABORT_STACK_START";
                sectionInst.output_section[3].output_sections_end    = "__ABORT_STACK_END";
                sectionInst.output_section[3].input_section.create(1);
                sectionInst.output_section[3].input_section[0].$name = ". = . + __ABORT_STACK_SIZE;";
                sectionInst.output_section[4].$name                  = ".undefinedstack";
                sectionInst.output_section[4].output_sections_start  = "__UNDEFINED_STACK_START";
                sectionInst.output_section[4].output_sections_end    = "__UNDEFINED_STACK_END";
                sectionInst.output_section[4].input_section.create(1);
                sectionInst.output_section[4].input_section[0].$name = ". = . + __UNDEFINED_STACK_SIZE;";
            }
            else if(ind == 7){
                sectionInst.load_memory                  = "OCRAM";
                sectionInst.$name                        = "Initialization and Exception Handling";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name      = ".ARM.exidx";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".init_array";
                sectionInst.output_section[1].palignment = true;
                sectionInst.output_section[2].$name      = ".fini_array";
                sectionInst.output_section[2].palignment = true;
            }
            else if(ind == 8){
                sectionInst.load_memory                 = "USER_SHM_MEM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.$name                       = "User Shared Memory";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.user_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 9){
                sectionInst.load_memory                 = "LOG_SHM_MEM";
                sectionInst.$name                       = "Log Shared Memory";
                sectionInst.group                       = false;
                sectionInst.type                        = "NOLOAD";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.log_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 10){
                sectionInst.load_memory                 = "RTOS_NORTOS_IPC_SHM_MEM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.$name                       = "IPC Shared Memory";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.ipc_vring_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 11){
                sectionInst.load_memory                 = "MAILBOX_HSM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.$name                       = "SIPC HSM Queue Memory";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.sipc_hsm_queue_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 12){
                sectionInst.load_memory                 = "MAILBOX_R5F";
                sectionInst.$name                       = "SIPC R5F Queue Memory";
                sectionInst.group                       = false;
                sectionInst.type                        = "NOLOAD";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.sipc_r5f_queue_mem";
                sectionInst.output_section[0].alignment = 0;
            }
        }
    }
    else if( device == "AM263Px" ){
        if( core.includes("r5f") ) {
            if(ind == 1) {
                sectionInst.load_memory                  = "R5F_VECS";
                sectionInst.group                        = false;
                sectionInst.$name                        = "Vector Table";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name      = ".vectors";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 2){
                sectionInst.load_memory                  = "OCRAM";
                sectionInst.$name                        = "Text Segments";
                sectionInst.output_section.create(5);
                sectionInst.output_section[0].$name      = ".text.hwi";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".text.cache";
                sectionInst.output_section[1].palignment = true;
                sectionInst.output_section[2].$name      = ".text.mpu";
                sectionInst.output_section[2].palignment = true;
                sectionInst.output_section[3].$name      = ".text.boot";
                sectionInst.output_section[3].palignment = true;
                sectionInst.output_section[4].$name      = ".text:abort";
                sectionInst.output_section[4].palignment = true;
            }
            else if(ind == 3){
                sectionInst.load_memory                  = "OCRAM";
                sectionInst.$name                        = "Code and Read-Only Data";
                sectionInst.output_section.create(2);
                sectionInst.output_section[0].$name      = ".text";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".rodata";
                sectionInst.output_section[1].palignment = true;
            }
            else if(ind == 4){
                sectionInst.load_memory                  = "OCRAM";
                sectionInst.$name                        = "Data Segment";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name      = ".data";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 5){
                sectionInst.load_memory                             = "OCRAM";
                sectionInst.$name                                   = "Memory Segments";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name                 = ".bss";
                sectionInst.output_section[0].output_sections_start = "__BSS_START";
                sectionInst.output_section[0].output_sections_end   = "__BSS_END";
                sectionInst.output_section[0].palignment            = true;
                sectionInst.output_section[1].$name                 = ".sysmem";
                sectionInst.output_section[1].palignment            = true;
                sectionInst.output_section[2].$name                 = ".stack";
                sectionInst.output_section[2].palignment            = true;
            }
            else if(ind == 6){
                sectionInst.load_memory                              = "OCRAM";
                sectionInst.$name                                    = "Stack Segments";
                sectionInst.output_section.create(5);
                sectionInst.output_section[0].$name                  = ".irqstack";
                sectionInst.output_section[0].output_sections_start  = "__IRQ_STACK_START";
                sectionInst.output_section[0].output_sections_end    = "__IRQ_STACK_END";
                sectionInst.output_section[0].input_section.create(1);
                sectionInst.output_section[0].input_section[0].$name = ". = . + __IRQ_STACK_SIZE;";
                sectionInst.output_section[1].$name                  = ".fiqstack";
                sectionInst.output_section[1].output_sections_start  = "__FIQ_STACK_START";
                sectionInst.output_section[1].output_sections_end    = "__FIQ_STACK_END";
                sectionInst.output_section[1].input_section.create(1);
                sectionInst.output_section[1].input_section[0].$name = ". = . + __FIQ_STACK_SIZE;";
                sectionInst.output_section[2].$name                  = ".svcstack";
                sectionInst.output_section[2].output_sections_start  = "__SVC_STACK_START";
                sectionInst.output_section[2].output_sections_end    = "__SVC_STACK_END";
                sectionInst.output_section[2].input_section.create(1);
                sectionInst.output_section[2].input_section[0].$name = ". = . + __SVC_STACK_SIZE;";
                sectionInst.output_section[3].$name                  = ".abortstack";
                sectionInst.output_section[3].output_sections_start  = "__ABORT_STACK_START";
                sectionInst.output_section[3].output_sections_end    = "__ABORT_STACK_END";
                sectionInst.output_section[3].input_section.create(1);
                sectionInst.output_section[3].input_section[0].$name = ". = . + __ABORT_STACK_SIZE;";
                sectionInst.output_section[4].$name                  = ".undefinedstack";
                sectionInst.output_section[4].output_sections_start  = "__UNDEFINED_STACK_START";
                sectionInst.output_section[4].output_sections_end    = "__UNDEFINED_STACK_END";
                sectionInst.output_section[4].input_section.create(1);
                sectionInst.output_section[4].input_section[0].$name = ". = . + __UNDEFINED_STACK_SIZE;";
            }
            else if(ind == 7){
                sectionInst.load_memory                  = "OCRAM";
                sectionInst.$name                        = "Initialization and Exception Handling";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name      = ".ARM.exidx";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".init_array";
                sectionInst.output_section[1].palignment = true;
                sectionInst.output_section[2].$name      = ".fini_array";
                sectionInst.output_section[2].palignment = true;
            }
            else if(ind == 8){
                sectionInst.load_memory                 = "USER_SHM_MEM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.$name                       = "User Shared Memory";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.user_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 9){
                sectionInst.load_memory                 = "LOG_SHM_MEM";
                sectionInst.$name                       = "Log Shared Memory";
                sectionInst.group                       = false;
                sectionInst.type                        = "NOLOAD";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.log_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 10){
                sectionInst.load_memory                 = "RTOS_NORTOS_IPC_SHM_MEM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.$name                       = "IPC Shared Memory";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.ipc_vring_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 11){
                sectionInst.load_memory                 = "MAILBOX_HSM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.$name                       = "SIPC HSM Queue Memory";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.sipc_hsm_queue_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 12){
                sectionInst.load_memory                 = "MAILBOX_R5F";
                sectionInst.$name                       = "SIPC R5F Queue Memory";
                sectionInst.group                       = false;
                sectionInst.type                        = "NOLOAD";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.sipc_r5f_queue_mem";
                sectionInst.output_section[0].alignment = 0;
            }
        }
    }
    else if( device == "AM64x" || device == "AM243x_ALV_beta" || device == "AM243x_ALX_beta"){
        if( core.includes("r5f") ) {
            if(ind == 1) {
                sectionInst.$name                        = "Vector Table";
                sectionInst.load_memory                  = "R5F_VECS";
                sectionInst.group                        = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name      = ".vectors";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 2){
                sectionInst.$name                        = "Text Segments";
                sectionInst.load_memory                  = "MSRAM";
                sectionInst.output_section.create(5);
                sectionInst.output_section[0].$name      = ".text.hwi";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".text.cache";
                sectionInst.output_section[1].palignment = true;
                sectionInst.output_section[2].$name      = ".text.mpu";
                sectionInst.output_section[2].palignment = true;
                sectionInst.output_section[3].$name      = ".text.boot";
                sectionInst.output_section[3].palignment = true;
                sectionInst.output_section[4].$name      = ".text:abort";
                sectionInst.output_section[4].palignment = true;
            }
            else if(ind == 3){
                sectionInst.$name                        = "Code and Read-Only Data";
                sectionInst.load_memory                  = "MSRAM";
                sectionInst.output_section.create(2);
                sectionInst.output_section[0].$name      = ".text";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".rodata";
                sectionInst.output_section[1].palignment = true;
                            }
            else if(ind == 4){
                sectionInst.$name                        = "Data Segment";
                sectionInst.load_memory                  = "MSRAM";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name      = ".data";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 5){
                sectionInst.$name                                   = "Memory Segments";
                sectionInst.load_memory                             = "MSRAM";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name                 = ".bss";
                sectionInst.output_section[0].palignment            = true;
                sectionInst.output_section[0].output_sections_start = "__BSS_START";
                sectionInst.output_section[0].output_sections_end   = "__BSS_END";
                sectionInst.output_section[1].$name                 = ".sysmem";
                sectionInst.output_section[1].palignment            = true;
                sectionInst.output_section[2].$name                 = ".stack";
                sectionInst.output_section[2].palignment            = true;
            }
            else if(ind == 6){
                sectionInst.$name                                    = "Stack Segments";
                sectionInst.load_memory                              = "MSRAM";
                sectionInst.output_section.create(5);
                sectionInst.output_section[0].$name                  = ".irqstack";
                sectionInst.output_section[0].output_sections_start  = "__IRQ_STACK_START";
                sectionInst.output_section[0].output_sections_end    = "__IRQ_STACK_END";
                sectionInst.output_section[0].input_section.create(1);
                sectionInst.output_section[0].input_section[0].$name = ". = . + __IRQ_STACK_SIZE;";
                sectionInst.output_section[1].$name                  = ".fiqstack";
                sectionInst.output_section[1].output_sections_start  = "__FIQ_STACK_START";
                sectionInst.output_section[1].output_sections_end    = "__FIQ_STACK_END";
                sectionInst.output_section[1].input_section.create(1);
                sectionInst.output_section[1].input_section[0].$name = ". = . + __FIQ_STACK_SIZE;";
                sectionInst.output_section[2].$name                  = ".svcstack";
                sectionInst.output_section[2].output_sections_start  = "__SVC_STACK_START";
                sectionInst.output_section[2].output_sections_end    = "__SVC_STACK_END";
                sectionInst.output_section[2].input_section.create(1);
                sectionInst.output_section[2].input_section[0].$name = ". = . + __SVC_STACK_SIZE;";
                sectionInst.output_section[3].$name                  = ".abortstack";
                sectionInst.output_section[3].output_sections_start  = "__ABORT_STACK_START";
                sectionInst.output_section[3].output_sections_end    = "__ABORT_STACK_END";
                sectionInst.output_section[3].input_section.create(1);
                sectionInst.output_section[3].input_section[0].$name = ". = . + __ABORT_STACK_SIZE;";
                sectionInst.output_section[4].$name                  = ".undefinedstack";
                sectionInst.output_section[4].output_sections_start  = "__UNDEFINED_STACK_START";
                sectionInst.output_section[4].output_sections_end    = "__UNDEFINED_STACK_END";
                sectionInst.output_section[4].input_section.create(1);
                sectionInst.output_section[4].input_section[0].$name = ". = . + __UNDEFINED_STACK_SIZE;";
            }
            else if(ind == 7){
                sectionInst.$name                        = "Initialization and Exception Handling";
                sectionInst.load_memory                  = "MSRAM";
                sectionInst.output_section.create(3);
                sectionInst.output_section[0].$name      = ".ARM.exidx";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".init_array";
                sectionInst.output_section[1].palignment = true;
                sectionInst.output_section[2].$name      = ".fini_array";
                sectionInst.output_section[2].palignment = true;
            }
            else if(ind == 8){
                sectionInst.$name                       = "User Shared Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.load_memory                 = "USER_SHM_MEM";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.user_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 9){
                sectionInst.$name                       = "Log Shared Memory";
                sectionInst.load_memory                 = "LOG_SHM_MEM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.log_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 10){
                sectionInst.$name                       = "IPC Shared Memory";
                sectionInst.type                        = "NOLOAD";
                sectionInst.load_memory                 = "RTOS_NORTOS_IPC_SHM_MEM";
                sectionInst.group                       = false;
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.ipc_vring_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 11){
                sectionInst.$name                       = "Non Cacheable Memory";
                sectionInst.load_memory                 = "NON_CACHE_MEM";
                sectionInst.group                       = false;
                sectionInst.type                        = "NOLOAD";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.nocache";
                sectionInst.output_section[0].alignment = 0;
            }
        }
        else if( core.includes("m4f") ) {
            if(ind == 1) {
                sectionInst.load_memory                  = "M4F_VECS";
                sectionInst.group                        = false;
                sectionInst.$name                        = "Vector Table";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name      = ".vectors";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 2){
                sectionInst.load_memory                  = "M4F_IRAM";
                sectionInst.group                        = false;
                sectionInst.$name                        = "Text Segments";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name      = ".text";
                sectionInst.output_section[0].palignment = true;
            }
            else if(ind == 3){
                sectionInst.group                                   = false;
                sectionInst.load_memory                             = "M4F_DRAM";
                sectionInst.$name                                   = "Bss Segment";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name                 = ".bss";
                sectionInst.output_section[0].output_sections_start = "__BSS_START";
                sectionInst.output_section[0].output_sections_end   = "__BSS_END";
                            }
            else if(ind == 4){
                sectionInst.load_memory                  = "M4F_DRAM";
                sectionInst.group                        = false;
                sectionInst.$name                        = "Code and Read-Only Data";
                sectionInst.output_section.create(2);
                sectionInst.output_section[0].$name      = ".data";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".rodata";
                sectionInst.output_section[1].palignment = true;
            }
            else if(ind == 5){
                sectionInst.load_memory                  = "M4F_IRAM";
                sectionInst.group                        = false;
                sectionInst.$name                        = "Memory and Stack Segments";
                sectionInst.output_section.create(5);
                sectionInst.output_section[0].$name      = ".sysmem";
                sectionInst.output_section[0].palignment = true;
                sectionInst.output_section[1].$name      = ".stack";
                sectionInst.output_section[1].palignment = true;
                sectionInst.output_section[2].$name      = ".ARM.exidx";
                sectionInst.output_section[2].palignment = true;
                sectionInst.output_section[3].$name      = ".init_array";
                sectionInst.output_section[3].palignment = true;
                sectionInst.output_section[4].$name      = ".fini_array";
                sectionInst.output_section[4].palignment = true;
            }
            else if(ind == 6){
                sectionInst.type                        = "NOLOAD";
                sectionInst.load_memory                 = "USER_SHM_MEM";
                sectionInst.group                       = false;
                sectionInst.$name                       = "User Shared Memory";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.user_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 7){
                sectionInst.load_memory                 = "LOG_SHM_MEM";
                sectionInst.group                       = false;
                sectionInst.type                        = "NOLOAD";
                sectionInst.$name                       = "Log Shared Memory";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.log_shared_mem";
                sectionInst.output_section[0].alignment = 0;
            }
            else if(ind == 8){
                sectionInst.load_memory                 = "IPC_VRING_MEM";
                sectionInst.type                        = "NOLOAD";
                sectionInst.group                       = false;
                sectionInst.$name                       = "IPC Shared Memory";
                sectionInst.output_section.create(1);
                sectionInst.output_section[0].$name     = ".bss.ipc_vring_mem";
                sectionInst.output_section[0].alignment = 0;
            }
        }
        else if( core.includes("a53") ) {
            if(ind == 1) {

            }
            else if(ind == 2){

            }
            else if(ind == 3){

            }
            else if(ind == 4){

            }
            else if(ind == 5){

            }
            else if(ind == 6){

            }
            else if(ind == 7){

            }
        }
    }
}

exports = {
    populate_memory_regions: def_memory_regions,
    populate_sections_regions: def_sections,
    populate_shared_region_name_change: def_shared_region_name_change
}