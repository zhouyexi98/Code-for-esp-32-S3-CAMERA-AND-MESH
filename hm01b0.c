#include "hm01b0.h"
#include <stddef.h>

hm01b0_status_e hm01b0_write_reg(hm01b0_cfg_t *psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes){
    if(psCfg && psCfg->interface && psCfg->interface->write) return psCfg->interface->write(psCfg, ui16Reg, pui8Value, ui32NumBytes, psCfg->interface->arg);
    return HM01B0_ERR_UNIMPLEMENTED;
}

hm01b0_status_e hm01b0_read_reg(hm01b0_cfg_t *psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes){
    if(psCfg && psCfg->interface && psCfg->interface->read) return psCfg->interface->read(psCfg, ui16Reg, pui8Value, ui32NumBytes, psCfg->interface->arg);
    return HM01B0_ERR_UNIMPLEMENTED;
}

hm01b0_status_e hm01b0_load_script(hm01b0_cfg_t *psCfg, hm_script_t *psScript, uint32_t ui32ScriptCmdNum){
    if(psScript == NULL || psCfg == NULL) return HM01B0_ERR_PARAMS;
    for(uint32_t i=0;i<ui32ScriptCmdNum;i++){
        hm_script_t *cmd = &psScript[i];
        hm01b0_status_e st = hm01b0_write_reg(psCfg, cmd->ui16Reg, &cmd->ui8Val, 1);
        if(st != HM01B0_ERR_OK) return st;
    }
    return HM01B0_ERR_OK;
}

hm01b0_status_e hm01b0_init_system(hm01b0_cfg_t *psCfg, hm_script_t *psScript, uint32_t ui32ScriptCmdNum){
    return hm01b0_load_script(psCfg, psScript, ui32ScriptCmdNum);
}

hm01b0_status_e hm01b0_get_modelid(hm01b0_cfg_t *psCfg, uint16_t *pui16MID){
    if(pui16MID == NULL) return HM01B0_ERR_PARAMS;
    uint8_t hi = 0, lo = 0;
    hm01b0_status_e st = hm01b0_read_reg(psCfg, 0x0000, &hi, 1);
    if(st != HM01B0_ERR_OK) return st;
    st = hm01b0_read_reg(psCfg, 0x0001, &lo, 1);
    if(st != HM01B0_ERR_OK) return st;
    *pui16MID = ((uint16_t)hi << 8) | lo;
    return HM01B0_ERR_OK;
}

hm01b0_status_e hm01b0_mclk_enable(hm01b0_cfg_t *psCfg){
    if(psCfg && psCfg->interface && psCfg->interface->mclk) return psCfg->interface->mclk(psCfg, true, psCfg->interface->arg);
    return HM01B0_ERR_UNIMPLEMENTED;
}

hm01b0_status_e hm01b0_init_if(hm01b0_cfg_t *psCfg){
    if(psCfg && psCfg->interface && psCfg->interface->init) return psCfg->interface->init(psCfg, psCfg->interface->arg);
    return HM01B0_ERR_UNIMPLEMENTED;
}

hm01b0_status_e hm01b0_deinit_if(hm01b0_cfg_t *psCfg){
    if(psCfg && psCfg->interface && psCfg->interface->deinit) return psCfg->interface->deinit(psCfg, psCfg->interface->arg);
    return HM01B0_ERR_UNIMPLEMENTED;
}
