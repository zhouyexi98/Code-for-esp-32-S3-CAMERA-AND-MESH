#ifndef HM01B0_H
#define HM01B0_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HM01B0_DEFAULT_ADDRESS (0x24)

// QVGA Resolution
#define HM01B0_PIXEL_X_NUM (320)
#define HM01B0_PIXEL_Y_NUM (240)

typedef enum {
    HM01B0_ERR_OK = 0x00,
    HM01B0_ERR,
    HM01B0_ERR_I2C,
    HM01B0_ERR_MODE,
    HM01B0_ERR_AE_NOT_CONVERGED,
    HM01B0_ERR_MCLK,
    HM01B0_ERR_INIT,
    HM01B0_ERR_DEINIT,
    HM01B0_ERR_PARAMS,
    HM01B0_ERR_UNIMPLEMENTED,
    HM01B0_NUM_ERR
} hm01b0_status_e;

typedef struct _hm01b0_cfg_t hm01b0_cfg_t;

typedef hm01b0_status_e (*hm01b0_if_fn_t)(hm01b0_cfg_t* psCfg, void* arg);
typedef hm01b0_status_e (*hm01b0_if_i2c_fn_t)(hm01b0_cfg_t* psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes, void* arg);
typedef hm01b0_status_e (*hm01b0_if_on_off_fn_t)(hm01b0_cfg_t* psCfg, bool enable, void* arg);

typedef struct {
    hm01b0_if_fn_t      init;
    hm01b0_if_i2c_fn_t  write;
    hm01b0_if_i2c_fn_t  read;
    hm01b0_if_on_off_fn_t mclk;
    hm01b0_if_on_off_fn_t trig;
    hm01b0_if_fn_t        deinit;
    void*                 arg;
} hm01b0_if_t;

typedef struct {
    uint16_t ui16Reg;
    uint8_t ui8Val;
} hm_script_t;

struct _hm01b0_cfg_t {
    hm01b0_if_t* interface;
};

// core API used by higher-level code
hm01b0_status_e hm01b0_write_reg(hm01b0_cfg_t *psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes);
hm01b0_status_e hm01b0_read_reg(hm01b0_cfg_t *psCfg, uint16_t ui16Reg, uint8_t *pui8Value, uint32_t ui32NumBytes);
hm01b0_status_e hm01b0_load_script(hm01b0_cfg_t *psCfg, hm_script_t *psScript, uint32_t ui32ScriptCmdNum);
hm01b0_status_e hm01b0_init_system(hm01b0_cfg_t *psCfg, hm_script_t *psScript, uint32_t ui32ScriptCmdNum);
hm01b0_status_e hm01b0_get_modelid(hm01b0_cfg_t *psCfg, uint16_t *pui16MID);
hm01b0_status_e hm01b0_mclk_enable(hm01b0_cfg_t *psCfg);
hm01b0_status_e hm01b0_init_if(hm01b0_cfg_t *psCfg);
hm01b0_status_e hm01b0_deinit_if(hm01b0_cfg_t *psCfg);

#ifdef __cplusplus
}
#endif

#endif // HM01B0_H
