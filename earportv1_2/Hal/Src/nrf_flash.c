#include "stdint.h"

#include "nrf_flash.h"

#include "nrf_fstorage.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#include "nrf_log.h"
#include "nrf_soc.h"
#include "nrf_delay.h"


static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            // NRF_LOG_DEBUG("--> Event received: wrote %d bytes at address 0x%x.",p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            // NRF_LOG_DEBUG("--> Event received: erased %d page from address 0x%x.",p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = FLASH_START_ADDR,
    .end_addr   = FLASH_END_ADDR,
};


static void nrf_flash_wait_ready(nrf_fstorage_t const* pFstorageHandle)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(pFstorageHandle))
    {
    }
}


int hal_flash_init(void)
{
    //内部flash初始化.
    int ret = 0;
    nrf_fstorage_api_t * p_fs_api = &nrf_fstorage_sd;
    ret = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
    return ret;
}

FLASH_ERR_EU hal_flash_read(uint32_t addr, void* para, int len)
{
    uint8_t ret = FLASH_SUCCESS;
    if ((NULL == para) || (0 == len))
    {
        return ARG_ERR;
    }

    if ((ret = nrf_fstorage_read(&fstorage, addr, para, len)) != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read error,%d", ret);
        return FLASH_READ_ERR;
    }

    return ret;
}

FLASH_ERR_EU hal_flash_write(uint32_t addr, void* para, int len)
{
    uint8_t ret = FLASH_SUCCESS;

    if ((NULL == para) || (0 == len))
    {
        return ARG_ERR;
    }

    if ((ret = nrf_fstorage_write(&fstorage, addr, para, len, NULL)) != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_write error,%d", ret);
        return FLASH_WRITE_ERR;
    }
    nrf_flash_wait_ready(&fstorage);
    return ret;
}

FLASH_ERR_EU hal_flash_erase(uint32_t addr)
{
    uint8_t ret = FLASH_SUCCESS;

    if ((ret = nrf_fstorage_erase(&fstorage, addr, 1, NULL)) != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_erase error,%d", ret);
        return FLASH_ERASE_ERR;
    }
    nrf_flash_wait_ready(&fstorage);
    return ret;
}



