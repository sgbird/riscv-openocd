#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include "target/riscv/riscv.h"

#define DPUSPI_ENDIAN_MSB          0
#define DPUSPI_ENDIAN_LSB          1

#define READID                      0
#define CHIPERASE                   1
#define CHIPRSTATUS                 2
#define CHIPWRIEN                   3

#define DPUSPI_FLASH_OFF            0x1e00000
#define DPUSPI_CONT_OFF             0x1e08000
#define CTRL_SS_REG                 0x8

#define BIT(n)      (1 << n)
#define WIP         BIT(0)

#define IN                      0x1
#define OUT                     0x0

struct dpuspi_flash_bank {
	int probed;
	target_addr_t mem_base;
	target_addr_t ctrl_base;
	target_addr_t flash_base;
	const struct flash_device *dev;
};

struct flash_cmd {
    uint8_t opcode;
    uint8_t dummy_cycle;
};

static struct flash_cmd dpuspi_flash_commands[] = {
    [READID] = {0x90, 0x3},
    [CHIPERASE] = {0x60, 0x0},
    [CHIPRSTATUS] = {0x05, 0x0},
    [CHIPWRIEN] = {0x06, 0x0},
};

FLASH_BANK_COMMAND_HANDLER(dpuspi_flash_bank_command)
{
	struct dpuspi_flash_bank *dpuspi_info;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	dpuspi_info = malloc(sizeof(struct dpuspi_flash_bank));
	if (dpuspi_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = dpuspi_info;
	dpuspi_info->probed = 0;
	dpuspi_info->ctrl_base = 0;
	if (CMD_ARGC >= 7) {
		COMMAND_PARSE_ADDRESS(CMD_ARGV[6], dpuspi_info->mem_base);
        dpuspi_info->ctrl_base = DPUSPI_CONT_OFF + dpuspi_info->mem_base;
        dpuspi_info->flash_base = DPUSPI_FLASH_OFF + dpuspi_info->mem_base;
	}

	return ERROR_OK;
}

#define MAKECMD(len, opcode, dummy_cycle)    \
            ((len << 13) | (dummy_cycle << 10) | (opcode << 2))

static int dpuspi_flashcmd(struct flash_bank *bank, uint8_t index, uint32_t *value, bool in)
{
	struct target *target = bank->target;
	struct dpuspi_flash_bank *dpuspi_info = bank->driver_priv;
    struct flash_cmd *cmd = &(dpuspi_flash_commands[index]);
    uint8_t opcode = cmd->opcode;
    uint8_t dummy_cycle = cmd->dummy_cycle;
    uint32_t data;
    int retval;

    if (in) {
        data = dpuspi_info->flash_base + MAKECMD(3, opcode, dummy_cycle);
        retval = target_read_u32(target, data, value);
        if (retval != ERROR_OK) {
            return retval;
        }
    } else {
        data = dpuspi_info->flash_base + MAKECMD(0, opcode, dummy_cycle);
        retval = target_write_u32(target, data, 0);
    }
	return ERROR_OK;
}

static int dpuspi_wait_wip(struct flash_bank *bank)
{
    int retval;
	int64_t start = timeval_ms();

	while (1) {
		uint32_t val;
        retval = dpuspi_flashcmd(bank, CHIPRSTATUS, &val, IN);
        if (retval != ERROR_OK)
            return retval;
		if (!(val & WIP))
			break;
		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("Wait too long for WIP\n");
			return ERROR_TARGET_TIMEOUT;
		}
	}
    
    return ERROR_OK;
}

static void dpuspi_cs(struct flash_bank *bank, bool en)
{
	struct target *target = bank->target;
	struct dpuspi_flash_bank *dpuspi_info = bank->driver_priv;
    int val = (en) ? 1 : 2;
    
    target_write_u8(target, dpuspi_info->ctrl_base + CTRL_SS_REG, val);
}

__attribute__((unused)) static int dpuspi_erase_sector(struct flash_bank *bank, int sector)
{
	struct dpuspi_flash_bank *dpuspi_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;
    uint32_t addr, data;

    retval = dpuspi_flashcmd(bank, CHIPWRIEN, 0, OUT);
	if (retval != ERROR_OK)
		return retval;

	sector = bank->sectors[sector].offset;

    addr = dpuspi_info->flash_base + (3 << 13);
    data = (dpuspi_info->dev->erase_cmd << 24) | sector;

    dpuspi_cs(bank, 1);
    target_write_u32(target, addr, data);
    dpuspi_cs(bank, 0);

    retval = dpuspi_wait_wip(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int dpuspi_erase_chip(struct flash_bank *bank)
{
    int retval;

    retval = dpuspi_flashcmd(bank, CHIPWRIEN, 0, OUT);
    retval = dpuspi_flashcmd(bank, CHIPERASE, 0, OUT);

    return retval;
}

static int dpuspi_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct dpuspi_flash_bank *dpuspi_info = bank->driver_priv;
	int retval = ERROR_OK;
	int sector;

	LOG_DEBUG("%s: from sector %d to sector %d", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(dpuspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	if (dpuspi_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;


	/* poll WIP */
    retval = dpuspi_wait_wip(bank);
	if (retval != ERROR_OK)
		goto done;
/*
	for (sector = first; sector <= last; sector++) {
		retval = dpuspi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			goto done;
		keep_alive();
	}
*/
    retval = dpuspi_erase_chip(bank);
    if (retval != ERROR_OK)
        goto done;

    retval = dpuspi_wait_wip(bank);

done:
	return retval;
}

static int dpuspi_protect(struct flash_bank *bank, int set,
		int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int dpuspi_write_mem(struct flash_bank *bank, uint32_t addr, uint32_t value, size_t size)
{
	struct target *target = bank->target;
	struct dpuspi_flash_bank *dpuspi_info = bank->driver_priv;

    dpuspi_flashcmd(bank, CHIPWRIEN, 0, OUT);
    //dpuspi_cs(bank, 1);
    if (size == 4)
        target_write_u32(target, dpuspi_info->mem_base + addr, value);
    else 
        target_write_u8(target, dpuspi_info->mem_base + addr, (uint8_t)value);
    //dpuspi_cs(bank, 0);
    dpuspi_wait_wip(bank);

    return ERROR_OK;
}

static int slow_dpuspi_write_buffer(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t len)
{
	uint32_t cnt_word = len / 4;
    uint32_t cnt_byte = len % 4;
    uint32_t i, j, tmp = 0;

	for (i = 0; i < cnt_word; i++) {
		if (dpuspi_write_mem(bank, offset + (i * 4), *(uint32_t *)(buffer + (i * 4)), 4) != ERROR_OK)
			return ERROR_FAIL;
	}

    // this controller only support read/write in word
    if (cnt_byte) {
        for (j = 0; j < cnt_byte; j++) {
            tmp |= *(buffer + (i * 4) + j) << (8 * j);
        }
        if (dpuspi_write_mem(bank, offset + (i * 4), tmp, 4) != ERROR_OK)
            return ERROR_FAIL;
    }

	keep_alive();

	return ERROR_OK;
}

// TODO: Implement a standalone application to run directly on CPU
static int dpuspi_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
    slow_dpuspi_write_buffer(bank, buffer, offset, count);
	return ERROR_OK;
}

static int dpuspi_read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
    int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* poll WIP */
    retval = dpuspi_wait_wip(bank);
	if (retval != ERROR_OK)
		return retval;

    retval = dpuspi_flashcmd(bank, READID, id, IN); 

	return retval;
}

static int dpuspi_probe(struct flash_bank *bank)
{
	//struct target *target = bank->target;
	struct dpuspi_flash_bank *dpuspi_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0;
	uint32_t sectorsize;
    int retval;

	if (dpuspi_info->probed)
		free(bank->sectors);
	dpuspi_info->probed = 0;

	if (dpuspi_info->ctrl_base == 0) {
		LOG_ERROR("Invalid DPUSPI controller base address\n");
	} else {
	  LOG_DEBUG("Assuming DPUSPI as specified at address " TARGET_ADDR_FMT
			  " with ctrl at " TARGET_ADDR_FMT, dpuspi_info->ctrl_base,
			  bank->base);
	}

	/* read and decode flash ID; returns in SW mode */
	//if (dpuspi_write_reg(bank, DPUSPI_REG_TXCTRL, DPUSPI_TXWM(1)) != ERROR_OK)
	//	return ERROR_FAIL;

	retval = dpuspi_read_flash_id(bank, &id);
	if (retval != ERROR_OK)
		return retval;

    id = 0x1640ef;  // TODO: temp patch
	dpuspi_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			dpuspi_info->dev = p;
			break;
		}

	if (!dpuspi_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
			dpuspi_info->dev->name, dpuspi_info->dev->device_id);

	/* Set correct size value */
	bank->size = dpuspi_info->dev->size_in_bytes;

	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = dpuspi_info->dev->sectorsize ?
		dpuspi_info->dev->sectorsize : dpuspi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = dpuspi_info->dev->size_in_bytes / sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	dpuspi_info->probed = 1;
	return ERROR_OK;
}

static int dpuspi_auto_probe(struct flash_bank *bank)
{
	struct dpuspi_flash_bank *dpuspi_info = bank->driver_priv;
	if (dpuspi_info->probed)
		return ERROR_OK;
	return dpuspi_probe(bank);
}

static int dpuspi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_dpuspi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct dpuspi_flash_bank *dpuspi_info = bank->driver_priv;

	if (!(dpuspi_info->probed)) {
		snprintf(buf, buf_size,
				"\nDPUSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nDPUSPI flash information:\n"
			"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
			dpuspi_info->dev->name, dpuspi_info->dev->device_id);

	return ERROR_OK;
}

const struct flash_driver dpuspi_flash = {
	.name = "dpuspi",
	.flash_bank_command = dpuspi_flash_bank_command,
	.erase = dpuspi_erase,
	.protect = dpuspi_protect,
	.write = dpuspi_write,
	.read = default_flash_read,
	.probe = dpuspi_probe,
	.auto_probe = dpuspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = dpuspi_protect_check,
	.info = get_dpuspi_info,
	.free_driver_priv = default_flash_free_driver_priv
};
