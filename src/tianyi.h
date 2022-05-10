#ifndef __TIANYI_H__
#define __TIANYI_H__

#include <stdint.h>
#include <stdarg.h>
#include <param/param_types.h>
#include <csp/csp.h>
#include <conf_uv.h>

#if SUPPORT_LFS
#include "lfs.h"
#endif

#define MAX_USART_RX_LEN	128
#define MAX_ARGS		10
#define MAGIC_55		0x55
#define I2C_LOOP		10000
#define USART_LOOP		10000
#define FLASH_LOOP		1000000
#define IRQ2NVIC_PRIOR(x)	((x)<<4)
#define DELAY_TIMER_ID_MIN	0x100000
#define DELAY_TIMER_ID_MAX	0x200000
#define WATCH_TIMER_ID_MIN	0x1000
#define WATCH_TIMER_ID_MAX	0x2000
#define UV_CSP_PING_TIMER_ID	0x6000
#define CYCLES_PER_US           20
#define FLASH_BASEADDR		0x08000000
#define FLASH_PAGESIZE		4096 /* 4K Dual Bank*/
#define FLASH_SIZE_REG		MMIO16(0x1FFF75E0)
#define DEFAULT_UV_NODE		5
#define DEFAULT_EPS_NODE	2
#define DEFAULT_OBC_NODE	1

#define SPI_RESERVED_SECTOR	16

#define SPI_MAIN_CFG_SECTOR	0
#define SPI_BACKUP_CFG_SECTOR	8

#define FLASH_FS_ADDR		0x08100000
#define FLASH_MAIN_CFG_ADDR	0x081ff000 /* last page for 2M */
#define FLASH_BACKUP_CFG_ADDR	0x081fc000 /* last 4 page for 2M */

#define APP_ADDRESS		0x20000000

#define CSP_PORT_RPARAM		7
#define CSP_PORT_FTP		9

#define EPS_PORT_HARDRESET	20
#define EPS_PORT_RESET_WDT_GND	16
#define EPS_PORT_RESET_COUNTERS	15
#define EPS_PORT_SET_OUTPUT	9 /* SAME AS FTP, IGNORED */
#define EPS_PORT_SET_SINGLE_OUTPUT	10
#define EPS_PORT_HK		8

#define IWDG_TS			6000

#define FLASH_SAVE_INTERVAL	7200

struct g_uv_sdr_config
{
	uint32_t freq;
	uint32_t baud;
	uint16_t guard;

	uint8_t csp_hmac, csp_rs, csp_crc, csp_rand;
	uint8_t csp_hmac_key[17];
};

#define IO_SETTINGS_COUNT     64
#define IO_SETTINGS_ON_COUNT  12
#define IO_SETTINGS_OFF_COUNT 52
 
struct g_params_config
{
	uint32_t base; /* flash base address */
	uint32_t backup_addr; /* flash backup option address */
	int len; /* length of parameters table */
	int count; /* count of parameters */

	void *buffer;
	void *tmp;
	void *orig; /* buffer to store original spi flash data */

	const struct g_param_table *table;

	uint8_t csp_node, kiss_usart, gosh_usart, mode;
	uint8_t ram_image, version;
	//uint8_t debug_mask;

	uint8_t cfg_read_err_cnt, cfg_write_err_cnt;

	uint32_t baudrate;
	char swload_image[32];
	uint8_t swload_count;
	uint16_t poweron_cnt;
	uint8_t boot_cause;

	uint32_t gnd_wdt, gnd_wdt_left;
	uint8_t gnd_wdt_kick_cnt;

	uint32_t flash_fs_pos;

	/* UV */
	struct g_uv_sdr_config rx, tx;

	uint32_t tx_inhibit;
	uint8_t tx_pwr;

	uint32_t tx_count, rx_count;
	uint32_t tx_bytes, rx_bytes;

	int16_t bgnd_rssi, last_rssi, last_rferr;
	uint32_t tot_tx_count, tot_rx_count;
	uint32_t tot_tx_bytes, tot_rx_bytes;
};

struct cmd_args {
	char ptr[MAX_USART_RX_LEN];
	char *argv[MAX_ARGS];
	int argc, timer_id, interval, enabled;
};

struct table_offset {
	uint16_t start, end;
};

#define MAX_RPARAM_TABLE	6

typedef int (*timer_action_handler)(void *);

typedef void (*gosh_cmd_handler)(char **, int);

/* USART */
uint32_t swapu32(uint32_t);
void usart_send_hex(uint32_t, uint8_t *, int, int, int);
void usart_send_string(uint32_t, uint8_t *);
void setup_usart_speed(uint32_t, uint32_t, int);
void append_usart_rx_queue(uint32_t, char);
int start_usart_tasks(void);
void setup_gosh_callback(gosh_cmd_handler);
void generic_usart_handler(void *);
void handle_console_input(char, void *);

/* TIMER */
int unregister_timer_action_handler(uint32_t);
int register_timer_ts_action_handler(uint32_t, uint32_t, int, timer_action_handler, void *, uint32_t, uint32_t);
int unregister_timer_section_action_handler(uint32_t, uint32_t);
void recalibrate_timer_action_vars(void);
void uv_timer_service(void *);

/* FLASH */
int flash_timeout_wait_for_last_operation(void);
int flash_write_double_word(uint32_t, uint32_t, uint32_t);
int flash_reset_page(uint32_t);
int flash_reset_all_pages(void);
int flash_write_data(uint32_t, char *, int);
int flash_read_data(uint32_t, int, char *);

int write_file(char *, int, char *, int);
char * read_file(char *, int *);
int printf_(const char *format, ...);

#define std_printf printf_

void task_ftp(void *);
void uv_hook_init(void);
int watcher_handler(void *p);
int init_can1(void);
uint32_t make_can_dest(uint8_t);

extern uint8_t this_can_id, log_mask;

extern TickType_t last_timer_tick;

#define log_info(fmt, args...) \
	printf_("[INF] (%s.%d.%s %d.%03d) " fmt, __FILE__, __LINE__, __FUNCTION__, last_timer_tick / 1000, last_timer_tick % 1000, ##args);

#ifndef TIANYI_RELEASE
#define LOG_INFO(fmt, args...) \
	do { if (log_mask & 0x01) printf_("[INF] (%s.%d.%s %d.%03d) " fmt, __FILE__, __LINE__, __FUNCTION__, last_timer_tick / 1000, last_timer_tick % 1000, ##args); } while(0)
#define ERROR_LOG(fmt, args...) \
	do { if (log_mask & 0x08) printf_("[ERR] (%s.%d.%s %d.%03d) " fmt, __FILE__, __LINE__, __FUNCTION__, last_timer_tick / 1000, last_timer_tick % 1000, ##args); } while(0)
#define LOG_BPX(fmt, args...) \
	do { if (log_mask & 0x10) printf_("[BPX] (%s.%d.%s %d.%03d) " fmt, __FILE__, __LINE__, __FUNCTION__, last_timer_tick / 1000, last_timer_tick % 1000, ##args); } while(0)
#else
#define ERROR_LOG(fmt, args...) do {;} while (0)
#define LOG_INFO(fmt, args...) do {;} while (0)
#define LOG_BPX(fmt, args...) do {;} while (0)
#endif

struct g_param_table {
	uint16_t addr;
	param_type_t type;
	uint8_t size;
	uint8_t flags;
	void *p;
	char name[MAX_PARAM_NAME_LEN];
};

void rparam_service_handler(csp_conn_t *, csp_packet_t *);
void handle_rparam_table_cmd(csp_conn_t *, csp_packet_t *, rparam_query *);
void handle_rparam_get_cmd(csp_conn_t *, csp_packet_t *, rparam_query *);
void handle_rparam_set_cmd(csp_conn_t *, csp_packet_t *, rparam_query *);
void handle_rparam_copy_cmd(csp_conn_t *, csp_packet_t *, rparam_query *);

#define CSP_MTU 250

void boot_loader(char *);

int lfs_copy_file(char *, char *);

enum cfp_ext_t {
	CFP_RESET = 0,
	CFP_SINGLE = 1,
	CFP_BEGIN = 2,
	CFP_MORE = 3,
	CFP_TIMESYNC = 4
};

int libcsp_can_init(uint8_t);

void usart_timeout_putc(uint32_t, char);
void console_puts(char *);
void kiss_usart_putc(char);
void console_putc(char);

void my_delay_us(uint32_t);

int read_params_config(void *);
int save_params_config(void *);
int check_params_config(void *, int);
int backup_params_config(void *);
int restore_params_config(void *, int);

void print_hex(uint8_t *, int);
void print_ios(uint8_t *, int);
void hex_dump(char *, int);

void param_set_data(char *, int, char *, int);

void auto_calibrate_us_cnt(void);
int init_spi_flash(void);
void check_pvd_status(void);
void setup_pvd(void);
void start_usart_task(void);
uint8_t get_boot_cause(void);
void list_params_config(struct g_params_config *, int, int, uint8_t, uint8_t);
int set_params_value(struct g_params_config *, char *, char *);
char * lfs_err_str(int);
void lfs_operation(int, char **);

#define CSP_PORT_TIANYI         10
#define TY_GROUND_STATION       10

void append_amateur_msg(char *msg, int len);

void spi_test_all(int, char **);
uint32_t random_(void);
void setup_rng(void);

/* 80 BYTES */
struct __attribute__ ((__packed__)) uv_hk_t {
	uint8_t version; /* 4 BYTES */
	uint8_t cfg_rd_err_cnt;
	uint8_t cfg_wr_err_cnt;
	uint8_t spi_err_cnt;

	int8_t pa_temp;
	uint8_t boot_cause;
	uint8_t app_flag;
	uint8_t gnd_wdt_kick_cnt;

	uint8_t pa_power, tx_err_cnt;
	uint16_t poweron_cnt;

	uint32_t gnd_wdt_left;
	uint32_t uptime;

	/* AX5043 */
	uint32_t csp_ax5043_rx, csp_ax5043_tx;
	uint32_t csp_ax5043_rx_bytes, csp_ax5043_tx_bytes;
	uint32_t csp_ax5043_rx_error, csp_ax5043_tx_error;

	uint32_t csp_ax5043_tx_duty;
	int16_t last_rssi, last_rferr;

	/* CAN */
	uint32_t csp_can_rx, csp_can_tx;
	uint32_t csp_can_rx_bytes, csp_can_tx_bytes;
	uint32_t csp_can_rx_error, csp_can_tx_error;

	/* CRC */
	uint32_t crc;
};

int read_tmp102(uint32_t, uint16_t, uint32_t, uint16_t, uint8_t, int8_t *);

void dump_call_stack(unsigned int *);
void print_csp_status(void);
void check_bin_file(int, int, uint32_t *, uint32_t *);
int init_spi_flash(void);
int write_check_spiflash(int);
int spi_flash_read_buffer(uint8_t *, uint32_t, int);
int spi_flash_write_buffer(uint8_t *, uint32_t, int);
int spi_flash_erase_sector(uint32_t);
void uv_hw_wdg_reset(void);
void cpu_reset(void);

int decode_golay24(uint32_t *);
int decode_ax100(uint8_t *, int, int, int, uint8_t);
int csp_hmac_memory(const uint8_t *, uint32_t, const uint8_t *, uint32_t, uint8_t *);
void csp_sha1_memory(const uint8_t *, uint32_t, uint8_t *);
int encode_ax100(uint8_t *, uint8_t, uint8_t *, int *, int, int, int, int, uint8_t *);

#if SUPPORT_LFS
void flash_lfs_init(struct lfs_config *, int);
void spi_flash_lfs_init(struct lfs_config *, int);
void boot_loader(char *);
#endif

#endif
