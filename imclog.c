#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <argp.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <signal.h>

#define AST_SPI_CTRL2_BASE			0x1e631000
#define AST_SPI2_FLASH_PHYS_ADDR	0x38000000
#define AST_SPI_CONFIG_REG			0x00
#define SPI2_CE0_CTL_REG			0x10

#define SPI2_CE0_EN_BIT				0x00010000	/* bit16 */
#define SPI2_CE0_HI					0x00000004	/* bit2 */
#define SPI2_CMD_USER_MODE			0x00000003	/* bit[0:1] */
#define AST_HW_strap_table_BASE		0x1e6e2070
#define IMC_LOG_LOCATION		"/tmp/imc_log"
#define GPIOIMCRDY "/sys/class/gpio/gpio395/value"
void *ast_spi_ctrl2_virt;
void *ast_spi_ctrl2_mem_virt;
#if 0
void *ast_spi_hw_strap_table_mem_virt;
#endif
uint32_t readl(void *addr)
{
	asm volatile("" : : : "memory");
	return *(volatile uint32_t *)addr;
}

uint32_t readb(void *addr)
{
	asm volatile("" : : : "memory");
	return *(volatile uint8_t *)addr;
}

void writel(uint32_t val, void *addr)
{
	asm volatile("" : : : "memory");
	*(volatile uint32_t *)addr = val;
}

void writeb(uint8_t val, void *addr)
{
	asm volatile("" : : : "memory");
	*(volatile uint8_t *)addr = val;
}

int init_spi_ctrl_mem_virt(void)
{
	int mem_fd;

	mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mem_fd < 0) {
		perror("Unable to open /dev/mem");
		return -1;
	}

	ast_spi_ctrl2_virt = mmap(NULL, getpagesize(), PROT_READ | PROT_WRITE,
							  MAP_SHARED, mem_fd, AST_SPI_CTRL2_BASE);
	if (ast_spi_ctrl2_virt == MAP_FAILED) {
		perror("Unable to map SPI_CFG register memory");
		return -1;
	}

	ast_spi_ctrl2_mem_virt = mmap(NULL, getpagesize(), PROT_READ | PROT_WRITE,
								  MAP_SHARED, mem_fd, AST_SPI2_FLASH_PHYS_ADDR);
	if (ast_spi_ctrl2_mem_virt == MAP_FAILED) {
		perror("Unable to map SPI2_MEM_ADDR register memory");
		return -1;
	}
	close(mem_fd);
	return 0;
}

void enable_spi_wirte(void)
{
	int32_t ret_val;

	ret_val = readl(ast_spi_ctrl2_virt + AST_SPI_CONFIG_REG);
	ret_val |= SPI2_CE0_EN_BIT;
	writel(ret_val, ast_spi_ctrl2_virt + AST_SPI_CONFIG_REG);
}

void active_spi2_cs0(void)
{
	int32_t ret_val;

	ret_val = readl(ast_spi_ctrl2_virt + SPI2_CE0_CTL_REG);
	ret_val |= SPI2_CMD_USER_MODE;
	ret_val &= ~(SPI2_CE0_HI);
	writel(ret_val, ast_spi_ctrl2_virt + SPI2_CE0_CTL_REG);
	return;
}

void deactive_spi2_cs0(void)
{
	int32_t ret_val;

	ret_val = readl(ast_spi_ctrl2_virt + SPI2_CE0_CTL_REG);
	ret_val |= SPI2_CE0_HI | SPI2_CMD_USER_MODE;
	writel(ret_val, ast_spi_ctrl2_virt + SPI2_CE0_CTL_REG);
	return;
}

void spi2_write_read_transfer(unsigned char *write_buf, unsigned int write_len,
							  unsigned char *read_buf, unsigned int read_len)
{
	int i = 0;
	active_spi2_cs0();
	while(write_len) {
		writeb(write_buf[i],ast_spi_ctrl2_mem_virt);
		i++;
		write_len--;
	}
	i = 0;
	while(read_len) {
		read_buf[i] = readb(ast_spi_ctrl2_mem_virt);
		i++;
		read_len--;
	}
	deactive_spi2_cs0();
	return;
}
int write_imc_log(void)
{
        unsigned char write_buf[16];
        unsigned char read_buf[256];
        unsigned int write_len;
        unsigned int read_len;
        int i = 0,j =0;
        int32_t ret_val;
        FILE *fp;

        memset(write_buf, 0x00, sizeof(write_buf));
        memset(read_buf, 0x00, sizeof(read_buf));
        write_buf[0] = 0x0b;
        write_buf[1] = 0x03;
        write_buf[2] = 0xa0;
        write_buf[3] = 0x00;
        write_len = 4;
        read_len = 0xff;
        fp = fopen(IMC_LOG_LOCATION, "wb");
        for(i = 0xa0; i < 0xf0; i++)
        {
                spi2_write_read_transfer(write_buf, write_len, read_buf, read_len);
                fprintf(fp, "%s", read_buf+4);
                memset(read_buf, 0x00, sizeof(read_buf));
                write_buf[2]++;
        }
        fclose(fp);
        return 0;

}

int get_imc_ready(void)
{
        FILE *fp;
        char config[8] = {0};//use to transform string to int
        int imcready = 0;

        fp = fopen(GPIOIMCRDY, "r");
        if (fp == NULL) {
                return 0;
        }
        fgets(config,8,fp);
        imcready = atoi(config);
        fclose(fp);
        return imcready;
}

int main()
{
	if (init_spi_ctrl_mem_virt()) exit(0);

	enable_spi_wirte();

	while(1){
		if(get_imc_ready() == 1)
			write_imc_log();
		sleep(10);
	}
	return 0;
}
