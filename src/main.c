#include <errno.h>
#include <stdio.h>
#include "libopencm3/stm32/gpio.h"
#include <libopencm3/stm32/usart.h>
#include "libopencm3/stm32/spi.h"
#include "libopencm3/stm32/exti.h"
#include "libopencm3/cm3/nvic.h"
#include <libopencm3/stm32/timer.h>

#include "mcp2515.h"

/* SPI1 */
#define RCC_SPI_PORT (GPIOA)
#define RCC_SPI_CS_PORT (GPIOB)

#define SPI_SCK_PIN (GPIO5)  /* Arduino-D13 pin. */
#define SPI_MISO_PIN (GPIO6) /* Arduino-D12 pin. */
#define SPI_MOSI_PIN (GPIO7) /* Arduino-D11 pin. */
#define SPI_CS_PIN (GPIO6)   /* Arduino-D10 pin. */
#define SPI_AF (GPIO_AF5)

#define INT_PORT (GPIOC)
#define INT_PIN (GPIO7) /* Arduino-D9 pin. */
#define INT_EXTI (EXTI7)
#define INT_IRQ (NVIC_EXTI9_5_IRQ)

/* USART */
#define USART_BAUDRATE (115200)
#define RCC_USART_TX_GPIO (RCC_GPIOA)
#define GPIO_USART_TX_PORT (GPIOA)
#define GPIO_USART_TX_PIN (GPIO2) /* Arduino-D1. */
#define GPIO_USART_RX_PORT (GPIOA)
#define GPIO_USART_RX_PIN (GPIO3) /* Arduino-D1. */
#define GPIO_USART_AF (GPIO_AF7)  /* Table-11 in DS10693 */

// USART命令緩衝區
#define CMD_BUFFER_SIZE 64
char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_index = 0;
volatile bool cmd_ready = false;

#define CanID (0x01 )

volatile bool Tx_complete = false;

void mcp2515_send_can_frame(can_frame_t *can_frame);

can_frame_t rx_frame;
const can_frame_t *last_tx_frame_ptr = NULL;
mcp2515_handle_t mcp2515;

void rcc_setup(void)
{
  /*
  Using 168MHz as system frequency with 8MHz HSE source
  ahb_frequency  = 168MHz,
  apb1_frequency = 84MHz,
  apb2_frequency = 168MHz,
*/
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_USART_TX_GPIO);
  rcc_periph_clock_enable(RCC_USART2);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
//   rcc_periph_clock_enable(RCC_GPIOC);
}

void usart_setup(void)
{
  /* Set USART-Tx pin to alternate function. */
  gpio_mode_setup(GPIO_USART_TX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART_TX_PIN);
  gpio_set_af(GPIO_USART_TX_PORT,
              GPIO_USART_AF,
              GPIO_USART_TX_PIN);
  /*Set USART-Rx*/
  gpio_mode_setup(GPIO_USART_RX_PORT,
              GPIO_MODE_AF,
              GPIO_PUPD_NONE,      
              GPIO_USART_RX_PIN);        
  gpio_set_af(GPIO_USART_RX_PORT,
            GPIO_USART_AF,
            GPIO_USART_RX_PIN);

  /* Config USART params. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX); /* Tx_Rx mode. */

  usart_enable(USART2);

  /* 啟用 RX 中斷 */
  usart_enable_rx_interrupt(USART2);

    /* 啟用 NVIC 中斷 */
  nvic_enable_irq(NVIC_USART2_IRQ);
}


void spi_setup(void)
{
  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_SYSCFG); /* For EXTI. */

  gpio_mode_setup(RCC_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);
  gpio_set_af(RCC_SPI_PORT, SPI_AF, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);
  gpio_set_output_options(RCC_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SPI_SCK_PIN | SPI_MOSI_PIN);

  gpio_mode_setup(RCC_SPI_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_CS_PIN);
  gpio_set_output_options(RCC_SPI_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SPI_CS_PIN);
  gpio_set(RCC_SPI_CS_PORT, SPI_CS_PIN); /* Deselect. */

  uint32_t spi = SPI1;
  spi_disable(spi);
//   spi_reset(spi);
  spi_init_master(spi, SPI_CR1_BAUDRATE_FPCLK_DIV_32, /* Max 10 MHz. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,    /* CPOL=0. */
                  SPI_CR1_CPHA_CLK_TRANSITION_1,      /* CPHA=0. */
                  SPI_CR1_DFF_8BIT,                   /* Data frame 8-bit. */
                  SPI_CR1_MSBFIRST);                  /* Order: MSB First. */
  spi_set_full_duplex_mode(spi);
  spi_enable_software_slave_management(spi);
  spi_set_nss_high(spi);
  mcp2515_deselect();
  spi_enable(spi);
}

void int_pin_setup(void)
{
  gpio_mode_setup(INT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, INT_PIN);
  exti_select_source(INT_EXTI, INT_PORT);
  exti_set_trigger(INT_EXTI, EXTI_TRIGGER_FALLING);
  exti_enable_request(INT_EXTI);
  nvic_enable_irq(INT_IRQ);
}

void mcp2515_deselect()
{
  while (!(SPI_SR(SPI1) &
           SPI_SR_TXE)) /* Wait for 'Transmit buffer empty' flag to set. */
  {
  }
  while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }
  gpio_set(RCC_SPI_CS_PORT, SPI_CS_PIN); /* CS pin output high to deselect. */
}

void mcp2515_select()
{
  gpio_clear(RCC_SPI_CS_PORT, SPI_CS_PIN); /* CS pin output low to select. */
}

uint8_t mcp2515_spi_transfer(uint8_t data)
{
  uint16_t rec = spi_xfer(SPI1, data);
  return rec & 0xFF;
}

bool mcp2515_rx_buffer_full(const mcp2515_handle_t *mcp2515_handle)
{
  uint8_t int_flag = mcp2515_getInterrupts(mcp2515_handle);

  if ((int_flag & CANINTF_RX0IF) && (int_flag & CANINTF_RX1IF))
  {
    return true;
  }
  return false;
}

bool mcp2515_check_tx_err_msg(ERROR err_msg)
{
  if (err_msg == ERROR_OK)
  {
    return true;
  }
  else if (err_msg == ERROR_FAILTX)
  {
    printf("err發生\n");
    return false;
  }
  else if (err_msg == ERROR_ALLTXBUSY)
  {
    printf("err發生\n");
    return false;
  }
}

bool mcp2515_check_after_rx_tx(const mcp2515_handle_t *mcp2515_handle)
{
  uint8_t int_flag = mcp2515_getInterrupts(mcp2515_handle);
  bool tx_ok = true;

  if (int_flag & CANINTF_MERRF)
  {
    mcp2515_clearMERR_Interrupt(&mcp2515_handle);
    mcp2515_clearTXn_Interrupts(&mcp2515_handle);
    tx_ok = false;
  }

  if (int_flag & CANINTF_ERRIF)
  {
    uint8_t err_flag = mcp2515_getErrorFlags(mcp2515_handle);
    mcp2515_error_dealing(err_flag, mcp2515_handle);  // 處理MCP_EFLG暫存器的中斷
    mcp2515_clearERRIF_Interrupt(mcp2515_handle);     // 處理CANINTF bit5的ERROR中斷
    mcp2515_clearTXn_Interrupts(mcp2515_handle);
    tx_ok = false;
  }
  return tx_ok;
}

void mcp2515_print_can_frame(can_frame_t *can_frame)
{
  printf("ID = %lx ", can_frame->can_id);
  printf("DLC = %x \ndata: ", can_frame->can_dlc);
  /* print the data */
  for (int i = 0; i < can_frame->can_dlc; i++)
  {
    printf("%x ", can_frame->data[i]);
  }
  printf(" \r\n");
}

void mcp2515_error_dealing(uint8_t err, const mcp2515_handle_t *mcp2515_handle)
{
  if (err & EFLG_RX1OVR)
  {
    printf("EEFLG_RX1OVR\n");
    mcp2515_clearRXnOVRFlags(mcp2515_handle);  // 清除EFLG內的RX1OVR和RX0OVR(bit7和bit6)，bus off 那邊沒有處理
  }

  if (err & EFLG_RX0OVR)
  {
    printf("EFLG_RX0OVR\n");
    mcp2515_clearRXnOVRFlags(mcp2515_handle);  // 清除EFLG內的RX1OVR和RX0OVR(bit7和bit6)，bus off 那邊沒有處理
  }

  if (err & EFLG_TXBO)  // TX error num >= 255
  {
    printf("EFLG_TXBO  TX error num=%d\n", mcp2515_errorCountTX(mcp2515_handle));
  }
  if (err & EFLG_TXEP)  // TX error num >= 128
  {
    printf("EFLG_TXEP  TX error num=%d\n", mcp2515_errorCountTX(mcp2515_handle));
  }
  if (err & EFLG_RXEP)  // RX error num >= 128
  {
    printf("EFLG_RXEP  RX error num=%d\n", mcp2515_errorCountRX(mcp2515_handle));
  }
  if (err & EFLG_TXWAR)  // TX error num >= 96
  {
    printf("EFLG_TXWAR  TX error num=%d\n", mcp2515_errorCountTX(mcp2515_handle));
  }
  if (err & EFLG_RXWAR)  // RX error num >= 96
  {
    printf("EFLG_RXWAR  RX error num=%d\n", mcp2515_errorCountRX(mcp2515_handle));
  }

  if (err & EFLG_EWARN)  // When RX or TX error num >=96
  {
    printf("EFLG_EWARN\n");
    printf(" TX error num=%d\n", mcp2515_errorCountTX(mcp2515_handle));
    printf(" RX error num=%d\n", mcp2515_errorCountRX(mcp2515_handle));
  }
}

void mcp2515_decode_package(can_frame_t *frame)
{
    uint8_t dlc = frame->can_dlc;
    for (uint8_t i = 0; i < dlc; i++)
    {
        uint8_t data = frame->data[i];
        switch (i)
        {
        case 0:
                printf("模式 data[%d]=0x%02X\n", i, data);
            break;
        case 1:
                printf("角度 data[%d]=0x%02X\n", i, data);
            break;
        case 2:
                printf("電流限制 data[%d]=0x%02X\n",i,data);
            break;
        case 3:
                printf("速度限制 data[%d]=0x%02X\n",i,data);
            break;
        default:
            printf("其餘資料 data[%d]=0x%02X\n", i, data);
            break;
        }
    }
}

static void delay(uint32_t value)
{
  for (uint32_t i = 0; i < value; i++)
  {
    __asm__("nop"); /* Do nothing. */
  }
}

void mcp2515_delay_ms(uint32_t ms)
{
  for (; ms > 0; ms--)
  {
    for (uint16_t j = 0; j < 25000; j++)
    {
      __asm__("nop"); /* Do nothing. */
    }
  } 
}

bool mcp2515_init(const mcp2515_handle_t *mcp2515_handle)
{
  for (int i = 0; i < 10; i++)
  {
    if (mcp2515_reset(mcp2515_handle) != ERROR_OK)
    {
      printf("Mcp2515_Reset FAILED\n");
      continue;
    }
    if (mcp2515_setBitrate(mcp2515_handle, CAN_125KBPS, MCP_8MHZ) != ERROR_OK)
    {
      printf("Mcp2515_setBitrate FAILED\n");
      continue;
    }
    if (mcp2515_setNormalMode(mcp2515_handle) != ERROR_OK)
    {
      printf("Mcp2515_setNormalMode FAILED\n");
      continue;
    }
    return true;
  }
  return false;
}

can_frame_t can_frame = {
    .can_id = CanID,
    .can_dlc = 8,
    .data[0] = 0x80,              // 1 0 11 11 10
    .data[1] = 0xEF,              // 1 11 0 11 11 往上200,不換,夾爪開,rc推往後
    .data[2] = 0x80,  // 恢復水平
    .data[3] = 0x69,
    .data[4] = 0xFF,
    .data[5] = 0xEF,
    .data[6] = 0xEE,
    .data[7] = 0x70,
};

// 處理USART命令的函數
void process_usart_command(char *cmd)
{
    // 去掉頭尾空白與換行
    while (*cmd == ' ' || *cmd == '\t') cmd++; // 去掉前面空白
    int len = strlen(cmd);
    while (len > 0 && (cmd[len-1] == '\r' || cmd[len-1] == '\n' || cmd[len-1] == ' ' || cmd[len-1] == '\t')) {
        cmd[--len] = '\0';
    }

    if (len == 0) {
        // 空字串，直接跳過
        return;
    }

    // 解析 DLC + data 的格式，例如: "3 02 FF GG"
    int dlc = 0;
    uint8_t data[8] = {0};
    char *ptr = cmd;
    if (sscanf(ptr, "%d", &dlc) == 1 && dlc > 0 && dlc <= 8)
    {
        ptr = strchr(cmd, ' '); // 找到第一個空格
        if (ptr)
        {
            ptr++; // 指向第一個資料
            int i;
            for (i = 0; i < dlc; i++)
            {
                int val;
                if (sscanf(ptr, "%x", &val) == 1)
                {
                    data[i] = (uint8_t)val;
                    ptr = strchr(ptr, ' ');
                    if (!ptr && i != dlc-1) {
                        printf("資料不足，剩餘使用0填充\n");
                        break;
                    }
                    if (ptr) ptr++;
                }
                else
                {
                    printf("資料格式錯誤，停止解析\n");
                    break;
                }
            }
            can_frame.can_dlc = dlc;
            memcpy(can_frame.data, data, 8);
            mcp2515_send_can_frame(&can_frame);
            return;
        }
    }

    // 處理原有命令
    if (strcmp(cmd, "send") == 0)
    {
        printf("發送當前CAN框架:\n");
        mcp2515_send_can_frame(&can_frame);
    }
    else if (strcmp(cmd, "show") == 0)
    {
        printf("當前CAN框架內容:\n");
        mcp2515_print_can_frame(&can_frame);
    }
    else if (strcmp(cmd, "help") == 0)
    {
        printf("可用命令:\n");
        printf("  <dlc> <data0> <data1> ... - 傳送CAN frame\n");
        printf("  send                      - 發送當前CAN框架\n");
        printf("  show                      - 顯示當前CAN框架\n");
        printf("  help                      - 顯示幫助訊息\n");
        printf("範例:\n");
        printf("  3 02 FF 10    - 傳送 DLC=3, data={0x02,0xFF,0x10}\n");
    }
    else
    {
        printf("未知命令: %s\n", cmd);
        printf("輸入 'help' 查看可用命令\n");
    }
}

int main(void)
{
    rcc_setup();
    usart_setup();
    spi_setup();
    int_pin_setup();
    mcp2515_make_handle(&mcp2515_select, &mcp2515_deselect, &mcp2515_spi_transfer, &mcp2515_delay_ms, &mcp2515);
    mcp2515_init(&mcp2515) ? printf("INIT SUCCESS\n") : printf("INIT Fault\n");

    // mcp2515_send_can_frame(&can_frame);
    delay(1000);
    while (1)
    {
      // 檢查是否有命令待處理
        if (cmd_ready)
        {
            cmd_ready = false;
            process_usart_command(cmd_buffer);
            cmd_index = 0;
            memset(cmd_buffer, 0, CMD_BUFFER_SIZE);
        }
    }
    return 0;   
}

/* For printf(). */
int _write(int file, char *ptr, int len)
{
  int i;

  if (file == 1)
  {
    for (i = 0; i < len; i++)
    {
      usart_send_blocking(USART2, ptr[i]);
    }
    return i;
  }

  errno = EIO;
  return -1;
}

void usart2_isr(void)
{
    /* 確認是 RXNE 事件 */
    if (usart_get_flag(USART2, USART_SR_RXNE)) {
        uint8_t indata = usart_recv(USART2);       /* 讀取資料 (自動清除 RXNE flag) */
         // 回顯字元（可選）
        usart_send_blocking(USART2, indata);
        
        // 處理命令輸入
        if (indata == '\r' || indata == '\n')
        {
            // 命令結束
            if (cmd_index > 0)
            {
                cmd_buffer[cmd_index] = '\0';
                cmd_ready = true;
            }
            usart_send_blocking(USART2, '\n');
        }
        else if (indata == '\b' || indata == 127) // 退格鍵
        {
            if (cmd_index > 0)
            {
                cmd_index--;
                cmd_buffer[cmd_index] = '\0';
            }
        }
        else if (cmd_index < CMD_BUFFER_SIZE - 1)
        {
            // 正常字元
            cmd_buffer[cmd_index++] = indata;
        }
    }
}

void exti9_5_isr(void)
{
  printf("========接收========\n");
    uint8_t intf = mcp2515_getInterrupts(&mcp2515);
    printf("Irq 觸發: ");
    bool printed = false;

    if (intf & CANINTF_MERRF)   { printf("MERRF "); printed = true; mcp2515_clearMERR_Interrupt(&mcp2515); }
    if (intf & CANINTF_RX1IF)   { printf("RX1IF "); printed = true; }
    if (intf & CANINTF_ERRIF)   { printf("ERRIF "); printed = true; 
                                   uint8_t err_flag = mcp2515_getErrorFlags(&mcp2515);
                                   mcp2515_error_dealing(err_flag, &mcp2515);
                                   mcp2515_clearERRIF_Interrupt(&mcp2515); }
    if (intf & CANINTF_TX2IF)   { printf("TX2IF "); printed = true; mcp2515_clearTX2_Interrupts(&mcp2515); }
    if (intf & CANINTF_TX1IF)   { printf("TX1IF "); printed = true; mcp2515_clearTX1_Interrupts(&mcp2515); }
    if (intf & CANINTF_TX0IF)   { printf("TX0IF "); printed = true; mcp2515_clearTX0_Interrupts(&mcp2515); }
    if (intf & CANINTF_RX0IF)   { printf("RX0IF "); printed = true; }
    if (intf & CANINTF_WAKIF)   { printf("WAKIF "); printed = true; mcp2515_clearWAKIF_Interrupt(&mcp2515); }

    if (!printed) printf("無觸發");
    printf("\n");

    // 處理 RX 緩衝區
    while (intf & (CANINTF_RX0IF | CANINTF_RX1IF))
    {
        ERROR ERR_msg = mcp2515_readMessage(&mcp2515, &rx_frame);

        if (ERR_msg == ERROR_OK && rx_frame.can_id == CanID)
        {
            printf("MCP接收");
            mcp2515_print_can_frame(&rx_frame);
            mcp2515_decode_package(&rx_frame);
        }
        else if (ERR_msg != ERROR_OK)
        {
            printf("RX失敗 irq flags: 0x%02X\n", intf);
        }

        // 清除對應的 RX 中斷
        intf = mcp2515_getInterrupts(&mcp2515);
        if (intf & CANINTF_RX0IF) mcp2515_clearRX0IF_Interrupt(&mcp2515);
        if (intf & CANINTF_RX1IF) mcp2515_clearRX1IF_Interrupt(&mcp2515);

        mcp2515_check_after_rx_tx(&mcp2515);
        intf = mcp2515_getInterrupts(&mcp2515);
    }

    exti_reset_request(INT_EXTI);
    printf("========結束========\n");
}

void mcp2515_send_can_frame(can_frame_t *can_frame)
{
  printf("========傳送========\n");
  mcp2515_print_can_frame(can_frame);
  printf("========傳送完成=====\n");
  last_tx_frame_ptr = can_frame;
  Tx_complete = false;
  for (int i = 0; i < 10; i++)
  {
    // 等待RX buffer清空
    while (mcp2515_rx_buffer_full(&mcp2515))
    {
      // 清空接收緩衝區以避免緩衝區溢出
      ERROR rx_msg = mcp2515_readMessage(&mcp2515, &rx_frame);
      if (rx_msg != ERROR_OK)
      {
        printf("接收清理錯誤: %d\n", rx_msg);
      }
    }
    ERROR tx_msg = mcp2515_sendMessage(&mcp2515, last_tx_frame_ptr);
    Tx_complete = mcp2515_check_tx_err_msg(tx_msg) && mcp2515_check_after_rx_tx(&mcp2515);
    delay(1000);
    if (Tx_complete) { break; }
  }
}