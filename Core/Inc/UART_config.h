#define _RX_BUF_SIZE 64
char 		RX_buff[_RX_BUF_SIZE]; 	// буфер
uint8_t 		RX_put_ptr, RX_get_ptr; 	// вказівники на буфер
volatile uint8_t 	RXchar;
#define 		RX_available() (RX_put_ptr - RX_get_ptr)
uint8_t RX_read(void) {
	uint8_t ch;
	if(RX_put_ptr != RX_get_ptr){
		ch = RX_buff[RX_get_ptr++];
		if(RX_get_ptr >= _RX_BUF_SIZE) RX_get_ptr = 0;
		return ch;
	} return 0;
}

int __io_putchar(int ch)
{
    while((USART1->SR & (0x1UL << 7)) == 0); // бит7:TXE (1)

    USART1->DR = (uint8_t) ch;
    return ch;
}
