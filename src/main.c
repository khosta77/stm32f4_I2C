#include "../system/include/cmsis/stm32f4xx.h"

void I2C1_init(void){
    //// Настройка GPIO
    // PB6 -> SCL, BP7 ->SDA
    // 0. Включаем тактирование
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // 1. Настраиваем скорость(Максимальную)
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR6;
    
    // 2. Настраиваем на альтернативную функцию
    GPIOB->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_1;
    
    // 3. Установите выход на открытый слив 
    GPIOB->OTYPER |= GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_6;
    
    // 4. Настраиваем на релим PULL-UP
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR6_0;
    
    // 5. Включаем альтернативную функцию
    GPIOB->AFR[0] |= GPIO_AFRL_AFRL7_2 | GPIO_AFRL_AFRL6_2;
    
    //// Настройка I2C
    // 0. Включаем тактирование
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN ;
    
    // 1. Устанавливает тактовую частоту периферийного устройства
    I2C1->CR2 = 0x2A;
    
    // 2. Задаем частоту работы, есть два режима стандартная и повышенная, выставлена стандартная
    I2C1->CCR = 0xD2;
    
    // 3. Настройка скорости
    I2C1->TRISE = 0x2B;
    
    // 4. Включаем переферию
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_write_bytes(uint8_t addr, uint8_t *data, uint8_t len) {
	// 0. Ждем не занят ли шина I2C
	while (I2C1->SR2 & I2C_SR2_BUSY);
	
    // 1. Запускаем передачу
	I2C1->CR1 |= I2C_CR1_START;
	
    // 2. Ждем пока будет отправлен начальный бит
	while (!(I2C1->SR1 & I2C_SR1_SB));
	
    // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
    //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	I2C1->DR = (addr << 1);

	// 4. Ждем пока предет подверждение получения адреса
	while (!(I2C1->SR1 & I2C_SR1_ADDR));

	// 5. Считываем SR1 и SR2 для того чтобы сбросить их
	(void) I2C1->SR2;

	// 6. Начинаем запись массива
	for (int i = 0; i < len; i++, *data++){
		I2C1->DR = *data;
		
        // 6.1. Ждем потверждения передачи
		while (!(I2C1->SR1 & I2C_SR1_BTF));
	}

	// 7. Выключаем передачу устанавливая стоповый бит
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_write_code(uint8_t address, uint8_t code, uint8_t data) {
    // 0. Ждем не занят ли шина I2C
	while (I2C1->SR2 & I2C_SR2_BUSY);

    // 1. Запускаем передачу
	I2C1->CR1 |= I2C_CR1_START;
    
    // 2. Ждем пока будет отправлен начальный бит
	while (!(I2C1->SR1 & I2C_SR1_SB));

    // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
    //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	I2C1->DR = (address << 1);

	// 4. Ждем пока предет подверждение получения адреса	
    while (!(I2C1->SR1 & I2C_SR1_ADDR));

	// 5. Считываем SR1 и SR2 для того чтобы сбросить их
	(void)I2C1->SR2;

	// 4. Передаем код
	I2C1->DR = code;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// 7. Передаем данные
	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// 8. Останавливаем передачу
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_write_byte(uint8_t address, uint8_t data) {
    // 0. Ждем не занят ли шина I2C
	while (I2C1->SR2 & I2C_SR2_BUSY);

    // 1. Запускаем передачу
	I2C1->CR1 |= I2C_CR1_START;
    
    // 2. Ждем пока будет отправлен начальный бит
	while (!(I2C1->SR1 & I2C_SR1_SB));

    // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
    //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	I2C1->DR = (address << 1);

	// 4. Ждем пока предет подверждение получения адреса	
    while (!(I2C1->SR1 & I2C_SR1_ADDR));

	// 5. Считываем SR1 и SR2 для того чтобы сбросить их
	(void)I2C1->SR2;

	// 4. Передаем код
	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// 5. Останавливаем передачу
	I2C1->CR1 |= I2C_CR1_STOP;
}



void I2C1_read_bytes(uint8_t addr, uint8_t *data, uint8_t len) {
	while(I2C1->SR2 & I2C_SR2_BUSY);          //Wait if bus busy
	I2C1->CR1 |= I2C_CR1_START;               //Start genera
	while(!(I2C1->SR1 & I2C_SR1_SB));         //Wait start condition generated
	I2C1->CR1 |= I2C_CR1_ACK;                 //Enable acknowledge
	I2C1->DR = addr;               //Write slave address
	while(!(I2C1->SR1 & I2C_SR1_ADDR))        //Wait send addsess
    {
		if(I2C1->SR1 & I2C_SR1_AF)          //Acknowledge failure
        {
            I2C1->CR1 |= I2C_CR1_STOP;       //Stop generation
            return;
        }
    }
	(void)I2C1->SR2;                          //Read SR2
	DMA1_Stream5->M0AR = (uint32_t)data;        //Set address buf
	DMA1_Stream5->NDTR = len;                 //Set len
	DMA1_Stream5->CR |= DMA_SxCR_EN;            //Enable DMA
	while(!(DMA1->HISR & DMA_HISR_TCIF5));    //Wait recive all data
	DMA1->HIFCR |= DMA_HIFCR_CTCIF5;          //Clear DMA event
	I2C1->CR1 |= I2C_CR1_STOP;                //Stop generation
	I2C1->CR1 &=~ I2C_CR1_ACK;                //Disable acknowledge
}

#define I2C_MODE_WRITE    0
#define I2C_MODE_READ    1
#define ADR 0x68
#define I2C_ADDRESS(x, y)        ((y) ? (((x) << 1) | 0x01) : (((x) << 1) & 0xFE))

uint8_t I2C_read_byte(uint8_t reg_addr)
{
	uint8_t data;
	//стартуем
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){};
	(void) I2C1->SR1;

	//передаем адрес устройства
	I2C1->DR = (ADR << 1);//I2C_ADDRESS(ADR,I2C_MODE_WRITE);
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};
	(void) I2C1->SR1;
	(void) I2C1->SR2;

	//передаем адрес регистра
	I2C1->DR = reg_addr;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){};
	I2C1->CR1 |= I2C_CR1_STOP;

	//рестарт!!!
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){};
	(void) I2C1->SR1;

	//передаем адрес устройства, но теперь для чтения
	I2C1->DR = ((ADR << 1) | 0x1);  // I2C_ADDRESS(ADR,I2C_MODE_READ);
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};
	(void) I2C1->SR1;
	(void) I2C1->SR2;

	//читаем
	I2C1->CR1 &= ~I2C_CR1_ACK;
	while(!(I2C1->SR1 & I2C_SR1_RXNE)){};
	data = I2C1->DR;
	I2C1->CR1 |= I2C_CR1_STOP;

	return data;
}

uint8_t MPU6050_ReadBit(uint8_t address, uint8_t mem) {
	uint8_t data;
	//CHECK THAT LINE IS NOT BUSY
	while(I2C1->SR2 & I2C_SR2_BUSY){}

	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB)){};
	(void) I2C1->SR1;

	I2C1->DR = address;
	while (!(I2C1->SR1 & I2C_SR1_ADDR)){};
	(void) I2C1->SR1;
	(void) I2C1->SR2;

	I2C1->DR = mem;
	while (!(I2C1->SR1 & I2C_SR1_BTF)){};

	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB)){};
	(void) I2C1->SR1;

	I2C1->DR = address;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)) {
		if(I2C1->SR1 & I2C_SR1_AF) {
			I2C1->CR1 |= I2C_CR1_STOP;       //Stop generation
	        return -1;
	    }
	}
	(void)I2C1->SR2;
	while(!(I2C1->SR1 & I2C_SR1_RXNE));   //Wait, data register is not empty
	data = I2C1->DR;
	I2C1->CR1 |= I2C_CR1_STOP;
	I2C1->CR1 &=~ I2C_CR1_ACK;                 //Disable acknowledge

/*
	while (!(I2C1->SR1 & I2C_SR1_ADDR)){};
	(void) I2C1->SR1;
	(void) I2C1->SR2;
	while (!(I2C1->SR1 & I2C_SR1_RXNE)){};

	data = I2C1->DR;

	I2C1->CR1 |= I2C_CR1_STOP;
*/
	return data;
}


int main(void) {
	while(1) {

	}
}


