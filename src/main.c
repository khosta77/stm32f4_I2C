#include "../system/include/cmsis/stm32f4xx.h"

void I2C1_init() {
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
	(void) I2C1->SR2;

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
	(void) I2C1->SR2;

	// 4. Передаем код
	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// 5. Останавливаем передачу
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_read_bytes(uint8_t address, uint8_t *data, uint8_t size) {
    while (I2C1->SR2 & I2C_SR2_BUSY);
    
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    
    // Повторяющиеся шаги я расписывать не буду
    // 1. Включаем подтверждение возвращается после получения байта
    I2C1->CR1 |= I2C_CR1_ACK;
    
    // 2. Отправляем в канал адрес, для того чтобы происходила чтение данных \
    //    его надо сместить в лево на 1 бит и установим 1 первым битом
    I2C1->DR = ((address << 1) | 0x01);
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
    (void) I2C1->SR2;
    
    // 3. Считывание массива данных
    while (size--) {
        // 3.1. Ждем, что регистр данных не пуст
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        *data++ = I2C1->DR;
    }

    I2C1->CR1 |= I2C_CR1_STOP;

    // 4. Выключаем подтверждение возвращается после получения байта 
    I2C1->CR1 &=~ I2C_CR1_ACK;
}

/** @breif I2C_read_byte - при работе с некоторой перефирией продуман сценарий \
 *                         получения 1 байта передачей команды
 * */
uint8_t I2C_read_byte(uint8_t address, uint8_t code) {
    while (I2C1->SR2 & I2C_SR2_BUSY);

	uint8_t data; // Возращаемые данные 
	//// В начале передадим код регистра, который надо считать, а после считаем регистр
    // 0. Передача кода
    I2C_write_byte(address, code);
	
    // 1. Чтение данных 
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->CR1 |= I2C_CR1_ACK;

	I2C1->DR = ((address << 1) | 0x1);
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void) I2C1->SR2;

	while (!(I2C1->SR1 & I2C_SR1_RXNE));
	data = I2C1->DR;

	I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 &= ~I2C_CR1_ACK;
	return data;
}

int main(void) {
    I2C1_init();

	while(1) {

	}
}


