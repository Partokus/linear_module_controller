
; Created: 25.01.2021 23:04:30
; Author : Alexey Lepeshkin



.include "m32def.inc"

.equ SREG_temp = 0x012B


.equ slave_adress = 0x27 // задаём I2C адресс дисплея


// определения для обработчика кнопок
.equ debounce_time = 0x0A  ; задаём время подавления дребезга НАЖАТИЯ кнопки в миллисекундах
.equ check_pin_time = 0x14 ; задаём время проверки на помехи
.equ debounce_end_time = 0x0A ; задаём время подавления дребезга ОТПУСКАНИЯ кнопки в миллисекундах
.equ RAM_BUTTON_UP = 0x01     ; присваиваем кнопке значение 0x.. для идентификации
.equ RAM_BUTTON_DOWN = 0x02   ; присваиваем кнопке значение 0x.. для идентификации
.equ RAM_BUTTON_OK = 0x03     ; присваиваем кнопке значение 0x.. для идентификации
.equ RAM_BUTTON_HOME = 0x04   ; присваиваем кнопке значение 0x.. для идентификации
.equ RAM_BUTTON_PROG = 0x05   ; присваиваем кнопке значение 0x.. для идентификации
.equ RAM_BUTTON_STEP = 0x06   ; присваиваем кнопке значение 0x.. для идентификации
.equ RAM_BUTTON_BLOCK = 0x07  ; присваиваем кнопке значение 0x.. для идентификации
.equ RAM_ACTIVE_BUTTON = 0x0103 ; адрес в ОЗУ, в который будет записываться идентификатор активной кнопки, чтобы разные кнопки не могли обрабатываться одновременно
.equ access_to_button_flags = 0x0111 ; байт в ОЗУ, с помощью которого можно устанавливать флаги, чтобы отключать какие-то кнопки во время работы определённых функций
.equ B1 = 3 ; PA3 = 3 номер бита в порте А ножки B1 энкодера





// инициализация для управления шаговым двигателем
.equ RAM_step_size = 0x0104 ; адрес байта в оперативной памяти, куда будет записываться текущее значения шага (0,1 мм, 1 мм, 10 мм)
.equ RAM_position_number = 0x0107 ; адрес байта, куда будет записываться номер позиции (0-100)
.equ SHAGOVIK_timer1_on = 0x0112 ; переменная показывает включён таймер 1 или нет. 0x00 - выкл; 0x01 - вкл.
/* 
   Минимальное значение OCR1A при StelthChop: 2000 (DEC).
   Минимальное значение OCR1A при SpreadCycle: ? (DEC).
*/
//.equ SHAGOVIK_freq_maxH = 0x0B ; старший байт максимальная частота импульсов STEP. В виде регистра сравнения OCR1A
//.equ SHAGOVIK_freq_maxL = 0xB8 ; младший байт максимальная частота импульсов STEP. В виде регистра сравнения OCR1A
.equ SHAGOVIK_freq_maxH = 0x02 ; старший байт максимальная частота импульсов STEP. В виде регистра сравнения OCR1A
.equ SHAGOVIK_freq_maxL = 0xAB ; младший байт максимальная частота импульсов STEP. В виде регистра сравнения OCR1A
.equ SHAGOVIK_freq_minH = 0x07 ; старший байт минимальная частота импульсов STEP. В виде регистра сравнения OCR1A
.equ SHAGOVIK_freq_minL = 0xD0 ; младший байт минимальная частота импульсов STEP. В виде регистра сравнения OCR1A
.equ SHAGOVIK_pid_prop = 0x6C ; пропорциональный коэффициент ПИДа
.equ SHAGOVIK_step = 0x10     ; шаг (1, 1/2, 1/4, 1/8, 1/16)
.equ SHAGOVIK_X_TRULY_HIGH_EEPROM = 0x00 ; ст. б. физическое значение положения в EEPROM(используется в случае движения с помощью энкодера, чтобы текущее полож. не было привязано к позиции)
.equ SHAGOVIK_X_TRULY_LOW_EEPROM = 0x03  ; мл. б.
.equ SHAGOVIK_X_TRULY_HIGH_RAM = 0x0132  ; ст. б. физическое значение положения в ОЗУ
.equ SHAGOVIK_X_TRULY_LOW_RAM = 0x0133   ; мл. б. физическое значение положения в ОЗУ
.equ SHAGOVIK_X0_HIGH_RAM = 0x0108    ; старший байт начального положения
.equ SHAGOVIK_X0_LOW_RAM = 0x0109     ; младший байт начального положения
.equ SHAGOVIK_X1_HIGH_RAM = 0x010A    ; старший байт половины пути 
.equ SHAGOVIK_X1_LOW_RAM = 0x010B     ; младший байт половины пути
.equ SHAGOVIK_X2_HIGH_RAM = 0x010C    ; старший байт конца пути 
.equ SHAGOVIK_X2_LOW_RAM = 0x010D     ; младший байт конца пути
.equ SHAGOVIK_OCR1A_BOTTOM_HIGH_RAM = 0x010F ; старший байт значения OCR1A, которого удастся достичь в первой части пути
.equ SHAGOVIK_OCR1A_BOTTOM_LOW_RAM = 0x0110 ; ; младший байт значения OCR1A, которого удастся достичь в первой части пути
.equ SHAGOVIK_DELTA_L_HIGH = 0x01; в этом регистре будет храниться константа, определяющая количество импульсов STEP, при которых будет пройден путь 0,1мм при шаге 1/16
.equ SHAGOVIK_DELTA_L_LOW = 0x90 ; в этом регистре будет храниться константа, определяющая количество импульсов STEP, при которых будет пройден путь 0,1мм при шаге 1/16
//.equ SHAGOVIK_DELTA_L_HIGH = 0x00; 
//.equ SHAGOVIK_DELTA_L_LOW = 0x05 ;

// временное хранение регистров, пока шаговик передвигается
.equ SHAGOVIK_r31 = 0x0117
.equ SHAGOVIK_r30 = 0x0118
.equ SHAGOVIK_r29 = 0x0119
.equ SHAGOVIK_r28 = 0x011A
.equ SHAGOVIK_r27 = 0x011B
.equ SHAGOVIK_r26 = 0x011C
.equ SHAGOVIK_r25 = 0x011D
.equ SHAGOVIK_r24 = 0x011E
.equ SHAGOVIK_r23 = 0x011F
.equ SHAGOVIK_r22 = 0x0126
.equ SHAGOVIK_r21 = 0x0127
.equ SHAGOVIK_r20 = 0x0128
.equ SHAGOVIK_r19 = 0x0129
.equ SHAGOVIK_r18 = 0x012A

.equ STEP = 5
.equ DIR = 4
// для функции HOME
.equ INDSIGNAL = 7
/*
 Этапы хоуминга:
 0 - ИД изначально был активен, поэтому едем вперёд 2 мм на медленной скорости, а потом сразу переходим к 4 этапу
 1 - ИД изначально не был активен, поэтому едем назад на быстрой скорости, пока ИД не станет активным
 2 - когда ИД стал активен, едем вперёд 1 мм на медленной скорости
 3 - едем назад, пока ИД не станет активным
 4 - ИД стал активным, едем назад ровно 1 мм в ноль
*/
// сколько нужно пройти на этапе 0 
.equ HOME_SHAGOVIK_DELTA_HIGH_0 = 0x00 
.equ HOME_SHAGOVIK_DELTA_LOW_0 = 0x14 
// сколько нужно пройти на этапе 2 
.equ HOME_SHAGOVIK_DELTA_HIGH_2 = 0x00 
.equ HOME_SHAGOVIK_DELTA_LOW_2 = 0x14 
// сколько нужно пройти на этапе 4 
.equ HOME_SHAGOVIK_DELTA_HIGH_4 = 0x00 
.equ HOME_SHAGOVIK_DELTA_LOW_4 = 0x01

.equ HOME_SHAGOVIK_freq_maxH = 0x02 ; старший байт максимальная частота импульсов STEP. В виде регистра сравнения OCR1A 
.equ HOME_SHAGOVIK_freq_maxL = 0xAB ; младший байт максимальная частота импульсов STEP. В виде регистра сравнения OCR1A 
.equ HOME_SHAGOVIK_freq_minH = 0x07 ; старший байт минимальная частота импульсов STEP. В виде регистра сравнения OCR1A
.equ HOME_SHAGOVIK_freq_minL = 0xD0 ; младший байт минимальная частота импульсов STEP. В виде регистра сравнения OCR1A
.equ INIT_FLAG = 0x0113 ; переменная для обозначения, что происходит инициализация, а не нажатие кнопки, чтобы правильно выводить значения на дисплей



// инициализация для I2c
.equ I2C_DATA = 0x0120 ; адрес в оперативной памяти, с которого хранятся байты для передачи по i2c
.equ RAM_slave_adress = 0x0100 ; адрес в оперативной памяти, куда будет записывать адрес i2c
.equ START = 0x08 ; статус старта для i2c
.equ REPEATED_START = 0x10 ; статус повторного старта
.equ MT_SLA_ACK = 0x18 ; адрес с командой записи была передана. ACK был получен
.equ MT_DATA_ACK = 0x28 ; данные были переданы. ACK был получен


// инициализация для контроля шагового двигателя энкодером
.equ ENCODER_CONTR_MOT_PRESS_OK = 0x0114 ; флаг вращения энкодера, чтобы вызвать функцию кнопки ОК
.equ ENCODER_CONTR_MOT_PRINT_X2 = 0x0115 ; флаг для того, чтобы вывести на экран позицию Х2, в которую надо приехать
.equ ENCODER_CONTR_MOT_ENCODER_PROCESSING = 0x0116 ; флаг чтобы показать, что передвижение происходит за счёт энкодера
.equ ENCODER_CONTR_MOT_SAVE_X = 0x012C  ; флаг начала отсчёта 3 секунд, чтобы по истечении сохранить текущую позицию Х в EEPROM
.equ ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH = 0x012F ; старший байт счётчика 3 секунд
.equ ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID = 0x012E
.equ ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW = 0x012D ; младший байт счётчика 3 секунд
.equ ENCODER_CONTR_MOT_SAVE_X_TIME_HIGH = 0x01 ; старший байт времени в тактовых импульсах, которое должно пройти перед сохранением
.equ ENCODER_CONTR_MOT_SAVE_X_TIME_MID = 0xFF ; средний байт времени в тактовых импульсах, которое должно пройти перед сохранением
.equ ENCODER_CONTR_MOT_SAVE_X_TIME_LOW = 0xFF  ; младший байт времени в тактовых импульсах, которое должно пройти перед сохранением


// инициализация для светодиодов
.equ LED_prog = 5
.equ LED_ok = 7
.equ LED_block = 4
.equ LED_home = 6


// для таймера 2
.equ TIM2_counter_high = 0x0131 ; счетчик миллисекунд
.equ TIM2_counter_low = 0x0130

// для функции PROG
.equ PROG_FUNC_WORK_STATUS_RAM = 0x0134 ; флаг, показывающий, работает кнопка PROG или нет. Если нажать BLOCK, PROG и OK одновременно, то кнопка PROG будет работать

/*
// для счётчика миллисекунд в формате uint32_t
.equ MILLIS_1 = 0x0135 // старший байт
.equ MILLIS_2 = 0x0136
.equ MILLIS_3 = 0x0137
.equ MILLIS_4 = 0x0138 // младший байт
*/

// для установки блока через SET_BLOCK_CLK_TIME секунд
.equ SET_BLOCK_CLK_COUNTER_HIGH = 0x0135
.equ SET_BLOCK_CLK_COUNTER_MID = 0x0136
.equ SET_BLOCK_CLK_COUNTER_LOW = 0x0137
.equ SET_BLOCK_CLK_FLAG = 0x0138
.equ SET_BLOCK_CLK_TIME_HIGH = 0x08 // 0x12
.equ SET_BLOCK_CLK_TIME_MID = 0xFF
.equ SET_BLOCK_CLK_TIME_LOW = 0xFF

// для выхода из режима настроек через EXIT_FROM_SETTINGS_CLK_TIME секунд
.equ EXIT_FROM_SETTINGS_CLK_COUNTER_HIGH = 0x0139
.equ EXIT_FROM_SETTINGS_CLK_COUNTER_MID = 0x013A
.equ EXIT_FROM_SETTINGS_CLK_COUNTER_LOW = 0x013B
.equ EXIT_FROM_SETTINGS_CLK_FLAG = 0x013C
.equ EXIT_FROM_SETTINGS_CLK_TIME_HIGH = 0x08 // 0x12
.equ EXIT_FROM_SETTINGS_CLK_TIME_MID = 0xFF
.equ EXIT_FROM_SETTINGS_CLK_TIME_LOW = 0xFF

.equ DIG_0 = 0x30
.equ DIG_1 = 0x31
.equ DIG_2 = 0x32
.equ DIG_3 = 0x33
.equ DIG_4 = 0x34
.equ DIG_5 = 0x35
.equ DIG_6 = 0x36
.equ DIG_7 = 0x37
.equ DIG_8 = 0x38
.equ DIG_9 = 0x39

.equ SYM_P = 0x50
.equ SYM_A = 0x41
.equ SYM_R = 0x52
.equ SYM_K = 0x4B

/*  комбинации клавиш:
	если нажать BLOCK и OK одновременно, заполняем нулями ячейки EEPROM, где хранятся значения положений позиций
	если нажать кнопку ОК, то запускаем тестировку
	если нажать кнопку HOME, то включается или отключается автохоуминг при включении
	если нажать BLOCK, PROG и OK одновременно, то кнопка PROG будет работать
*/

/* Последний использованный адрес в ОЗУ = 0x013C*/

jmp RESET 
jmp EXT_INT0 
jmp EXT_INT1 
jmp EXT_INT2 
jmp TIM2_COMP
jmp TIM2_OVF
jmp TIM1_CAPT
jmp TIM1_COMPA
jmp TIM1_COMPB
jmp TIM1_OVF
jmp TIM0_COMP
jmp TIM0_OVF
jmp SPI_STC
jmp USART_RXC
jmp USART_UDRE
jmp USART_TXC
jmp ADC1
jmp EE_RDY
jmp ANA_COMP
jmp TWI
jmp SPM_RDY


;
RESET: 
	ldi r16, high(0x042F); Main program start для 
	out SPH,r16 ; 
	ldi r16, low(0x042F) ; половина от обьёма оперативной памяти ATmega32
	out SPL,r16
	
	rcall init_values
	rcall init_port ; настраиваем порты
	rcall init_I2C ; настраиваем I2c
	rcall init_timer0 ; настраиваем timer0 для обработчика кнопок
	rcall init_timer1 ; настраиваем timer1 для управления шаговым двигателем
	//rcall init_timer2 ; настравиваем timer2 для отсчёта миллисекунд, для различных таймеров
	
	call set_value_of_positions_to_eeprom ; если нажать BLOCK и PROG одновременно, заполняем нулями ячейки EEPROM, где хранятся значения положения позиций
	
	rcall lcd1602_init ; инициализируем LCD1602
	rcall Create_char ; создаём русские символы
	rcall init_parameters ; инициализируем параметры, считываем из ПЗУ и расставляем по полочкам (ОЗУ)
	rcall start_title ; выводим начальную надпись и начальные значения на экран

	rcall test_random ; если нажать кнопку ОК, то запускаем тестировку
	rcall home_and_go_to_last_position ; возвращаемся в ноль, а потом на последнюю позицию, которая была перед выключением
	call check_block_prog_ok ; проверяем нажатие этих трёх кнопок, который включают или выключают режим программирования
	// по умолчанию
	clr r16
	clr r17
	
	
	rcall Button_block_func ; блокируем все кнопки 
	
	//ldi r16,0b11110000
	//out PORTC,r16
	
	sei ; Enable interrupts








main:
rcall interrupt_process ; ф-ция в которой находимся как в цикле while, пока включён таймер 1 (пока подаём на ШД step-сигналы)
rcall save_position_X_after_3_sec ; сохраняние истинного значения позиции через 3 секунды после последнего вращения энкодера
call set_block_clk               // установление блокировки через 10 секунд, если не нажимать кнопки
call exit_from_settings_mode_clk // выход из режима настроек через 10 секунд, если не нажимать кнопки
call check_block_prog_ok

lds r16,access_to_button_flags ; проверяем какие кнопки можно нажимать в данный момент
SBRS r16,0 ; если в access_to_button_flags сброшен бит 0, то  запретить нажатие некоторых кнопок
rjmp main
rcall Button_block
lds r16,access_to_button_flags ; проверяем какие кнопки можно нажимать в данный момент
SBRS r16,1 ; если сброшен бит 1, разрешить нажатие только кнопки Блок
rjmp main

rcall Button_up
rcall Button_down
rcall Button_ok
rcall Button_prog
rcall Button_home
rcall Button_step

rcall check_encoder_press // проверяем вращение энкодера

rjmp main














// настраиваем I2C
init_I2C:
	//ldi r16,0b00000011
	//out PORTC,r16
	; настраиваем частоту SCL (передачи). SCLfreq = CPU clock frequency / (16 + 2 * (TWBR) * 4 * (PrescalerValue))
	ldi r16,0x0A
	out TWBR,r16 ; Bit rate register

	ldi r16,0x00 
	out TWSR,r16 ; предделитель частоты ставим равным 1

	; Для Slave мода. Устанавливает собственный адресс
	;ldi r16,0x27 
	;lsr r16 ; сдвигаем влево с добавлением нуля в младший бит, так как адрес имеет лишь 7 битов, и последний бит в регистре TWAR
			; отвечает за функцию распознавания, она не нужна нам
	;sts TWAR,r16 ; загружаем сразу адрес дисплея 1602, равный 0x27 в шестнадцатиричной системе счисления
ret




i2c_write: nop
; TWEN - включаем I2C
; TWSTA - отправляем START condition
; TWSTO - отправляем STOP condition
; TWINT - устанавливаем 1 для сброса флага



; загружаем в регистр данных байт, который нужно передать. очищаем бит TWINT, чтобы начать передачу данных
	ld r16,Z+ ; загружаем из оперативной памяти байт, адрес ячейки которого находится в Z, который надо передать
	out TWDR, r16       
	ldi r16, (1<<TWINT) | (1<<TWEN) 
	out TWCR, r16

; ждём пока флаг TWINT установится. Это покажет то, что команда байт был передан и  от слейва получен ACK/NACK
wait3: 
	in r16,TWCR 
	sbrs r16,TWINT 
	rjmp wait3

; проверяем регистр статуса TWSR, если статус не соответствует MT_DATA_ACK (ACK не получен), то фиксируем ошибку. Маска битов предделителя
; DATA has been transmitted; ACK has been received
	in r16,TWSR 
	andi r16, 0xF8 
	cpi r16, MT_DATA_ACK ; .equ START = 0x18
	brne ERROR
	ret

ERROR:

ret




i2c_start:
; отправляем START
	ldi r16, (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (0<<TWSTO)
	out TWCR, r16

; ждём пока TWINT станем равным 1, что говорит о том, что передача START закончена 
wait1: 
	in r16,TWCR 
	sbrs r16,TWINT 
	rjmp wait1

; проверяем статус START, если статус не соответствует START, то фиксируем ошибку. Маска битов предделителя
	in r16,TWSR 
	andi r16, 0xF8 
	cpi r16, START ; .equ START = 0x08
	breq next1 ; если в первый раз передаём СТАРТ, то продолжаем передачу
	cpi r16, REPEATED_START ; иначе проверяем статус повтроного старта
	brne ERROR_start ; если это не повторный старт, то фиксируем ошибку

next1:
; загружаем из оперативной памяти байт с адресом и командой записи в регистр данных байт SLA_W - адрес + команда записи
	lds r16,RAM_slave_adress ; .equ SLA_W = 0b01001110 - адрес 0x27 и младший бит 0 для записи
	lsl r16 ; сдвигаем влево, чтобы получить формат XXXX XXX0 - где слева адрес, а 0 это команда записи
	out TWDR, r16 
	ldi r16, (1<<TWINT) | (1<<TWEN) 
	out TWCR, r16

; ждём пока флаг TWINT установится. Это покажет то, что команда SLA_W была передана и от слейва получен ACK/NACK
wait2: 
	in r16,TWCR 
	sbrs r16,TWINT 
	rjmp wait2

; проверяем регистр статуса TWSR, если статус не соответствует MT_SLA_ACK (ACK не получен), то фиксируем ошибку. Маска битов предделителя
; SLA+W has been transmitted; ACK has been received
	in r16,TWSR 
	andi r16, 0xF8 
	cpi r16, MT_SLA_ACK ; .equ START = 0x18
	brne ERROR_start
	ret

ERROR_start:
ret



i2c_stop:
; передаём STOP condition
	ldi r16, (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) 
	out TWCR, r16 

	ldi r16, (1<<TWINT) | (0<<TWEN) | (0<<TWSTO) | (0<<TWSTA) ; выключаем всё
	out TWCR, r16 
	ret

ret













lcd1602_init: nop
    clr r17
	sts 0x0102,r17 ; загружаем в оперативную память нулевые значения RS и R/W

	ldi r16,0x10
	rcall nopdelay_1ms ; задержка на 16 миллисекунд

	; посылаем инструкцию настройки интерфейса
	ldi r16,0b00110011 ; восьмибитный интерфейс 
	rcall LCD1602_send_8bit_command ; 

	ldi r16,0x05
	rcall nopdelay_1ms ; задержка на 4 миллисекунды

	ldi r16,0b00110011 ; восьмибитный интерфейс 
	rcall LCD1602_send_8bit_command ; 

	ldi r16,0x01
	rcall nopdelay_1ms ; задержка на 1 мсек

	ldi r16,0b00100010 ; меняем на 4-битный интерфейс
	rcall LCD1602_send_8bit_command
	
	ldi r16,0b00101000 ; 
	rcall LCD1602_send_command
	
	ldi r16,0b00001000
	rcall LCD1602_send_command
	
	ldi r16,0b00000001 ; команда очистки дисплея
	rcall LCD1602_send_command

	ldi r16,0x03 ; пауза, так как очистка экрана занимает дольше всех времени
	rcall nopdelay_1ms
	
	ldi r16,0b00000110 ; направление вывода символов слева-направо, запрет сдвига экрана при выводе символов
	rcall LCD1602_send_command
	
	ldi r16,0b00001100 ; включить дисплей (D=1), Отключить отображение курсора (C=0), отключить функцию мигания курсора (B=0)
	rcall LCD1602_send_command
ret





; функция задержки. значения в r16 показываем на сколько миллисекунд будет задержка. Если r16 = 60, то задержка 60 мсек
nopdelay_1ms: 
cycle3:
ldi r18,0x40
cycle2:
ldi r17,0x31
nop
nop
cycle1:
nop
nop
dec r17
brne cycle1
dec r18
brne cycle2
dec r16
brne cycle3
ret





; функция выполняет 8-ю инструкцию дисплея, где записывается адрес места, где будет выведен символ. Для 1-ой строки адреса: 0x00 - 0x0F. Для второй: 0x40 - 0x4F
Set_cursor: nop
	mov r20,r16 ; сохраняем в регистре r20, чтобы знать, значение какой величины отображаем. Это нужно для вывода чисел в разном формате (00.0 или 00 к примеру)
	ori r16,0b10000000 ; активируем 8-ю инструкцию, где DB7 равен единице
	clr r17
	sts 0x0102,r17 ; RS и R/W в 0 для записи адреса курсора
	rcall LCD1602_send_command ; отправляем данные
ret









Send_instruction: nop
	clr r17
	sts 0x0102,r17 ; RS и R/W в 0 для записи адреса курсора
	rcall LCD1602_send_command ; отправляем данные
ret








; отправляет в дислей байт D7-D0 по 4-битному интерфейсу. Данные фиксируются по спаду на E. D7 - D6 - D5 - D4 - LED - E - R/W - RS
LCD1602_send_command: nop
    mov r27,r16  ; сохраняем наши данные в регистре r25
	ori r16,0b00001100 ; оставляем включённой подсветку и устанавливаем единичку на выводе тактирования Е
	lds r17,0x0102 ; загружаем байт, в котором установлены значения RS и R/W 
	or r16,r17 ; загружаем значения RS и R/W 
	andi r16,0b11111101 ; очищаем R/W
	sts I2C_DATA,r16 ; загружаем в ОЗУ первый байт, который надо передать

    ;ldi r16,slave_adress
    ;sts 0x0100,r16 ; загружаем в оперативную память байт с адресом слейва 

	lds r16,I2C_DATA ; считываем байт, который передали
	andi r16,0b11111011 ; линию тактирования в 0
	sts 0x0121,r16 ; загружаем в ОЗУ 2-й байт, который надо передать
	; сдвигаем четыре бита влево, чтобы можно было передать их
	
	lsl r27
	lsl r27
	lsl r27
	lsl r27 
	mov r16,r27 ; копируем в r16
	
	ori r16,0b00001100 ; оставляем включённой подсветку и устанавливаем единичку на выводе тактирования Е
	lds r17,0x0102
	or r16,r17
	andi r16,0b11111101 ; очищаем R/W
	sts 0x0122,r16 ; загружаем в ОЗУ 3-й байт, который надо передать


	lds r16,0x0122 ; считываем байт, который передали
	andi r16,0b11111011 ; линию тактирования в 0
	sts 0x0123,r16 ; загружаем в ОЗУ 4-й байт, который надо передать

	push r31 ; чтобы не испортить 
	push r30
	// загружаем в рег. пару Z адрес ячейки ОЗУ байта, который надо передать по i2c
	ldi r31,0x01
	ldi r30,0x20 
	rcall i2c_start ; начинаем передачу по i2c
	// передаём старшую часть байта комманды, Е = 1
	rcall i2c_write ; записываем данные
	// передаём старшую часть байта комманды, Е = 0. Данные в дисплее фиксируются по спаду. Данные зафиксировались
	rcall i2c_write ; записываем данные
	// передаём младшую часть байта комманды, Е = 1
	rcall i2c_write ; записываем данные
	// передаём младшую  часть байта комманды, Е = 0. 
	rcall i2c_write ; записываем данные
	rcall i2c_stop ; заканчиваем передачу

	pop r30
	pop r31
ret







Write_symbol: nop
	ldi r17,0x01 ; RS = 1, R/W = 0 
	sts 0x0102,r17 ; 
	rcall LCD1602_send_command ; отправляем данные
ret








; эта функция нужна только для инициализации дисплея, так как в начале он использует 8-битный интерфейс
LCD1602_send_8bit_command: nop
    mov r25,r16  ; сохраняем наши данные в регистре r25
	ori r16,0b00001100 ; оставляем включённой подсветку и устанавливаем единичку на выводе тактирования Е
	andi r16,0b11111100 ; обнуляем последние два бита
	sts I2C_DATA,r16 ; загружаем в оп память байт, который нужно передать
	// загружаем в рег. пару Z адрес ячейки ОЗУ байта, который надо передать по i2c
	ldi r31,0x01
	ldi r30,0x20 

    ldi r16,slave_adress
    sts 0x0100,r16 ; загружаем в оперативную память байт с адресом слейва 
	rcall i2c_start ; начинаем передачу по i2c
	rcall i2c_write ; передаём старшую часть байта комманды, Е = 1
	rcall i2c_stop  ; заканчиваем передачу по i2c
	// загружаем в рег. пару Z адрес ячейки ОЗУ байта, который надо передать по i2c
	ldi r31,0x01
	ldi r30,0x20 

	lds r16,I2C_DATA ; считываем байт, который передали
	andi r16,0b11111011 ; линию тактирования в 0
	sts I2C_DATA,r16 ; загружаем в оп память байт, который нужно передать
	; передаём старшую часть байта комманды, Е = 0. Данные в дисплее фиксируются по спаду. Данные зафиксировались
	rcall i2c_start ; начинаем передачу по i2c
	rcall i2c_write ; передаём байт по i2c
	rcall i2c_stop  ; заканчиваем передачу по i2c
ret






init_timer0:
// настраиваем TIMER0 для обработчика кнопок
	clr r16 
	out TCNT0,r16 ; очищаем на всякий случай

	ldi r16,0b00000000  ; пока не запускаем таймер
	out TCCR0,r16 ; 

	ldi r16,0xF9 ; F9
	out OCR0,r16 ; записываем число 249 в регистр сравнения А, так как при достижении счётчиком этого числа пройдёт ровно 1 мсек

	in r16,TIMSK
	ori r16,0b00000010  
	out TIMSK,r16 ; устанавливаем прерывание по совпадению А. Используется sts, так как адрес регистра TIMSK0 находится в локации
	out TIFR,r16  ; сбрасываем регистр, чтобы прерывания пока не запускались. TIFR0 находится в локации "64 I/O Registers" (0x0020 - 0x005F)


ret



// Настраиваем Timer1 для управления шаговым двигателем
init_timer1:
	// настраиваем на режим СТС
	ldi r16, (0<<WGM10) | (0<<WGM11) 
	out TCCR1A,r16 

	;ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; предделитель равен 1.
	ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (0<<CS10) ; пока не запускаем таймер
	out TCCR1B,r16

	// обнуляем TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; записываем в TCNT1 r17:r16

	// записываем в OCR1A. Минимальную частоту
	ldi r16,SHAGOVIK_freq_minL
	ldi r17,SHAGOVIK_freq_minH
	rcall TIM16_WriteOCR1A ; записываем в OCR1A r17:r16
	// разрешаем прерывание по совпадению с OCR1A
	in r16,TIMSK
	ldi r17, (1<<OCIE1A)
	or r16,r17
	out TIMSK,r16
	// обнуляем флаг прерывания
	out TIFR,r16
ret
/*
// Prescaler = 32. Ticks = 250. OCR2 period = 0.0005. Для 10 секунд должно произойти (OCR2 period * 20000) прерываний
init_timer2:
// настраиваем TIMER0 для обработчика кнопок
	clr r16 
	out TCNT2,r16 ; очищаем на всякий случай

	ldi r16,0b00000011 // prescaler = 32  
	out TCCR2,r16 

	ldi r16,0xF9 ; 249
	out OCR2,r16 ; записываем число 249 в регистр сравнения А, так как при достижении счётчиком этого числа пройдёт ровно 1 мсек

	in r16,TIMSK
	ori r16,0b10000000  
	out TIMSK,r16 ; устанавливаем прерывание по совпадению А. Используется sts, так как адрес регистра TIMSK0 находится в локации
	out TIFR,r16  ; TIFR0 находится в локации "64 I/O Registers" (0x0020 - 0x005F)
ret
*/

// для записи в TCNT1
TIM16_WriteTCNT1:
	// сохраняем регистров флагов 
	in r18,SREG
	// отключаем прерывания
	cli
	// записываем в TCNT1 число r17:r16
	out TCNT1H,r17
	out TCNT1L,r16
	// восстанавливаем регистр флагов
	out SREG,r18
ret

// для чтения из TCNT1
TIM16_ReadTCNT1:
	// сохраняем регистров флагов 
	in r18,SREG
	// отключаем прерывания
	cli
	// считываем TCNT1 в r17:r16
	in r16,TCNT1L
	in r17,TCNT1H
	// восстанавливаем регистр флагов
	out SREG,r18
ret

// для записи в OCR1A
TIM16_WriteOCR1A:
	// сохраняем регистров флагов 
	in r18,SREG
	// отключаем прерывания
	cli
	// записываем в TCNT1 число r17:r16
	out OCR1AH,r17
	out OCR1AL,r16
	// восстанавливаем регистр флагов
	out SREG,r18
ret


// для чтения из OCR1A
TIM16_ReadOCR1A:
	// сохраняем регистров флагов 
	in r18,SREG
	// отключаем прерывания
	cli
	// считываем OCR1A в r17:r16
	in r16,OCR1AL
	in r17,OCR1AH
	// восстанавливаем регистр флагов
	out SREG,r18
ret

/*
// Настраиваем Timer2 для управления шаговым двигателем
init_timer2:
	clr r16
	out TCNT2,r16 ; очищаем на всякий случай

	ldi r16,0b00001000 ; пока не запускаем таймер. WGM21 = 1 для режима совпадения
	out TCCR2,r16 ; 

	ldi r16,0xC7 ; Задаваемая частота частота 10 кГц при OCR2 = 249
	out OCR2,r16 ; регистр совпадения

	in r16,TIMSK ; чтобы не испортить другие значения регистра
	ori r16,0b10000000
	out TIMSK,r16 ; устанавливаем прерывание по совпадению. OCIE2 = 1 
	out TIFR,r16  ; сбрасываем регистр единицей. TIFR0 находится в локации "64 I/O Registers" (0x0020 - 0x005F)
ret
*/


init_port: nop
	// настраиваем кнопки
	clr r16
	out DDRB,r16 ; PB1-PB4 на вход
	ldi r16,0b00000000
	out PORTB,r16 ; PB1-PB4 подтяжка
	clr r16
	out DDRA,r16 ; PA0-PA3 на вход
	ldi r16,0b00000000
	out PORTA,r16 ; PA0-PA3 подтяжка

	// настраиваем пин А для энкодера 
	clr r16
	out DDRD,r16 ; пин PD2(INT0) на вход
	ldi r16,0b00000100
	out PORTD,r16 ; ; пин PD2(INT0) подтяжка
	; настраиваем PD2(INT0) на прерывание по фронту (т.к. там стоит инвертирующий триггер Шмитта)
	in r16,MCUCR
	//ori r16,0b00000011
	ldi r16,0b00000011
	out MCUCR,r16 ; ISC01 и ISC00 в единицы для прерывания по фронту 
	ldi r16,0b01000000
	out GICR,r16 ; INT0 в единицу для включения прерывания на пине INT0

	// настраиваем светодиоды 
	ldi r16,0b11110000
	out DDRC,r16 ; PC4-PC7 на выход
	clr r16
	out PORTC,r16 ; на выходе нули
	
	// настраиваем пины STEP, DIR, ENABLE
	in r16,DDRA
	ori r16,0b01110000
	out DDRA,r16 ; STEP, DIR , ENABLE на выход
	in r16,PORTA
	andi r16,0b10001111
	out PORTA,r16 ; STEP, DIR нули на выходе

	// настраиваем вход для индуктивного датчика
	in r16,DDRA
	andi r16,0b01111111
	out DDRA,r16 ; INDSIGNAL на вход
	in r16,PORTA
	andi r16,0b01111111
	out PORTA,r16 ; INDSIGNAL без подтяжки
ret



/*
[Х][ ][ ][ ][ ][ ][Ш][А][Г][ ][ ][ ][П][О][З][.]
[5][2][.][0][ ][ ][1][0][.][0][ ][ ][ ][9][9][ ]
*/
start_title: nop
    ldi r16,0x00 ; устанавливаем курсора на начало на всякий
	rcall Set_cursor

	; выводим символы
	ldi r16,0x58 ; выводим символ "X"
	rcall Write_symbol

	ldi r16,0x2C ; выводим символ ","
	rcall Write_symbol

	ldi r16,0x6D ; выводим символ "m"
	rcall Write_symbol

	ldi r16,0x6D ; выводим символ "m"
	rcall Write_symbol

	ldi r16,0x06 
	rcall Set_cursor ; ставим курсор на седьмой столбик верхней строки

	ldi r16,0x00 ; выводим символ "Ш"
	rcall Write_symbol

	ldi r16,0x41 ; выводим символ "А"
	rcall Write_symbol

	ldi r16,0x01 ; выводим символ "Г"
	rcall Write_symbol

	ldi r16,0x0C 
	rcall Set_cursor ; ставим курсор на 13-й столбик верхней строки

	ldi r16,0x02 ; выводим символ "П"
	rcall Write_symbol

	ldi r16,0x4F ; выводим символ "O"
	rcall Write_symbol

	ldi r16,0x03 ; выводим символ "З"
	rcall Write_symbol

	ldi r16,0x2E ; выводим символ "."
	rcall Write_symbol

	// устанавливаем курсора на начало второй строки
	ldi r16,0x40 
	rcall Set_cursor
	
	/* Выводим на экран значения*/
	// считываем текущее положение X, чтобы вывести на экран
	lds r26,SHAGOVIK_X2_HIGH_RAM 
	lds r25,SHAGOVIK_X2_LOW_RAM 

	rcall lcd_print_number ; вызываем ф-цию, выводящую на дисплей число r26:r25 в десятичном формате

	// выводим значение ШАГа
	ldi r16,0x46
	rcall Set_cursor

	clr r26 ; подготавливаем старший байт для расчётов
	ldi r25,0x0A 
	sts RAM_step_size,r25 ; записываем в ОЗУ текущий ШАГ
	rcall lcd_print_number ; вызываем ф-цию, выводящую на дисплей число r26:r25 в десятичном формате

	// выводим номер позиции
	clr r26 ; подготавливаем старший байт для расчётов
	lds r25,RAM_position_number ; считываем из ОЗУ номер позиции (0-99)
	// call function that deside print number or name of position
	rcall lcd_print_number_or_name_of_position
	
ret

// сбрасываем переменные, чтобы в них не было мусора
init_values:
	clr r16
	sts SHAGOVIK_timer1_on,r16

	sts SET_BLOCK_CLK_FLAG,r16 // не запускаем таймер блока 
	
	ldi r16,0x01
	sts EXIT_FROM_SETTINGS_CLK_FLAG,r16 // запускаем таймер выхода из настроек

	ldi r16,0x00
	sts PROG_FUNC_WORK_STATUS_RAM,r16 ; записываем ноль в флаг работы кнопки ПРОГ. 0 - кнопка ПРОГ не работает. 1 - работает.
ret




// инициализируем параметры, считываем из ПЗУ и расставляем по полочкам (ОЗУ)
init_parameters:
	// считываем номер последней позиции из EEPROM (0-100) (адрес в EEPROM - 0x0000) адресация EEPROM ---- --XX XXXX XXXX. То есть 10 битов информации
	ldi r18,0x00
	ldi r17,0x00 ; адрес в EEPROM 00 0000 0000
	rcall EEPROM_read ; считываем байт из ПЗУ, где r18:r17 это адрес байта, в r16
	sts RAM_position_number,r16 ; сохраняем в ОЗУ номер позиции

	rcall get_position_value_from_eeprom ; берём из EEPROM значение положения Х исходя из текущего номера позиции
	// проверяем включён ли автохоуминг при включении или нет
	ldi r18,0x00
	ldi r17,0x02 ; EEPROM адрес 0x0002
	rcall EEPROM_read

	cpi r16,0xFF 
	breq position_priority // автохоуминг при включении включён, значит считываем положение, привязанное к позиции

// автохоуминг отключён, значит считываем последнюю позицию, в которой физически находились
last_move_priority:
	ldi r17,0x03
	ldi r18,0x00 ; адрес младшего байта в EEPROM 0x0003
	rcall EEPROM_read
	sts SHAGOVIK_X2_LOW_RAM,r16 ; записываем в ОЗУ младший байт текущей позиции 
	sts SHAGOVIK_X_TRULY_LOW_RAM,r16 ; сохраняем физическое положение в ОЗУ

	ldi r17,0x04
	ldi r18,0x00 ; адрес старшего байта в EEPROM 0x0003
	rcall EEPROM_read
	sts SHAGOVIK_X2_HIGH_RAM,r16 ; записываем в ОЗУ старший байт текущей позиции
	sts SHAGOVIK_X_TRULY_HIGH_RAM,r16 ; сохраняем физическое положение в ОЗУ
	

	
// автохоуминг при включении включён, значит считываем положение, привязанное к позиции
position_priority:
	// записываем начальное положение Х0 в ОЗУ из Х2
	lds r16,SHAGOVIK_X2_HIGH_RAM 
	sts SHAGOVIK_X0_HIGH_RAM,r16
	sts SHAGOVIK_X_TRULY_HIGH_RAM,r16 ; сохраняем физическое положение в ОЗУ
	lds r16,SHAGOVIK_X2_LOW_RAM 
	sts SHAGOVIK_X0_LOW_RAM,r16
	sts SHAGOVIK_X_TRULY_LOW_RAM,r16 ; сохраняем физическое положение в ОЗУ

	ldi r16,0b00000011
	sts access_to_button_flags,r16 ; разрашаем нажатие всех кнопок
ret


get_position_value_from_eeprom:
	// умножаем номер позиции на два, так как данные номера позиции имеют размер 2 байта, ответ в r0
	lds r16,RAM_position_number
	ldi r17,0x02
	mul r16,r17 

	// прибавляем к номеру позиции, умноженному на два, число 0x0A, так как с 0x0A в EEPROM начинается хранение данных позиции (положение Х) в виде двух байт
	ldi r16,0x0A
	add r0,r16

	// полученный адрес в EEPROM перекидываем в регистры, чтобы прочитать старший байт положения Х последней позиции
	ldi r18,0x00
	mov r17,r0
	rcall EEPROM_read
	sts SHAGOVIK_X2_HIGH_RAM,r16 ; записываем в ОЗУ старший байт текущей позиции
	; mov r26,r16 ; считываем старший байт положения в r26 для вывода на дисплей
	// считываем млайдший байт текущей позиции
	inc r17 ; увеличиваем адрес в EEPROM на 1, чтобы считать младший байт 
	rcall EEPROM_read
	sts SHAGOVIK_X2_LOW_RAM,r16 ; записываем в ОЗУ младший байт текущей позиции 
	; mov r25,r16 ; считываем старший байт положения в r25 для вывода на дисплей
ret


TIM0_COMP:
	push r16 ; сохраняем значения регистра в стеке, чтобы не испортить работу других функций
	in r16,SREG
	push r16
	clr r16
	out TCNT0,r16 ; обнуляем счётчик таймера 0
	SBRS r17,3 ; если выполняется какая-то функция, то не менять биты в r17, чтобы не помешать работе какой-то функции
	SBR r17,2 ; устанавливаем единичный бит как флаг, что произошло прерывание, то есть прошло 10 мсек
	pop r16
	out SREG,r16
	pop r16  ; возвращаем значение r16 из стека
reti


// ф-ция прибавляет номер позиции
Button_up_func: nop
	rcall Clock_start_for_block // включаем таймер, который включит блокировку через 10 секунд

	clr r26 ; для расчётов
	lds r25,RAM_position_number ; считываем текущий номер позиции
	inc r25 ; прибавляем к номеру позиции

	cpi r25,0x64 ; если номер позиции не оказался равен 100
	brne print_num_up ; то переходим к выводу на дисплей
	ldi r25,0x00 ; иначе приравниваем к 0 
	
print_num_up:
	sts RAM_position_number,r25 ; запоминаем новое значение в ОЗУ

	// считываем значения положения Х исходя из номера позиции
	rcall get_position_value_from_eeprom ; считываем из EEPROM положение Х по номеру позиции

	// call function that deside print number or name of position
	rcall lcd_print_number_or_name_of_position
	
	ldi r16,0x40
	rcall Set_cursor
	
	// считываем значение текущего положения Х
	lds r26,SHAGOVIK_X2_HIGH_RAM  
	lds r25,SHAGOVIK_X2_LOW_RAM 
	rcall lcd_print_number ; выводим на дисплей 
	
ret



Button_down_func:
rcall Clock_start_for_block // включаем таймер, который включит блокировку через 10 секунд

clr r26 ; для расчётов
	lds r25,RAM_position_number ; считываем текущий номер позиции
	dec r25 ; прибавляем к номеру позиции

	cpi r25,0xFF ; если номер позиции не перешёл за 0
	brne print_num_down ; то переходим к выводу на дисплей
	ldi r25,0x63 ; иначе приравниваем к 99
	
print_num_down:
	sts RAM_position_number,r25 ; запоминаем новое значение в ОЗУ
	// считываем значения положения Х исходя из номера позиции
	mov r16,r25 ; запоминаем номер позиции для считывания положения Х
	rcall get_position_value_from_eeprom ; считываем из EEPROM положение Х по номеру позиции
	// выводим номер позиции на дисплей
	
	// call function that deside print number or name of position
	rcall lcd_print_number_or_name_of_position

	ldi r16,0x40
	rcall Set_cursor
	
	// считываем значение текущего положения Х
	lds r26,SHAGOVIK_X2_HIGH_RAM  
	lds r25,SHAGOVIK_X2_LOW_RAM 
	rcall lcd_print_number ; выводим на дисплей 
ret


Button_ok_func: nop
	cli ; запрет прерываний
	
	rcall Clock_start_for_block // включаем таймер, который включит блокировку через 10 секунд

// первый этап
shagovik_stage1:
	// считываем текущее положение X0 (начальное)
	lds r27,SHAGOVIK_X0_HIGH_RAM
	lds r26,SHAGOVIK_X0_LOW_RAM
	// считываем положение X2, куда надо отправиться
	lds r31,SHAGOVIK_X2_HIGH_RAM
	lds r30,SHAGOVIK_X2_LOW_RAM 

	sts SHAGOVIK_X_TRULY_HIGH_RAM,r31 ; записываем в ОЗУ старший байт позиции
    sts SHAGOVIK_X_TRULY_LOW_RAM,r30  ; записываем в ОЗУ младший байт позиции
	// проверяем, не равно ли новое положение начальному положению, при том, что это та же позиция
	CPSE r27,r31 ; если равны, то пропустить след. команду
	rjmp find_mid ; если не равны, то едем дальше
	CPSE r26,r30 ; если равны, то пропустить след. команду
	rjmp find_mid ; если не равны, то едем дальше
	// если начальное положение оказалось равное новому, то теперь проверяем
	rcall encoder_print_num ; выводим на экран значение позиции Х2, если вращался энкодер
	// равен ли номер позиции в EEPROM позиции, которую пытаемся сохранить
	ldi r17,0x00
	ldi r18,0x00 ; адрес номера позиции в EEPROM 0x0000
	rcall EEPROM_read
	lds r17,RAM_position_number ; считываем из ОЗУ номер текущей позиции
	CPSE r17,r16 ; если равны, то пропустить след. команду
	rjmp save_pos_number_to_eeprom ; если номера позиций не равны, то просто сохраняем новый номер позиции в EEPROM 
	// если номера позиций равны, то получается, что шаговый двигатель некуда крутить, и номер позиции
	// нет необходимости сохранять, выходим из функции
	ldi r16,0x00
    sts ENCODER_CONTR_MOT_ENCODER_PROCESSING,r16 ; сбрасываем флаг того, что вращение происходит по энкодеру
	clr r17 ; чтобы кнопки работали
	rjmp ok_exit ; выходим из функции
// сохраняем новый номер позиции в EEPROM
save_pos_number_to_eeprom:
	mov r16,r17
	ldi r17,0x00
	ldi r18,0x00 ; адрес номера позиции в EEPROM 0x0000
	rcall EEPROM_write
	clr r17 ; чтобы кнопки работали
	rjmp ok_exit ; выходим из функции

find_mid:
	// включаем светодиод
	SBI PORTC,PC7
	// находим среднее положение X1. X1 = (|X2 - X0|)/2 +(-) X0) 
	// например, X0 = 14.2, X2 = 26.2 -> X1 = (26.2 - 14.2)/2 + 14.2 = 20.2
	// X2 - X0
	sub r30,r26
	sbc r31,r27
	BRPL poloj ; если разность оказалсь положительной, то переходим на метку

	// иначе разность оказалась отрицательной
	CBI PORTA,DIR ; показываем, что движемся назад
	// считываем ещё раз положение Х2, так как оно было испорчено
	lds r31,SHAGOVIK_X2_HIGH_RAM
	lds r30,SHAGOVIK_X2_LOW_RAM 
	// находим разность уже зная, что Х0 больше, чем Х2
	sub r26,r30
	sbc r27,r31
	// делим на 2 сдвигом вправо
	lsr r27
	ror r26
	// считываем ещё раз положение Х0, так как оно было испорчено. WARNING: записали Х0 в регистры где обычно Х1 хранится
	lds r29,SHAGOVIK_X0_HIGH_RAM
	lds r28,SHAGOVIK_X0_LOW_RAM
	// вычитаем из Х0 полученную разницу, делённую на два
	sub r28,r26
	sbc r29,r27
	// теперь в регистрах r29:r28 хранится X1
	rjmp save_X1_to_ram // сохраняем Х1 в ОЗУ
	 
poloj:
	SBI PORTA,DIR ; показываем, что движемся вперёд
	// делим полученную разность на 2 сдвигом вправо
	lsr r31
	ror r30
	// перекладываем Х0 в регистры для Х1
	mov r28,r26
	mov r29,r27
	// прибавляем к Х0 полученную разницу, делённую на два
	add r28,r30
	adc r29,r31
	// теперь в регистрах r29:r28 хранится X1
// сохраняем Х1 в ОЗУ
save_X1_to_ram: 
	sts SHAGOVIK_X1_HIGH_RAM,r29
	sts SHAGOVIK_X1_LOW_RAM,r28
	// сохраняем в ПЗУ номер последней позиции, чтобы при следующем включении работа началась с этой позиции
	lds r16,INIT_FLAG
	cpi r16,0x00
	breq ok_next1 ; если было включение прибора, а не нажатие кнопки, то не сохраняем в ПЗУ значение текущей позиции, т.к. оно и так там находится
// сохраняем новое значение позиции в ПЗУ
position_number_save_to_eeprom:
	clr r18 ; для адреса
	ldi r17,0x00 ; адрес номера позиции в ПЗУ = 0x00
	lds r16,RAM_position_number ; считываем текущую позицию из EEPROM
	rcall EEPROM_write ; записываем в EEPROM
ok_next1:
	// считываем текущее положение X0 (начальное)
	lds r27,SHAGOVIK_X0_HIGH_RAM
	lds r26,SHAGOVIK_X0_LOW_RAM
	// считываем положение X2, куда надо отправиться
	lds r31,SHAGOVIK_X2_HIGH_RAM
	lds r30,SHAGOVIK_X2_LOW_RAM 

// второй этап
shagovik_stage2:
	// подготавливаем все нужные переменные
	mov r25,r27 ; в этих регистрах будет храниться текущее положение
	mov r24,r26
	;ldi r25,SHAGOVIK_freq_maxH ; в этом регистре будет храниться старший байт минимальное значение OCR1A (максимальная частота)
	;ldi r24,SHAGOVIK_freq_maxL ; в этом регистре будет храниться младший байт минимальное значение OCR1A (максимальная частота)
	;ldi r23,SHAGOVIK_freq_minH ; в этом регистре будет храниться старший байт максимальное значение OCR1A (минимальная частота)
	;ldi r22,SHAGOVIK_freq_minL ; в этом регистре будет храниться младший байт максимальное значение OCR1A (минимальная частота)
	;ldi r22,SHAGOVIK_pid_prop ; в этом регистре будет храниться пропорциональный коэффициент ПИДа
	;;ldi r21,SHAGOVIK_step ; в этом регистре будет храниться шаг драйвера шагового двигателя (ШД)
	ldi r21,SHAGOVIK_DELTA_L_LOW ; в этом регистре будет храниться константа, определяющая количество импульсов STEP, при которых будет пройден путь 0,1мм при шаге 1/16
	ldi r22,SHAGOVIK_DELTA_L_HIGH ; старший байт константы

	; в регистрах OCR1AH:OCR1AL находится текущая частота ШД

	// инициализируем таймер1 
	// обнуляем TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; записываем в TCNT1 r17:r16

	// записываем в OCR1A. Минимальную частоту, с которой начинается движение. В регистрах OCR1AH:OCR1AL находится текущая частота ШД
	ldi r16,SHAGOVIK_freq_minL
	ldi r17,SHAGOVIK_freq_minH
	rcall TIM16_WriteOCR1A ; записываем в OCR1A r17:r16


	// запускаем таймер1 
	//ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; предделитель 1
	//out TCCR1B,r16
	ldi r16,0x01 
	sts SHAGOVIK_timer1_on,r16 ; показываем, что timer1 включён

	//ldi r16,0b00000000
	//out GICR,r16 ; выключаем энкодер

	lds r16,INIT_FLAG ; если было влкючение, то выводим начальное положение
	cpi r16,0x00
	breq ok_exit

	// если функция вызвалась вращением энкодера, то не выводить на экран текущую позицию
	lds r16,ENCODER_CONTR_MOT_PRESS_OK
	cpi r16,0x01
	breq ok_exit
print_num:
	// сохраняем регистры
	push r16
	push r17
	push r18
	push r19
	push r20
	push r21 
	push r22
	push r23
	push r24
	push r25
	push r26
	push r27

	// выводим на дисплей новое положение Х
	ldi r16,0x40
	rcall Set_cursor
	
	// считываем значение текущего положения Х
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; выводим на дисплей 

	pop r27
	pop r26
	pop r25 
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
ok_exit:
	// сохраняем в ОЗУ регистры для передвижение шаговика
	sts SHAGOVIK_r31,r31
	sts SHAGOVIK_r30,r30
	sts SHAGOVIK_r29,r29
	sts SHAGOVIK_r28,r28
	sts SHAGOVIK_r27,r27
	sts SHAGOVIK_r26,r26
	sts SHAGOVIK_r25,r25
	sts SHAGOVIK_r24,r24
	sts SHAGOVIK_r23,r23
	sts SHAGOVIK_r22,r22
	sts SHAGOVIK_r21,r21
	sts SHAGOVIK_r20,r20
	sts SHAGOVIK_r19,r19

sei ;разрешаем прерывания
ret











Button_home_func: nop
	rcall Clock_start_for_block // включаем таймер, который включит блокировку через 10 секунд

	SBI PORTC,LED_home ; включаем светодиод кнопки HOME

	// проверяем напряжение на индуктивном датчике (ИД). Если ИД активен, то едем вперёд
	SBIC PINA,INDSIGNAL ; если активен
	SBI PORTA,DIR ; едем вперёд
	
	SBIS PINA,INDSIGNAL ; если не активен
	CBI PORTA,DIR ; едем назад

	// проверяем едем вперёд или назад, то есть на каком этапе находимся
	SBIS PORTA,DIR 
	rjmp edem_nazad
	ldi r26,0x00
	rcall low_speed ; ставим медленную скорость вращения

	ldi r22,HOME_SHAGOVIK_DELTA_HIGH_0 ; сколько нужно пройти на этапе 0 старший байт
	ldi r21,HOME_SHAGOVIK_DELTA_LOW_0  ; сколько нужно пройти на этапе 0 младший байт
	
	rjmp home_next
// если на 0 этапе
edem_nazad:
	// если на 1 этапе
	ldi r26,0x01 /* переменная, где хранится текущий этап режима HOME.
				    0 - ИД изначально был активен, поэтому едем вперёд 2 мм на медленной скорости, а потом сразу переходим к 4 этапу
				    1 - ИД изначально не был активен, поэтому едем назад на быстрой скорости, пока ИД не станет активным
				    2 - когда ИД стал активен, едем вперёд 1 мм на медленной скорости
				    3 - едем назад, пока ИД не станет активным
				    4 - ИД стал активным, едем назад ровно 1 мм в ноль
				 */
	rcall fast_speed ; ставим высокую скорость вращения
home_next:
	// инициализируем таймер1 
	// обнуляем TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; записываем в TCNT1 r17:r16

	// запускаем таймер1 
	//ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; предделитель 1
	//out TCCR1B,r16
	ldi r16,0x01 
	sts SHAGOVIK_timer1_on,r16 ; показываем, что timer1 включён

	ldi r16,0b00000000
	out GICR,r16 ; выключаем энкодер

	ldi r23,SHAGOVIK_DELTA_L_LOW
	ldi r24,SHAGOVIK_DELTA_L_HIGH
	// сохраняем в ОЗУ регистры для передвижение шаговика
	sts SHAGOVIK_r31,r31
	sts SHAGOVIK_r30,r30
	sts SHAGOVIK_r29,r29
	sts SHAGOVIK_r28,r28
	sts SHAGOVIK_r27,r27
	sts SHAGOVIK_r26,r26
	sts SHAGOVIK_r25,r25
	sts SHAGOVIK_r24,r24
	sts SHAGOVIK_r23,r23
	sts SHAGOVIK_r22,r22
	sts SHAGOVIK_r21,r21
	sts SHAGOVIK_r20,r20
	sts SHAGOVIK_r19,r19
ret










Button_prog_func:
	

	rcall Clock_start_for_block // включаем таймер, который включит блокировку через 10 секунд

	// проверяем, работает ли кнопка ПРОГ
	lds r16,PROG_FUNC_WORK_STATUS_RAM ; считываем флаг работы кнопки ПРОГ
	cpi r16,0x01 
	breq prog_next1 ; если во флаге оказалась единичка, то следуем дальше
	ret ; иначе выходим

prog_next1:
	
	rcall blink_prog_led



	// считываем текущий номер позиции
	lds r16,RAM_position_number

	// записываем положение Х данной позиции в ПЗУ 263 (26.3)
	ldi r17,0x02
	mul r16,r17
	mov r16,r0
	ldi r17,0x0A
	add r17,r16
	clr r18

	lds r16,SHAGOVIK_X2_HIGH_RAM  ; сначала старший байт текущего положения Х
	rcall EEPROM_write

	inc r17
	lds r16,SHAGOVIK_X2_LOW_RAM  ; теперь младший байт текущего положения Х
	rcall EEPROM_write

	

ret

blink_prog_led:
	cli
	// включаем светодиод кнопки ПРОГ
	sbi PORTC,LED_prog
		
	// пауза
	ldi r16,0x64
	rcall nopdelay_1ms
	cbi PORTC,LED_prog ; выключаем светодиод ПРОГа

	// пауза
	ldi r16,0x64
	rcall nopdelay_1ms
	sbi PORTC,LED_prog ; включаем светодиод ПРОГа

	// пауза
	//ldi r16,0x64
	//rcall nopdelay_1ms
	//cbi PORTC,LED_prog ; выключаем светодиод ПРОГа
	sei
ret


Button_block_func:


SBIS PORTC,LED_block ; если выключён светодиод кнопки Блок, то включаем и запрещаем нажатие всех кнопок и энкодера
rjmp block_on

block_off: ; выключаем режим Блока
CBI PORTC,LED_block
ldi r16,0b00000011 
sts access_to_button_flags,r16 ; запрещаем нажатие всех кнопок, кроме Блок
ldi r16,0b01000000
out GIFR,r16 ; убираем флаг сработавшего прерывания
out GICR,r16 ; выключаем прерывание по INT0, чтобы отключить обработку энкодера

rcall Clock_start_for_block // включаем таймер, который включит блокировку через 10 секунд

ret

block_on:; включаем режим Блока
SBI PORTC,LED_block
ldi r16,0b00000001 
sts access_to_button_flags,r16 ; разрешаем нажатие всех кнопок
ldi r16,0b00000000
out GICR,r16 ; INT0 в единицу для включения прерывания на пине INT0

rcall Clock_stop_for_block // вЫключаем таймер, который включит блокировку через 10 секунд

ret


// ф-ция изменения ШАГА. Варианты: 10.0, 01.0, 00.1
Button_step_func:
	rcall Clock_start_for_block // включаем таймер, который включит блокировку через 10 секунд

	lds r16,RAM_step_size ; считываем из ОЗУ текущее значения ШАГА

	cpi r16,0x0A ; если шаг равен 10 (01.0)
	breq set_step_1

	cpi r16,0x01 ; если шаг равен 1 (00.1)
	breq set_step_2

	; если шаг был равен 100 (10.0)
	ldi r16,0x01 ; новый ШАГ равен 1 (00.1)
	rjmp save_step

	set_step_1:
	ldi r16,0x64 ; новый ШАГ равен 100 (10.0)
	rjmp save_step

	set_step_2:
	ldi r16,0x0A ; новый ШАГ равен 10 (01.0)
	// сохранить новое значение ШАГА в ОЗУ
	save_step:
	sts RAM_step_size,r16

	// выводим значение ШАГА на дисплей
	mov r25,r16 ; сохраняем значения шага для дальнешего вывода на экран
	clr r26 
	ldi r16,0x46
	rcall Set_cursor ; ставим курсор для вывода значения шага

	rcall lcd_print_number ; выводим значение шага на дисплей

ret







// функция для создания символов. Максимум можно создать 8 символов. 
/*
byte bukva_B[8]   = {B11110,B10000,B10000,B11110,B10001,B10001,B11110,B00000,}; // Буква "Б"
byte bukva_G[8]   = {B11111,B10001,B10000,B10000,B10000,B10000,B10000,B00000,}; // Буква "Г"
byte bukva_D[8]   = {B01111,B00101,B00101,B01001,B10001,B11111,B10001,B00000,}; // Буква "Д"
byte bukva_ZH[8]  = {B10101,B10101,B10101,B11111,B10101,B10101,B10101,B00000,}; // Буква "Ж"
byte bukva_Z[8]   = {B01110,B10001,B00001,B00010,B00001,B10001,B01110,B00000,}; // Буква "З"
byte bukva_I[8]   = {B10001,B10011,B10011,B10101,B11001,B11001,B10001,B00000,}; // Буква "И"
byte bukva_IY[8]  = {B01110,B00000,B10001,B10011,B10101,B11001,B10001,B00000,}; // Буква "Й"
byte bukva_L[8]   = {B00011,B00111,B00101,B00101,B01101,B01001,B11001,B00000,}; // Буква "Л"
byte bukva_P[8]   = {B11111,B10001,B10001,B10001,B10001,B10001,B10001,B00000,}; // Буква "П"
byte bukva_Y[8]   = {B10001,B10001,B10001,B01010,B00100,B01000,B10000,B00000,}; // Буква "У"
byte bukva_F[8]   = {B00100,B11111,B10101,B10101,B11111,B00100,B00100,B00000,}; // Буква "Ф"
byte bukva_TS[8]  = {B10010,B10010,B10010,B10010,B10010,B10010,B11111,B00001,}; // Буква "Ц"
byte bukva_CH[8]  = {B10001,B10001,B10001,B01111,B00001,B00001,B00001,B00000,}; // Буква "Ч"
byte bukva_Sh[8]  = {B10101,B10101,B10101,B10101,B10101,B10101,B11111,B00000,}; // Буква "Ш"
byte bukva_Shch[8]= {B10101,B10101,B10101,B10101,B10101,B10101,B11111,B00001,}; // Буква "Щ"
byte bukva_Mz[8]  = {B10000,B10000,B10000,B11110,B10001,B10001,B11110,B00000,}; // Буква "Ь"
byte bukva_IYI[8] = {B10001,B10001,B10001,B11001,B10101,B10101,B11001,B00000,}; // Буква "Ы"
byte bukva_Yu[8]  = {B10010,B10101,B10101,B11101,B10101,B10101,B10010,B00000,}; // Буква "Ю"
byte bukva_Ya[8]  = {B01111,B10001,B10001,B01111,B00101,B01001,B10001,B00000,}; // Буква "Я"
*/
Create_Char:
	ldi r16,0b01000000 ; активируем 7-ю инструкцию для обращения к энергонезависимой памяти CGRAM, где DB6 равен единице, и адрес начинается с 0x00
	clr r17
	sts 0x0102,r17 ; RS и R/W в 0 для 7-й инструкции 
	rcall LCD1602_send_command ; отправляем данные

	// подготавливаем 8 байт для создания 1-ого символа в адрес 0x00 
	// символ "Ш"
	ldi r17,0b10101
	ldi r18,0b10101
	ldi r19,0b10101
	ldi r20,0b10101
	ldi r21,0b10101
	ldi r22,0b10101
	ldi r23,0b11111
	ldi r24,0b00000
	// записываем символ в дисплей
	rcall write_new_symbol

	// подготавливаем 8 байт для создания 2-ого символа в адрес 0x01 
	// символ "Г"
	ldi r17,0b11111
	ldi r18,0b10001
	ldi r19,0b10000
	ldi r20,0b10000
	ldi r21,0b10000
	ldi r22,0b10000
	ldi r23,0b10000
	ldi r24,0b00000
	// записываем символ в дисплей
	rcall write_new_symbol
	
	// подготавливаем 8 байт для создания 3-ого символа в адрес 0x02 
	// символ "П"
	ldi r17,0b11111
	ldi r18,0b10001
	ldi r19,0b10001
	ldi r20,0b10001
	ldi r21,0b10001
	ldi r22,0b10001
	ldi r23,0b10001
	ldi r24,0b00000
	// записываем символ в дисплей
	rcall write_new_symbol


	// подготавливаем 8 байт для создания 3-ого символа в адрес 0x03 
	// символ "З"
	ldi r17,0b01110
	ldi r18,0b10001
	ldi r19,0b00001
	ldi r20,0b00010
	ldi r21,0b00001
	ldi r22,0b10001
	ldi r23,0b01110
	ldi r24,0b00000
	// записываем символ в дисплей
	rcall write_new_symbol
ret


write_new_symbol:
// записываем символ в дисплей
	mov r16,r17
	rcall Write_symbol

	mov r16,r18
	rcall Write_symbol

	mov r16,r19
	rcall Write_symbol

	mov r16,r20
	rcall Write_symbol

	mov r16,r21
	rcall Write_symbol

	mov r16,r22
	rcall Write_symbol

	mov r16,r23
	rcall Write_symbol

	mov r16,r24
	rcall Write_symbol
ret






// ф-ция для вычитания двух байтов из двух
subtraction_2x2:
SUB r25,r23 ; вычитаем младшие байты
SBC r26,r24 ; вычитаем старшие байты, учитывая перенос
ret

// ф-ция для сложения двух байтов к двум
addition_2x2:
add r25,r23 ; складываем младшие байты
adc r26,r24 ; складываем старшие байты, учитывая перенос
ret




// функция деления два байта на два (рекомендуется только деление на 10). в r25 - целая часть. в r16 - остаток. r26,r26 / r24,r23
division_2x2: nop
	clr r16  ; счётчик вычитаний
	push r26 ; сохраняем старший байт в стеке
	push r25 ; сохраняем младший байт в стеке 
division_proc:
	sub r25,r23 ; вычитаем из уменьшаемого вычитаемое пока число не станет отрицательный
	sbc r26,r24 ; вычитаем, учитывая перенос
	inc r16     ; прибавляем к счётчик вычитаний, который в итоге даст ответ деления (его целой части)
	brcc division_proc

	dec r16 ; в r16 находится целая часть
	mul r23,r16 ; умножаем делитель на колиство полученных вычитаний, чтобы с помощью этого потом вычисл. остаток
	pop r25 ; возвращаем сохранённое значение из стека
	pop r26 ; возвращаем сохранённое значение из стека
	sub r25,r0  ; вычитаем из делимого целую часть полученную при делении, таким образом в r25 лежит остаток
	mov r0,r16  
	mov r16,r25 ; положить остаток в r16
	mov r25,r0  ; положить целую часть в r25
	clr r26     ; очистить r26 для последующих вычислений
ret








// функция конвертирует значение двух байт, чтобы вывести на дисплей в десятичном формате, XX.X
lcd_print_number: nop
	// сохраняем на всякий случай
	push r18
	push r17

	// начинаем процесс
	clr r18 ; счётчик
	clr r24
	cycle22:
	ldi r23,0x0A ; деление на 10
	rcall division_2x2 ; найти остаток от деления на 10, который будет в r16, целая часть от деления сохранится в r25
	ldi r24,0x30
	add r16,r24 ; прибавляем к остатоку 30h, посколько в 30h находится символ "0", таким образом выведется какая-то цифра
	clr r24
	// если вычленили младший десятичный символ
	cpi r18,0x00
	breq save_r2 ; то сохраняем в r2

	// если вычленили средний десятичный символ
	cpi r18,0x01
	breq save_r3 ; то сохраняем в r1

	// если вычленили старший десятичный символ
	cpi r18,0x02
	breq save_r4 ; то сохраняем в r0

	save_r4:
	mov r4,r16
	rjmp incc

	save_r3:
	mov r3,r16
	rjmp incc

	save_r2:
	mov r2,r16

	incc:
	inc r18 ; прибавляем к счётчик
	cpi r18,0x03
	brne cycle22

	cpi r20,0x4C
	breq print_next ; если курсор установлен на вывод позиции, то пропускаем вывод старшего символа

	mov r16,r4
	rcall Write_symbol ; выводим старший символ числа

print_next:
	mov r16,r3
	rcall Write_symbol ; выводим средний символ числа

	cpi r20,0x4C
	breq print_next2 ; если установлен флаг вывода номера позиции, то пропускаем вывод точки

	ldi r16,0x2E
	rcall Write_symbol ; выводим символ "."
print_next2:
	mov r16,r2
	rcall Write_symbol ; выводим младший символ числа
	clr r20
	pop r17
	pop r18
ret

/*  Set name of position
	For positions:
	00 - PARK
	01 - 450
	02 - 650
	03 - 900
	04 - 1350
*/
lcd_print_name_of_position: nop
	push r16
	push r22
	push r23
	push r24
	push r25

	// check position number and print name
	//if position number is 0
	cpi r25,0x00 
	breq print_PARK
	//if position number is 1
	cpi r25,0x01 
	breq print_450
	//if position number is 2
	cpi r25,0x02
	breq print_650
	//if position number is 3
	cpi r25,0x03
	breq print_900
	//if position number is 4
	cpi r25,0x04
	breq print_1350

print_PARK:
	ldi r22,SYM_P
	ldi r23,SYM_A
	ldi r24,SYM_R
	ldi r25,SYM_K
	rjmp print_and_exit
print_450:
	ldi r22,DIG_0
	ldi r23,DIG_4
	ldi r24,DIG_5
	ldi r25,DIG_0
	rjmp print_and_exit
print_650:
	ldi r22,DIG_0
	ldi r23,DIG_6
	ldi r24,DIG_5
	ldi r25,DIG_0	
	rjmp print_and_exit
print_900:
	ldi r22,DIG_0
	ldi r23,DIG_9
	ldi r24,DIG_0
	ldi r25,DIG_0
	rjmp print_and_exit
print_1350:
	ldi r22,DIG_1
	ldi r23,DIG_3
	ldi r24,DIG_5
	ldi r25,DIG_0
	rjmp print_and_exit

print_and_exit:
	ldi r16,0x4C
	rcall Set_cursor 

	mov r16,r22
	rcall Write_symbol

	mov r16,r23
	rcall Write_symbol

	mov r16,r24
	rcall Write_symbol

	mov r16,r25
	rcall Write_symbol

	pop r25
	pop r24
	pop r23
	pop r22
	pop r16
ret

lcd_print_number_or_name_of_position: nop
	cpi r25,0x05
	brsh print_number1 //branch if same or higher
	rcall lcd_print_name_of_position
	ret

print_number1:
	ldi r16,0x4C
	rcall Set_cursor 
	
	ldi r16,0x20
	rcall Write_symbol ; выводим символ " "

	rcall lcd_print_number ; выводим на дисплей

	ldi r16,0x20
	rcall Write_symbol ; выводим символ " "
ret






// обработчик кнопки. Функция вызывается спустя 100 мсек после нажатия кнопки
button_press: nop

	SBRC r17,0 ; если уже находились в процессе обработки кнопки
	rjmp next  ; то перейти к основной обработке

	// иначе 
	clr r18 ; счётчик миллисекунд

	// запускаем таймер
	clr r16 
	out TCNT0,r16 ; сбрасываем счётчик таймера 0
	ldi r16,0b00001011  ; WGM01 = 1 - CTC режим
	out TCCR0,r16 ; устанавливаем предделитель частоты на 64
	in r16,TIFR ; сохраняем в r16, чтобы не испортить значения регистра
	ori r16,0b00000010 
	out TIFR,r16 ; сбрасываем флаг прерывания
	SBR r17,1 ; устанавливаем флаг обработки прерывания
	ret ; выходим из прерывания

next: 
	SBRS r17,1 ; если произошло очередное прерывание
	ret ; то не выходить

	inc r18 ; прибавляем к счётчику миллисекунд
	CBR r17,2 ; сбрасываем единичный бит обратно для ожидания очередного прерывания таймера0

	SBRC r17,2 ; если пока находимся в стадии дребезга контактов
	rjmp main_process ; то не переходить к основной обработке
	
	SBRC r17,4 ; если пройден основной этап обработки кнопки и теперь отслеживаем отпускание
	rjmp end_process ; то перейти к отслеживанию отпускания кнопки

// участок подавления дребезга при нажатии на кнопку
debounce_process:
	cpi r18,debounce_time ; смотрим сколько прошло миллисекунд
	brne exit ; если ещё не прошло время дребезга, то выходим
	SBR r17,4 ; устанавливаем второй бит, показывая, что этап дребезга пройден
	clr r18   ; сбрасываем счётчик, чтобы начать новый отсчёт для этапа проверки на помехи
	rjmp check_pin ; проверяем уровень на пине

// основная обработка, когда дребезг устранили
main_process:
	cpi r18,check_pin_time ; проверяем прошло ли время проверки на помехи
	breq finish ; если прошло, то закругляемся и устанавливаем флаг вызова функции, на которую настроена кнопка
	rjmp check_pin

//
end_process:
	SBRS r17,5 ; если ещё не было зафиксировано отпускания кнопки
	rjmp check_pin ; то продолжаем отслеживать отпускание
    // иначе гасим дребезг при отпускании кнопки
	cpi r18,debounce_end_time ; если прошло debounce_end_time миллисекунд, то завершаем полностью обработку кнопки
	breq the_end ; завершаем обработку кнопки
	ret

// иначе проверяем кнопку на помеху (каждую миллисекунду)
check_pin:
	BRTS stop_end_exit ; если на пине оказался 1, то завершить обработку, так как это оказалась помеха
	ret ; иначе выйти

// финиш
finish:
	ori r17,0b00011000 ; устанавливаем 3-й бит, флаг разрешения вызова функции, прикреплённой на кнопку
                       ; устанавливаем 4-й бит, показывающий, что основной этап пройден и начан этап отслеживания отпускания
	CBR r17,4 ; сбрасываем 2-й бит, чтобы больше не заходить в main_process
	ret

// выйти и закончить обработку кнопки, так как произошла помеха
stop_end_exit:
	SBRS r17,4 ; если кнопка успешно сработала, то мы зафиксировали отпускание кнопки (end_process)
	rjmp the_end ; если не было успешного срабатывания кнопки, то завершаем обработку кнопки
	SBR r17,32 ; устанавливаем флаг того, что было зафиксировано отпускание кнопки после успешного его срабатывания
	clr r18    ; сбрасываем счётчик времени
	ret
	the_end:
	clr r16
	out TCCR0,r16 ; выключаем таймер
	clr r17 ; очищаем регистр флагов

exit:
ret







// обработчик кнопки прибавления
Button_up: nop
	SBRS r17,0 ; пропустить следующую команду, если уже находимся в процессе обработки какой-то кнопки
	rjmp step_up_next1 ; переходим далее
	//если уже находимся в процессе обработки кнопки, то проверяем, ту ли кнопку сейчас обрабатываем
	lds r16,RAM_ACTIVE_BUTTON ; считываем из ОЗУ, какая кнопка сейчас активна
	cpi r16,RAM_BUTTON_UP ; сравниваем
	breq step_up_next1 ; если это кнопка данной функции, то идём обрабатывать дальше
	ret ; иначе это не та кнопка, которая сейчас обрабатывается, выходим

// если находимся в процессе обработки данной кнопки или нет никакого процесса обработки кнопки
step_up_next1:
	in r16,PINB ; считываем состояние пинов порта 
	com r16 ; инвертируем состояние пинов. убрать эту строчку, если активный уровень нажатия кнопки = 0.
	BST r16,PB4 ; записываем фо флаг Т значение пина
	SBRC r17,0 ; если в бите 1, ты выполнить нижнию команду
	rjmp step_up_next3 ; если уже находились в процессе обработки данной кнопки 
	BRTC step_up_next2 ; если ещё не находились в процессе обработки кнопки и если такая-то кнопка была нажата
	ret ; если кнопка не нажималась и не было никакого процесса обработки, то просто выходим

// так как кнопка нажалась впервые, то сохраняем в ОЗУ информацию о том, какую кнопку нажали, чтобы обрабатывать только её
step_up_next2:
    ldi r16,RAM_BUTTON_UP
	sts RAM_ACTIVE_BUTTON,r16
step_up_next3:
	rcall button_press ; вызываем обработчик нажатия кнопки, в котором ф-ция вызывается спустя 100 мсек
	push r17 ; сохраняем регистр прежде чем перейти в другую функцию
	push r18
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не вызывать
	rcall Button_up_func ; вызвать ф-цию, прикреплённую на кнопку
	pop r18
	pop r17 ; возвращаем r17
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не сбрасывать регистр флагов
	CBR r17,8 ; очистить флаг вызова функции, чтобы больше её не вызывать
ret










// обработчик кнопки прибавления
Button_down: nop
	SBRS r17,0 ; пропустить следующую команду, если уже находимся в процессе обработки какой-то кнопки
	rjmp step_down_next1 ; переходим далее
	//если уже находимся в процессе обработки кнопки, то проверяем, ту ли кнопку сейчас обрабатываем
	lds r16,RAM_ACTIVE_BUTTON ; считываем из ОЗУ, какая кнопка сейчас активна
	cpi r16,RAM_BUTTON_DOWN ; сравниваем
	breq step_down_next1 ; если это кнопка данной функции, то идём обрабатывать дальше
	ret ; иначе это не та кнопка, которая сейчас обрабатывается, выходим

// если находимся в процессе обработки данной кнопки или нет никакого процесса обработки кнопки
step_down_next1:
	in r16,PINB ; считываем состояние пинов порта 
	com r16 ; инвертируем состояние пинов. убрать эту строчку, если активный уровень нажатия кнопки = 0.
	BST r16,PB3 ; записываем фо флаг Т значение пина
	SBRC r17,0 ; если в бите 1, ты выполнить нижнию команду
	rjmp step_down_next3 ; если уже находились в процессе обработки данной кнопки 
	BRTC step_down_next2 ; если ещё не находились в процессе обработки кнопки и если такая-то кнопка была нажата
	ret ; если кнопка не нажималась и не было никакого процесса обработки, то просто выходим

// так как кнопка нажалась впервые, то сохраняем в ОЗУ информацию о том, какую кнопку нажали, чтобы обрабатывать только её
step_down_next2:
    ldi r16,RAM_BUTTON_DOWN
	sts RAM_ACTIVE_BUTTON,r16
step_down_next3:
	rcall button_press ; вызываем обработчик нажатия кнопки, в котором ф-ция вызывается спустя 100 мсек
	push r17 ; сохраняем регистр прежде чем перейти в другую функцию
	push r18
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не вызывать
	rcall Button_down_func ; вызвать ф-цию, прикреплённую на кнопку
	pop r18
	pop r17 ; возвращаем r17
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не сбрасывать регистр флагов
	CBR r17,8 ; очистить флаг вызова функции, чтобы больше её не вызывать
ret








// обработчик кнопки прибавления
Button_ok: nop
	SBRS r17,0 ; пропустить следующую команду, если уже находимся в процессе обработки какой-то кнопки
	rjmp step_ok_next1 ; переходим далее
	//если уже находимся в процессе обработки кнопки, то проверяем, ту ли кнопку сейчас обрабатываем
	lds r16,RAM_ACTIVE_BUTTON ; считываем из ОЗУ, какая кнопка сейчас активна
	cpi r16,RAM_BUTTON_OK ; сравниваем
	breq step_ok_next1 ; если это кнопка данной функции, то идём обрабатывать дальше
	ret ; иначе это не та кнопка, которая сейчас обрабатывается, выходим

// если находимся в процессе обработки данной кнопки или нет никакого процесса обработки кнопки
step_ok_next1:
	in r16,PINB ; считываем состояние пинов порта 
	com r16 ; инвертируем состояние пинов. убрать эту строчку, если активный уровень нажатия кнопки = 0.
	BST r16,PB2 ; записываем фо флаг Т значение пина
	SBRC r17,0 ; если в бите 1, ты выполнить нижнию команду
	rjmp step_ok_next3 ; если уже находились в процессе обработки данной кнопки 
	BRTC step_ok_next2 ; если ещё не находились в процессе обработки кнопки и если такая-то кнопка была нажата
	ret ; если кнопка не нажималась и не было никакого процесса обработки, то просто выходим

// так как кнопка нажалась впервые, то сохраняем в ОЗУ информацию о том, какую кнопку нажали, чтобы обрабатывать только её
step_ok_next2:
    ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16
step_ok_next3:
	rcall button_press ; вызываем обработчик нажатия кнопки, в котором ф-ция вызывается спустя 100 мсек
	push r17 ; сохраняем регистр прежде чем перейти в другую функцию
	push r18
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не вызывать
	rcall Button_ok_func ; вызвать ф-цию, прикреплённую на кнопку
	pop r18
	pop r17 ; возвращаем r17
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не сбрасывать регистр флагов
	CBR r17,8 ; очистить флаг вызова функции, чтобы больше её не вызывать
ret









// обработчик кнопки прибавления
Button_home: nop
	SBRS r17,0 ; пропустить следующую команду, если уже находимся в процессе обработки какой-то кнопки
	rjmp step_home_next1 ; переходим далее
	//если уже находимся в процессе обработки кнопки, то проверяем, ту ли кнопку сейчас обрабатываем
	lds r16,RAM_ACTIVE_BUTTON ; считываем из ОЗУ, какая кнопка сейчас активна
	cpi r16,RAM_BUTTON_HOME ; сравниваем
	breq step_home_next1 ; если это кнопка данной функции, то идём обрабатывать дальше
	ret ; иначе это не та кнопка, которая сейчас обрабатывается, выходим

// если находимся в процессе обработки данной кнопки или нет никакого процесса обработки кнопки
step_home_next1:
	in r16,PINA ; считываем состояние пинов порта 
	com r16 ; инвертируем состояние пинов. убрать эту строчку, если активный уровень нажатия кнопки = 0.
	BST r16,PA1 ; записываем фо флаг Т значение пина
	SBRC r17,0 ; если в бите 1, ты выполнить нижнию команду
	rjmp step_home_next3 ; если уже находились в процессе обработки данной кнопки 
	BRTC step_home_next2 ; если ещё не находились в процессе обработки кнопки и если такая-то кнопка была нажата
	ret ; если кнопка не нажималась и не было никакого процесса обработки, то просто выходим

// так как кнопка нажалась впервые, то сохраняем в ОЗУ информацию о том, какую кнопку нажали, чтобы обрабатывать только её
step_home_next2:
    ldi r16,RAM_BUTTON_HOME
	sts RAM_ACTIVE_BUTTON,r16
step_home_next3:
	rcall button_press ; вызываем обработчик нажатия кнопки, в котором ф-ция вызывается спустя 100 мсек
	push r17 ; сохраняем регистр прежде чем перейти в другую функцию
	push r18
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не вызывать
	rcall Button_home_func ; вызвать ф-цию, прикреплённую на кнопку
	pop r18
	pop r17 ; возвращаем r17
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не сбрасывать регистр флагов
	CBR r17,8 ; очистить флаг вызова функции, чтобы больше её не вызывать
ret





// обработчик кнопки прибавления
Button_prog: nop
	SBRS r17,0 ; пропустить следующую команду, если уже находимся в процессе обработки какой-то кнопки
	rjmp step_prog_next1 ; переходим далее
	//если уже находимся в процессе обработки кнопки, то проверяем, ту ли кнопку сейчас обрабатываем
	lds r16,RAM_ACTIVE_BUTTON ; считываем из ОЗУ, какая кнопка сейчас активна
	cpi r16,RAM_BUTTON_PROG ; сравниваем
	breq step_prog_next1 ; если это кнопка данной функции, то идём обрабатывать дальше
	ret ; иначе это не та кнопка, которая сейчас обрабатывается, выходим

// если находимся в процессе обработки данной кнопки или нет никакого процесса обработки кнопки
step_prog_next1:
	in r16,PINA ; считываем состояние пинов порта 
	com r16 ; инвертируем состояние пинов. убрать эту строчку, если активный уровень нажатия кнопки = 0.
	BST r16,PA0 ; записываем фо флаг Т значение пина
	SBRC r17,0 ; если в бите 1, ты выполнить нижнию команду
	rjmp step_prog_next3 ; если уже находились в процессе обработки данной кнопки 
	BRTC step_prog_next2 ; если ещё не находились в процессе обработки кнопки и если такая-то кнопка была нажата
	ret ; если кнопка не нажималась и не было никакого процесса обработки, то просто выходим

// так как кнопка нажалась впервые, то сохраняем в ОЗУ информацию о том, какую кнопку нажали, чтобы обрабатывать только её
step_prog_next2:
    ldi r16,RAM_BUTTON_PROG
	sts RAM_ACTIVE_BUTTON,r16
step_prog_next3:
	rcall button_press ; вызываем обработчик нажатия кнопки, в котором ф-ция вызывается спустя 100 мсек
	push r17 ; сохраняем регистр прежде чем перейти в другую функцию
	push r18
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не вызывать
	rcall Button_prog_func ; вызвать ф-цию, прикреплённую на кнопку
	pop r18
	pop r17 ; возвращаем r17
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не сбрасывать регистр флагов
	CBR r17,8 ; очистить флаг вызова функции, чтобы больше её не вызывать
ret









// обработчик кнопки прибавления
Button_step: nop
	SBRS r17,0 ; пропустить следующую команду, если уже находимся в процессе обработки какой-то кнопки
	rjmp step_step_next1 ; переходим далее
	//если уже находимся в процессе обработки кнопки, то проверяем, ту ли кнопку сейчас обрабатываем
	lds r16,RAM_ACTIVE_BUTTON ; считываем из ОЗУ, какая кнопка сейчас активна
	cpi r16,RAM_BUTTON_STEP ; сравниваем
	breq step_step_next1 ; если это кнопка данной функции, то идём обрабатывать дальше
	ret ; иначе это не та кнопка, которая сейчас обрабатывается, выходим

// если находимся в процессе обработки данной кнопки или нет никакого процесса обработки кнопки
step_step_next1:
	in r16,PINA ; считываем состояние пинов порта 
	com r16 ; инвертируем состояние пинов. убрать эту строчку, если активный уровень нажатия кнопки = 0.
	BST r16,PA2 ; записываем фо флаг Т значение пина
	SBRC r17,0 ; если в бите 1, ты выполнить нижнию команду
	rjmp step_step_next3 ; если уже находились в процессе обработки данной кнопки 
	BRTC step_step_next2 ; если ещё не находились в процессе обработки кнопки и если такая-то кнопка была нажата
	ret ; если кнопка не нажималась и не было никакого процесса обработки, то просто выходим

// так как кнопка нажалась впервые, то сохраняем в ОЗУ информацию о том, какую кнопку нажали, чтобы обрабатывать только её
step_step_next2:
    ldi r16,RAM_BUTTON_STEP
	sts RAM_ACTIVE_BUTTON,r16
step_step_next3:
	rcall button_press ; вызываем обработчик нажатия кнопки, в котором ф-ция вызывается спустя 100 мсек
	push r17 ; сохраняем регистр прежде чем перейти в другую функцию
	push r18
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не вызывать
	rcall Button_step_func ; вызвать ф-цию, прикреплённую на кнопку
	pop r18
	pop r17 ; возвращаем r17
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не сбрасывать регистр флагов
	CBR r17,8 ; очистить флаг вызова функции, чтобы больше её не вызывать
ret



// обработчик кнопки прибавления
Button_block: nop
	SBRS r17,0 ; пропустить следующую команду, если уже находимся в процессе обработки какой-то кнопки
	rjmp block_block_next1 ; переходим далее
	//если уже находимся в процессе обработки кнопки, то проверяем, ту ли кнопку сейчас обрабатываем
	lds r16,RAM_ACTIVE_BUTTON ; считываем из ОЗУ, какая кнопка сейчас активна
	cpi r16,RAM_BUTTON_BLOCK ; сравниваем
	breq block_block_next1 ; если это кнопка данной функции, то идём обрабатывать дальше
	ret ; иначе это не та кнопка, которая сейчас обрабатывается, выходим

// если находимся в процессе обработки данной кнопки или нет никакого процесса обработки кнопки
block_block_next1:
	in r16,PINB ; считываем состояние пинов порта 
	com r16 ; инвертируем состояние пинов. убрать эту строчку, если активный уровень нажатия кнопки = 0.
	BST r16,PB1 ; записываем фо флаг Т значение пина
	SBRC r17,0 ; если в бите 1, ты выполнить нижнию команду
	rjmp block_block_next3 ; если уже находились в процессе обработки данной кнопки 
	BRTC block_block_next2 ; если ещё не находились в процессе обработки кнопки и если такая-то кнопка была нажата
	ret ; если кнопка не нажималась и не было никакого процесса обработки, то просто выходим

// так как кнопка нажалась впервые, то сохраняем в ОЗУ информацию о том, какую кнопку нажали, чтобы обрабатывать только её
block_block_next2:
    ldi r16,RAM_BUTTON_BLOCK
	sts RAM_ACTIVE_BUTTON,r16
block_block_next3:
	rcall button_press ; вызываем обработчик нажатия кнопки, в котором ф-ция вызывается спустя 100 мсек
	push r17 ; сохраняем регистр прежде чем перейти в другую функцию
	push r18
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не вызывать
	rcall Button_block_func ; вызвать ф-цию, прикреплённую на кнопку
	pop r18
	pop r17 ; возвращаем r17
	SBRC r17,3 ; если флаг разрешения вызова функции, прикреплённой на кнопку, не установлен, то не сбрасывать регистр флагов
	CBR r17,8 ; очистить флаг вызова функции, чтобы больше её не вызывать
ret







EEPROM_write: nop
	cli ; выключаем прерывания, т.к. во время ожидания предыдущей записи возникшие прерывания могут испортить регистры r16,r17,r18

	// Ждём пока закончится предыдущая запись
	sbic EECR,EEWE
	rjmp EEPROM_write
	// Устанавливаем адрес (r18:r17) в регистер адреса
	out EEARH, r18
	out EEARL, r17
	// Записываем байт (r16) в регистр данных, которые нужно записать
	out EEDR,r16
	// Записываем логическую единицу в EEMWE
	sbi EECR,EEMWE
	// Начинаем запись в EEPROM установлением бита EEWE в регистре EECR
	sbi EECR,EEWE

	sei
ret


EEPROM_read:
	// Ждём пока закончится предыдущая запись
	cli

	sbic EECR,EEWE
	rjmp EEPROM_read
	// Устанавливаем адрес (r18:r17) в регистер адреса
	out EEARH, r18
	out EEARL, r17
	// Начинаем чтение из EEPROM установлением бита EERE в регистре EECR
	sbi EECR,EERE
	// считываем байт из регистра данных
	in r16,EEDR

	sei
ret




// ф-ция перемножения двух байт. r19:r18 * r17:r16. Рез-т в r7:r6:r5:r4
umnoj_mul_2x2:
clr r7 ; очищаем регистр под старший байт рез-тата 
clr r6 ; очищаем регистр под 3-й байт рез-тата 
clr r5 ; очищаем регистр под 2-й байт рез-тата 
clr r4 ; очищаем регистр под младший байт рез-тата 
ldi r20,0x00 ; 

mul r18,r16 ; умножаем младший байт множимого на младший байт множителя
add r4,r0 ; 
adc r5,r1
adc r6,r20
adc r7,r20
mul r18,r17 ; умножаем младший байт множимого на старший байт множителя
add r5,r0
adc r6,r1
adc r7,r20
mul r19,r16 ; умножаем старший байт множимого на младший байт множителя
add r5,r0
adc r6,r1
adc r7,r20
mul r19,r17 ; умножаем старший байт множимого на старший байт множителя
add r6,r0
adc r7,r1
ret


// Для кнопки HOME. Активировать быстрое вращение ШД
fast_speed:
// записываем в OCR1A частоту быстрого вращения для HOME
	ldi r16,HOME_SHAGOVIK_freq_maxL
	ldi r17,HOME_SHAGOVIK_freq_maxH
	rcall TIM16_WriteOCR1A ; записываем в OCR1A r17:r16

	// обнуляем TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; записываем в TCNT1 r17:r16
ret

// Для кнопки HOME. Активировать медленно вращение ШД
low_speed:
// записываем в OCR1A частоту медленного вращения для HOME
	ldi r16,HOME_SHAGOVIK_freq_minL
	ldi r17,HOME_SHAGOVIK_freq_minH
	rcall TIM16_WriteOCR1A ; записываем в OCR1A r17:r16

	// обнуляем TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; записываем в TCNT1 r17:r16
ret

otladka:
	
/*
	ldi r16,0x01
	sts RAM_step_size,r16
	ldi r16,0x00
	sts SHAGOVIK_X2_HIGH_RAM ,r16
	sts RAM_position_number,r16
	ldi r19,0x63
	go:
	push r16
	rcall Button_up_func
	pop r16
	clr r30
	sts SHAGOVIK_X2_HIGH_RAM ,r30
	rcall EXT_INT0
	inc r16
	push r16
	rcall Button_prog_func
	pop r16
	dec r19
	brne go
	*/
	/*
	ldi r19,0x64
	ldi r16,0x00
	sts SHAGOVIK_X2_HIGH_RAM ,r16
	sts RAM_position_number,r16
	go1:
	rcall Button_up_func
	inc r16
	push r16
	rcall get_position_value_from_eeprom
	rcall Button_prog_func
	pop r16
	dec r19
	brne go1
	*/







	// записываем номер последней позиции в ПЗУ
	/*ldi r19,0x64
	ldi r17,0x00
	otladka1:
	clr r18
	ldi r16,0x00 ; 3-я позиция
	rcall EEPROM_write
	inc r17
	dec r19
	brne otladka1
	*/
	/*
	ldi r16,0x01
	sts RAM_position_number,r16

	// записываем положение Х данной позиции в ПЗУ 263 (26.3)
	ldi r17,0x02
	mul r17,r16
	ldi r17,0x0A
	add r17,r0
	clr r18
	ldi r16,0x01 ; сначала старший байт положения Х
	rcall EEPROM_write
	ldi r16,0x01
	sts SHAGOVIK_X2_HIGH_RAM ,r16

	inc r17
	ldi r16,0x07 ; младший байт положения Х
	rcall EEPROM_write
	ldi r16,0x07
	sts SHAGOVIK_X2_LOW_RAM ,r16

	ldi r16,0x0A
	sts RAM_step_size,r16
	*/
	
ret

// крутимся здесь, пока доберёмся до Х2
interrupt_process:
	lds r16,SHAGOVIK_timer1_on ; смотрим включён ли таймер 1 (двигается ли ШД)
	SBRS r16,0 ; если бит 0 установлен, то заходим в цикл, пока таймер 1 не отключится
	ret ; иначе выходим из ф-ции
	// включаем timer1
	ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; предделитель 1
	out TCCR1B,r16
	// выключаем таймер 0
	in r16,TIMSK
	andi r16,0b11111101  
	out TIMSK,r16 
wait_untill_timer1_stop:
	//rcall lcd_printing_number
	rcall encoder_print_num ; выводим на экран значение позиции Х2, если вращался энкодер
	lds r16,SHAGOVIK_timer1_on ; смотрим включён ли таймер 1 (двигается ли ШД)
	SBRC r16,0 ; 
	rjmp wait_untill_timer1_stop
	ldi r17,0x00
ret



home_and_go_to_last_position:
	// участок, отвечающий за включение и выключение автоматического включение этой функции при включении машины
	// считываем статус автоматического включения функции
	ldi r18,0x00
	ldi r17,0x02 ; EEPROM адрес 0x0002
	rcall EEPROM_read
	// сначала проверяем нужно включить автоматическое включение этой функции или нет (проверяем нажата ли кнопка HOME)
	SBIS PINA,1 ; если кнопка HOME нажата, то пропустить следующую команду
	rjmp hag_1

	SBIC PINB,1 ; если кнопка BLOCK нажата, то переходим на hag_1
	rjmp hag_1

	SBIC PINB,2 ; если кнопка OK нажата, то переходим на hag_1
	rjmp hag_1


	// если кнопка оказалась нажатой, то проверяем равно ли значение статуса авт. вкл. нулю и если нет, то записываем 0xff иначе 0x00
	cpi r16,0x00
	breq set0xff
	// если ставим 0
	lds r16,SHAGOVIK_X0_LOW_RAM
	ldi r17,0x03
	clr r18
	rcall EEPROM_write
	lds r16,SHAGOVIK_X0_HIGH_RAM
	ldi r17,0x04
	clr r18
	rcall EEPROM_write
	clr r16
	rjmp save_auto_to_eeprom
set0xff:
	ldi r16,0xFF ; инвертируем считанное значение из EEPROM
save_auto_to_eeprom:
	ldi r18,0x00
	ldi r17,0x02 ; EEPROM адрес 0x0002
	rcall EEPROM_write ; записываем новый статус автоматического включения
	cpi r16,0x00 ; если статус авт. вкл. ф-ции равен 0, то не включаем функцию автоматически
	breq auto_on ; если статус был равен 0xFF, то автоматически включить
	wait_press_of_ok1:
	SBIC PINA,1
	rjmp wait_press_of_ok1
	//CBI PORTC,LED_ok ; гасим светодиод кнопки ОК
	rjmp ext99
// проверяем включено ли автоматическое включение функции при включении машины
hag_1:
	cpi r16,0x00 ; если статус авт. вкл. ф-ции равен 0, то не включаем функцию автоматически
	//brne auto_on ; если статус был равен 0xFF, то автоматически включить
	breq auto_on ; если статус был равен 0xFF, то автоматически включить

	SBIC PINB,1 ; если кнопка BLOCK нажата, то переходим на ext99
	rjmp ext99

	SBIC PINB,2 ; если кнопка OK нажата, то переходим на ext99
	rjmp ext99
	// иначе ждём, пока нажмётся кнопка ОК
	// зажигаем светодиод кнопки ОК
	//SBI PORTC,LED_ok
wait_press_of_ok:
	SBIC PINA,1
	rjmp wait_press_of_ok
	//CBI PORTC,LED_ok ; гасим светодиод кнопки ОК
	rjmp ext99
auto_on:
	ldi r16,0x00
	sts INIT_FLAG,r16 ; показываем, что было не нажатие кнопки, а включение 
	ldi r16,RAM_BUTTON_HOME
	sts RAM_ACTIVE_BUTTON,r16 ; показываем, что обрабатываем кнопку HOME
	rcall Button_home_func ; возвращаемся в нулевое положение
	rcall interrupt_process ; крутимся здесь, пока не достигнем нулевого положение
	rcall get_position_value_from_eeprom ; считываем положение, в которое надо придти 
	ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16 ; показываем, что обрабатываем кнопку OK
	rcall Button_ok_func ; едем в положение, которое было в последний раз
	rcall interrupt_process ; крутимся здесь, пока не достигнем нужного положения
	ldi r16,0x01
	sts INIT_FLAG,r16 ; показываем, что включение было произведено, чтобы при нажатии на кнопку HOME, в конце производился вывод нулевого положения на экран
ext99:
    ldi r16,0x01
	sts INIT_FLAG,r16 ; показываем, что включение было произведено, чтобы при нажатии на кнопку HOME, в конце производился вывод нулевого положения на экран
ret



save_many_value_to_eeprom:
	ldi r16,0x00 ; записываемые значения
	ldi r18,0x00 ; старший байт адреса в ПЗУ
	ldi r17,0x0A ; младший байт адреса в ПЗУ
	ldi r19,0x64 ; сколько значений подряд надо записать в ПЗУ
	save_many_cycle:
	rcall EEPROM_Write
	ldi r20,0x01
	add r17,r20 ; получаем следующий адрес, в который надо записать данные
	
	dec r19
	brne save_many_cycle ; если пока не записали r19 значение, то продолжаем записывать в ПЗУ
	ldi r16,0x00 ; записываемые значения
	ldi r18,0x00 ; старший байт адреса в ПЗУ
	ldi r17,0x00 ; младший байт адреса в ПЗУ
	rcall EEPROM_Write
ret

/*
lcd_printing_number:
	cpi r21,SHAGOVIK_DELTA_L
	brne print_exit
	// выводим на дисплей новое положение Х
	rcall push_group_registers
	push r25
	push r24
	ldi r16,0x40
	rcall Set_cursor
	pop r24
	pop r25
	// считываем значение текущего положения Х
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; выводим на дисплей 
	rcall pop_group_registers
print_exit:
ret
*/


TIM2_COMP: nop
	
	push r16
	push r17
	push r18
	push r19
	push r20
	// сохраняем SREG, чтобы не испортить флаги в случае выполнения breq, brne и т.п. в обычных функциях. В конце каждой функции возвращаем обратно, как с push и pop
	in r16,SREG
	sts SREG_temp,r16 

	//rcall millis // прибавляем к счётчику миллисекунд
	

	clr r16
	out TCNT2,r16
	// считываем регистры счётчика из ОЗУ
	lds r18,TIM2_counter_high
	lds r17,TIM2_counter_low
	ldi r16,0x01

	// прибавляем единицу
	add r17,r16
	clr r16
	adc r18,r16

	// проверяем входит ли в диапазон от 0 до 520
	ldi r16,0xF5
	CPSE r17,r16
	rjmp save_counter
	ldi r16,0x01
	CPSE r18,r16
	rjmp save_counter
// если уже 521, то обнуляем
obnulenie_counter:
	clr r16
	sts TIM2_counter_high,r16
	sts TIM2_counter_low,r16
	rjmp tim2_ext
// просто сохраняем
save_counter:
	sts TIM2_counter_high,r18
	sts TIM2_counter_low,r17

// конец
tim2_ext:
	pop r20
	pop r19
	pop r18
	pop r17
	// возвращаем SREG, чтобы не испортить флаги в случае выполнения breq, brne и т.п. в обычных функциях. В конце каждой функции возвращаем обратно, как с push и pop
	lds r16,SREG_temp
	out SREG,r16
	pop r16 ; возвращаем обязательно (для ф-ции interrupt_process)
reti




// прерывание обрабатывает вращение энкодера
EXT_INT0:
    push r16
    push r17
	push r18
	push r19
	push r20
	push r21 
	push r22
	push r23
	push r24
	push r25
	push r26
	push r27

	

	// сохраняем SREG, чтобы не испортить флаги в случае выполнения breq, brne и т.п. в обычных функциях. В конце каждой функции возвращаем обратно, как с push и pop
	in r16,SREG
	sts SREG_temp,r16 

	lds r16,PROG_FUNC_WORK_STATUS_RAM
	cpi r16,0x00
	breq ext_int0_exit

	rcall Clock_start_for_block // включаем таймер, который включит блокировку через 10 секунд

SBIS PINA,B1 ; пропустить следующую команду, если 1 на ножке B1 (если вращение вперёд)
rjmp rotate_forward ; если на ножке B1 0, то идёт вращение вперёд

rotate_reverse:
/*вычитаем из позиции шаг и выводим на дисплей*/
// считываем позицию
lds r26,SHAGOVIK_X_TRULY_HIGH_RAM  ; считываем старший байт физического положения из ОЗУ
lds r25,SHAGOVIK_X_TRULY_LOW_RAM   ; считываем младший байт физического положения из ОЗУ
//считываем шаг
clr r24 ; очищаем старший байт вычитаемого
lds r23,RAM_step_size ; считываем шаг (младший байт вычитаемого) 
// вычитаем шаг (вычитаемое) из позиции (уменьшаемого), разность записывается в позицию
rcall subtraction_2x2 ; вызываем функцию вычитания (r26-r25 - r24-r23)
cpi r26,0xFF    ; если в старшем байте позиции оказалось число около 65000, то значения шага оказалось больше,
breq obnulenie	; поэтому приравниваем значение позиции к нулю, т.к. отрицательной позиции нет

rjmp save_position ; сохраняем значение позиции в ОЗУ
obnulenie:
clr r25
clr r26
rjmp save_position ; сохраняем значение позиции в ОЗУ

rotate_forward:
/*прибавляем к позиции шаг и выводим на дисплей*/
// считываем позицию
lds r26,SHAGOVIK_X_TRULY_HIGH_RAM  ; считываем старший байт физического положения из ОЗУ
lds r25,SHAGOVIK_X_TRULY_LOW_RAM   ; считываем младший байт физического положения из ОЗУ
//считываем шаг
clr r24 ; очищаем старший байт 
lds r23,RAM_step_size ; считываем шаг (младший байт вычитаемого) 
// прибавляем шаг (второе слагаемое) к позиции (первое слагаемое), сумма записывается в позицию
rcall addition_2x2 ; вызываем функцию сложения (r26-r25 + r24-r23)
// если в старшем байте позиции оказалось число больше 700 (70.0 мм) (0x02BC)
cpi r26,0x02 ; сравниваем старший байт сначала
BRSH priravnenie_max ; если старший байт позиции оказался больше 0x03 или равно ему 
cpi r25,0xF5 ; если старший байт оказался всё-таки меньше, то сравнимаем теперь младшую часть
BRLO save_position ; если же старшая часть оказалась меньше 0xBD, то переходим к сохранению
cpi r26,0x01 ; если старший байт позиции при большой младшей части равен 0x02, то число оказалось больше 700
breq priravnenie_max
; иначе позиция оказалась меньше 700 (70.0), тогда сохраняем без изменений
rjmp save_position
priravnenie_max:
ldi r26,0x01
ldi r25,0xF4 ; придаём позиции значение 700 (70.0)

save_position:
sts SHAGOVIK_X2_HIGH_RAM,r26 ; записываем в ОЗУ старший байт позиции
sts SHAGOVIK_X2_LOW_RAM,r25  ; записываем в ОЗУ младший байт позиции

sts SHAGOVIK_X_TRULY_HIGH_RAM ,r26 ; записываем в ОЗУ старший байт позиции
sts SHAGOVIK_X_TRULY_LOW_RAM,r25  ; записываем в ОЗУ младший байт позиции

/*
ldi r16,0x40
rcall Set_cursor ; ставим курсор на место числа позиции
rcall lcd_print_number ; выводим на экран [r26][r25] (позицию) в десятичном формате
*/
rcall controlling_motor

ext_int0_exit:
	// возвращаем SREG, чтобы не испортить флаги в случае выполнения breq, brne и т.п. в обычных функциях. В конце каждой функции возвращаем обратно, как с push и pop
	lds r16,SREG_temp
	out SREG,r16

	pop r27
	pop r26
	pop r25 
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
reti

// функция отвечает за передвижение шагового двигателя при вращении энкодера
controlling_motor:
	
	// выставить флаг, чтобы вывести число на экран и чтобы нажать кнопку ОК
	ldi r16,0x01
	sts ENCODER_CONTR_MOT_PRESS_OK,r16 
	sts ENCODER_CONTR_MOT_PRINT_X2,r16
	sts ENCODER_CONTR_MOT_ENCODER_PROCESSING,r16 ; показываем, что кнопка нажимается за счёт энкодера
ret

// функция проверяет было ли вращение энкодера и если было, то вызывает функцию кнопки ОК
check_encoder_press:
	lds r16,ENCODER_CONTR_MOT_PRESS_OK
	cpi r16,0x01
	brne c_e_p_exit
	ldi r16,0x01
    sts ENCODER_CONTR_MOT_ENCODER_PROCESSING,r16 ; показываем, что кнопка нажимается за счёт энкодера
	ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16 ; показываем, что якобы нажали кнопку ОК - необходимо, чтобы зайти глубоко в функции прерывания, т.к. там проверка на кнопку
	rcall Button_ok_func    // вызываем функцию кнопки ОК
	
	// устанавливаем флаг начала отсчёта 3 секунд, чтобы по истечении сохранить текущую позицию Х в EEPROM
	ldi r16,0x01
	sts ENCODER_CONTR_MOT_SAVE_X,r16
	clr r16
	sts ENCODER_CONTR_MOT_PRESS_OK,r16 // показываем, что кнопку нажали
c_e_p_exit: ; выход
	
ret

encoder_print_num:
	//cli
	lds r16,ENCODER_CONTR_MOT_PRINT_X2 ; проверяем нужно ли вывести на экран число (если вращался энкодер)
	cpi r16,0x01
	brne exit001 ; если флаг не равен единице, то просто выйти
	//sei
	// иначе выводим на экран число
	ldi r16,0x40
	rcall Set_cursor ; ставим курсор на место числа позиции
	
	lds r26,SHAGOVIK_X2_HIGH_RAM ; считываем из ОЗУ старший байт позиции Х2
	lds r25,SHAGOVIK_X2_LOW_RAM  ; считываем из ОЗУ младший байт позиции Х2

	rcall lcd_print_number ; выводим на экран [r26][r25] (позицию) в десятичном формате
	clr r16
	sts ENCODER_CONTR_MOT_PRINT_X2,r16 ; по умолчанию
exit001:
//sei
ret

// функция сохраняет текущую позицию в EEPROM при вращении энкодера спустя три секунды после того, как остановились двигаться. Если в течение трёх секунд
// движение опять начнётся, то начинается новый отсчёт времени. Ф-ция нужна для экономичного использования памяти EEPROM
save_position_X_after_3_sec:
	push r18
	push r19
	push r20
	// записываем в регистры счётчик времени
	lds r20,ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH
	lds r19,ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID
	lds r18,ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW
	// считываем флаг сброса счётчика
	lds r16,ENCODER_CONTR_MOT_SAVE_X

	// проверяем флаг.
	cpi r16,0x01
	brne spx_next1
flag_1: // если флаг равен 1
	// сбрасываем счётчик времени в единицу
	ldi r18,0x01
	clr r19
	clr r20
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH,r20
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID,r19
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW,r18
	clr r16
	sts ENCODER_CONTR_MOT_SAVE_X,r16 ; сбрасываем флаг
spx_next1: // если флаг равен 0
	// проверяем чему равен счётчик времени. Если счётчик равен 0, то просто выйти
	clr r16
	CPSE r18,r16
	rjmp spx_next2 ; выйти
	CPSE r19,r16
	rjmp spx_next2 ; выйти
	CPSE r20,r16
	rjmp spx_next2 ; выйти
	rjmp spx_exit
spx_next2: // если счётчик не равен 0
	// прибавляем 1 к счётчику
	ldi r16,0x01
	add r18,r16
	clr r16
	adc r19,r16
	adc r20,r16
	// сравнием счётчик с целевым числом (прошло ли установленное количество времени)
	ldi r16,ENCODER_CONTR_MOT_SAVE_X_TIME_LOW
	CPSE r18,r16
	rjmp spx_next3
	ldi r16,ENCODER_CONTR_MOT_SAVE_X_TIME_MID
	CPSE r19,r16
	rjmp spx_next3
	ldi r16,ENCODER_CONTR_MOT_SAVE_X_TIME_HIGH
	CPSE r20,r16
	rjmp spx_next3
final_time1: // если счётчик равен целевому числу (прошло установленное время)
	// считываем младший байт положения в котором находимся
	lds r16,SHAGOVIK_X0_LOW_RAM
	// сохраняем его в EEPROM с адресом 0x0003

	cli

	ldi r17,0x03
	ldi r18,0x00
	rcall EEPROM_write

	// считываем старший байт положения в котором находимся
	lds r16,SHAGOVIK_X0_HIGH_RAM
	// сохраняем его в EEPROM с адресом 0x0004
	ldi r17,0x04
	ldi r18,0x00
	rcall EEPROM_write

	sei

	// ставим всё по умолчанию
	clr r18
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH,r18
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID,r18
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW,r18
	rjmp spx_exit
spx_next3: // если счётчик не равен целевому числу (не прошло установленное время)
	// просто сохраняем счётчик
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH,r20
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID,r19
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW,r18
spx_exit:
pop r20
pop r19
pop r18
ret














// прерывание управляет процессом передвижения шагового двигателя
TIM1_COMPA: 
	SBI PORTA,STEP ; устанавливаем 1 на PA5(STEP)
	nop ; для TMC2100 минимум равен 85 нс
	nop ; ждём 65*2=130нс
	CBI PORTA,STEP ; устанавливаем 0 на PA5(STEP)

	rcall Clock_start_for_block // возобнавляем таймер блокировки и выхода из режима настроек через 10 секунд

	//rcall push_group_registers
	push r16 // сохраняем обязательно (для функции interrupt_process)

	// сохраняем SREG, чтобы не испортить флаги в случае выполнения breq, brne и т.п. в обычных функциях. В конце каждой функции возвращаем обратно, как с push и pop
	in r16,SREG
	sts SREG_temp,r16 
	// сохраняем, чтобы не испортить для других функций
	push r17
	push r18
	push r19
	push r20
	push r21 
	push r22
	push r23
	push r24
	push r25
	push r26
	push r27
	push r28
	push r29
	push r30
	push r31

	// считываем из ОЗУ регистры для передвижение шаговика
	lds r31,SHAGOVIK_r31
	lds r30,SHAGOVIK_r30
	lds r29,SHAGOVIK_r29
	lds r28,SHAGOVIK_r28
	lds r27,SHAGOVIK_r27
	lds r26,SHAGOVIK_r26
	lds r25,SHAGOVIK_r25
	lds r24,SHAGOVIK_r24
	lds r23,SHAGOVIK_r23
	lds r22,SHAGOVIK_r22
	lds r21,SHAGOVIK_r21
	lds r20,SHAGOVIK_r20
	lds r19,SHAGOVIK_r19
	// проверяем нажали ОК или HOME
	lds r16,RAM_ACTIVE_BUTTON
	SBRS r16,0 ; если бит 0 не установлен, что говорит о том, что это не кнопка ОК, то переходим на метку, иначе была нажата кнопка ОК
	rjmp home_func ; если нажали на ОК, то едем дальше, иначе переходим на метку


	
	

// проходим путь 0,1 мм
do_0_1mm:
	ldi r16,0x01
	sub r21,r16
	breq check_for_zero
	clr r16
	sbc r22,r16
	rjmp exit_but_ok

check_for_zero:
	clr r16
	sbc r22,r16
	breq obrabotka
	rjmp exit_but_ok

	//dec r21 ; прошли ещё 1/16 шага
	//breq obrabotka ; если прошли путь 0,1 мм, то обрабатываем: расчитываем ПИД, выводим на дисплей текущее положение и т.д...
	//rjmp exit1 ; иначе покидаем прерывание

// расчитываем ПИД, выводим на дисплей текущее положение и т.д...
obrabotka:
	ldi r21,SHAGOVIK_DELTA_L_LOW
	ldi r22,SHAGOVIK_DELTA_L_HIGH
	; в r31:30 хранится положение, куда должны приехать, в r29:r28 среднее положение, в r27:r26 начальное положение и в r25:r24 текущее положение
	; в r22 хранится пропорциональный коэфициента ПИДа
	SBIS PORTA,DIR ; проверяем направление движения
	rjmp nazad ; если на DIR ноль, то движемся назад

// иначе движемся вперёд
vpered:
	// прибавляем к текущему положению 0,1 мм (0x01), тем самым получая новое текущее положение
	ldi r16,0x01
	clr r17
	add r24,r16
	adc r25,r17
	// здесь код, который проверяет, достигли ли мы конца пути, и если да, то закругляемся
	CPSE r30,r24 ; если равны, пропустить след. команду
	rjmp next13 ; если не равны, идём дальше
	CPSE r31,r25 ; если равны, пропустить след. команду
	rjmp next13 ; если не равны, идём дальше
	rjmp THE_END1 ; иначе закругляемся

	next13:
	// находим расстояние, которое уже преодолели
	mov r16,r24
	mov r17,r25 ; записываем новое текущее положение, чтобы не потерять

	// проверяем какую часть пути проходим
	ldi r18,0x00
	CPSE r29,r18
	rjmp chast1
	CPSE r28,r18
	rjmp chast1
// если проходим 2 часть пути
chast2:
	// вычитаем из целевого положения Х2 текущее положение X
	push r30
	push r31
	sub r30,r16
	sbc r31,r17
	mov r16,r30
	mov r17,r31
	pop r31
	pop r30
	rjmp pid1
// если проходим 1 часть пути
chast1:
	// вычитаем из нового текущего положения Х начальное положение Х0
	sub r16,r26
	sbc r17,r27

	rjmp pid1 ; переходим дальше, где направление движения уже не играет роли


// в случае движения назад
nazad:
	// вычитаем из текущего положения 0,1 мм (0x01), тем самым получая новое текущее положение
	ldi r16,0x01
	clr r17
	sub r24,r16
	sbc r25,r17
	// здесь код, который проверяет, достигли ли мы конца пути, и если да, то закругляемся
	CPSE r30,r24 ; если равны, пропустить след. команду
	rjmp next14 ; если не равны, идём дальше
	CPSE r31,r25 ; если равны, пропустить след. команду
	rjmp next13 ; если не равны, идём дальше
	rjmp THE_END1 ; иначе закругляемся

	next14:
	// находим расстояние, которое уже преодолели
	mov r16,r26
	mov r17,r27 ; записываем начальное положение Х0, чтобы не потерять

	// проверяем какую часть пути проходим
	ldi r18,0x00
	CPSE r29,r18
	rjmp chast11
	CPSE r28,r18
	rjmp chast11
// если проходим 2 часть пути
chast22:
	// вычитаем из текущего положения Х целевое положение X2
	mov r16,r24
	mov r17,r25

	sub r16,r30
	sbc r17,r31
	rjmp pid1
// если проходим 1 часть пути
chast11:
	// вычитаем из начального положения Х0 новое текущее положение Х 
	sub r16,r24
	sbc r17,r25
	// вычитаем из текущего положение X целевое положения Х2
	// вычитаем из начального положения Х0 новое текущее положение Х 
	

// выполняем дальше алгоритмы ПИДа
pid1:
/*
	// сохраняем регистры
	push r16
	push r17
	push r20
	push r21 
	push r23
	push r24
	push r25
	push r26
	push r27

	// выводим на дисплей новое положение Х
	ldi r16,0x40
	rcall Set_cursor
	
	// считываем значение текущего положения Х
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; выводим на дисплей 

	pop r27
	pop r26
	pop r25 
	pop r24
	pop r23
	pop r21
	pop r20
	pop r17
	pop r16
	*/
	// умножаем разницу на пропорциональный коэффициент 
	ldi r18,SHAGOVIK_pid_prop ; записываем в r16 коэффициент
	clr r19 ; подготавливаем для расчётов
	rcall umnoj_mul_2x2 ; перемножаем r19:r18 * r17:r16. Рез-т в r7:r6:r5:r4
	mov r17,r7 ; это неправильно вроде, но на всякий случай оставлю тут
	mov r16,r6 ; это неправильно вроде, но на всякий случай оставлю тут
	// проверяем, прошли ли половину пути (если прошли, то в r29:r28 должны быть нули)
	clr r18
	CPSE r29,r18 ; если равен нулю, то пропустить
	rjmp chast_puti1 ; если не равен нулю, то пока не дошли до половины
	CPSE r28,r18 ; если равен нулю, то пропустить
	rjmp chast_puti1 ; если не равен нулю, то пока не дошли до половины


/* Описание ПИД-регулирования второй части пути:
   OCR1A = |(X2 - X)| * Freq(min)
   Минимальная частота импульсов STEP умножается на разницу между целевым положение Х2 и текущим положением.
   Это зеркальное вычисление по отношению к первой части.
*/
chast_puti2:
	// смотрим не превышает ли  разница, умноженная на коэф. проп. максимального значения регистра сравнения 
	cpi r17,0x00 
	brne priravn_OCR1A_min; если число больше 65535, то приравнять к минимальному OCR1A
	cpi r16,0x00
	brne priravn_OCR1A_min; если число больше 65535, то приравнять к минимальному OCR1A

	mov r17,r5 ; перекидываем delta * kp в другие регистры
	mov r16,r4
	
	ldi r19,SHAGOVIK_freq_minH ; записываем максимальное значение OCR1A
	ldi r18,SHAGOVIK_freq_minL

	// OCR1A(MAX) - delta * Kp
	sub r18,r16
	sbc r19,r17 
	
	BRLO priravn_OCR1A_min_2 ; если получилось вообще отрицательное число, то приравниваем к достигнутой в первой части частоте
	// если r19:r18 меньше
	cpi r19,SHAGOVIK_freq_maxH
	BRCS priravn_OCR1A_min_2
	cpi r18,SHAGOVIK_freq_maxL
	BRCC higher2 ; 
	cpi r19,SHAGOVIK_freq_maxH
	breq priravn_OCR1A_min_2
	rjmp higher2


priravn_OCR1A_min_2:
	// записываем в OCR1A достигнутую в первой части частоту
	lds r16,SHAGOVIK_OCR1A_BOTTOM_LOW_RAM
	lds r17,SHAGOVIK_OCR1A_BOTTOM_HIGH_RAM
	rjmp set_OCR1A
higher2:
	mov r17,r19 ; копируем новое значение OCR1A
	mov r16,r18
	rjmp set_OCR1A
/*
	// смотрим не превышает ли  разница, умноженная на коэф. проп. максимального значения регистра сравнения (минимальной частоты)
	cpi r17,0x00 
	brne priravn_OCR1A_max; если число больше 65535, то приравнять к максимальному OCR1A
	cpi r16,0x00
	brne priravn_OCR1A_max; если число больше 65535, то приравнять к максимальному OCR1A

	mov r17,r5
	mov r16,r4
	
	// считываем значение OCR1A, которое удалось достичь в первой части пути
	lds r19,SHAGOVIK_OCR1A_BOTTOM_HIGH_RAM
	lds r18,SHAGOVIK_OCR1A_BOTTOM_LOW_RAM
	
	// считываем минимальную частоту
	ldi r19,SHAGOVIK_freq_minH
	ldi r18,SHAGOVIK_freq_minL


	sub r18,r16
	sbc r19,r17

	BRCS priravn_OCR1A_max // если возникло переполнение, то число оказалось больше 65535, приравниваем к верхней границе

	// если r17:r16 меньше
	cpi r17,SHAGOVIK_freq_minH
	BRCS lower1
	cpi r16,SHAGOVIK_freq_minL
	BRCC priravn_OCR1A_max ; 
	cpi r17,SHAGOVIK_freq_minH
	breq lower1

priravn_OCR1A_max:
	// записываем в OCR1A. Минимальную частоту
	ldi r16,SHAGOVIK_freq_minL
	ldi r17,SHAGOVIK_freq_minH
	rjmp set_OCR1A
lower1:
	rjmp set_OCR1A
	*/



// если пока не дошли до половины. Для этого случая формула высчитывания OCR1A: OCR1A = OCR1A(MAX) - delta * Kp
// где OCR1A(MAX) - верхняя граница значения регистра сравнения, которая задаётся в SHAGOVIK_freq_min;
// delta - разница, которая берётся от начального положения до половины пути
chast_puti1:
	// смотрим не превышает ли  разница, умноженная на коэф. проп. максимального значения регистра сравнения 
	cpi r17,0x00 
	brne priravn_OCR1A_min; если число больше 65535, то приравнять к минимальному OCR1A
	cpi r16,0x00
	brne priravn_OCR1A_min; если число больше 65535, то приравнять к минимальному OCR1A

	mov r17,r5 ; перекидываем delta * kp в другие регистры
	mov r16,r4
	
	ldi r19,SHAGOVIK_freq_minH ; записываем максимальное значение OCR1A
	ldi r18,SHAGOVIK_freq_minL

	// OCR1A(MAX) - delta * Kp
	sub r18,r16
	sbc r19,r17 

	BRLO priravn_OCR1A_min ; если получилось вообще отрицательное число, то приравниваем к минимуму 
	// если r19:r18 меньше
	cpi r19,SHAGOVIK_freq_maxH
	BRCS priravn_OCR1A_min
	cpi r18,SHAGOVIK_freq_maxL
	BRCC higher1 ; 
	cpi r19,SHAGOVIK_freq_maxH
	breq priravn_OCR1A_min
	rjmp higher1


priravn_OCR1A_min:
	// записываем в OCR1A. Максимальную частоту
	ldi r16,SHAGOVIK_freq_maxL
	ldi r17,SHAGOVIK_freq_maxH
	rjmp check_half_way ; сравниваем текущее положение со средним положением
higher1:
	mov r17,r19 ; копируем новое значение OCR1A
	mov r16,r18
// сравниваем текущее положение со средним положением
check_half_way:
	CPSE r24,r28
	rjmp set_OCR1A
	CPSE r25,r29
	rjmp set_OCR1A
	// если мы достигли среднего положения, то
	/*
	mov r27,r29
	mov r26,r28 ; X1 записываем в Х0
	*/
	mov r27,r31
	mov r26,r30 ; X2 записываем в Х0
	clr r29
	clr r28 ; обнуляем Х1, что покажет, что мы прошли первую часть пути
	
	// записываем в ОЗУ значение OCR1A, которого удалость достичь в первой части пути
	sts SHAGOVIK_OCR1A_BOTTOM_HIGH_RAM,r17
	sts SHAGOVIK_OCR1A_BOTTOM_LOW_RAM,r16
set_OCR1A:
	rcall TIM16_WriteOCR1A ; записываем в OCR1A r17:r16
	rjmp exit_but_ok ; выходим из прерывания
THE_END1:
	
	CBI PORTC,7 ; выключаем светодиод кнопки ОК
	ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (0<<CS10) ; 
	out TCCR1B,r16 ; выключаем таймер

	// записываем в ОЗУ положение, в которое пришли
	sts SHAGOVIK_X0_HIGH_RAM,r25
	sts SHAGOVIK_X0_LOW_RAM,r24
	sts SHAGOVIK_X_TRULY_HIGH_RAM,r25 ; сохраняем физическое положение в ОЗУ
	sts SHAGOVIK_X_TRULY_LOW_RAM,r24 ; сохраняем физическое положение в ОЗУ
	// выводим на дисплей новое положение Х
	// сохраняем в EEPROM текущее положение
	ldi r16,0x01
	sts ENCODER_CONTR_MOT_SAVE_X,r16
	// если вращение ШД вызвал энкодер, а не нажатие кнопки ОК
	lds r16,ENCODER_CONTR_MOT_ENCODER_PROCESSING
	cpi r16,0x01

	breq if_encoder // то не выводим новое положение Х
	ldi r16,0x40
	rcall Set_cursor
	
	// считываем значение текущего положения Х
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; выводим на дисплей 

if_encoder:
	ldi r16,0x00
    sts ENCODER_CONTR_MOT_ENCODER_PROCESSING,r16 ; по умолчанию

	ldi r16,0b01000000
	out GICR,r16 ; включаем энкодер

	in r16,TIMSK
	ori r16,0b00000010  
	out TIMSK,r16 ; включаем таймер 0

	ldi r16,0x00 
	sts SHAGOVIK_timer1_on,r16 ; показываем, что timer1 выключён
	rjmp exit_but_ok ; выходим из прерывания

// едем в нулевое положение
home_func:
	// проходим путь 0,1 мм
do_0_1mm_home:
	ldi r16,0x01
	sub r23,r16
	breq check_for_zero_home
	clr r16
	sbc r24,r16
	rjmp exit_but_ok

check_for_zero_home:
	clr r16
	sbc r24,r16
	breq obrabotka_home
	rjmp exit_but_ok
	/* Этапы (stage1):
	   0 - ИД изначально был активен, поэтому едем вперёд 2 мм на медленной скорости, а потом сразу переходим к 4 этапу
	   1 - ИД изначально не был активен, поэтому едем назад на быстрой скорости, пока ИД не станет активным
	   2 - когда ИД стал активен, едем вперёд 1 мм на медленной скорости
	   3 - едем назад, пока ИД не станет активным
	   4 - ИД стал активным, едем назад ровно 1 мм в ноль
	*/
obrabotka_home:
	ldi r23,SHAGOVIK_DELTA_L_LOW
	ldi r24,SHAGOVIK_DELTA_L_HIGH

	cpi r26,0x00 
	breq stage0 

	cpi r26,0x01
	breq stage1 

	cpi r26,0x02
	breq stage2 

	cpi r26,0x03
	breq stage3 

	cpi r26,0x04
	breq stage4 

// ИД изначально был активен, поэтому едем вперёд 2 мм на медленной скорости, а потом сразу переходим к 4 этапу
stage0:
	// крутимся здесь пока не пройдём 2 мм
	// вычитаем единицу и сравниваем с нулём
	ldi r16,0x01
	sub r21,r16
	clr r16
	sbc r22,r16

	cpi r21,0x00
	brne zapasnoi_exit
	cpi r22,0x00
	brne zapasnoi_exit ; если r22:r21 равно не равно нулю, то выйти

	// если прошли 2 мм, то
	ldi r26,0x03 ; переходим ко этапу 3 на медленной скорости
	CBI PORTA,DIR ; меняем направление движения на назад
	// задаём дистанцию для этапа 4
	//ldi r22,HOME_SHAGOVIK_DELTA_HIGH_4 
	//ldi r21,HOME_SHAGOVIK_DELTA_LOW_4  
rjmp exit1

// ИД изначально не был активен, поэтому едем назад на быстрой скорости, пока ИД не станет активным
stage1:
	// смотрим не стал ли активен ИД
	in r16,PINA
	SBRS r16,INDSIGNAL 
	rjmp exit1
	// если ИД стал активен, то переходим к этапу 2
	//rcall low_speed ; ставим медленную скорость
	ldi r26,0x02 ; переходим ко этапу 2 с быстрой скоростью
	SBI PORTA,DIR ; меняем направление движения на вперёд
	// задаём дистанцию для этапа 2
	ldi r22,HOME_SHAGOVIK_DELTA_HIGH_2 
	ldi r21,HOME_SHAGOVIK_DELTA_LOW_2 
	ldi r16,0x64
	rcall nopdelay_1ms
rjmp exit1

// когда ИД стал активен, едем вперёд 1 мм на быстрой скорости
stage2:
	// крутимся здесь пока не пройдём 1 мм
	// вычитаем единицу и сравниваем с нулём
	ldi r16,0x01
	sub r21,r16
	clr r16
	sbc r22,r16

	clr r16
	cpi r21,0x00
	brne zapasnoi_exit
	cpi r22,0x00
	brne zapasnoi_exit ; если r22:r21 равно не равно нулю, то выйти
	// если прошли 1 мм, то
	rcall low_speed ; ставим медленную скорость передвижения
	ldi r26,0x03 ; переходим ко этапу 3 на медленной скорости
	CBI PORTA,DIR ; меняем направление движения на назад
rjmp exit1

// едем назад, пока ИД не станет активным
stage3:
	// смотрим не стал ли активен ИД
	in r16,PINA
	SBRS r16,INDSIGNAL 
	rjmp exit1
	// если ИД стал активен, то переходим к этапу 4
	ldi r26,0x04 ; переходим ко этапу 4 
	// задаём дистанцию для этапа 4
	ldi r22,HOME_SHAGOVIK_DELTA_HIGH_4 
	ldi r21,HOME_SHAGOVIK_DELTA_LOW_4  
zapasnoi_exit:
rjmp exit1

// ИД стал активным, едем назад ровно 1 мм в ноль
stage4:
	// крутимся здесь пока не пройдём 1 мм
	// вычитаем единицу и сравниваем с нулём
	ldi r16,0x01
	sub r21,r16
	clr r16
	sbc r22,r16

	clr r16
	cpi r21,0x00
	brne zapasnoi_exit
	cpi r22,0x00
	brne zapasnoi_exit ; если r22:r21 равно не равно нулю, то выйти
	// если прошли 1 мм и пришли в ноль, то
	CBI PORTC,LED_home ; выключаем светодиод кнопки HOME
	ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (0<<CS10) ; 
	out TCCR1B,r16 ; выключаем таймер

	// записываем в ОЗУ положение нуля, в которое пришли, и положение в которое мы придём нажав на кнопку ОК
	clr r25
	clr r24
	sts SHAGOVIK_X0_HIGH_RAM,r25
	sts SHAGOVIK_X0_LOW_RAM,r24
	sts SHAGOVIK_X_TRULY_HIGH_RAM,r25 ; сохраняем физическое положение в ОЗУ
	sts SHAGOVIK_X_TRULY_LOW_RAM,r24 ; сохраняем физическое положение в ОЗУ
	sts SHAGOVIK_X2_HIGH_RAM,r25  
	sts SHAGOVIK_X2_LOW_RAM,r24  
	
	// сохраняем в EEPROM текущее положение
	ldi r16,0x01
	sts ENCODER_CONTR_MOT_SAVE_X,r16

	lds r16,INIT_FLAG
	cpi r16,0x00
	breq home_next1 ; если было включение прибора, а не нажатие кнопки, то не выводим на экран положение 0

	// выводим на дисплей нулевое положение
	ldi r16,0x40
	rcall Set_cursor
	
	// считываем значение текущего положения Х
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; выводим на дисплей 


home_next1:

	ldi r16,0b01000000
	out GICR,r16 ; включаем энкодер

	in r16,TIMSK
	ori r16,0b00000010  
	out TIMSK,r16 ; включаем таймер 0

	ldi r16,0x00 
	sts SHAGOVIK_timer1_on,r16 ; показываем, что timer1 выключён
rjmp exit1

exit_but_ok:
// сохраняем в стеке регистры, чтобы их не испортило вращение энкодера и вывод чисел на экран
	
exit1:
// сохраняем в ОЗУ регистры для передвижение шаговика
	sts SHAGOVIK_r31,r31
	sts SHAGOVIK_r30,r30
	sts SHAGOVIK_r29,r29
	sts SHAGOVIK_r28,r28
	sts SHAGOVIK_r27,r27
	sts SHAGOVIK_r26,r26
	sts SHAGOVIK_r25,r25
	sts SHAGOVIK_r24,r24
	sts SHAGOVIK_r23,r23
	sts SHAGOVIK_r22,r22
	sts SHAGOVIK_r21,r21
	sts SHAGOVIK_r20,r20
	sts SHAGOVIK_r19,r19
	// возращаем, чтобы не испортить для других функций
	pop r31
	pop r30
	pop r29
	pop r28
	pop r27
	pop r26
	pop r25 
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
// общий выход
// возвращаем SREG, чтобы не испортить флаги в случае выполнения breq, brne и т.п. в обычных функциях. В конце каждой функции возвращаем обратно, как с push и pop
	lds r16,SREG_temp
	out SREG,r16
	pop r16 ; возвращаем обязательно (для ф-ции interrupt_process)
reti

















// функция тестирования с рандомным перемещением. Источник случайности - число тактовых импульсов в диапазоне расстояния перемещения шаговым двигателем чего-либо
test_random:
	SBIC PINA,1 ; если кнопка HOME нажата, то просто выходим
	ret ; просто выходим

	SBIC PINB,1 ; если кнопка BLOCK нажата, то просто выходим
	ret ; просто выходим


	// проверяем зажата ли кнопка ОК для включения тестировки
	in r16,PINB
	SBRC r16,2 ; если кнопка ОК не нажата, то пропустить след. команду 
	rjmp test_start ; если ОК нажата, то запускаем тестировку
	ret ; иначе просто выходим


test_start: ; тестировка запускается
	// сначала проверяем работает ли автоматический автохоуминг, если не работает, то всё запустить.
	// проверяем статус авт. автохоуминга
	//ldi r18,0x00
	//ldi r17,0x02 ; EEPROM адрес 0x0002
	//rcall EEPROM_read

	//cpi r16,0xFF ; если автохоуминг включён
	//breq test_random_next2 ; то не автохоумимся и просто включаем тестировку

test_auto_on: ; иначе автохоумимся
	ldi r16,0x00
	sts INIT_FLAG,r16 ; показываем, что было не нажатие кнопки, а включение 
	ldi r16,RAM_BUTTON_HOME
	sts RAM_ACTIVE_BUTTON,r16 ; показываем, что обрабатываем кнопку HOME
	rcall Button_home_func ; возвращаемся в нулевое положение
	rcall interrupt_process ; крутимся здесь, пока не достигнем нулевого положение
	rcall get_position_value_from_eeprom ; считываем положение, в которое надо придти 
	ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16 ; показываем, что обрабатываем кнопку OK
	rcall Button_ok_func ; едем в положение, которое было в последний раз
	rcall interrupt_process ; крутимся здесь, пока не достигнем нужного положения
	ldi r16,0x01
	sts INIT_FLAG,r16 ; показываем, что включение было произведено, чтобы при нажатии на кнопку HOME, в конце производился вывод нулевого положения на экран


test_random_next2: ; запускает тестировку
	// инициализируем таймер 2, который будет считать тактовые импульсы
	ldi r16,0b00001111 ; CTC; 1024 Precaler
	out TCCR2,r16 

	clr r16
	out TCNT2,r16 ; очищаем счётчик

	ldi r16,0x0F ; 15 ~ 1 миллисекунда
	out OCR2,r16

	in r16,TIMSK
	ori r16,0b10000000 ; прерывание на сравнении
	out TIMSK,r16

	cli
	clr r16
	clr r17
	sts TIM2_counter_high,r17
	sts TIM2_counter_low,r16
	sei
// основной код
while1:
	rcall Button_block_func
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	ldi r16,0xFF
	rcall nopdelay_1ms
	rcall Button_block_func

	lds r17,TIM2_counter_high
	lds r16,TIM2_counter_low
	//ldi r17,0x01
	//ldi r16,0x2c
	sts SHAGOVIK_X2_HIGH_RAM,r17 ; записываем в ОЗУ старший байт позиции
    sts SHAGOVIK_X2_LOW_RAM,r16  ; записываем в ОЗУ младший байт позиции
	
	
	push r17
	push r16
	ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16 ; показываем, что обрабатываем кнопку OK
	rcall Button_ok_func ; едем в рандомное положение

	rcall interrupt_process ; крутимся здесь, пока не достигнем нужного положения
	
	pop r16
	pop r17

	rjmp while1

ret






/*
get_adc_value:
	ldi r16,0b11000111 
	out ADCSRA,r16    ; ADC enable; Start conversation; Prescaler for freq  = 128
// ждём пока 6 бит регистра ADCSRA сбросится
wait_value:
	in r16,ADCSRA
	SBRC r16,6
	rjmp wait_value ; если бит Start conversation сбросится, то значит преобразование закончилось и можно идти дальше
	*/


; функция заполняет нулями ячейки EEPROM, в которых хранятся положения позиций от 0 до (r18 - 1) (от 0 до 99)
set_value_of_positions_to_eeprom:
	// если зажать кнопку BLOCK и кнопку OK, то все значения позиций обнулятся

	// проверяем зажата ли кнопка OK для включения обнуления значений
	in r16,PINB
	SBRC r16,2 ; если кнопка OK не нажата, то пропустить след. команду 
	rjmp check_block ; если OK нажата, то начинаем проверять нажатие кнопки BLOCK
	ret ; иначе просто выходим

check_block:
	// проверяем зажата ли кнопка BLOCK для включения обнуления значений
	in r16,PINB
	SBRC r16,1 ; если кнопка BLOCK не нажата, то пропустить след. команду 
	rjmp clear_values_start ; если BLOCK нажата, то запускаем обнуление значений
	ret ; иначе просто выходим

clear_values_start:
	
	ldi r18,0x64 ; количество позиций в которые надо сохранить значение положения
	clr r17 ; счётчик
	sts SHAGOVIK_X2_HIGH_RAM,r17 ; ставим значение 0
	sts SHAGOVIK_X2_LOW_RAM,r17 ; ставим значение 0
	sts RAM_position_number,r17  ; выбираем нулевую позицию
// запускаем цикл записи
go1:
	; сохраняем в EEPROM значение SHAGOVIK_X2_HIGH_RAM позиции RAM_position_number
	// включаем светодиод кнопки ПРОГ
	push r17
	push r18
	sbi PORTC,LED_prog
	// считываем текущий номер позиции
	lds r16,RAM_position_number

	// записываем положение Х данной позиции в ПЗУ 263 (26.3)
	ldi r17,0x02
	mul r16,r17
	mov r16,r0
	ldi r17,0x0A
	add r17,r16
	clr r18

	lds r16,SHAGOVIK_X2_HIGH_RAM  ; сначала старший байт текущего положения Х
	rcall EEPROM_write

	inc r17
	lds r16,SHAGOVIK_X2_LOW_RAM  ; теперь младший байт текущего положения Х
	rcall EEPROM_write
	cbi PORTC,LED_prog
	pop r18
	pop r17

	inc r17
	sts RAM_position_number,r17 ; выбираем следующую позицию
	dec r18 
	brne go1 ; если r18 не равен нулю, то повторяем

	// под конец обнуляем истинное значение положения 
	clr r16
	ldi r17,0x00
	clr r18
	rcall EEPROM_write

	inc r17
	rcall EEPROM_write
	inc r17
	rcall EEPROM_write
	inc r17
	rcall EEPROM_write
	inc r17
	rcall EEPROM_write
	


ret


// проверяем нажатие этих трёх кнопок, который включают или выключают режим программирования
check_block_prog_ok:

	//ldi r16,0x00
	//sts PROG_FUNC_WORK_STATUS_RAM,r16 ; записываем ноль в флаг работы кнопки ПРОГ. 0 - кнопка ПРОГ не работает. 1 - работает.

	SBIS PINA,0 ; если кнопка PROG нажата, то не выходим
	ret ; иначе выходим

	SBIS PINB,1 ; если кнопка BLOCK нажата, то не выходим
	ret ; иначе выходим

	SBIS PINB,2 ; если кнопка OK нажата, то не выходим
	ret ; иначе выходим

	// если все три кнопки получились нажатыми, делаем кнопку ПРОГ работающей
	ldi r16,0x01
	sts PROG_FUNC_WORK_STATUS_RAM,r16 ; записываем единицу в флаг работы кнопки ПРОГ. 0 - кнопка ПРОГ не работает. 1 - работает.
	sts EXIT_FROM_SETTINGS_CLK_FLAG,r16

	// выключаем BLOCK
	CBI PORTC,LED_block
	ldi r16,0b00000011 
	sts access_to_button_flags,r16 ; запрещаем нажатие всех кнопок, кроме Блок
	ldi r16,0b01000000
	out GIFR,r16 ; убираем флаг сработавшего прерывания
	out GICR,r16 ; выключаем прерывание по INT0, чтобы отключить обработку энкодера
	rcall Clock_start_for_block // включаем таймер, который включит блокировку через 10 секунд

	call blink_prog_led // мигаем светодиодом кнопки PROG

// ждём пока отпустят кнопку BLOCK
wait_block_release:
	SBIC PINB,1 ; если кнопка отпущена, то пропустить след. команду
	rjmp wait_block_release

// ждём пока отпустят кнопку HOME
wait_home_release:
	SBIC PINA,1 ; если кнопка отпущена, то пропустить след. команду
	rjmp wait_home_release

// ждём пока отпустят кнопку OK
wait_ok_release:
	SBIC PINB,2 ; если кнопка отпущена, то пропустить след. команду
	rjmp wait_ok_release

ret

// Таймер, который включает блокировку, если кнопки не нажимались в течение 10 секунд. При вызове этой функции таймер каждый раз возобновляется
Clock_start_for_block:
	push r16
	ldi r16,0x01
	sts SET_BLOCK_CLK_FLAG,r16
	sts EXIT_FROM_SETTINGS_CLK_FLAG,r16 // заодно и возобнавляем таймер для выхода из режима настроек
	pop r16
ret

// для отключения таймера блокировки
Clock_stop_for_block:
	push r16
	clr r16
	sts SET_BLOCK_CLK_FLAG,r16
	sts SET_BLOCK_CLK_COUNTER_HIGH,r16
	sts SET_BLOCK_CLK_COUNTER_MID,r16
	sts SET_BLOCK_CLK_COUNTER_LOW,r16
	pop r16
ret
/*
millis:
// считываем счётчик миллисекунд из ОЗУ
	lds r20,MILLIS_1
	lds r19,MILLIS_2
	lds r18,MILLIS_3
	lds r17,MILLIS_4

	// прибавляем к счётчику миллисекунд единицу
	ldi r16,0x01
	add r17,r16
	clr r16
	adc r18,r16
	adc r19,r16
	adc r20,r16

	// записываем счётчик миллисекунд в ОЗУ
	sts MILLIS_1,r20
	sts MILLIS_2,r19
	sts MILLIS_3,r18
	sts MILLIS_4,r17
ret
*/




set_block_clk:
	push r16
	push r18
	push r19
	push r20
	// записываем в регистры счётчик времени
	lds r20,SET_BLOCK_CLK_COUNTER_HIGH
	lds r19,SET_BLOCK_CLK_COUNTER_MID
	lds r18,SET_BLOCK_CLK_COUNTER_LOW
	// считываем флаг сброса счётчика
	lds r16,SET_BLOCK_CLK_FLAG

	// проверяем флаг.
	cpi r16,0x01
	brne set_block_spx_next1
set_block_flag: // если флаг равен 1
	// сбрасываем счётчик времени в единицу
	ldi r18,0x01
	clr r19
	clr r20
	sts SET_BLOCK_CLK_COUNTER_HIGH,r20
	sts SET_BLOCK_CLK_COUNTER_MID,r19
	sts SET_BLOCK_CLK_COUNTER_LOW,r18
	clr r16
	sts SET_BLOCK_CLK_FLAG,r16 ; сбрасываем флаг
set_block_spx_next1: // если флаг равен 0
	// проверяем чему равен счётчик времени. Если счётчик равен 0, то просто выйти
	clr r16
	CPSE r18,r16
	rjmp set_block_spx_next2 ; выйти
	CPSE r19,r16
	rjmp set_block_spx_next2 ; выйти
	CPSE r20,r16
	rjmp set_block_spx_next2 ; выйти
	rjmp set_block_spx_exit
set_block_spx_next2: // если счётчик не равен 0
	// прибавляем 1 к счётчику
	ldi r16,0x01
	add r18,r16
	clr r16
	adc r19,r16
	adc r20,r16
	// сравнием счётчик с целевым числом (прошло ли установленное количество времени)
	ldi r16,SET_BLOCK_CLK_TIME_LOW
	CPSE r18,r16
	rjmp set_block_spx_next3
	ldi r16,SET_BLOCK_CLK_TIME_MID
	CPSE r19,r16
	rjmp set_block_spx_next3
	ldi r16,SET_BLOCK_CLK_TIME_HIGH
	CPSE r20,r16
	rjmp set_block_spx_next3
set_block_final_time: // если счётчик равен целевому числу (прошло установленное время)
	// включаем режим Блока
	SBI PORTC,LED_block
	ldi r16,0b00000001 
	sts access_to_button_flags,r16 ; разрешаем нажатие всех кнопок
	ldi r16,0b00000000
	out GICR,r16 ; INT0 в единицу для включения прерывания на пине INT0


	// ставим всё по умолчанию
	clr r18
	sts SET_BLOCK_CLK_COUNTER_HIGH,r18
	sts SET_BLOCK_CLK_COUNTER_MID,r18
	sts SET_BLOCK_CLK_COUNTER_LOW,r18
	rjmp set_block_spx_exit
set_block_spx_next3: // если счётчик не равен целевому числу (не прошло установленное время)
	// просто сохраняем счётчик
	sts SET_BLOCK_CLK_COUNTER_HIGH,r20
	sts SET_BLOCK_CLK_COUNTER_MID,r19
	sts SET_BLOCK_CLK_COUNTER_LOW,r18
set_block_spx_exit:
pop r20
pop r19
pop r18
pop r16
ret







exit_from_settings_mode_clk:

	push r16
	push r18
	push r19
	push r20

	// проверяем, был ли совершён уже выход из режима настроек, и если да, то выйти из функции
	lds r16,PROG_FUNC_WORK_STATUS_RAM // 0 - кнопка ПРОГ не работает. 1 - работает.
	cpi r16,0x00 // если во флаге ноль, то выйти из функции
	breq exit_from_settings_spx_exit

	// записываем в регистры счётчик времени
	lds r20,EXIT_FROM_SETTINGS_CLK_COUNTER_HIGH
	lds r19,EXIT_FROM_SETTINGS_CLK_COUNTER_MID
	lds r18,EXIT_FROM_SETTINGS_CLK_COUNTER_LOW
	// считываем флаг сброса счётчика
	lds r16,EXIT_FROM_SETTINGS_CLK_FLAG

	// проверяем флаг.
	cpi r16,0x01
	brne exit_from_settings_spx_next1
exit_from_settings_flag: // если флаг равен 1
	// сбрасываем счётчик времени в единицу
	ldi r18,0x01
	clr r19
	clr r20
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_HIGH,r20
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_MID,r19
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_LOW,r18
	clr r16
	sts EXIT_FROM_SETTINGS_CLK_FLAG,r16 ; сбрасываем флаг
exit_from_settings_spx_next1: // если флаг равен 0
	// проверяем чему равен счётчик времени. Если счётчик равен 0, то просто выйти
	clr r16
	CPSE r18,r16
	rjmp exit_from_settings_spx_next2 ; выйти
	CPSE r19,r16
	rjmp exit_from_settings_spx_next2 ; выйти
	CPSE r20,r16
	rjmp exit_from_settings_spx_next2 ; выйти
	rjmp exit_from_settings_spx_exit
exit_from_settings_spx_next2: // если счётчик не равен 0
	// прибавляем 1 к счётчику
	ldi r16,0x01
	add r18,r16
	clr r16
	adc r19,r16
	adc r20,r16
	// сравнием счётчик с целевым числом (прошло ли установленное количество времени)
	ldi r16,EXIT_FROM_SETTINGS_CLK_TIME_LOW
	CPSE r18,r16
	rjmp exit_from_settings_spx_next3
	ldi r16,EXIT_FROM_SETTINGS_CLK_TIME_MID
	CPSE r19,r16
	rjmp exit_from_settings_spx_next3
	ldi r16,EXIT_FROM_SETTINGS_CLK_TIME_HIGH
	CPSE r20,r16
	rjmp exit_from_settings_spx_next3
exit_from_settings_final_time: // если счётчик равен целевому числу (прошло установленное время)
	// выходим из режима настройки, записав во флаг ноль
	ldi r16,0x00
	sts PROG_FUNC_WORK_STATUS_RAM,r16
	cbi PORTC,LED_prog // выключаем светодиод над кнопкой PROG

	// ставим всё по умолчанию
	clr r18
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_HIGH,r18
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_MID,r18
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_LOW,r18
	rjmp exit_from_settings_spx_exit
exit_from_settings_spx_next3: // если счётчик не равен целевому числу (не прошло установленное время)
	// просто сохраняем счётчик
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_HIGH,r20
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_MID,r19
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_LOW,r18
exit_from_settings_spx_exit:
pop r20
pop r19
pop r18
pop r16

ret











;EXT_INT0: reti
EXT_INT1: reti
EXT_INT2: reti
;TIM2_COMP: reti
TIM2_OVF: reti
TIM1_CAPT: reti
;TIM1_COMPA: reti
TIM1_COMPB: reti
TIM1_OVF: reti
;TIM0_COMP: reti
TIM0_OVF: reti
SPI_STC: reti
USART_RXC: reti
USART_UDRE: reti
USART_TXC: reti
ADC1: reti
EE_RDY: reti
ANA_COMP: reti
TWI: reti
SPM_RDY: reti


