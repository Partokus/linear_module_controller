
; Created: 25.01.2021 23:04:30
; Author : Alexey Lepeshkin



.include "m32def.inc"

.equ SREG_temp = 0x012B


.equ slave_adress = 0x27 // ����� I2C ������ �������


// ����������� ��� ����������� ������
.equ debounce_time = 0x0A  ; ����� ����� ���������� �������� ������� ������ � �������������
.equ check_pin_time = 0x14 ; ����� ����� �������� �� ������
.equ debounce_end_time = 0x0A ; ����� ����� ���������� �������� ���������� ������ � �������������
.equ RAM_BUTTON_UP = 0x01     ; ����������� ������ �������� 0x.. ��� �������������
.equ RAM_BUTTON_DOWN = 0x02   ; ����������� ������ �������� 0x.. ��� �������������
.equ RAM_BUTTON_OK = 0x03     ; ����������� ������ �������� 0x.. ��� �������������
.equ RAM_BUTTON_HOME = 0x04   ; ����������� ������ �������� 0x.. ��� �������������
.equ RAM_BUTTON_PROG = 0x05   ; ����������� ������ �������� 0x.. ��� �������������
.equ RAM_BUTTON_STEP = 0x06   ; ����������� ������ �������� 0x.. ��� �������������
.equ RAM_BUTTON_BLOCK = 0x07  ; ����������� ������ �������� 0x.. ��� �������������
.equ RAM_ACTIVE_BUTTON = 0x0103 ; ����� � ���, � ������� ����� ������������ ������������� �������� ������, ����� ������ ������ �� ����� �������������� ������������
.equ access_to_button_flags = 0x0111 ; ���� � ���, � ������� �������� ����� ������������� �����, ����� ��������� �����-�� ������ �� ����� ������ ����������� �������
.equ B1 = 3 ; PA3 = 3 ����� ���� � ����� � ����� B1 ��������





// ������������� ��� ���������� ������� ����������
.equ RAM_step_size = 0x0104 ; ����� ����� � ����������� ������, ���� ����� ������������ ������� �������� ���� (0,1 ��, 1 ��, 10 ��)
.equ RAM_position_number = 0x0107 ; ����� �����, ���� ����� ������������ ����� ������� (0-100)
.equ SHAGOVIK_timer1_on = 0x0112 ; ���������� ���������� ������� ������ 1 ��� ���. 0x00 - ����; 0x01 - ���.
/* 
   ����������� �������� OCR1A ��� StelthChop: 2000 (DEC).
   ����������� �������� OCR1A ��� SpreadCycle: ? (DEC).
*/
//.equ SHAGOVIK_freq_maxH = 0x0B ; ������� ���� ������������ ������� ��������� STEP. � ���� �������� ��������� OCR1A
//.equ SHAGOVIK_freq_maxL = 0xB8 ; ������� ���� ������������ ������� ��������� STEP. � ���� �������� ��������� OCR1A
.equ SHAGOVIK_freq_maxH = 0x02 ; ������� ���� ������������ ������� ��������� STEP. � ���� �������� ��������� OCR1A
.equ SHAGOVIK_freq_maxL = 0xAB ; ������� ���� ������������ ������� ��������� STEP. � ���� �������� ��������� OCR1A
.equ SHAGOVIK_freq_minH = 0x07 ; ������� ���� ����������� ������� ��������� STEP. � ���� �������� ��������� OCR1A
.equ SHAGOVIK_freq_minL = 0xD0 ; ������� ���� ����������� ������� ��������� STEP. � ���� �������� ��������� OCR1A
.equ SHAGOVIK_pid_prop = 0x6C ; ���������������� ����������� ����
.equ SHAGOVIK_step = 0x10     ; ��� (1, 1/2, 1/4, 1/8, 1/16)
.equ SHAGOVIK_X_TRULY_HIGH_EEPROM = 0x00 ; ��. �. ���������� �������� ��������� � EEPROM(������������ � ������ �������� � ������� ��������, ����� ������� �����. �� ���� ��������� � �������)
.equ SHAGOVIK_X_TRULY_LOW_EEPROM = 0x03  ; ��. �.
.equ SHAGOVIK_X_TRULY_HIGH_RAM = 0x0132  ; ��. �. ���������� �������� ��������� � ���
.equ SHAGOVIK_X_TRULY_LOW_RAM = 0x0133   ; ��. �. ���������� �������� ��������� � ���
.equ SHAGOVIK_X0_HIGH_RAM = 0x0108    ; ������� ���� ���������� ���������
.equ SHAGOVIK_X0_LOW_RAM = 0x0109     ; ������� ���� ���������� ���������
.equ SHAGOVIK_X1_HIGH_RAM = 0x010A    ; ������� ���� �������� ���� 
.equ SHAGOVIK_X1_LOW_RAM = 0x010B     ; ������� ���� �������� ����
.equ SHAGOVIK_X2_HIGH_RAM = 0x010C    ; ������� ���� ����� ���� 
.equ SHAGOVIK_X2_LOW_RAM = 0x010D     ; ������� ���� ����� ����
.equ SHAGOVIK_OCR1A_BOTTOM_HIGH_RAM = 0x010F ; ������� ���� �������� OCR1A, �������� ������� ������� � ������ ����� ����
.equ SHAGOVIK_OCR1A_BOTTOM_LOW_RAM = 0x0110 ; ; ������� ���� �������� OCR1A, �������� ������� ������� � ������ ����� ����
.equ SHAGOVIK_DELTA_L_HIGH = 0x01; � ���� �������� ����� ��������� ���������, ������������ ���������� ��������� STEP, ��� ������� ����� ������� ���� 0,1�� ��� ���� 1/16
.equ SHAGOVIK_DELTA_L_LOW = 0x90 ; � ���� �������� ����� ��������� ���������, ������������ ���������� ��������� STEP, ��� ������� ����� ������� ���� 0,1�� ��� ���� 1/16
//.equ SHAGOVIK_DELTA_L_HIGH = 0x00; 
//.equ SHAGOVIK_DELTA_L_LOW = 0x05 ;

// ��������� �������� ���������, ���� ������� �������������
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
// ��� ������� HOME
.equ INDSIGNAL = 7
/*
 ����� ��������:
 0 - �� ���������� ��� �������, ������� ���� ����� 2 �� �� ��������� ��������, � ����� ����� ��������� � 4 �����
 1 - �� ���������� �� ��� �������, ������� ���� ����� �� ������� ��������, ���� �� �� ������ ��������
 2 - ����� �� ���� �������, ���� ����� 1 �� �� ��������� ��������
 3 - ���� �����, ���� �� �� ������ ��������
 4 - �� ���� ��������, ���� ����� ����� 1 �� � ����
*/
// ������� ����� ������ �� ����� 0 
.equ HOME_SHAGOVIK_DELTA_HIGH_0 = 0x00 
.equ HOME_SHAGOVIK_DELTA_LOW_0 = 0x14 
// ������� ����� ������ �� ����� 2 
.equ HOME_SHAGOVIK_DELTA_HIGH_2 = 0x00 
.equ HOME_SHAGOVIK_DELTA_LOW_2 = 0x14 
// ������� ����� ������ �� ����� 4 
.equ HOME_SHAGOVIK_DELTA_HIGH_4 = 0x00 
.equ HOME_SHAGOVIK_DELTA_LOW_4 = 0x01

.equ HOME_SHAGOVIK_freq_maxH = 0x02 ; ������� ���� ������������ ������� ��������� STEP. � ���� �������� ��������� OCR1A 
.equ HOME_SHAGOVIK_freq_maxL = 0xAB ; ������� ���� ������������ ������� ��������� STEP. � ���� �������� ��������� OCR1A 
.equ HOME_SHAGOVIK_freq_minH = 0x07 ; ������� ���� ����������� ������� ��������� STEP. � ���� �������� ��������� OCR1A
.equ HOME_SHAGOVIK_freq_minL = 0xD0 ; ������� ���� ����������� ������� ��������� STEP. � ���� �������� ��������� OCR1A
.equ INIT_FLAG = 0x0113 ; ���������� ��� �����������, ��� ���������� �������������, � �� ������� ������, ����� ��������� �������� �������� �� �������



// ������������� ��� I2c
.equ I2C_DATA = 0x0120 ; ����� � ����������� ������, � �������� �������� ����� ��� �������� �� i2c
.equ RAM_slave_adress = 0x0100 ; ����� � ����������� ������, ���� ����� ���������� ����� i2c
.equ START = 0x08 ; ������ ������ ��� i2c
.equ REPEATED_START = 0x10 ; ������ ���������� ������
.equ MT_SLA_ACK = 0x18 ; ����� � �������� ������ ���� ��������. ACK ��� �������
.equ MT_DATA_ACK = 0x28 ; ������ ���� ��������. ACK ��� �������


// ������������� ��� �������� �������� ��������� ���������
.equ ENCODER_CONTR_MOT_PRESS_OK = 0x0114 ; ���� �������� ��������, ����� ������� ������� ������ ��
.equ ENCODER_CONTR_MOT_PRINT_X2 = 0x0115 ; ���� ��� ����, ����� ������� �� ����� ������� �2, � ������� ���� ��������
.equ ENCODER_CONTR_MOT_ENCODER_PROCESSING = 0x0116 ; ���� ����� ��������, ��� ������������ ���������� �� ���� ��������
.equ ENCODER_CONTR_MOT_SAVE_X = 0x012C  ; ���� ������ ������� 3 ������, ����� �� ��������� ��������� ������� ������� � � EEPROM
.equ ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH = 0x012F ; ������� ���� �������� 3 ������
.equ ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID = 0x012E
.equ ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW = 0x012D ; ������� ���� �������� 3 ������
.equ ENCODER_CONTR_MOT_SAVE_X_TIME_HIGH = 0x01 ; ������� ���� ������� � �������� ���������, ������� ������ ������ ����� �����������
.equ ENCODER_CONTR_MOT_SAVE_X_TIME_MID = 0xFF ; ������� ���� ������� � �������� ���������, ������� ������ ������ ����� �����������
.equ ENCODER_CONTR_MOT_SAVE_X_TIME_LOW = 0xFF  ; ������� ���� ������� � �������� ���������, ������� ������ ������ ����� �����������


// ������������� ��� �����������
.equ LED_prog = 5
.equ LED_ok = 7
.equ LED_block = 4
.equ LED_home = 6


// ��� ������� 2
.equ TIM2_counter_high = 0x0131 ; ������� �����������
.equ TIM2_counter_low = 0x0130

// ��� ������� PROG
.equ PROG_FUNC_WORK_STATUS_RAM = 0x0134 ; ����, ������������, �������� ������ PROG ��� ���. ���� ������ BLOCK, PROG � OK ������������, �� ������ PROG ����� ��������

/*
// ��� �������� ����������� � ������� uint32_t
.equ MILLIS_1 = 0x0135 // ������� ����
.equ MILLIS_2 = 0x0136
.equ MILLIS_3 = 0x0137
.equ MILLIS_4 = 0x0138 // ������� ����
*/

// ��� ��������� ����� ����� SET_BLOCK_CLK_TIME ������
.equ SET_BLOCK_CLK_COUNTER_HIGH = 0x0135
.equ SET_BLOCK_CLK_COUNTER_MID = 0x0136
.equ SET_BLOCK_CLK_COUNTER_LOW = 0x0137
.equ SET_BLOCK_CLK_FLAG = 0x0138
.equ SET_BLOCK_CLK_TIME_HIGH = 0x08 // 0x12
.equ SET_BLOCK_CLK_TIME_MID = 0xFF
.equ SET_BLOCK_CLK_TIME_LOW = 0xFF

// ��� ������ �� ������ �������� ����� EXIT_FROM_SETTINGS_CLK_TIME ������
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

/*  ���������� ������:
	���� ������ BLOCK � OK ������������, ��������� ������ ������ EEPROM, ��� �������� �������� ��������� �������
	���� ������ ������ ��, �� ��������� ����������
	���� ������ ������ HOME, �� ���������� ��� ����������� ����������� ��� ���������
	���� ������ BLOCK, PROG � OK ������������, �� ������ PROG ����� ��������
*/

/* ��������� �������������� ����� � ��� = 0x013C*/

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
	ldi r16, high(0x042F); Main program start ��� 
	out SPH,r16 ; 
	ldi r16, low(0x042F) ; �������� �� ������ ����������� ������ ATmega32
	out SPL,r16
	
	rcall init_values
	rcall init_port ; ����������� �����
	rcall init_I2C ; ����������� I2c
	rcall init_timer0 ; ����������� timer0 ��� ����������� ������
	rcall init_timer1 ; ����������� timer1 ��� ���������� ������� ����������
	//rcall init_timer2 ; ������������ timer2 ��� ������� �����������, ��� ��������� ��������
	
	call set_value_of_positions_to_eeprom ; ���� ������ BLOCK � PROG ������������, ��������� ������ ������ EEPROM, ��� �������� �������� ��������� �������
	
	rcall lcd1602_init ; �������������� LCD1602
	rcall Create_char ; ������ ������� �������
	rcall init_parameters ; �������������� ���������, ��������� �� ��� � ����������� �� �������� (���)
	rcall start_title ; ������� ��������� ������� � ��������� �������� �� �����

	rcall test_random ; ���� ������ ������ ��, �� ��������� ����������
	rcall home_and_go_to_last_position ; ������������ � ����, � ����� �� ��������� �������, ������� ���� ����� �����������
	call check_block_prog_ok ; ��������� ������� ���� ��� ������, ������� �������� ��� ��������� ����� ����������������
	// �� ���������
	clr r16
	clr r17
	
	
	rcall Button_block_func ; ��������� ��� ������ 
	
	//ldi r16,0b11110000
	//out PORTC,r16
	
	sei ; Enable interrupts








main:
rcall interrupt_process ; �-��� � ������� ��������� ��� � ����� while, ���� ������� ������ 1 (���� ����� �� �� step-�������)
rcall save_position_X_after_3_sec ; ���������� ��������� �������� ������� ����� 3 ������� ����� ���������� �������� ��������
call set_block_clk               // ������������ ���������� ����� 10 ������, ���� �� �������� ������
call exit_from_settings_mode_clk // ����� �� ������ �������� ����� 10 ������, ���� �� �������� ������
call check_block_prog_ok

lds r16,access_to_button_flags ; ��������� ����� ������ ����� �������� � ������ ������
SBRS r16,0 ; ���� � access_to_button_flags ������� ��� 0, ��  ��������� ������� ��������� ������
rjmp main
rcall Button_block
lds r16,access_to_button_flags ; ��������� ����� ������ ����� �������� � ������ ������
SBRS r16,1 ; ���� ������� ��� 1, ��������� ������� ������ ������ ����
rjmp main

rcall Button_up
rcall Button_down
rcall Button_ok
rcall Button_prog
rcall Button_home
rcall Button_step

rcall check_encoder_press // ��������� �������� ��������

rjmp main














// ����������� I2C
init_I2C:
	//ldi r16,0b00000011
	//out PORTC,r16
	; ����������� ������� SCL (��������). SCLfreq = CPU clock frequency / (16 + 2 * (TWBR) * 4 * (PrescalerValue))
	ldi r16,0x0A
	out TWBR,r16 ; Bit rate register

	ldi r16,0x00 
	out TWSR,r16 ; ������������ ������� ������ ������ 1

	; ��� Slave ����. ������������� ����������� ������
	;ldi r16,0x27 
	;lsr r16 ; �������� ����� � ����������� ���� � ������� ���, ��� ��� ����� ����� ���� 7 �����, � ��������� ��� � �������� TWAR
			; �������� �� ������� �������������, ��� �� ����� ���
	;sts TWAR,r16 ; ��������� ����� ����� ������� 1602, ������ 0x27 � ����������������� ������� ���������
ret




i2c_write: nop
; TWEN - �������� I2C
; TWSTA - ���������� START condition
; TWSTO - ���������� STOP condition
; TWINT - ������������� 1 ��� ������ �����



; ��������� � ������� ������ ����, ������� ����� ��������. ������� ��� TWINT, ����� ������ �������� ������
	ld r16,Z+ ; ��������� �� ����������� ������ ����, ����� ������ �������� ��������� � Z, ������� ���� ��������
	out TWDR, r16       
	ldi r16, (1<<TWINT) | (1<<TWEN) 
	out TWCR, r16

; ��� ���� ���� TWINT �����������. ��� ������� ��, ��� ������� ���� ��� ������� �  �� ������ ������� ACK/NACK
wait3: 
	in r16,TWCR 
	sbrs r16,TWINT 
	rjmp wait3

; ��������� ������� ������� TWSR, ���� ������ �� ������������� MT_DATA_ACK (ACK �� �������), �� ��������� ������. ����� ����� ������������
; DATA has been transmitted; ACK has been received
	in r16,TWSR 
	andi r16, 0xF8 
	cpi r16, MT_DATA_ACK ; .equ START = 0x18
	brne ERROR
	ret

ERROR:

ret




i2c_start:
; ���������� START
	ldi r16, (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (0<<TWSTO)
	out TWCR, r16

; ��� ���� TWINT ������ ������ 1, ��� ������� � ���, ��� �������� START ��������� 
wait1: 
	in r16,TWCR 
	sbrs r16,TWINT 
	rjmp wait1

; ��������� ������ START, ���� ������ �� ������������� START, �� ��������� ������. ����� ����� ������������
	in r16,TWSR 
	andi r16, 0xF8 
	cpi r16, START ; .equ START = 0x08
	breq next1 ; ���� � ������ ��� ������� �����, �� ���������� ��������
	cpi r16, REPEATED_START ; ����� ��������� ������ ���������� ������
	brne ERROR_start ; ���� ��� �� ��������� �����, �� ��������� ������

next1:
; ��������� �� ����������� ������ ���� � ������� � �������� ������ � ������� ������ ���� SLA_W - ����� + ������� ������
	lds r16,RAM_slave_adress ; .equ SLA_W = 0b01001110 - ����� 0x27 � ������� ��� 0 ��� ������
	lsl r16 ; �������� �����, ����� �������� ������ XXXX XXX0 - ��� ����� �����, � 0 ��� ������� ������
	out TWDR, r16 
	ldi r16, (1<<TWINT) | (1<<TWEN) 
	out TWCR, r16

; ��� ���� ���� TWINT �����������. ��� ������� ��, ��� ������� SLA_W ���� �������� � �� ������ ������� ACK/NACK
wait2: 
	in r16,TWCR 
	sbrs r16,TWINT 
	rjmp wait2

; ��������� ������� ������� TWSR, ���� ������ �� ������������� MT_SLA_ACK (ACK �� �������), �� ��������� ������. ����� ����� ������������
; SLA+W has been transmitted; ACK has been received
	in r16,TWSR 
	andi r16, 0xF8 
	cpi r16, MT_SLA_ACK ; .equ START = 0x18
	brne ERROR_start
	ret

ERROR_start:
ret



i2c_stop:
; ������� STOP condition
	ldi r16, (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) 
	out TWCR, r16 

	ldi r16, (1<<TWINT) | (0<<TWEN) | (0<<TWSTO) | (0<<TWSTA) ; ��������� ��
	out TWCR, r16 
	ret

ret













lcd1602_init: nop
    clr r17
	sts 0x0102,r17 ; ��������� � ����������� ������ ������� �������� RS � R/W

	ldi r16,0x10
	rcall nopdelay_1ms ; �������� �� 16 �����������

	; �������� ���������� ��������� ����������
	ldi r16,0b00110011 ; ������������ ��������� 
	rcall LCD1602_send_8bit_command ; 

	ldi r16,0x05
	rcall nopdelay_1ms ; �������� �� 4 ������������

	ldi r16,0b00110011 ; ������������ ��������� 
	rcall LCD1602_send_8bit_command ; 

	ldi r16,0x01
	rcall nopdelay_1ms ; �������� �� 1 ����

	ldi r16,0b00100010 ; ������ �� 4-������ ���������
	rcall LCD1602_send_8bit_command
	
	ldi r16,0b00101000 ; 
	rcall LCD1602_send_command
	
	ldi r16,0b00001000
	rcall LCD1602_send_command
	
	ldi r16,0b00000001 ; ������� ������� �������
	rcall LCD1602_send_command

	ldi r16,0x03 ; �����, ��� ��� ������� ������ �������� ������ ���� �������
	rcall nopdelay_1ms
	
	ldi r16,0b00000110 ; ����������� ������ �������� �����-�������, ������ ������ ������ ��� ������ ��������
	rcall LCD1602_send_command
	
	ldi r16,0b00001100 ; �������� ������� (D=1), ��������� ����������� ������� (C=0), ��������� ������� ������� ������� (B=0)
	rcall LCD1602_send_command
ret





; ������� ��������. �������� � r16 ���������� �� ������� ����������� ����� ��������. ���� r16 = 60, �� �������� 60 ����
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





; ������� ��������� 8-� ���������� �������, ��� ������������ ����� �����, ��� ����� ������� ������. ��� 1-�� ������ ������: 0x00 - 0x0F. ��� ������: 0x40 - 0x4F
Set_cursor: nop
	mov r20,r16 ; ��������� � �������� r20, ����� �����, �������� ����� �������� ����������. ��� ����� ��� ������ ����� � ������ ������� (00.0 ��� 00 � �������)
	ori r16,0b10000000 ; ���������� 8-� ����������, ��� DB7 ����� �������
	clr r17
	sts 0x0102,r17 ; RS � R/W � 0 ��� ������ ������ �������
	rcall LCD1602_send_command ; ���������� ������
ret









Send_instruction: nop
	clr r17
	sts 0x0102,r17 ; RS � R/W � 0 ��� ������ ������ �������
	rcall LCD1602_send_command ; ���������� ������
ret








; ���������� � ������ ���� D7-D0 �� 4-������� ����������. ������ ����������� �� ����� �� E. D7 - D6 - D5 - D4 - LED - E - R/W - RS
LCD1602_send_command: nop
    mov r27,r16  ; ��������� ���� ������ � �������� r25
	ori r16,0b00001100 ; ��������� ���������� ��������� � ������������� �������� �� ������ ������������ �
	lds r17,0x0102 ; ��������� ����, � ������� ����������� �������� RS � R/W 
	or r16,r17 ; ��������� �������� RS � R/W 
	andi r16,0b11111101 ; ������� R/W
	sts I2C_DATA,r16 ; ��������� � ��� ������ ����, ������� ���� ��������

    ;ldi r16,slave_adress
    ;sts 0x0100,r16 ; ��������� � ����������� ������ ���� � ������� ������ 

	lds r16,I2C_DATA ; ��������� ����, ������� ��������
	andi r16,0b11111011 ; ����� ������������ � 0
	sts 0x0121,r16 ; ��������� � ��� 2-� ����, ������� ���� ��������
	; �������� ������ ���� �����, ����� ����� ���� �������� ��
	
	lsl r27
	lsl r27
	lsl r27
	lsl r27 
	mov r16,r27 ; �������� � r16
	
	ori r16,0b00001100 ; ��������� ���������� ��������� � ������������� �������� �� ������ ������������ �
	lds r17,0x0102
	or r16,r17
	andi r16,0b11111101 ; ������� R/W
	sts 0x0122,r16 ; ��������� � ��� 3-� ����, ������� ���� ��������


	lds r16,0x0122 ; ��������� ����, ������� ��������
	andi r16,0b11111011 ; ����� ������������ � 0
	sts 0x0123,r16 ; ��������� � ��� 4-� ����, ������� ���� ��������

	push r31 ; ����� �� ��������� 
	push r30
	// ��������� � ���. ���� Z ����� ������ ��� �����, ������� ���� �������� �� i2c
	ldi r31,0x01
	ldi r30,0x20 
	rcall i2c_start ; �������� �������� �� i2c
	// ������� ������� ����� ����� ��������, � = 1
	rcall i2c_write ; ���������� ������
	// ������� ������� ����� ����� ��������, � = 0. ������ � ������� ����������� �� �����. ������ ���������������
	rcall i2c_write ; ���������� ������
	// ������� ������� ����� ����� ��������, � = 1
	rcall i2c_write ; ���������� ������
	// ������� �������  ����� ����� ��������, � = 0. 
	rcall i2c_write ; ���������� ������
	rcall i2c_stop ; ����������� ��������

	pop r30
	pop r31
ret







Write_symbol: nop
	ldi r17,0x01 ; RS = 1, R/W = 0 
	sts 0x0102,r17 ; 
	rcall LCD1602_send_command ; ���������� ������
ret








; ��� ������� ����� ������ ��� ������������� �������, ��� ��� � ������ �� ���������� 8-������ ���������
LCD1602_send_8bit_command: nop
    mov r25,r16  ; ��������� ���� ������ � �������� r25
	ori r16,0b00001100 ; ��������� ���������� ��������� � ������������� �������� �� ������ ������������ �
	andi r16,0b11111100 ; �������� ��������� ��� ����
	sts I2C_DATA,r16 ; ��������� � �� ������ ����, ������� ����� ��������
	// ��������� � ���. ���� Z ����� ������ ��� �����, ������� ���� �������� �� i2c
	ldi r31,0x01
	ldi r30,0x20 

    ldi r16,slave_adress
    sts 0x0100,r16 ; ��������� � ����������� ������ ���� � ������� ������ 
	rcall i2c_start ; �������� �������� �� i2c
	rcall i2c_write ; ������� ������� ����� ����� ��������, � = 1
	rcall i2c_stop  ; ����������� �������� �� i2c
	// ��������� � ���. ���� Z ����� ������ ��� �����, ������� ���� �������� �� i2c
	ldi r31,0x01
	ldi r30,0x20 

	lds r16,I2C_DATA ; ��������� ����, ������� ��������
	andi r16,0b11111011 ; ����� ������������ � 0
	sts I2C_DATA,r16 ; ��������� � �� ������ ����, ������� ����� ��������
	; ������� ������� ����� ����� ��������, � = 0. ������ � ������� ����������� �� �����. ������ ���������������
	rcall i2c_start ; �������� �������� �� i2c
	rcall i2c_write ; ������� ���� �� i2c
	rcall i2c_stop  ; ����������� �������� �� i2c
ret






init_timer0:
// ����������� TIMER0 ��� ����������� ������
	clr r16 
	out TCNT0,r16 ; ������� �� ������ ������

	ldi r16,0b00000000  ; ���� �� ��������� ������
	out TCCR0,r16 ; 

	ldi r16,0xF9 ; F9
	out OCR0,r16 ; ���������� ����� 249 � ������� ��������� �, ��� ��� ��� ���������� ��������� ����� ����� ������ ����� 1 ����

	in r16,TIMSK
	ori r16,0b00000010  
	out TIMSK,r16 ; ������������� ���������� �� ���������� �. ������������ sts, ��� ��� ����� �������� TIMSK0 ��������� � �������
	out TIFR,r16  ; ���������� �������, ����� ���������� ���� �� �����������. TIFR0 ��������� � ������� "64 I/O Registers" (0x0020 - 0x005F)


ret



// ����������� Timer1 ��� ���������� ������� ����������
init_timer1:
	// ����������� �� ����� ���
	ldi r16, (0<<WGM10) | (0<<WGM11) 
	out TCCR1A,r16 

	;ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; ������������ ����� 1.
	ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (0<<CS10) ; ���� �� ��������� ������
	out TCCR1B,r16

	// �������� TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; ���������� � TCNT1 r17:r16

	// ���������� � OCR1A. ����������� �������
	ldi r16,SHAGOVIK_freq_minL
	ldi r17,SHAGOVIK_freq_minH
	rcall TIM16_WriteOCR1A ; ���������� � OCR1A r17:r16
	// ��������� ���������� �� ���������� � OCR1A
	in r16,TIMSK
	ldi r17, (1<<OCIE1A)
	or r16,r17
	out TIMSK,r16
	// �������� ���� ����������
	out TIFR,r16
ret
/*
// Prescaler = 32. Ticks = 250. OCR2 period = 0.0005. ��� 10 ������ ������ ��������� (OCR2 period * 20000) ����������
init_timer2:
// ����������� TIMER0 ��� ����������� ������
	clr r16 
	out TCNT2,r16 ; ������� �� ������ ������

	ldi r16,0b00000011 // prescaler = 32  
	out TCCR2,r16 

	ldi r16,0xF9 ; 249
	out OCR2,r16 ; ���������� ����� 249 � ������� ��������� �, ��� ��� ��� ���������� ��������� ����� ����� ������ ����� 1 ����

	in r16,TIMSK
	ori r16,0b10000000  
	out TIMSK,r16 ; ������������� ���������� �� ���������� �. ������������ sts, ��� ��� ����� �������� TIMSK0 ��������� � �������
	out TIFR,r16  ; TIFR0 ��������� � ������� "64 I/O Registers" (0x0020 - 0x005F)
ret
*/

// ��� ������ � TCNT1
TIM16_WriteTCNT1:
	// ��������� ��������� ������ 
	in r18,SREG
	// ��������� ����������
	cli
	// ���������� � TCNT1 ����� r17:r16
	out TCNT1H,r17
	out TCNT1L,r16
	// ��������������� ������� ������
	out SREG,r18
ret

// ��� ������ �� TCNT1
TIM16_ReadTCNT1:
	// ��������� ��������� ������ 
	in r18,SREG
	// ��������� ����������
	cli
	// ��������� TCNT1 � r17:r16
	in r16,TCNT1L
	in r17,TCNT1H
	// ��������������� ������� ������
	out SREG,r18
ret

// ��� ������ � OCR1A
TIM16_WriteOCR1A:
	// ��������� ��������� ������ 
	in r18,SREG
	// ��������� ����������
	cli
	// ���������� � TCNT1 ����� r17:r16
	out OCR1AH,r17
	out OCR1AL,r16
	// ��������������� ������� ������
	out SREG,r18
ret


// ��� ������ �� OCR1A
TIM16_ReadOCR1A:
	// ��������� ��������� ������ 
	in r18,SREG
	// ��������� ����������
	cli
	// ��������� OCR1A � r17:r16
	in r16,OCR1AL
	in r17,OCR1AH
	// ��������������� ������� ������
	out SREG,r18
ret

/*
// ����������� Timer2 ��� ���������� ������� ����������
init_timer2:
	clr r16
	out TCNT2,r16 ; ������� �� ������ ������

	ldi r16,0b00001000 ; ���� �� ��������� ������. WGM21 = 1 ��� ������ ����������
	out TCCR2,r16 ; 

	ldi r16,0xC7 ; ���������� ������� ������� 10 ��� ��� OCR2 = 249
	out OCR2,r16 ; ������� ����������

	in r16,TIMSK ; ����� �� ��������� ������ �������� ��������
	ori r16,0b10000000
	out TIMSK,r16 ; ������������� ���������� �� ����������. OCIE2 = 1 
	out TIFR,r16  ; ���������� ������� ��������. TIFR0 ��������� � ������� "64 I/O Registers" (0x0020 - 0x005F)
ret
*/


init_port: nop
	// ����������� ������
	clr r16
	out DDRB,r16 ; PB1-PB4 �� ����
	ldi r16,0b00000000
	out PORTB,r16 ; PB1-PB4 ��������
	clr r16
	out DDRA,r16 ; PA0-PA3 �� ����
	ldi r16,0b00000000
	out PORTA,r16 ; PA0-PA3 ��������

	// ����������� ��� � ��� �������� 
	clr r16
	out DDRD,r16 ; ��� PD2(INT0) �� ����
	ldi r16,0b00000100
	out PORTD,r16 ; ; ��� PD2(INT0) ��������
	; ����������� PD2(INT0) �� ���������� �� ������ (�.�. ��� ����� ������������� ������� ������)
	in r16,MCUCR
	//ori r16,0b00000011
	ldi r16,0b00000011
	out MCUCR,r16 ; ISC01 � ISC00 � ������� ��� ���������� �� ������ 
	ldi r16,0b01000000
	out GICR,r16 ; INT0 � ������� ��� ��������� ���������� �� ���� INT0

	// ����������� ���������� 
	ldi r16,0b11110000
	out DDRC,r16 ; PC4-PC7 �� �����
	clr r16
	out PORTC,r16 ; �� ������ ����
	
	// ����������� ���� STEP, DIR, ENABLE
	in r16,DDRA
	ori r16,0b01110000
	out DDRA,r16 ; STEP, DIR , ENABLE �� �����
	in r16,PORTA
	andi r16,0b10001111
	out PORTA,r16 ; STEP, DIR ���� �� ������

	// ����������� ���� ��� ������������ �������
	in r16,DDRA
	andi r16,0b01111111
	out DDRA,r16 ; INDSIGNAL �� ����
	in r16,PORTA
	andi r16,0b01111111
	out PORTA,r16 ; INDSIGNAL ��� ��������
ret



/*
[�][ ][ ][ ][ ][ ][�][�][�][ ][ ][ ][�][�][�][.]
[5][2][.][0][ ][ ][1][0][.][0][ ][ ][ ][9][9][ ]
*/
start_title: nop
    ldi r16,0x00 ; ������������� ������� �� ������ �� ������
	rcall Set_cursor

	; ������� �������
	ldi r16,0x58 ; ������� ������ "X"
	rcall Write_symbol

	ldi r16,0x2C ; ������� ������ ","
	rcall Write_symbol

	ldi r16,0x6D ; ������� ������ "m"
	rcall Write_symbol

	ldi r16,0x6D ; ������� ������ "m"
	rcall Write_symbol

	ldi r16,0x06 
	rcall Set_cursor ; ������ ������ �� ������� ������� ������� ������

	ldi r16,0x00 ; ������� ������ "�"
	rcall Write_symbol

	ldi r16,0x41 ; ������� ������ "�"
	rcall Write_symbol

	ldi r16,0x01 ; ������� ������ "�"
	rcall Write_symbol

	ldi r16,0x0C 
	rcall Set_cursor ; ������ ������ �� 13-� ������� ������� ������

	ldi r16,0x02 ; ������� ������ "�"
	rcall Write_symbol

	ldi r16,0x4F ; ������� ������ "O"
	rcall Write_symbol

	ldi r16,0x03 ; ������� ������ "�"
	rcall Write_symbol

	ldi r16,0x2E ; ������� ������ "."
	rcall Write_symbol

	// ������������� ������� �� ������ ������ ������
	ldi r16,0x40 
	rcall Set_cursor
	
	/* ������� �� ����� ��������*/
	// ��������� ������� ��������� X, ����� ������� �� �����
	lds r26,SHAGOVIK_X2_HIGH_RAM 
	lds r25,SHAGOVIK_X2_LOW_RAM 

	rcall lcd_print_number ; �������� �-���, ��������� �� ������� ����� r26:r25 � ���������� �������

	// ������� �������� ����
	ldi r16,0x46
	rcall Set_cursor

	clr r26 ; �������������� ������� ���� ��� ��������
	ldi r25,0x0A 
	sts RAM_step_size,r25 ; ���������� � ��� ������� ���
	rcall lcd_print_number ; �������� �-���, ��������� �� ������� ����� r26:r25 � ���������� �������

	// ������� ����� �������
	clr r26 ; �������������� ������� ���� ��� ��������
	lds r25,RAM_position_number ; ��������� �� ��� ����� ������� (0-99)
	// call function that deside print number or name of position
	rcall lcd_print_number_or_name_of_position
	
ret

// ���������� ����������, ����� � ��� �� ���� ������
init_values:
	clr r16
	sts SHAGOVIK_timer1_on,r16

	sts SET_BLOCK_CLK_FLAG,r16 // �� ��������� ������ ����� 
	
	ldi r16,0x01
	sts EXIT_FROM_SETTINGS_CLK_FLAG,r16 // ��������� ������ ������ �� ��������

	ldi r16,0x00
	sts PROG_FUNC_WORK_STATUS_RAM,r16 ; ���������� ���� � ���� ������ ������ ����. 0 - ������ ���� �� ��������. 1 - ��������.
ret




// �������������� ���������, ��������� �� ��� � ����������� �� �������� (���)
init_parameters:
	// ��������� ����� ��������� ������� �� EEPROM (0-100) (����� � EEPROM - 0x0000) ��������� EEPROM ---- --XX XXXX XXXX. �� ���� 10 ����� ����������
	ldi r18,0x00
	ldi r17,0x00 ; ����� � EEPROM 00 0000 0000
	rcall EEPROM_read ; ��������� ���� �� ���, ��� r18:r17 ��� ����� �����, � r16
	sts RAM_position_number,r16 ; ��������� � ��� ����� �������

	rcall get_position_value_from_eeprom ; ���� �� EEPROM �������� ��������� � ������ �� �������� ������ �������
	// ��������� ������� �� ����������� ��� ��������� ��� ���
	ldi r18,0x00
	ldi r17,0x02 ; EEPROM ����� 0x0002
	rcall EEPROM_read

	cpi r16,0xFF 
	breq position_priority // ����������� ��� ��������� �������, ������ ��������� ���������, ����������� � �������

// ����������� ��������, ������ ��������� ��������� �������, � ������� ��������� ����������
last_move_priority:
	ldi r17,0x03
	ldi r18,0x00 ; ����� �������� ����� � EEPROM 0x0003
	rcall EEPROM_read
	sts SHAGOVIK_X2_LOW_RAM,r16 ; ���������� � ��� ������� ���� ������� ������� 
	sts SHAGOVIK_X_TRULY_LOW_RAM,r16 ; ��������� ���������� ��������� � ���

	ldi r17,0x04
	ldi r18,0x00 ; ����� �������� ����� � EEPROM 0x0003
	rcall EEPROM_read
	sts SHAGOVIK_X2_HIGH_RAM,r16 ; ���������� � ��� ������� ���� ������� �������
	sts SHAGOVIK_X_TRULY_HIGH_RAM,r16 ; ��������� ���������� ��������� � ���
	

	
// ����������� ��� ��������� �������, ������ ��������� ���������, ����������� � �������
position_priority:
	// ���������� ��������� ��������� �0 � ��� �� �2
	lds r16,SHAGOVIK_X2_HIGH_RAM 
	sts SHAGOVIK_X0_HIGH_RAM,r16
	sts SHAGOVIK_X_TRULY_HIGH_RAM,r16 ; ��������� ���������� ��������� � ���
	lds r16,SHAGOVIK_X2_LOW_RAM 
	sts SHAGOVIK_X0_LOW_RAM,r16
	sts SHAGOVIK_X_TRULY_LOW_RAM,r16 ; ��������� ���������� ��������� � ���

	ldi r16,0b00000011
	sts access_to_button_flags,r16 ; ��������� ������� ���� ������
ret


get_position_value_from_eeprom:
	// �������� ����� ������� �� ���, ��� ��� ������ ������ ������� ����� ������ 2 �����, ����� � r0
	lds r16,RAM_position_number
	ldi r17,0x02
	mul r16,r17 

	// ���������� � ������ �������, ����������� �� ���, ����� 0x0A, ��� ��� � 0x0A � EEPROM ���������� �������� ������ ������� (��������� �) � ���� ���� ����
	ldi r16,0x0A
	add r0,r16

	// ���������� ����� � EEPROM ������������ � ��������, ����� ��������� ������� ���� ��������� � ��������� �������
	ldi r18,0x00
	mov r17,r0
	rcall EEPROM_read
	sts SHAGOVIK_X2_HIGH_RAM,r16 ; ���������� � ��� ������� ���� ������� �������
	; mov r26,r16 ; ��������� ������� ���� ��������� � r26 ��� ������ �� �������
	// ��������� �������� ���� ������� �������
	inc r17 ; ����������� ����� � EEPROM �� 1, ����� ������� ������� ���� 
	rcall EEPROM_read
	sts SHAGOVIK_X2_LOW_RAM,r16 ; ���������� � ��� ������� ���� ������� ������� 
	; mov r25,r16 ; ��������� ������� ���� ��������� � r25 ��� ������ �� �������
ret


TIM0_COMP:
	push r16 ; ��������� �������� �������� � �����, ����� �� ��������� ������ ������ �������
	in r16,SREG
	push r16
	clr r16
	out TCNT0,r16 ; �������� ������� ������� 0
	SBRS r17,3 ; ���� ����������� �����-�� �������, �� �� ������ ���� � r17, ����� �� �������� ������ �����-�� �������
	SBR r17,2 ; ������������� ��������� ��� ��� ����, ��� ��������� ����������, �� ���� ������ 10 ����
	pop r16
	out SREG,r16
	pop r16  ; ���������� �������� r16 �� �����
reti


// �-��� ���������� ����� �������
Button_up_func: nop
	rcall Clock_start_for_block // �������� ������, ������� ������� ���������� ����� 10 ������

	clr r26 ; ��� ��������
	lds r25,RAM_position_number ; ��������� ������� ����� �������
	inc r25 ; ���������� � ������ �������

	cpi r25,0x64 ; ���� ����� ������� �� �������� ����� 100
	brne print_num_up ; �� ��������� � ������ �� �������
	ldi r25,0x00 ; ����� ������������ � 0 
	
print_num_up:
	sts RAM_position_number,r25 ; ���������� ����� �������� � ���

	// ��������� �������� ��������� � ������ �� ������ �������
	rcall get_position_value_from_eeprom ; ��������� �� EEPROM ��������� � �� ������ �������

	// call function that deside print number or name of position
	rcall lcd_print_number_or_name_of_position
	
	ldi r16,0x40
	rcall Set_cursor
	
	// ��������� �������� �������� ��������� �
	lds r26,SHAGOVIK_X2_HIGH_RAM  
	lds r25,SHAGOVIK_X2_LOW_RAM 
	rcall lcd_print_number ; ������� �� ������� 
	
ret



Button_down_func:
rcall Clock_start_for_block // �������� ������, ������� ������� ���������� ����� 10 ������

clr r26 ; ��� ��������
	lds r25,RAM_position_number ; ��������� ������� ����� �������
	dec r25 ; ���������� � ������ �������

	cpi r25,0xFF ; ���� ����� ������� �� ������� �� 0
	brne print_num_down ; �� ��������� � ������ �� �������
	ldi r25,0x63 ; ����� ������������ � 99
	
print_num_down:
	sts RAM_position_number,r25 ; ���������� ����� �������� � ���
	// ��������� �������� ��������� � ������ �� ������ �������
	mov r16,r25 ; ���������� ����� ������� ��� ���������� ��������� �
	rcall get_position_value_from_eeprom ; ��������� �� EEPROM ��������� � �� ������ �������
	// ������� ����� ������� �� �������
	
	// call function that deside print number or name of position
	rcall lcd_print_number_or_name_of_position

	ldi r16,0x40
	rcall Set_cursor
	
	// ��������� �������� �������� ��������� �
	lds r26,SHAGOVIK_X2_HIGH_RAM  
	lds r25,SHAGOVIK_X2_LOW_RAM 
	rcall lcd_print_number ; ������� �� ������� 
ret


Button_ok_func: nop
	cli ; ������ ����������
	
	rcall Clock_start_for_block // �������� ������, ������� ������� ���������� ����� 10 ������

// ������ ����
shagovik_stage1:
	// ��������� ������� ��������� X0 (���������)
	lds r27,SHAGOVIK_X0_HIGH_RAM
	lds r26,SHAGOVIK_X0_LOW_RAM
	// ��������� ��������� X2, ���� ���� �����������
	lds r31,SHAGOVIK_X2_HIGH_RAM
	lds r30,SHAGOVIK_X2_LOW_RAM 

	sts SHAGOVIK_X_TRULY_HIGH_RAM,r31 ; ���������� � ��� ������� ���� �������
    sts SHAGOVIK_X_TRULY_LOW_RAM,r30  ; ���������� � ��� ������� ���� �������
	// ���������, �� ����� �� ����� ��������� ���������� ���������, ��� ���, ��� ��� �� �� �������
	CPSE r27,r31 ; ���� �����, �� ���������� ����. �������
	rjmp find_mid ; ���� �� �����, �� ���� ������
	CPSE r26,r30 ; ���� �����, �� ���������� ����. �������
	rjmp find_mid ; ���� �� �����, �� ���� ������
	// ���� ��������� ��������� ��������� ������ ������, �� ������ ���������
	rcall encoder_print_num ; ������� �� ����� �������� ������� �2, ���� �������� �������
	// ����� �� ����� ������� � EEPROM �������, ������� �������� ���������
	ldi r17,0x00
	ldi r18,0x00 ; ����� ������ ������� � EEPROM 0x0000
	rcall EEPROM_read
	lds r17,RAM_position_number ; ��������� �� ��� ����� ������� �������
	CPSE r17,r16 ; ���� �����, �� ���������� ����. �������
	rjmp save_pos_number_to_eeprom ; ���� ������ ������� �� �����, �� ������ ��������� ����� ����� ������� � EEPROM 
	// ���� ������ ������� �����, �� ����������, ��� ������� ��������� ������ �������, � ����� �������
	// ��� ������������� ���������, ������� �� �������
	ldi r16,0x00
    sts ENCODER_CONTR_MOT_ENCODER_PROCESSING,r16 ; ���������� ���� ����, ��� �������� ���������� �� ��������
	clr r17 ; ����� ������ ��������
	rjmp ok_exit ; ������� �� �������
// ��������� ����� ����� ������� � EEPROM
save_pos_number_to_eeprom:
	mov r16,r17
	ldi r17,0x00
	ldi r18,0x00 ; ����� ������ ������� � EEPROM 0x0000
	rcall EEPROM_write
	clr r17 ; ����� ������ ��������
	rjmp ok_exit ; ������� �� �������

find_mid:
	// �������� ���������
	SBI PORTC,PC7
	// ������� ������� ��������� X1. X1 = (|X2 - X0|)/2 +(-) X0) 
	// ��������, X0 = 14.2, X2 = 26.2 -> X1 = (26.2 - 14.2)/2 + 14.2 = 20.2
	// X2 - X0
	sub r30,r26
	sbc r31,r27
	BRPL poloj ; ���� �������� �������� �������������, �� ��������� �� �����

	// ����� �������� ��������� �������������
	CBI PORTA,DIR ; ����������, ��� �������� �����
	// ��������� ��� ��� ��������� �2, ��� ��� ��� ���� ���������
	lds r31,SHAGOVIK_X2_HIGH_RAM
	lds r30,SHAGOVIK_X2_LOW_RAM 
	// ������� �������� ��� ����, ��� �0 ������, ��� �2
	sub r26,r30
	sbc r27,r31
	// ����� �� 2 ������� ������
	lsr r27
	ror r26
	// ��������� ��� ��� ��������� �0, ��� ��� ��� ���� ���������. WARNING: �������� �0 � �������� ��� ������ �1 ��������
	lds r29,SHAGOVIK_X0_HIGH_RAM
	lds r28,SHAGOVIK_X0_LOW_RAM
	// �������� �� �0 ���������� �������, ������� �� ���
	sub r28,r26
	sbc r29,r27
	// ������ � ��������� r29:r28 �������� X1
	rjmp save_X1_to_ram // ��������� �1 � ���
	 
poloj:
	SBI PORTA,DIR ; ����������, ��� �������� �����
	// ����� ���������� �������� �� 2 ������� ������
	lsr r31
	ror r30
	// ������������� �0 � �������� ��� �1
	mov r28,r26
	mov r29,r27
	// ���������� � �0 ���������� �������, ������� �� ���
	add r28,r30
	adc r29,r31
	// ������ � ��������� r29:r28 �������� X1
// ��������� �1 � ���
save_X1_to_ram: 
	sts SHAGOVIK_X1_HIGH_RAM,r29
	sts SHAGOVIK_X1_LOW_RAM,r28
	// ��������� � ��� ����� ��������� �������, ����� ��� ��������� ��������� ������ �������� � ���� �������
	lds r16,INIT_FLAG
	cpi r16,0x00
	breq ok_next1 ; ���� ���� ��������� �������, � �� ������� ������, �� �� ��������� � ��� �������� ������� �������, �.�. ��� � ��� ��� ���������
// ��������� ����� �������� ������� � ���
position_number_save_to_eeprom:
	clr r18 ; ��� ������
	ldi r17,0x00 ; ����� ������ ������� � ��� = 0x00
	lds r16,RAM_position_number ; ��������� ������� ������� �� EEPROM
	rcall EEPROM_write ; ���������� � EEPROM
ok_next1:
	// ��������� ������� ��������� X0 (���������)
	lds r27,SHAGOVIK_X0_HIGH_RAM
	lds r26,SHAGOVIK_X0_LOW_RAM
	// ��������� ��������� X2, ���� ���� �����������
	lds r31,SHAGOVIK_X2_HIGH_RAM
	lds r30,SHAGOVIK_X2_LOW_RAM 

// ������ ����
shagovik_stage2:
	// �������������� ��� ������ ����������
	mov r25,r27 ; � ���� ��������� ����� ��������� ������� ���������
	mov r24,r26
	;ldi r25,SHAGOVIK_freq_maxH ; � ���� �������� ����� ��������� ������� ���� ����������� �������� OCR1A (������������ �������)
	;ldi r24,SHAGOVIK_freq_maxL ; � ���� �������� ����� ��������� ������� ���� ����������� �������� OCR1A (������������ �������)
	;ldi r23,SHAGOVIK_freq_minH ; � ���� �������� ����� ��������� ������� ���� ������������ �������� OCR1A (����������� �������)
	;ldi r22,SHAGOVIK_freq_minL ; � ���� �������� ����� ��������� ������� ���� ������������ �������� OCR1A (����������� �������)
	;ldi r22,SHAGOVIK_pid_prop ; � ���� �������� ����� ��������� ���������������� ����������� ����
	;;ldi r21,SHAGOVIK_step ; � ���� �������� ����� ��������� ��� �������� �������� ��������� (��)
	ldi r21,SHAGOVIK_DELTA_L_LOW ; � ���� �������� ����� ��������� ���������, ������������ ���������� ��������� STEP, ��� ������� ����� ������� ���� 0,1�� ��� ���� 1/16
	ldi r22,SHAGOVIK_DELTA_L_HIGH ; ������� ���� ���������

	; � ��������� OCR1AH:OCR1AL ��������� ������� ������� ��

	// �������������� ������1 
	// �������� TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; ���������� � TCNT1 r17:r16

	// ���������� � OCR1A. ����������� �������, � ������� ���������� ��������. � ��������� OCR1AH:OCR1AL ��������� ������� ������� ��
	ldi r16,SHAGOVIK_freq_minL
	ldi r17,SHAGOVIK_freq_minH
	rcall TIM16_WriteOCR1A ; ���������� � OCR1A r17:r16


	// ��������� ������1 
	//ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; ������������ 1
	//out TCCR1B,r16
	ldi r16,0x01 
	sts SHAGOVIK_timer1_on,r16 ; ����������, ��� timer1 �������

	//ldi r16,0b00000000
	//out GICR,r16 ; ��������� �������

	lds r16,INIT_FLAG ; ���� ���� ���������, �� ������� ��������� ���������
	cpi r16,0x00
	breq ok_exit

	// ���� ������� ��������� ��������� ��������, �� �� �������� �� ����� ������� �������
	lds r16,ENCODER_CONTR_MOT_PRESS_OK
	cpi r16,0x01
	breq ok_exit
print_num:
	// ��������� ��������
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

	// ������� �� ������� ����� ��������� �
	ldi r16,0x40
	rcall Set_cursor
	
	// ��������� �������� �������� ��������� �
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; ������� �� ������� 

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
	// ��������� � ��� �������� ��� ������������ ��������
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

sei ;��������� ����������
ret











Button_home_func: nop
	rcall Clock_start_for_block // �������� ������, ������� ������� ���������� ����� 10 ������

	SBI PORTC,LED_home ; �������� ��������� ������ HOME

	// ��������� ���������� �� ����������� ������� (��). ���� �� �������, �� ���� �����
	SBIC PINA,INDSIGNAL ; ���� �������
	SBI PORTA,DIR ; ���� �����
	
	SBIS PINA,INDSIGNAL ; ���� �� �������
	CBI PORTA,DIR ; ���� �����

	// ��������� ���� ����� ��� �����, �� ���� �� ����� ����� ���������
	SBIS PORTA,DIR 
	rjmp edem_nazad
	ldi r26,0x00
	rcall low_speed ; ������ ��������� �������� ��������

	ldi r22,HOME_SHAGOVIK_DELTA_HIGH_0 ; ������� ����� ������ �� ����� 0 ������� ����
	ldi r21,HOME_SHAGOVIK_DELTA_LOW_0  ; ������� ����� ������ �� ����� 0 ������� ����
	
	rjmp home_next
// ���� �� 0 �����
edem_nazad:
	// ���� �� 1 �����
	ldi r26,0x01 /* ����������, ��� �������� ������� ���� ������ HOME.
				    0 - �� ���������� ��� �������, ������� ���� ����� 2 �� �� ��������� ��������, � ����� ����� ��������� � 4 �����
				    1 - �� ���������� �� ��� �������, ������� ���� ����� �� ������� ��������, ���� �� �� ������ ��������
				    2 - ����� �� ���� �������, ���� ����� 1 �� �� ��������� ��������
				    3 - ���� �����, ���� �� �� ������ ��������
				    4 - �� ���� ��������, ���� ����� ����� 1 �� � ����
				 */
	rcall fast_speed ; ������ ������� �������� ��������
home_next:
	// �������������� ������1 
	// �������� TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; ���������� � TCNT1 r17:r16

	// ��������� ������1 
	//ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; ������������ 1
	//out TCCR1B,r16
	ldi r16,0x01 
	sts SHAGOVIK_timer1_on,r16 ; ����������, ��� timer1 �������

	ldi r16,0b00000000
	out GICR,r16 ; ��������� �������

	ldi r23,SHAGOVIK_DELTA_L_LOW
	ldi r24,SHAGOVIK_DELTA_L_HIGH
	// ��������� � ��� �������� ��� ������������ ��������
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
	

	rcall Clock_start_for_block // �������� ������, ������� ������� ���������� ����� 10 ������

	// ���������, �������� �� ������ ����
	lds r16,PROG_FUNC_WORK_STATUS_RAM ; ��������� ���� ������ ������ ����
	cpi r16,0x01 
	breq prog_next1 ; ���� �� ����� ��������� ��������, �� ������� ������
	ret ; ����� �������

prog_next1:
	
	rcall blink_prog_led



	// ��������� ������� ����� �������
	lds r16,RAM_position_number

	// ���������� ��������� � ������ ������� � ��� 263 (26.3)
	ldi r17,0x02
	mul r16,r17
	mov r16,r0
	ldi r17,0x0A
	add r17,r16
	clr r18

	lds r16,SHAGOVIK_X2_HIGH_RAM  ; ������� ������� ���� �������� ��������� �
	rcall EEPROM_write

	inc r17
	lds r16,SHAGOVIK_X2_LOW_RAM  ; ������ ������� ���� �������� ��������� �
	rcall EEPROM_write

	

ret

blink_prog_led:
	cli
	// �������� ��������� ������ ����
	sbi PORTC,LED_prog
		
	// �����
	ldi r16,0x64
	rcall nopdelay_1ms
	cbi PORTC,LED_prog ; ��������� ��������� �����

	// �����
	ldi r16,0x64
	rcall nopdelay_1ms
	sbi PORTC,LED_prog ; �������� ��������� �����

	// �����
	//ldi r16,0x64
	//rcall nopdelay_1ms
	//cbi PORTC,LED_prog ; ��������� ��������� �����
	sei
ret


Button_block_func:


SBIS PORTC,LED_block ; ���� �������� ��������� ������ ����, �� �������� � ��������� ������� ���� ������ � ��������
rjmp block_on

block_off: ; ��������� ����� �����
CBI PORTC,LED_block
ldi r16,0b00000011 
sts access_to_button_flags,r16 ; ��������� ������� ���� ������, ����� ����
ldi r16,0b01000000
out GIFR,r16 ; ������� ���� ������������ ����������
out GICR,r16 ; ��������� ���������� �� INT0, ����� ��������� ��������� ��������

rcall Clock_start_for_block // �������� ������, ������� ������� ���������� ����� 10 ������

ret

block_on:; �������� ����� �����
SBI PORTC,LED_block
ldi r16,0b00000001 
sts access_to_button_flags,r16 ; ��������� ������� ���� ������
ldi r16,0b00000000
out GICR,r16 ; INT0 � ������� ��� ��������� ���������� �� ���� INT0

rcall Clock_stop_for_block // ��������� ������, ������� ������� ���������� ����� 10 ������

ret


// �-��� ��������� ����. ��������: 10.0, 01.0, 00.1
Button_step_func:
	rcall Clock_start_for_block // �������� ������, ������� ������� ���������� ����� 10 ������

	lds r16,RAM_step_size ; ��������� �� ��� ������� �������� ����

	cpi r16,0x0A ; ���� ��� ����� 10 (01.0)
	breq set_step_1

	cpi r16,0x01 ; ���� ��� ����� 1 (00.1)
	breq set_step_2

	; ���� ��� ��� ����� 100 (10.0)
	ldi r16,0x01 ; ����� ��� ����� 1 (00.1)
	rjmp save_step

	set_step_1:
	ldi r16,0x64 ; ����� ��� ����� 100 (10.0)
	rjmp save_step

	set_step_2:
	ldi r16,0x0A ; ����� ��� ����� 10 (01.0)
	// ��������� ����� �������� ���� � ���
	save_step:
	sts RAM_step_size,r16

	// ������� �������� ���� �� �������
	mov r25,r16 ; ��������� �������� ���� ��� ���������� ������ �� �����
	clr r26 
	ldi r16,0x46
	rcall Set_cursor ; ������ ������ ��� ������ �������� ����

	rcall lcd_print_number ; ������� �������� ���� �� �������

ret







// ������� ��� �������� ��������. �������� ����� ������� 8 ��������. 
/*
byte bukva_B[8]   = {B11110,B10000,B10000,B11110,B10001,B10001,B11110,B00000,}; // ����� "�"
byte bukva_G[8]   = {B11111,B10001,B10000,B10000,B10000,B10000,B10000,B00000,}; // ����� "�"
byte bukva_D[8]   = {B01111,B00101,B00101,B01001,B10001,B11111,B10001,B00000,}; // ����� "�"
byte bukva_ZH[8]  = {B10101,B10101,B10101,B11111,B10101,B10101,B10101,B00000,}; // ����� "�"
byte bukva_Z[8]   = {B01110,B10001,B00001,B00010,B00001,B10001,B01110,B00000,}; // ����� "�"
byte bukva_I[8]   = {B10001,B10011,B10011,B10101,B11001,B11001,B10001,B00000,}; // ����� "�"
byte bukva_IY[8]  = {B01110,B00000,B10001,B10011,B10101,B11001,B10001,B00000,}; // ����� "�"
byte bukva_L[8]   = {B00011,B00111,B00101,B00101,B01101,B01001,B11001,B00000,}; // ����� "�"
byte bukva_P[8]   = {B11111,B10001,B10001,B10001,B10001,B10001,B10001,B00000,}; // ����� "�"
byte bukva_Y[8]   = {B10001,B10001,B10001,B01010,B00100,B01000,B10000,B00000,}; // ����� "�"
byte bukva_F[8]   = {B00100,B11111,B10101,B10101,B11111,B00100,B00100,B00000,}; // ����� "�"
byte bukva_TS[8]  = {B10010,B10010,B10010,B10010,B10010,B10010,B11111,B00001,}; // ����� "�"
byte bukva_CH[8]  = {B10001,B10001,B10001,B01111,B00001,B00001,B00001,B00000,}; // ����� "�"
byte bukva_Sh[8]  = {B10101,B10101,B10101,B10101,B10101,B10101,B11111,B00000,}; // ����� "�"
byte bukva_Shch[8]= {B10101,B10101,B10101,B10101,B10101,B10101,B11111,B00001,}; // ����� "�"
byte bukva_Mz[8]  = {B10000,B10000,B10000,B11110,B10001,B10001,B11110,B00000,}; // ����� "�"
byte bukva_IYI[8] = {B10001,B10001,B10001,B11001,B10101,B10101,B11001,B00000,}; // ����� "�"
byte bukva_Yu[8]  = {B10010,B10101,B10101,B11101,B10101,B10101,B10010,B00000,}; // ����� "�"
byte bukva_Ya[8]  = {B01111,B10001,B10001,B01111,B00101,B01001,B10001,B00000,}; // ����� "�"
*/
Create_Char:
	ldi r16,0b01000000 ; ���������� 7-� ���������� ��� ��������� � ����������������� ������ CGRAM, ��� DB6 ����� �������, � ����� ���������� � 0x00
	clr r17
	sts 0x0102,r17 ; RS � R/W � 0 ��� 7-� ���������� 
	rcall LCD1602_send_command ; ���������� ������

	// �������������� 8 ���� ��� �������� 1-��� ������� � ����� 0x00 
	// ������ "�"
	ldi r17,0b10101
	ldi r18,0b10101
	ldi r19,0b10101
	ldi r20,0b10101
	ldi r21,0b10101
	ldi r22,0b10101
	ldi r23,0b11111
	ldi r24,0b00000
	// ���������� ������ � �������
	rcall write_new_symbol

	// �������������� 8 ���� ��� �������� 2-��� ������� � ����� 0x01 
	// ������ "�"
	ldi r17,0b11111
	ldi r18,0b10001
	ldi r19,0b10000
	ldi r20,0b10000
	ldi r21,0b10000
	ldi r22,0b10000
	ldi r23,0b10000
	ldi r24,0b00000
	// ���������� ������ � �������
	rcall write_new_symbol
	
	// �������������� 8 ���� ��� �������� 3-��� ������� � ����� 0x02 
	// ������ "�"
	ldi r17,0b11111
	ldi r18,0b10001
	ldi r19,0b10001
	ldi r20,0b10001
	ldi r21,0b10001
	ldi r22,0b10001
	ldi r23,0b10001
	ldi r24,0b00000
	// ���������� ������ � �������
	rcall write_new_symbol


	// �������������� 8 ���� ��� �������� 3-��� ������� � ����� 0x03 
	// ������ "�"
	ldi r17,0b01110
	ldi r18,0b10001
	ldi r19,0b00001
	ldi r20,0b00010
	ldi r21,0b00001
	ldi r22,0b10001
	ldi r23,0b01110
	ldi r24,0b00000
	// ���������� ������ � �������
	rcall write_new_symbol
ret


write_new_symbol:
// ���������� ������ � �������
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






// �-��� ��� ��������� ���� ������ �� ����
subtraction_2x2:
SUB r25,r23 ; �������� ������� �����
SBC r26,r24 ; �������� ������� �����, �������� �������
ret

// �-��� ��� �������� ���� ������ � ����
addition_2x2:
add r25,r23 ; ���������� ������� �����
adc r26,r24 ; ���������� ������� �����, �������� �������
ret




// ������� ������� ��� ����� �� ��� (������������� ������ ������� �� 10). � r25 - ����� �����. � r16 - �������. r26,r26 / r24,r23
division_2x2: nop
	clr r16  ; ������� ���������
	push r26 ; ��������� ������� ���� � �����
	push r25 ; ��������� ������� ���� � ����� 
division_proc:
	sub r25,r23 ; �������� �� ������������ ���������� ���� ����� �� ������ �������������
	sbc r26,r24 ; ��������, �������� �������
	inc r16     ; ���������� � ������� ���������, ������� � ����� ���� ����� ������� (��� ����� �����)
	brcc division_proc

	dec r16 ; � r16 ��������� ����� �����
	mul r23,r16 ; �������� �������� �� �������� ���������� ���������, ����� � ������� ����� ����� ������. �������
	pop r25 ; ���������� ���������� �������� �� �����
	pop r26 ; ���������� ���������� �������� �� �����
	sub r25,r0  ; �������� �� �������� ����� ����� ���������� ��� �������, ����� ������� � r25 ����� �������
	mov r0,r16  
	mov r16,r25 ; �������� ������� � r16
	mov r25,r0  ; �������� ����� ����� � r25
	clr r26     ; �������� r26 ��� ����������� ����������
ret








// ������� ������������ �������� ���� ����, ����� ������� �� ������� � ���������� �������, XX.X
lcd_print_number: nop
	// ��������� �� ������ ������
	push r18
	push r17

	// �������� �������
	clr r18 ; �������
	clr r24
	cycle22:
	ldi r23,0x0A ; ������� �� 10
	rcall division_2x2 ; ����� ������� �� ������� �� 10, ������� ����� � r16, ����� ����� �� ������� ���������� � r25
	ldi r24,0x30
	add r16,r24 ; ���������� � �������� 30h, ��������� � 30h ��������� ������ "0", ����� ������� ��������� �����-�� �����
	clr r24
	// ���� ��������� ������� ���������� ������
	cpi r18,0x00
	breq save_r2 ; �� ��������� � r2

	// ���� ��������� ������� ���������� ������
	cpi r18,0x01
	breq save_r3 ; �� ��������� � r1

	// ���� ��������� ������� ���������� ������
	cpi r18,0x02
	breq save_r4 ; �� ��������� � r0

	save_r4:
	mov r4,r16
	rjmp incc

	save_r3:
	mov r3,r16
	rjmp incc

	save_r2:
	mov r2,r16

	incc:
	inc r18 ; ���������� � �������
	cpi r18,0x03
	brne cycle22

	cpi r20,0x4C
	breq print_next ; ���� ������ ���������� �� ����� �������, �� ���������� ����� �������� �������

	mov r16,r4
	rcall Write_symbol ; ������� ������� ������ �����

print_next:
	mov r16,r3
	rcall Write_symbol ; ������� ������� ������ �����

	cpi r20,0x4C
	breq print_next2 ; ���� ���������� ���� ������ ������ �������, �� ���������� ����� �����

	ldi r16,0x2E
	rcall Write_symbol ; ������� ������ "."
print_next2:
	mov r16,r2
	rcall Write_symbol ; ������� ������� ������ �����
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
	rcall Write_symbol ; ������� ������ " "

	rcall lcd_print_number ; ������� �� �������

	ldi r16,0x20
	rcall Write_symbol ; ������� ������ " "
ret






// ���������� ������. ������� ���������� ������ 100 ���� ����� ������� ������
button_press: nop

	SBRC r17,0 ; ���� ��� ���������� � �������� ��������� ������
	rjmp next  ; �� ������� � �������� ���������

	// ����� 
	clr r18 ; ������� �����������

	// ��������� ������
	clr r16 
	out TCNT0,r16 ; ���������� ������� ������� 0
	ldi r16,0b00001011  ; WGM01 = 1 - CTC �����
	out TCCR0,r16 ; ������������� ������������ ������� �� 64
	in r16,TIFR ; ��������� � r16, ����� �� ��������� �������� ��������
	ori r16,0b00000010 
	out TIFR,r16 ; ���������� ���� ����������
	SBR r17,1 ; ������������� ���� ��������� ����������
	ret ; ������� �� ����������

next: 
	SBRS r17,1 ; ���� ��������� ��������� ����������
	ret ; �� �� ��������

	inc r18 ; ���������� � �������� �����������
	CBR r17,2 ; ���������� ��������� ��� ������� ��� �������� ���������� ���������� �������0

	SBRC r17,2 ; ���� ���� ��������� � ������ �������� ���������
	rjmp main_process ; �� �� ���������� � �������� ���������
	
	SBRC r17,4 ; ���� ������� �������� ���� ��������� ������ � ������ ����������� ����������
	rjmp end_process ; �� ������� � ������������ ���������� ������

// ������� ���������� �������� ��� ������� �� ������
debounce_process:
	cpi r18,debounce_time ; ������� ������� ������ �����������
	brne exit ; ���� ��� �� ������ ����� ��������, �� �������
	SBR r17,4 ; ������������� ������ ���, ���������, ��� ���� �������� �������
	clr r18   ; ���������� �������, ����� ������ ����� ������ ��� ����� �������� �� ������
	rjmp check_pin ; ��������� ������� �� ����

// �������� ���������, ����� ������� ���������
main_process:
	cpi r18,check_pin_time ; ��������� ������ �� ����� �������� �� ������
	breq finish ; ���� ������, �� ������������ � ������������� ���� ������ �������, �� ������� ��������� ������
	rjmp check_pin

//
end_process:
	SBRS r17,5 ; ���� ��� �� ���� ������������� ���������� ������
	rjmp check_pin ; �� ���������� ����������� ����������
    // ����� ����� ������� ��� ���������� ������
	cpi r18,debounce_end_time ; ���� ������ debounce_end_time �����������, �� ��������� ��������� ��������� ������
	breq the_end ; ��������� ��������� ������
	ret

// ����� ��������� ������ �� ������ (������ ������������)
check_pin:
	BRTS stop_end_exit ; ���� �� ���� �������� 1, �� ��������� ���������, ��� ��� ��� ��������� ������
	ret ; ����� �����

// �����
finish:
	ori r17,0b00011000 ; ������������� 3-� ���, ���� ���������� ������ �������, ������������ �� ������
                       ; ������������� 4-� ���, ������������, ��� �������� ���� ������� � ����� ���� ������������ ����������
	CBR r17,4 ; ���������� 2-� ���, ����� ������ �� �������� � main_process
	ret

// ����� � ��������� ��������� ������, ��� ��� ��������� ������
stop_end_exit:
	SBRS r17,4 ; ���� ������ ������� ���������, �� �� ������������� ���������� ������ (end_process)
	rjmp the_end ; ���� �� ���� ��������� ������������ ������, �� ��������� ��������� ������
	SBR r17,32 ; ������������� ���� ����, ��� ���� ������������� ���������� ������ ����� ��������� ��� ������������
	clr r18    ; ���������� ������� �������
	ret
	the_end:
	clr r16
	out TCCR0,r16 ; ��������� ������
	clr r17 ; ������� ������� ������

exit:
ret







// ���������� ������ �����������
Button_up: nop
	SBRS r17,0 ; ���������� ��������� �������, ���� ��� ��������� � �������� ��������� �����-�� ������
	rjmp step_up_next1 ; ��������� �����
	//���� ��� ��������� � �������� ��������� ������, �� ���������, �� �� ������ ������ ������������
	lds r16,RAM_ACTIVE_BUTTON ; ��������� �� ���, ����� ������ ������ �������
	cpi r16,RAM_BUTTON_UP ; ����������
	breq step_up_next1 ; ���� ��� ������ ������ �������, �� ��� ������������ ������
	ret ; ����� ��� �� �� ������, ������� ������ ��������������, �������

// ���� ��������� � �������� ��������� ������ ������ ��� ��� �������� �������� ��������� ������
step_up_next1:
	in r16,PINB ; ��������� ��������� ����� ����� 
	com r16 ; ����������� ��������� �����. ������ ��� �������, ���� �������� ������� ������� ������ = 0.
	BST r16,PB4 ; ���������� �� ���� � �������� ����
	SBRC r17,0 ; ���� � ���� 1, �� ��������� ������ �������
	rjmp step_up_next3 ; ���� ��� ���������� � �������� ��������� ������ ������ 
	BRTC step_up_next2 ; ���� ��� �� ���������� � �������� ��������� ������ � ���� �����-�� ������ ���� ������
	ret ; ���� ������ �� ���������� � �� ���� �������� �������� ���������, �� ������ �������

// ��� ��� ������ �������� �������, �� ��������� � ��� ���������� � ���, ����� ������ ������, ����� ������������ ������ �
step_up_next2:
    ldi r16,RAM_BUTTON_UP
	sts RAM_ACTIVE_BUTTON,r16
step_up_next3:
	rcall button_press ; �������� ���������� ������� ������, � ������� �-��� ���������� ������ 100 ����
	push r17 ; ��������� ������� ������ ��� ������� � ������ �������
	push r18
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ��������
	rcall Button_up_func ; ������� �-���, ������������ �� ������
	pop r18
	pop r17 ; ���������� r17
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ���������� ������� ������
	CBR r17,8 ; �������� ���� ������ �������, ����� ������ � �� ��������
ret










// ���������� ������ �����������
Button_down: nop
	SBRS r17,0 ; ���������� ��������� �������, ���� ��� ��������� � �������� ��������� �����-�� ������
	rjmp step_down_next1 ; ��������� �����
	//���� ��� ��������� � �������� ��������� ������, �� ���������, �� �� ������ ������ ������������
	lds r16,RAM_ACTIVE_BUTTON ; ��������� �� ���, ����� ������ ������ �������
	cpi r16,RAM_BUTTON_DOWN ; ����������
	breq step_down_next1 ; ���� ��� ������ ������ �������, �� ��� ������������ ������
	ret ; ����� ��� �� �� ������, ������� ������ ��������������, �������

// ���� ��������� � �������� ��������� ������ ������ ��� ��� �������� �������� ��������� ������
step_down_next1:
	in r16,PINB ; ��������� ��������� ����� ����� 
	com r16 ; ����������� ��������� �����. ������ ��� �������, ���� �������� ������� ������� ������ = 0.
	BST r16,PB3 ; ���������� �� ���� � �������� ����
	SBRC r17,0 ; ���� � ���� 1, �� ��������� ������ �������
	rjmp step_down_next3 ; ���� ��� ���������� � �������� ��������� ������ ������ 
	BRTC step_down_next2 ; ���� ��� �� ���������� � �������� ��������� ������ � ���� �����-�� ������ ���� ������
	ret ; ���� ������ �� ���������� � �� ���� �������� �������� ���������, �� ������ �������

// ��� ��� ������ �������� �������, �� ��������� � ��� ���������� � ���, ����� ������ ������, ����� ������������ ������ �
step_down_next2:
    ldi r16,RAM_BUTTON_DOWN
	sts RAM_ACTIVE_BUTTON,r16
step_down_next3:
	rcall button_press ; �������� ���������� ������� ������, � ������� �-��� ���������� ������ 100 ����
	push r17 ; ��������� ������� ������ ��� ������� � ������ �������
	push r18
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ��������
	rcall Button_down_func ; ������� �-���, ������������ �� ������
	pop r18
	pop r17 ; ���������� r17
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ���������� ������� ������
	CBR r17,8 ; �������� ���� ������ �������, ����� ������ � �� ��������
ret








// ���������� ������ �����������
Button_ok: nop
	SBRS r17,0 ; ���������� ��������� �������, ���� ��� ��������� � �������� ��������� �����-�� ������
	rjmp step_ok_next1 ; ��������� �����
	//���� ��� ��������� � �������� ��������� ������, �� ���������, �� �� ������ ������ ������������
	lds r16,RAM_ACTIVE_BUTTON ; ��������� �� ���, ����� ������ ������ �������
	cpi r16,RAM_BUTTON_OK ; ����������
	breq step_ok_next1 ; ���� ��� ������ ������ �������, �� ��� ������������ ������
	ret ; ����� ��� �� �� ������, ������� ������ ��������������, �������

// ���� ��������� � �������� ��������� ������ ������ ��� ��� �������� �������� ��������� ������
step_ok_next1:
	in r16,PINB ; ��������� ��������� ����� ����� 
	com r16 ; ����������� ��������� �����. ������ ��� �������, ���� �������� ������� ������� ������ = 0.
	BST r16,PB2 ; ���������� �� ���� � �������� ����
	SBRC r17,0 ; ���� � ���� 1, �� ��������� ������ �������
	rjmp step_ok_next3 ; ���� ��� ���������� � �������� ��������� ������ ������ 
	BRTC step_ok_next2 ; ���� ��� �� ���������� � �������� ��������� ������ � ���� �����-�� ������ ���� ������
	ret ; ���� ������ �� ���������� � �� ���� �������� �������� ���������, �� ������ �������

// ��� ��� ������ �������� �������, �� ��������� � ��� ���������� � ���, ����� ������ ������, ����� ������������ ������ �
step_ok_next2:
    ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16
step_ok_next3:
	rcall button_press ; �������� ���������� ������� ������, � ������� �-��� ���������� ������ 100 ����
	push r17 ; ��������� ������� ������ ��� ������� � ������ �������
	push r18
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ��������
	rcall Button_ok_func ; ������� �-���, ������������ �� ������
	pop r18
	pop r17 ; ���������� r17
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ���������� ������� ������
	CBR r17,8 ; �������� ���� ������ �������, ����� ������ � �� ��������
ret









// ���������� ������ �����������
Button_home: nop
	SBRS r17,0 ; ���������� ��������� �������, ���� ��� ��������� � �������� ��������� �����-�� ������
	rjmp step_home_next1 ; ��������� �����
	//���� ��� ��������� � �������� ��������� ������, �� ���������, �� �� ������ ������ ������������
	lds r16,RAM_ACTIVE_BUTTON ; ��������� �� ���, ����� ������ ������ �������
	cpi r16,RAM_BUTTON_HOME ; ����������
	breq step_home_next1 ; ���� ��� ������ ������ �������, �� ��� ������������ ������
	ret ; ����� ��� �� �� ������, ������� ������ ��������������, �������

// ���� ��������� � �������� ��������� ������ ������ ��� ��� �������� �������� ��������� ������
step_home_next1:
	in r16,PINA ; ��������� ��������� ����� ����� 
	com r16 ; ����������� ��������� �����. ������ ��� �������, ���� �������� ������� ������� ������ = 0.
	BST r16,PA1 ; ���������� �� ���� � �������� ����
	SBRC r17,0 ; ���� � ���� 1, �� ��������� ������ �������
	rjmp step_home_next3 ; ���� ��� ���������� � �������� ��������� ������ ������ 
	BRTC step_home_next2 ; ���� ��� �� ���������� � �������� ��������� ������ � ���� �����-�� ������ ���� ������
	ret ; ���� ������ �� ���������� � �� ���� �������� �������� ���������, �� ������ �������

// ��� ��� ������ �������� �������, �� ��������� � ��� ���������� � ���, ����� ������ ������, ����� ������������ ������ �
step_home_next2:
    ldi r16,RAM_BUTTON_HOME
	sts RAM_ACTIVE_BUTTON,r16
step_home_next3:
	rcall button_press ; �������� ���������� ������� ������, � ������� �-��� ���������� ������ 100 ����
	push r17 ; ��������� ������� ������ ��� ������� � ������ �������
	push r18
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ��������
	rcall Button_home_func ; ������� �-���, ������������ �� ������
	pop r18
	pop r17 ; ���������� r17
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ���������� ������� ������
	CBR r17,8 ; �������� ���� ������ �������, ����� ������ � �� ��������
ret





// ���������� ������ �����������
Button_prog: nop
	SBRS r17,0 ; ���������� ��������� �������, ���� ��� ��������� � �������� ��������� �����-�� ������
	rjmp step_prog_next1 ; ��������� �����
	//���� ��� ��������� � �������� ��������� ������, �� ���������, �� �� ������ ������ ������������
	lds r16,RAM_ACTIVE_BUTTON ; ��������� �� ���, ����� ������ ������ �������
	cpi r16,RAM_BUTTON_PROG ; ����������
	breq step_prog_next1 ; ���� ��� ������ ������ �������, �� ��� ������������ ������
	ret ; ����� ��� �� �� ������, ������� ������ ��������������, �������

// ���� ��������� � �������� ��������� ������ ������ ��� ��� �������� �������� ��������� ������
step_prog_next1:
	in r16,PINA ; ��������� ��������� ����� ����� 
	com r16 ; ����������� ��������� �����. ������ ��� �������, ���� �������� ������� ������� ������ = 0.
	BST r16,PA0 ; ���������� �� ���� � �������� ����
	SBRC r17,0 ; ���� � ���� 1, �� ��������� ������ �������
	rjmp step_prog_next3 ; ���� ��� ���������� � �������� ��������� ������ ������ 
	BRTC step_prog_next2 ; ���� ��� �� ���������� � �������� ��������� ������ � ���� �����-�� ������ ���� ������
	ret ; ���� ������ �� ���������� � �� ���� �������� �������� ���������, �� ������ �������

// ��� ��� ������ �������� �������, �� ��������� � ��� ���������� � ���, ����� ������ ������, ����� ������������ ������ �
step_prog_next2:
    ldi r16,RAM_BUTTON_PROG
	sts RAM_ACTIVE_BUTTON,r16
step_prog_next3:
	rcall button_press ; �������� ���������� ������� ������, � ������� �-��� ���������� ������ 100 ����
	push r17 ; ��������� ������� ������ ��� ������� � ������ �������
	push r18
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ��������
	rcall Button_prog_func ; ������� �-���, ������������ �� ������
	pop r18
	pop r17 ; ���������� r17
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ���������� ������� ������
	CBR r17,8 ; �������� ���� ������ �������, ����� ������ � �� ��������
ret









// ���������� ������ �����������
Button_step: nop
	SBRS r17,0 ; ���������� ��������� �������, ���� ��� ��������� � �������� ��������� �����-�� ������
	rjmp step_step_next1 ; ��������� �����
	//���� ��� ��������� � �������� ��������� ������, �� ���������, �� �� ������ ������ ������������
	lds r16,RAM_ACTIVE_BUTTON ; ��������� �� ���, ����� ������ ������ �������
	cpi r16,RAM_BUTTON_STEP ; ����������
	breq step_step_next1 ; ���� ��� ������ ������ �������, �� ��� ������������ ������
	ret ; ����� ��� �� �� ������, ������� ������ ��������������, �������

// ���� ��������� � �������� ��������� ������ ������ ��� ��� �������� �������� ��������� ������
step_step_next1:
	in r16,PINA ; ��������� ��������� ����� ����� 
	com r16 ; ����������� ��������� �����. ������ ��� �������, ���� �������� ������� ������� ������ = 0.
	BST r16,PA2 ; ���������� �� ���� � �������� ����
	SBRC r17,0 ; ���� � ���� 1, �� ��������� ������ �������
	rjmp step_step_next3 ; ���� ��� ���������� � �������� ��������� ������ ������ 
	BRTC step_step_next2 ; ���� ��� �� ���������� � �������� ��������� ������ � ���� �����-�� ������ ���� ������
	ret ; ���� ������ �� ���������� � �� ���� �������� �������� ���������, �� ������ �������

// ��� ��� ������ �������� �������, �� ��������� � ��� ���������� � ���, ����� ������ ������, ����� ������������ ������ �
step_step_next2:
    ldi r16,RAM_BUTTON_STEP
	sts RAM_ACTIVE_BUTTON,r16
step_step_next3:
	rcall button_press ; �������� ���������� ������� ������, � ������� �-��� ���������� ������ 100 ����
	push r17 ; ��������� ������� ������ ��� ������� � ������ �������
	push r18
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ��������
	rcall Button_step_func ; ������� �-���, ������������ �� ������
	pop r18
	pop r17 ; ���������� r17
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ���������� ������� ������
	CBR r17,8 ; �������� ���� ������ �������, ����� ������ � �� ��������
ret



// ���������� ������ �����������
Button_block: nop
	SBRS r17,0 ; ���������� ��������� �������, ���� ��� ��������� � �������� ��������� �����-�� ������
	rjmp block_block_next1 ; ��������� �����
	//���� ��� ��������� � �������� ��������� ������, �� ���������, �� �� ������ ������ ������������
	lds r16,RAM_ACTIVE_BUTTON ; ��������� �� ���, ����� ������ ������ �������
	cpi r16,RAM_BUTTON_BLOCK ; ����������
	breq block_block_next1 ; ���� ��� ������ ������ �������, �� ��� ������������ ������
	ret ; ����� ��� �� �� ������, ������� ������ ��������������, �������

// ���� ��������� � �������� ��������� ������ ������ ��� ��� �������� �������� ��������� ������
block_block_next1:
	in r16,PINB ; ��������� ��������� ����� ����� 
	com r16 ; ����������� ��������� �����. ������ ��� �������, ���� �������� ������� ������� ������ = 0.
	BST r16,PB1 ; ���������� �� ���� � �������� ����
	SBRC r17,0 ; ���� � ���� 1, �� ��������� ������ �������
	rjmp block_block_next3 ; ���� ��� ���������� � �������� ��������� ������ ������ 
	BRTC block_block_next2 ; ���� ��� �� ���������� � �������� ��������� ������ � ���� �����-�� ������ ���� ������
	ret ; ���� ������ �� ���������� � �� ���� �������� �������� ���������, �� ������ �������

// ��� ��� ������ �������� �������, �� ��������� � ��� ���������� � ���, ����� ������ ������, ����� ������������ ������ �
block_block_next2:
    ldi r16,RAM_BUTTON_BLOCK
	sts RAM_ACTIVE_BUTTON,r16
block_block_next3:
	rcall button_press ; �������� ���������� ������� ������, � ������� �-��� ���������� ������ 100 ����
	push r17 ; ��������� ������� ������ ��� ������� � ������ �������
	push r18
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ��������
	rcall Button_block_func ; ������� �-���, ������������ �� ������
	pop r18
	pop r17 ; ���������� r17
	SBRC r17,3 ; ���� ���� ���������� ������ �������, ������������ �� ������, �� ����������, �� �� ���������� ������� ������
	CBR r17,8 ; �������� ���� ������ �������, ����� ������ � �� ��������
ret







EEPROM_write: nop
	cli ; ��������� ����������, �.�. �� ����� �������� ���������� ������ ��������� ���������� ����� ��������� �������� r16,r17,r18

	// ��� ���� ���������� ���������� ������
	sbic EECR,EEWE
	rjmp EEPROM_write
	// ������������� ����� (r18:r17) � �������� ������
	out EEARH, r18
	out EEARL, r17
	// ���������� ���� (r16) � ������� ������, ������� ����� ��������
	out EEDR,r16
	// ���������� ���������� ������� � EEMWE
	sbi EECR,EEMWE
	// �������� ������ � EEPROM ������������� ���� EEWE � �������� EECR
	sbi EECR,EEWE

	sei
ret


EEPROM_read:
	// ��� ���� ���������� ���������� ������
	cli

	sbic EECR,EEWE
	rjmp EEPROM_read
	// ������������� ����� (r18:r17) � �������� ������
	out EEARH, r18
	out EEARL, r17
	// �������� ������ �� EEPROM ������������� ���� EERE � �������� EECR
	sbi EECR,EERE
	// ��������� ���� �� �������� ������
	in r16,EEDR

	sei
ret




// �-��� ������������ ���� ����. r19:r18 * r17:r16. ���-� � r7:r6:r5:r4
umnoj_mul_2x2:
clr r7 ; ������� ������� ��� ������� ���� ���-���� 
clr r6 ; ������� ������� ��� 3-� ���� ���-���� 
clr r5 ; ������� ������� ��� 2-� ���� ���-���� 
clr r4 ; ������� ������� ��� ������� ���� ���-���� 
ldi r20,0x00 ; 

mul r18,r16 ; �������� ������� ���� ��������� �� ������� ���� ���������
add r4,r0 ; 
adc r5,r1
adc r6,r20
adc r7,r20
mul r18,r17 ; �������� ������� ���� ��������� �� ������� ���� ���������
add r5,r0
adc r6,r1
adc r7,r20
mul r19,r16 ; �������� ������� ���� ��������� �� ������� ���� ���������
add r5,r0
adc r6,r1
adc r7,r20
mul r19,r17 ; �������� ������� ���� ��������� �� ������� ���� ���������
add r6,r0
adc r7,r1
ret


// ��� ������ HOME. ������������ ������� �������� ��
fast_speed:
// ���������� � OCR1A ������� �������� �������� ��� HOME
	ldi r16,HOME_SHAGOVIK_freq_maxL
	ldi r17,HOME_SHAGOVIK_freq_maxH
	rcall TIM16_WriteOCR1A ; ���������� � OCR1A r17:r16

	// �������� TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; ���������� � TCNT1 r17:r16
ret

// ��� ������ HOME. ������������ �������� �������� ��
low_speed:
// ���������� � OCR1A ������� ���������� �������� ��� HOME
	ldi r16,HOME_SHAGOVIK_freq_minL
	ldi r17,HOME_SHAGOVIK_freq_minH
	rcall TIM16_WriteOCR1A ; ���������� � OCR1A r17:r16

	// �������� TCNT1
	ldi r16,0x00
	ldi r17,0x00
	rcall TIM16_WriteTCNT1 ; ���������� � TCNT1 r17:r16
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







	// ���������� ����� ��������� ������� � ���
	/*ldi r19,0x64
	ldi r17,0x00
	otladka1:
	clr r18
	ldi r16,0x00 ; 3-� �������
	rcall EEPROM_write
	inc r17
	dec r19
	brne otladka1
	*/
	/*
	ldi r16,0x01
	sts RAM_position_number,r16

	// ���������� ��������� � ������ ������� � ��� 263 (26.3)
	ldi r17,0x02
	mul r17,r16
	ldi r17,0x0A
	add r17,r0
	clr r18
	ldi r16,0x01 ; ������� ������� ���� ��������� �
	rcall EEPROM_write
	ldi r16,0x01
	sts SHAGOVIK_X2_HIGH_RAM ,r16

	inc r17
	ldi r16,0x07 ; ������� ���� ��������� �
	rcall EEPROM_write
	ldi r16,0x07
	sts SHAGOVIK_X2_LOW_RAM ,r16

	ldi r16,0x0A
	sts RAM_step_size,r16
	*/
	
ret

// �������� �����, ���� �������� �� �2
interrupt_process:
	lds r16,SHAGOVIK_timer1_on ; ������� ������� �� ������ 1 (��������� �� ��)
	SBRS r16,0 ; ���� ��� 0 ����������, �� ������� � ����, ���� ������ 1 �� ����������
	ret ; ����� ������� �� �-���
	// �������� timer1
	ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; ������������ 1
	out TCCR1B,r16
	// ��������� ������ 0
	in r16,TIMSK
	andi r16,0b11111101  
	out TIMSK,r16 
wait_untill_timer1_stop:
	//rcall lcd_printing_number
	rcall encoder_print_num ; ������� �� ����� �������� ������� �2, ���� �������� �������
	lds r16,SHAGOVIK_timer1_on ; ������� ������� �� ������ 1 (��������� �� ��)
	SBRC r16,0 ; 
	rjmp wait_untill_timer1_stop
	ldi r17,0x00
ret



home_and_go_to_last_position:
	// �������, ���������� �� ��������� � ���������� ��������������� ��������� ���� ������� ��� ��������� ������
	// ��������� ������ ��������������� ��������� �������
	ldi r18,0x00
	ldi r17,0x02 ; EEPROM ����� 0x0002
	rcall EEPROM_read
	// ������� ��������� ����� �������� �������������� ��������� ���� ������� ��� ��� (��������� ������ �� ������ HOME)
	SBIS PINA,1 ; ���� ������ HOME ������, �� ���������� ��������� �������
	rjmp hag_1

	SBIC PINB,1 ; ���� ������ BLOCK ������, �� ��������� �� hag_1
	rjmp hag_1

	SBIC PINB,2 ; ���� ������ OK ������, �� ��������� �� hag_1
	rjmp hag_1


	// ���� ������ ��������� �������, �� ��������� ����� �� �������� ������� ���. ���. ���� � ���� ���, �� ���������� 0xff ����� 0x00
	cpi r16,0x00
	breq set0xff
	// ���� ������ 0
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
	ldi r16,0xFF ; ����������� ��������� �������� �� EEPROM
save_auto_to_eeprom:
	ldi r18,0x00
	ldi r17,0x02 ; EEPROM ����� 0x0002
	rcall EEPROM_write ; ���������� ����� ������ ��������������� ���������
	cpi r16,0x00 ; ���� ������ ���. ���. �-��� ����� 0, �� �� �������� ������� �������������
	breq auto_on ; ���� ������ ��� ����� 0xFF, �� ������������� ��������
	wait_press_of_ok1:
	SBIC PINA,1
	rjmp wait_press_of_ok1
	//CBI PORTC,LED_ok ; ����� ��������� ������ ��
	rjmp ext99
// ��������� �������� �� �������������� ��������� ������� ��� ��������� ������
hag_1:
	cpi r16,0x00 ; ���� ������ ���. ���. �-��� ����� 0, �� �� �������� ������� �������������
	//brne auto_on ; ���� ������ ��� ����� 0xFF, �� ������������� ��������
	breq auto_on ; ���� ������ ��� ����� 0xFF, �� ������������� ��������

	SBIC PINB,1 ; ���� ������ BLOCK ������, �� ��������� �� ext99
	rjmp ext99

	SBIC PINB,2 ; ���� ������ OK ������, �� ��������� �� ext99
	rjmp ext99
	// ����� ���, ���� ������� ������ ��
	// �������� ��������� ������ ��
	//SBI PORTC,LED_ok
wait_press_of_ok:
	SBIC PINA,1
	rjmp wait_press_of_ok
	//CBI PORTC,LED_ok ; ����� ��������� ������ ��
	rjmp ext99
auto_on:
	ldi r16,0x00
	sts INIT_FLAG,r16 ; ����������, ��� ���� �� ������� ������, � ��������� 
	ldi r16,RAM_BUTTON_HOME
	sts RAM_ACTIVE_BUTTON,r16 ; ����������, ��� ������������ ������ HOME
	rcall Button_home_func ; ������������ � ������� ���������
	rcall interrupt_process ; �������� �����, ���� �� ��������� �������� ���������
	rcall get_position_value_from_eeprom ; ��������� ���������, � ������� ���� ������ 
	ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16 ; ����������, ��� ������������ ������ OK
	rcall Button_ok_func ; ���� � ���������, ������� ���� � ��������� ���
	rcall interrupt_process ; �������� �����, ���� �� ��������� ������� ���������
	ldi r16,0x01
	sts INIT_FLAG,r16 ; ����������, ��� ��������� ���� �����������, ����� ��� ������� �� ������ HOME, � ����� ������������ ����� �������� ��������� �� �����
ext99:
    ldi r16,0x01
	sts INIT_FLAG,r16 ; ����������, ��� ��������� ���� �����������, ����� ��� ������� �� ������ HOME, � ����� ������������ ����� �������� ��������� �� �����
ret



save_many_value_to_eeprom:
	ldi r16,0x00 ; ������������ ��������
	ldi r18,0x00 ; ������� ���� ������ � ���
	ldi r17,0x0A ; ������� ���� ������ � ���
	ldi r19,0x64 ; ������� �������� ������ ���� �������� � ���
	save_many_cycle:
	rcall EEPROM_Write
	ldi r20,0x01
	add r17,r20 ; �������� ��������� �����, � ������� ���� �������� ������
	
	dec r19
	brne save_many_cycle ; ���� ���� �� �������� r19 ��������, �� ���������� ���������� � ���
	ldi r16,0x00 ; ������������ ��������
	ldi r18,0x00 ; ������� ���� ������ � ���
	ldi r17,0x00 ; ������� ���� ������ � ���
	rcall EEPROM_Write
ret

/*
lcd_printing_number:
	cpi r21,SHAGOVIK_DELTA_L
	brne print_exit
	// ������� �� ������� ����� ��������� �
	rcall push_group_registers
	push r25
	push r24
	ldi r16,0x40
	rcall Set_cursor
	pop r24
	pop r25
	// ��������� �������� �������� ��������� �
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; ������� �� ������� 
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
	// ��������� SREG, ����� �� ��������� ����� � ������ ���������� breq, brne � �.�. � ������� ��������. � ����� ������ ������� ���������� �������, ��� � push � pop
	in r16,SREG
	sts SREG_temp,r16 

	//rcall millis // ���������� � �������� �����������
	

	clr r16
	out TCNT2,r16
	// ��������� �������� �������� �� ���
	lds r18,TIM2_counter_high
	lds r17,TIM2_counter_low
	ldi r16,0x01

	// ���������� �������
	add r17,r16
	clr r16
	adc r18,r16

	// ��������� ������ �� � �������� �� 0 �� 520
	ldi r16,0xF5
	CPSE r17,r16
	rjmp save_counter
	ldi r16,0x01
	CPSE r18,r16
	rjmp save_counter
// ���� ��� 521, �� ��������
obnulenie_counter:
	clr r16
	sts TIM2_counter_high,r16
	sts TIM2_counter_low,r16
	rjmp tim2_ext
// ������ ���������
save_counter:
	sts TIM2_counter_high,r18
	sts TIM2_counter_low,r17

// �����
tim2_ext:
	pop r20
	pop r19
	pop r18
	pop r17
	// ���������� SREG, ����� �� ��������� ����� � ������ ���������� breq, brne � �.�. � ������� ��������. � ����� ������ ������� ���������� �������, ��� � push � pop
	lds r16,SREG_temp
	out SREG,r16
	pop r16 ; ���������� ����������� (��� �-��� interrupt_process)
reti




// ���������� ������������ �������� ��������
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

	

	// ��������� SREG, ����� �� ��������� ����� � ������ ���������� breq, brne � �.�. � ������� ��������. � ����� ������ ������� ���������� �������, ��� � push � pop
	in r16,SREG
	sts SREG_temp,r16 

	lds r16,PROG_FUNC_WORK_STATUS_RAM
	cpi r16,0x00
	breq ext_int0_exit

	rcall Clock_start_for_block // �������� ������, ������� ������� ���������� ����� 10 ������

SBIS PINA,B1 ; ���������� ��������� �������, ���� 1 �� ����� B1 (���� �������� �����)
rjmp rotate_forward ; ���� �� ����� B1 0, �� ��� �������� �����

rotate_reverse:
/*�������� �� ������� ��� � ������� �� �������*/
// ��������� �������
lds r26,SHAGOVIK_X_TRULY_HIGH_RAM  ; ��������� ������� ���� ����������� ��������� �� ���
lds r25,SHAGOVIK_X_TRULY_LOW_RAM   ; ��������� ������� ���� ����������� ��������� �� ���
//��������� ���
clr r24 ; ������� ������� ���� �����������
lds r23,RAM_step_size ; ��������� ��� (������� ���� �����������) 
// �������� ��� (����������) �� ������� (������������), �������� ������������ � �������
rcall subtraction_2x2 ; �������� ������� ��������� (r26-r25 - r24-r23)
cpi r26,0xFF    ; ���� � ������� ����� ������� ��������� ����� ����� 65000, �� �������� ���� ��������� ������,
breq obnulenie	; ������� ������������ �������� ������� � ����, �.�. ������������� ������� ���

rjmp save_position ; ��������� �������� ������� � ���
obnulenie:
clr r25
clr r26
rjmp save_position ; ��������� �������� ������� � ���

rotate_forward:
/*���������� � ������� ��� � ������� �� �������*/
// ��������� �������
lds r26,SHAGOVIK_X_TRULY_HIGH_RAM  ; ��������� ������� ���� ����������� ��������� �� ���
lds r25,SHAGOVIK_X_TRULY_LOW_RAM   ; ��������� ������� ���� ����������� ��������� �� ���
//��������� ���
clr r24 ; ������� ������� ���� 
lds r23,RAM_step_size ; ��������� ��� (������� ���� �����������) 
// ���������� ��� (������ ���������) � ������� (������ ���������), ����� ������������ � �������
rcall addition_2x2 ; �������� ������� �������� (r26-r25 + r24-r23)
// ���� � ������� ����� ������� ��������� ����� ������ 700 (70.0 ��) (0x02BC)
cpi r26,0x02 ; ���������� ������� ���� �������
BRSH priravnenie_max ; ���� ������� ���� ������� �������� ������ 0x03 ��� ����� ��� 
cpi r25,0xF5 ; ���� ������� ���� �������� ��-���� ������, �� ���������� ������ ������� �����
BRLO save_position ; ���� �� ������� ����� ��������� ������ 0xBD, �� ��������� � ����������
cpi r26,0x01 ; ���� ������� ���� ������� ��� ������� ������� ����� ����� 0x02, �� ����� ��������� ������ 700
breq priravnenie_max
; ����� ������� ��������� ������ 700 (70.0), ����� ��������� ��� ���������
rjmp save_position
priravnenie_max:
ldi r26,0x01
ldi r25,0xF4 ; ������ ������� �������� 700 (70.0)

save_position:
sts SHAGOVIK_X2_HIGH_RAM,r26 ; ���������� � ��� ������� ���� �������
sts SHAGOVIK_X2_LOW_RAM,r25  ; ���������� � ��� ������� ���� �������

sts SHAGOVIK_X_TRULY_HIGH_RAM ,r26 ; ���������� � ��� ������� ���� �������
sts SHAGOVIK_X_TRULY_LOW_RAM,r25  ; ���������� � ��� ������� ���� �������

/*
ldi r16,0x40
rcall Set_cursor ; ������ ������ �� ����� ����� �������
rcall lcd_print_number ; ������� �� ����� [r26][r25] (�������) � ���������� �������
*/
rcall controlling_motor

ext_int0_exit:
	// ���������� SREG, ����� �� ��������� ����� � ������ ���������� breq, brne � �.�. � ������� ��������. � ����� ������ ������� ���������� �������, ��� � push � pop
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

// ������� �������� �� ������������ �������� ��������� ��� �������� ��������
controlling_motor:
	
	// ��������� ����, ����� ������� ����� �� ����� � ����� ������ ������ ��
	ldi r16,0x01
	sts ENCODER_CONTR_MOT_PRESS_OK,r16 
	sts ENCODER_CONTR_MOT_PRINT_X2,r16
	sts ENCODER_CONTR_MOT_ENCODER_PROCESSING,r16 ; ����������, ��� ������ ���������� �� ���� ��������
ret

// ������� ��������� ���� �� �������� �������� � ���� ����, �� �������� ������� ������ ��
check_encoder_press:
	lds r16,ENCODER_CONTR_MOT_PRESS_OK
	cpi r16,0x01
	brne c_e_p_exit
	ldi r16,0x01
    sts ENCODER_CONTR_MOT_ENCODER_PROCESSING,r16 ; ����������, ��� ������ ���������� �� ���� ��������
	ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16 ; ����������, ��� ����� ������ ������ �� - ����������, ����� ����� ������� � ������� ����������, �.�. ��� �������� �� ������
	rcall Button_ok_func    // �������� ������� ������ ��
	
	// ������������� ���� ������ ������� 3 ������, ����� �� ��������� ��������� ������� ������� � � EEPROM
	ldi r16,0x01
	sts ENCODER_CONTR_MOT_SAVE_X,r16
	clr r16
	sts ENCODER_CONTR_MOT_PRESS_OK,r16 // ����������, ��� ������ ������
c_e_p_exit: ; �����
	
ret

encoder_print_num:
	//cli
	lds r16,ENCODER_CONTR_MOT_PRINT_X2 ; ��������� ����� �� ������� �� ����� ����� (���� �������� �������)
	cpi r16,0x01
	brne exit001 ; ���� ���� �� ����� �������, �� ������ �����
	//sei
	// ����� ������� �� ����� �����
	ldi r16,0x40
	rcall Set_cursor ; ������ ������ �� ����� ����� �������
	
	lds r26,SHAGOVIK_X2_HIGH_RAM ; ��������� �� ��� ������� ���� ������� �2
	lds r25,SHAGOVIK_X2_LOW_RAM  ; ��������� �� ��� ������� ���� ������� �2

	rcall lcd_print_number ; ������� �� ����� [r26][r25] (�������) � ���������� �������
	clr r16
	sts ENCODER_CONTR_MOT_PRINT_X2,r16 ; �� ���������
exit001:
//sei
ret

// ������� ��������� ������� ������� � EEPROM ��� �������� �������� ������ ��� ������� ����� ����, ��� ������������ ���������. ���� � ������� ��� ������
// �������� ����� �������, �� ���������� ����� ������ �������. �-��� ����� ��� ������������ ������������� ������ EEPROM
save_position_X_after_3_sec:
	push r18
	push r19
	push r20
	// ���������� � �������� ������� �������
	lds r20,ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH
	lds r19,ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID
	lds r18,ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW
	// ��������� ���� ������ ��������
	lds r16,ENCODER_CONTR_MOT_SAVE_X

	// ��������� ����.
	cpi r16,0x01
	brne spx_next1
flag_1: // ���� ���� ����� 1
	// ���������� ������� ������� � �������
	ldi r18,0x01
	clr r19
	clr r20
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH,r20
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID,r19
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW,r18
	clr r16
	sts ENCODER_CONTR_MOT_SAVE_X,r16 ; ���������� ����
spx_next1: // ���� ���� ����� 0
	// ��������� ���� ����� ������� �������. ���� ������� ����� 0, �� ������ �����
	clr r16
	CPSE r18,r16
	rjmp spx_next2 ; �����
	CPSE r19,r16
	rjmp spx_next2 ; �����
	CPSE r20,r16
	rjmp spx_next2 ; �����
	rjmp spx_exit
spx_next2: // ���� ������� �� ����� 0
	// ���������� 1 � ��������
	ldi r16,0x01
	add r18,r16
	clr r16
	adc r19,r16
	adc r20,r16
	// �������� ������� � ������� ������ (������ �� ������������� ���������� �������)
	ldi r16,ENCODER_CONTR_MOT_SAVE_X_TIME_LOW
	CPSE r18,r16
	rjmp spx_next3
	ldi r16,ENCODER_CONTR_MOT_SAVE_X_TIME_MID
	CPSE r19,r16
	rjmp spx_next3
	ldi r16,ENCODER_CONTR_MOT_SAVE_X_TIME_HIGH
	CPSE r20,r16
	rjmp spx_next3
final_time1: // ���� ������� ����� �������� ����� (������ ������������� �����)
	// ��������� ������� ���� ��������� � ������� ���������
	lds r16,SHAGOVIK_X0_LOW_RAM
	// ��������� ��� � EEPROM � ������� 0x0003

	cli

	ldi r17,0x03
	ldi r18,0x00
	rcall EEPROM_write

	// ��������� ������� ���� ��������� � ������� ���������
	lds r16,SHAGOVIK_X0_HIGH_RAM
	// ��������� ��� � EEPROM � ������� 0x0004
	ldi r17,0x04
	ldi r18,0x00
	rcall EEPROM_write

	sei

	// ������ �� �� ���������
	clr r18
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH,r18
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID,r18
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW,r18
	rjmp spx_exit
spx_next3: // ���� ������� �� ����� �������� ����� (�� ������ ������������� �����)
	// ������ ��������� �������
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_HIGH,r20
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_MID,r19
	sts ENCODER_CONTR_MOT_SAVE_X_COUNTER_LOW,r18
spx_exit:
pop r20
pop r19
pop r18
ret














// ���������� ��������� ��������� ������������ �������� ���������
TIM1_COMPA: 
	SBI PORTA,STEP ; ������������� 1 �� PA5(STEP)
	nop ; ��� TMC2100 ������� ����� 85 ��
	nop ; ��� 65*2=130��
	CBI PORTA,STEP ; ������������� 0 �� PA5(STEP)

	rcall Clock_start_for_block // ������������ ������ ���������� � ������ �� ������ �������� ����� 10 ������

	//rcall push_group_registers
	push r16 // ��������� ����������� (��� ������� interrupt_process)

	// ��������� SREG, ����� �� ��������� ����� � ������ ���������� breq, brne � �.�. � ������� ��������. � ����� ������ ������� ���������� �������, ��� � push � pop
	in r16,SREG
	sts SREG_temp,r16 
	// ���������, ����� �� ��������� ��� ������ �������
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

	// ��������� �� ��� �������� ��� ������������ ��������
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
	// ��������� ������ �� ��� HOME
	lds r16,RAM_ACTIVE_BUTTON
	SBRS r16,0 ; ���� ��� 0 �� ����������, ��� ������� � ���, ��� ��� �� ������ ��, �� ��������� �� �����, ����� ���� ������ ������ ��
	rjmp home_func ; ���� ������ �� ��, �� ���� ������, ����� ��������� �� �����


	
	

// �������� ���� 0,1 ��
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

	//dec r21 ; ������ ��� 1/16 ����
	//breq obrabotka ; ���� ������ ���� 0,1 ��, �� ������������: ����������� ���, ������� �� ������� ������� ��������� � �.�...
	//rjmp exit1 ; ����� �������� ����������

// ����������� ���, ������� �� ������� ������� ��������� � �.�...
obrabotka:
	ldi r21,SHAGOVIK_DELTA_L_LOW
	ldi r22,SHAGOVIK_DELTA_L_HIGH
	; � r31:30 �������� ���������, ���� ������ ��������, � r29:r28 ������� ���������, � r27:r26 ��������� ��������� � � r25:r24 ������� ���������
	; � r22 �������� ���������������� ����������� ����
	SBIS PORTA,DIR ; ��������� ����������� ��������
	rjmp nazad ; ���� �� DIR ����, �� �������� �����

// ����� �������� �����
vpered:
	// ���������� � �������� ��������� 0,1 �� (0x01), ��� ����� ������� ����� ������� ���������
	ldi r16,0x01
	clr r17
	add r24,r16
	adc r25,r17
	// ����� ���, ������� ���������, �������� �� �� ����� ����, � ���� ��, �� ������������
	CPSE r30,r24 ; ���� �����, ���������� ����. �������
	rjmp next13 ; ���� �� �����, ��� ������
	CPSE r31,r25 ; ���� �����, ���������� ����. �������
	rjmp next13 ; ���� �� �����, ��� ������
	rjmp THE_END1 ; ����� ������������

	next13:
	// ������� ����������, ������� ��� ����������
	mov r16,r24
	mov r17,r25 ; ���������� ����� ������� ���������, ����� �� ��������

	// ��������� ����� ����� ���� ��������
	ldi r18,0x00
	CPSE r29,r18
	rjmp chast1
	CPSE r28,r18
	rjmp chast1
// ���� �������� 2 ����� ����
chast2:
	// �������� �� �������� ��������� �2 ������� ��������� X
	push r30
	push r31
	sub r30,r16
	sbc r31,r17
	mov r16,r30
	mov r17,r31
	pop r31
	pop r30
	rjmp pid1
// ���� �������� 1 ����� ����
chast1:
	// �������� �� ������ �������� ��������� � ��������� ��������� �0
	sub r16,r26
	sbc r17,r27

	rjmp pid1 ; ��������� ������, ��� ����������� �������� ��� �� ������ ����


// � ������ �������� �����
nazad:
	// �������� �� �������� ��������� 0,1 �� (0x01), ��� ����� ������� ����� ������� ���������
	ldi r16,0x01
	clr r17
	sub r24,r16
	sbc r25,r17
	// ����� ���, ������� ���������, �������� �� �� ����� ����, � ���� ��, �� ������������
	CPSE r30,r24 ; ���� �����, ���������� ����. �������
	rjmp next14 ; ���� �� �����, ��� ������
	CPSE r31,r25 ; ���� �����, ���������� ����. �������
	rjmp next13 ; ���� �� �����, ��� ������
	rjmp THE_END1 ; ����� ������������

	next14:
	// ������� ����������, ������� ��� ����������
	mov r16,r26
	mov r17,r27 ; ���������� ��������� ��������� �0, ����� �� ��������

	// ��������� ����� ����� ���� ��������
	ldi r18,0x00
	CPSE r29,r18
	rjmp chast11
	CPSE r28,r18
	rjmp chast11
// ���� �������� 2 ����� ����
chast22:
	// �������� �� �������� ��������� � ������� ��������� X2
	mov r16,r24
	mov r17,r25

	sub r16,r30
	sbc r17,r31
	rjmp pid1
// ���� �������� 1 ����� ����
chast11:
	// �������� �� ���������� ��������� �0 ����� ������� ��������� � 
	sub r16,r24
	sbc r17,r25
	// �������� �� �������� ��������� X ������� ��������� �2
	// �������� �� ���������� ��������� �0 ����� ������� ��������� � 
	

// ��������� ������ ��������� ����
pid1:
/*
	// ��������� ��������
	push r16
	push r17
	push r20
	push r21 
	push r23
	push r24
	push r25
	push r26
	push r27

	// ������� �� ������� ����� ��������� �
	ldi r16,0x40
	rcall Set_cursor
	
	// ��������� �������� �������� ��������� �
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; ������� �� ������� 

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
	// �������� ������� �� ���������������� ����������� 
	ldi r18,SHAGOVIK_pid_prop ; ���������� � r16 �����������
	clr r19 ; �������������� ��� ��������
	rcall umnoj_mul_2x2 ; ����������� r19:r18 * r17:r16. ���-� � r7:r6:r5:r4
	mov r17,r7 ; ��� ����������� �����, �� �� ������ ������ ������� ���
	mov r16,r6 ; ��� ����������� �����, �� �� ������ ������ ������� ���
	// ���������, ������ �� �������� ���� (���� ������, �� � r29:r28 ������ ���� ����)
	clr r18
	CPSE r29,r18 ; ���� ����� ����, �� ����������
	rjmp chast_puti1 ; ���� �� ����� ����, �� ���� �� ����� �� ��������
	CPSE r28,r18 ; ���� ����� ����, �� ����������
	rjmp chast_puti1 ; ���� �� ����� ����, �� ���� �� ����� �� ��������


/* �������� ���-������������� ������ ����� ����:
   OCR1A = |(X2 - X)| * Freq(min)
   ����������� ������� ��������� STEP ���������� �� ������� ����� ������� ��������� �2 � ������� ����������.
   ��� ���������� ���������� �� ��������� � ������ �����.
*/
chast_puti2:
	// ������� �� ��������� ��  �������, ���������� �� ����. ����. ������������� �������� �������� ��������� 
	cpi r17,0x00 
	brne priravn_OCR1A_min; ���� ����� ������ 65535, �� ���������� � ������������ OCR1A
	cpi r16,0x00
	brne priravn_OCR1A_min; ���� ����� ������ 65535, �� ���������� � ������������ OCR1A

	mov r17,r5 ; ������������ delta * kp � ������ ��������
	mov r16,r4
	
	ldi r19,SHAGOVIK_freq_minH ; ���������� ������������ �������� OCR1A
	ldi r18,SHAGOVIK_freq_minL

	// OCR1A(MAX) - delta * Kp
	sub r18,r16
	sbc r19,r17 
	
	BRLO priravn_OCR1A_min_2 ; ���� ���������� ������ ������������� �����, �� ������������ � ����������� � ������ ����� �������
	// ���� r19:r18 ������
	cpi r19,SHAGOVIK_freq_maxH
	BRCS priravn_OCR1A_min_2
	cpi r18,SHAGOVIK_freq_maxL
	BRCC higher2 ; 
	cpi r19,SHAGOVIK_freq_maxH
	breq priravn_OCR1A_min_2
	rjmp higher2


priravn_OCR1A_min_2:
	// ���������� � OCR1A ����������� � ������ ����� �������
	lds r16,SHAGOVIK_OCR1A_BOTTOM_LOW_RAM
	lds r17,SHAGOVIK_OCR1A_BOTTOM_HIGH_RAM
	rjmp set_OCR1A
higher2:
	mov r17,r19 ; �������� ����� �������� OCR1A
	mov r16,r18
	rjmp set_OCR1A
/*
	// ������� �� ��������� ��  �������, ���������� �� ����. ����. ������������� �������� �������� ��������� (����������� �������)
	cpi r17,0x00 
	brne priravn_OCR1A_max; ���� ����� ������ 65535, �� ���������� � ������������� OCR1A
	cpi r16,0x00
	brne priravn_OCR1A_max; ���� ����� ������ 65535, �� ���������� � ������������� OCR1A

	mov r17,r5
	mov r16,r4
	
	// ��������� �������� OCR1A, ������� ������� ������� � ������ ����� ����
	lds r19,SHAGOVIK_OCR1A_BOTTOM_HIGH_RAM
	lds r18,SHAGOVIK_OCR1A_BOTTOM_LOW_RAM
	
	// ��������� ����������� �������
	ldi r19,SHAGOVIK_freq_minH
	ldi r18,SHAGOVIK_freq_minL


	sub r18,r16
	sbc r19,r17

	BRCS priravn_OCR1A_max // ���� �������� ������������, �� ����� ��������� ������ 65535, ������������ � ������� �������

	// ���� r17:r16 ������
	cpi r17,SHAGOVIK_freq_minH
	BRCS lower1
	cpi r16,SHAGOVIK_freq_minL
	BRCC priravn_OCR1A_max ; 
	cpi r17,SHAGOVIK_freq_minH
	breq lower1

priravn_OCR1A_max:
	// ���������� � OCR1A. ����������� �������
	ldi r16,SHAGOVIK_freq_minL
	ldi r17,SHAGOVIK_freq_minH
	rjmp set_OCR1A
lower1:
	rjmp set_OCR1A
	*/



// ���� ���� �� ����� �� ��������. ��� ����� ������ ������� ������������ OCR1A: OCR1A = OCR1A(MAX) - delta * Kp
// ��� OCR1A(MAX) - ������� ������� �������� �������� ���������, ������� ������� � SHAGOVIK_freq_min;
// delta - �������, ������� ������ �� ���������� ��������� �� �������� ����
chast_puti1:
	// ������� �� ��������� ��  �������, ���������� �� ����. ����. ������������� �������� �������� ��������� 
	cpi r17,0x00 
	brne priravn_OCR1A_min; ���� ����� ������ 65535, �� ���������� � ������������ OCR1A
	cpi r16,0x00
	brne priravn_OCR1A_min; ���� ����� ������ 65535, �� ���������� � ������������ OCR1A

	mov r17,r5 ; ������������ delta * kp � ������ ��������
	mov r16,r4
	
	ldi r19,SHAGOVIK_freq_minH ; ���������� ������������ �������� OCR1A
	ldi r18,SHAGOVIK_freq_minL

	// OCR1A(MAX) - delta * Kp
	sub r18,r16
	sbc r19,r17 

	BRLO priravn_OCR1A_min ; ���� ���������� ������ ������������� �����, �� ������������ � �������� 
	// ���� r19:r18 ������
	cpi r19,SHAGOVIK_freq_maxH
	BRCS priravn_OCR1A_min
	cpi r18,SHAGOVIK_freq_maxL
	BRCC higher1 ; 
	cpi r19,SHAGOVIK_freq_maxH
	breq priravn_OCR1A_min
	rjmp higher1


priravn_OCR1A_min:
	// ���������� � OCR1A. ������������ �������
	ldi r16,SHAGOVIK_freq_maxL
	ldi r17,SHAGOVIK_freq_maxH
	rjmp check_half_way ; ���������� ������� ��������� �� ������� ����������
higher1:
	mov r17,r19 ; �������� ����� �������� OCR1A
	mov r16,r18
// ���������� ������� ��������� �� ������� ����������
check_half_way:
	CPSE r24,r28
	rjmp set_OCR1A
	CPSE r25,r29
	rjmp set_OCR1A
	// ���� �� �������� �������� ���������, ��
	/*
	mov r27,r29
	mov r26,r28 ; X1 ���������� � �0
	*/
	mov r27,r31
	mov r26,r30 ; X2 ���������� � �0
	clr r29
	clr r28 ; �������� �1, ��� �������, ��� �� ������ ������ ����� ����
	
	// ���������� � ��� �������� OCR1A, �������� �������� ������� � ������ ����� ����
	sts SHAGOVIK_OCR1A_BOTTOM_HIGH_RAM,r17
	sts SHAGOVIK_OCR1A_BOTTOM_LOW_RAM,r16
set_OCR1A:
	rcall TIM16_WriteOCR1A ; ���������� � OCR1A r17:r16
	rjmp exit_but_ok ; ������� �� ����������
THE_END1:
	
	CBI PORTC,7 ; ��������� ��������� ������ ��
	ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (0<<CS10) ; 
	out TCCR1B,r16 ; ��������� ������

	// ���������� � ��� ���������, � ������� ������
	sts SHAGOVIK_X0_HIGH_RAM,r25
	sts SHAGOVIK_X0_LOW_RAM,r24
	sts SHAGOVIK_X_TRULY_HIGH_RAM,r25 ; ��������� ���������� ��������� � ���
	sts SHAGOVIK_X_TRULY_LOW_RAM,r24 ; ��������� ���������� ��������� � ���
	// ������� �� ������� ����� ��������� �
	// ��������� � EEPROM ������� ���������
	ldi r16,0x01
	sts ENCODER_CONTR_MOT_SAVE_X,r16
	// ���� �������� �� ������ �������, � �� ������� ������ ��
	lds r16,ENCODER_CONTR_MOT_ENCODER_PROCESSING
	cpi r16,0x01

	breq if_encoder // �� �� ������� ����� ��������� �
	ldi r16,0x40
	rcall Set_cursor
	
	// ��������� �������� �������� ��������� �
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; ������� �� ������� 

if_encoder:
	ldi r16,0x00
    sts ENCODER_CONTR_MOT_ENCODER_PROCESSING,r16 ; �� ���������

	ldi r16,0b01000000
	out GICR,r16 ; �������� �������

	in r16,TIMSK
	ori r16,0b00000010  
	out TIMSK,r16 ; �������� ������ 0

	ldi r16,0x00 
	sts SHAGOVIK_timer1_on,r16 ; ����������, ��� timer1 ��������
	rjmp exit_but_ok ; ������� �� ����������

// ���� � ������� ���������
home_func:
	// �������� ���� 0,1 ��
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
	/* ����� (stage1):
	   0 - �� ���������� ��� �������, ������� ���� ����� 2 �� �� ��������� ��������, � ����� ����� ��������� � 4 �����
	   1 - �� ���������� �� ��� �������, ������� ���� ����� �� ������� ��������, ���� �� �� ������ ��������
	   2 - ����� �� ���� �������, ���� ����� 1 �� �� ��������� ��������
	   3 - ���� �����, ���� �� �� ������ ��������
	   4 - �� ���� ��������, ���� ����� ����� 1 �� � ����
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

// �� ���������� ��� �������, ������� ���� ����� 2 �� �� ��������� ��������, � ����� ����� ��������� � 4 �����
stage0:
	// �������� ����� ���� �� ������ 2 ��
	// �������� ������� � ���������� � ����
	ldi r16,0x01
	sub r21,r16
	clr r16
	sbc r22,r16

	cpi r21,0x00
	brne zapasnoi_exit
	cpi r22,0x00
	brne zapasnoi_exit ; ���� r22:r21 ����� �� ����� ����, �� �����

	// ���� ������ 2 ��, ��
	ldi r26,0x03 ; ��������� �� ����� 3 �� ��������� ��������
	CBI PORTA,DIR ; ������ ����������� �������� �� �����
	// ����� ��������� ��� ����� 4
	//ldi r22,HOME_SHAGOVIK_DELTA_HIGH_4 
	//ldi r21,HOME_SHAGOVIK_DELTA_LOW_4  
rjmp exit1

// �� ���������� �� ��� �������, ������� ���� ����� �� ������� ��������, ���� �� �� ������ ��������
stage1:
	// ������� �� ���� �� ������� ��
	in r16,PINA
	SBRS r16,INDSIGNAL 
	rjmp exit1
	// ���� �� ���� �������, �� ��������� � ����� 2
	//rcall low_speed ; ������ ��������� ��������
	ldi r26,0x02 ; ��������� �� ����� 2 � ������� ���������
	SBI PORTA,DIR ; ������ ����������� �������� �� �����
	// ����� ��������� ��� ����� 2
	ldi r22,HOME_SHAGOVIK_DELTA_HIGH_2 
	ldi r21,HOME_SHAGOVIK_DELTA_LOW_2 
	ldi r16,0x64
	rcall nopdelay_1ms
rjmp exit1

// ����� �� ���� �������, ���� ����� 1 �� �� ������� ��������
stage2:
	// �������� ����� ���� �� ������ 1 ��
	// �������� ������� � ���������� � ����
	ldi r16,0x01
	sub r21,r16
	clr r16
	sbc r22,r16

	clr r16
	cpi r21,0x00
	brne zapasnoi_exit
	cpi r22,0x00
	brne zapasnoi_exit ; ���� r22:r21 ����� �� ����� ����, �� �����
	// ���� ������ 1 ��, ��
	rcall low_speed ; ������ ��������� �������� ������������
	ldi r26,0x03 ; ��������� �� ����� 3 �� ��������� ��������
	CBI PORTA,DIR ; ������ ����������� �������� �� �����
rjmp exit1

// ���� �����, ���� �� �� ������ ��������
stage3:
	// ������� �� ���� �� ������� ��
	in r16,PINA
	SBRS r16,INDSIGNAL 
	rjmp exit1
	// ���� �� ���� �������, �� ��������� � ����� 4
	ldi r26,0x04 ; ��������� �� ����� 4 
	// ����� ��������� ��� ����� 4
	ldi r22,HOME_SHAGOVIK_DELTA_HIGH_4 
	ldi r21,HOME_SHAGOVIK_DELTA_LOW_4  
zapasnoi_exit:
rjmp exit1

// �� ���� ��������, ���� ����� ����� 1 �� � ����
stage4:
	// �������� ����� ���� �� ������ 1 ��
	// �������� ������� � ���������� � ����
	ldi r16,0x01
	sub r21,r16
	clr r16
	sbc r22,r16

	clr r16
	cpi r21,0x00
	brne zapasnoi_exit
	cpi r22,0x00
	brne zapasnoi_exit ; ���� r22:r21 ����� �� ����� ����, �� �����
	// ���� ������ 1 �� � ������ � ����, ��
	CBI PORTC,LED_home ; ��������� ��������� ������ HOME
	ldi r16, (1<<WGM12) | (0<<WGM13) | (0<<CS12) | (0<<CS11) | (0<<CS10) ; 
	out TCCR1B,r16 ; ��������� ������

	// ���������� � ��� ��������� ����, � ������� ������, � ��������� � ������� �� ����� ����� �� ������ ��
	clr r25
	clr r24
	sts SHAGOVIK_X0_HIGH_RAM,r25
	sts SHAGOVIK_X0_LOW_RAM,r24
	sts SHAGOVIK_X_TRULY_HIGH_RAM,r25 ; ��������� ���������� ��������� � ���
	sts SHAGOVIK_X_TRULY_LOW_RAM,r24 ; ��������� ���������� ��������� � ���
	sts SHAGOVIK_X2_HIGH_RAM,r25  
	sts SHAGOVIK_X2_LOW_RAM,r24  
	
	// ��������� � EEPROM ������� ���������
	ldi r16,0x01
	sts ENCODER_CONTR_MOT_SAVE_X,r16

	lds r16,INIT_FLAG
	cpi r16,0x00
	breq home_next1 ; ���� ���� ��������� �������, � �� ������� ������, �� �� ������� �� ����� ��������� 0

	// ������� �� ������� ������� ���������
	ldi r16,0x40
	rcall Set_cursor
	
	// ��������� �������� �������� ��������� �
	mov r26,r25 
	mov r25,r24
	rcall lcd_print_number ; ������� �� ������� 


home_next1:

	ldi r16,0b01000000
	out GICR,r16 ; �������� �������

	in r16,TIMSK
	ori r16,0b00000010  
	out TIMSK,r16 ; �������� ������ 0

	ldi r16,0x00 
	sts SHAGOVIK_timer1_on,r16 ; ����������, ��� timer1 ��������
rjmp exit1

exit_but_ok:
// ��������� � ����� ��������, ����� �� �� ��������� �������� �������� � ����� ����� �� �����
	
exit1:
// ��������� � ��� �������� ��� ������������ ��������
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
	// ���������, ����� �� ��������� ��� ������ �������
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
// ����� �����
// ���������� SREG, ����� �� ��������� ����� � ������ ���������� breq, brne � �.�. � ������� ��������. � ����� ������ ������� ���������� �������, ��� � push � pop
	lds r16,SREG_temp
	out SREG,r16
	pop r16 ; ���������� ����������� (��� �-��� interrupt_process)
reti

















// ������� ������������ � ��������� ������������. �������� ����������� - ����� �������� ��������� � ��������� ���������� ����������� ������� ���������� ����-����
test_random:
	SBIC PINA,1 ; ���� ������ HOME ������, �� ������ �������
	ret ; ������ �������

	SBIC PINB,1 ; ���� ������ BLOCK ������, �� ������ �������
	ret ; ������ �������


	// ��������� ������ �� ������ �� ��� ��������� ����������
	in r16,PINB
	SBRC r16,2 ; ���� ������ �� �� ������, �� ���������� ����. ������� 
	rjmp test_start ; ���� �� ������, �� ��������� ����������
	ret ; ����� ������ �������


test_start: ; ���������� �����������
	// ������� ��������� �������� �� �������������� �����������, ���� �� ��������, �� �� ���������.
	// ��������� ������ ���. ������������
	//ldi r18,0x00
	//ldi r17,0x02 ; EEPROM ����� 0x0002
	//rcall EEPROM_read

	//cpi r16,0xFF ; ���� ����������� �������
	//breq test_random_next2 ; �� �� ������������ � ������ �������� ����������

test_auto_on: ; ����� ������������
	ldi r16,0x00
	sts INIT_FLAG,r16 ; ����������, ��� ���� �� ������� ������, � ��������� 
	ldi r16,RAM_BUTTON_HOME
	sts RAM_ACTIVE_BUTTON,r16 ; ����������, ��� ������������ ������ HOME
	rcall Button_home_func ; ������������ � ������� ���������
	rcall interrupt_process ; �������� �����, ���� �� ��������� �������� ���������
	rcall get_position_value_from_eeprom ; ��������� ���������, � ������� ���� ������ 
	ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16 ; ����������, ��� ������������ ������ OK
	rcall Button_ok_func ; ���� � ���������, ������� ���� � ��������� ���
	rcall interrupt_process ; �������� �����, ���� �� ��������� ������� ���������
	ldi r16,0x01
	sts INIT_FLAG,r16 ; ����������, ��� ��������� ���� �����������, ����� ��� ������� �� ������ HOME, � ����� ������������ ����� �������� ��������� �� �����


test_random_next2: ; ��������� ����������
	// �������������� ������ 2, ������� ����� ������� �������� ��������
	ldi r16,0b00001111 ; CTC; 1024 Precaler
	out TCCR2,r16 

	clr r16
	out TCNT2,r16 ; ������� �������

	ldi r16,0x0F ; 15 ~ 1 ������������
	out OCR2,r16

	in r16,TIMSK
	ori r16,0b10000000 ; ���������� �� ���������
	out TIMSK,r16

	cli
	clr r16
	clr r17
	sts TIM2_counter_high,r17
	sts TIM2_counter_low,r16
	sei
// �������� ���
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
	sts SHAGOVIK_X2_HIGH_RAM,r17 ; ���������� � ��� ������� ���� �������
    sts SHAGOVIK_X2_LOW_RAM,r16  ; ���������� � ��� ������� ���� �������
	
	
	push r17
	push r16
	ldi r16,RAM_BUTTON_OK
	sts RAM_ACTIVE_BUTTON,r16 ; ����������, ��� ������������ ������ OK
	rcall Button_ok_func ; ���� � ��������� ���������

	rcall interrupt_process ; �������� �����, ���� �� ��������� ������� ���������
	
	pop r16
	pop r17

	rjmp while1

ret






/*
get_adc_value:
	ldi r16,0b11000111 
	out ADCSRA,r16    ; ADC enable; Start conversation; Prescaler for freq  = 128
// ��� ���� 6 ��� �������� ADCSRA ���������
wait_value:
	in r16,ADCSRA
	SBRC r16,6
	rjmp wait_value ; ���� ��� Start conversation ���������, �� ������ �������������� ����������� � ����� ���� ������
	*/


; ������� ��������� ������ ������ EEPROM, � ������� �������� ��������� ������� �� 0 �� (r18 - 1) (�� 0 �� 99)
set_value_of_positions_to_eeprom:
	// ���� ������ ������ BLOCK � ������ OK, �� ��� �������� ������� ���������

	// ��������� ������ �� ������ OK ��� ��������� ��������� ��������
	in r16,PINB
	SBRC r16,2 ; ���� ������ OK �� ������, �� ���������� ����. ������� 
	rjmp check_block ; ���� OK ������, �� �������� ��������� ������� ������ BLOCK
	ret ; ����� ������ �������

check_block:
	// ��������� ������ �� ������ BLOCK ��� ��������� ��������� ��������
	in r16,PINB
	SBRC r16,1 ; ���� ������ BLOCK �� ������, �� ���������� ����. ������� 
	rjmp clear_values_start ; ���� BLOCK ������, �� ��������� ��������� ��������
	ret ; ����� ������ �������

clear_values_start:
	
	ldi r18,0x64 ; ���������� ������� � ������� ���� ��������� �������� ���������
	clr r17 ; �������
	sts SHAGOVIK_X2_HIGH_RAM,r17 ; ������ �������� 0
	sts SHAGOVIK_X2_LOW_RAM,r17 ; ������ �������� 0
	sts RAM_position_number,r17  ; �������� ������� �������
// ��������� ���� ������
go1:
	; ��������� � EEPROM �������� SHAGOVIK_X2_HIGH_RAM ������� RAM_position_number
	// �������� ��������� ������ ����
	push r17
	push r18
	sbi PORTC,LED_prog
	// ��������� ������� ����� �������
	lds r16,RAM_position_number

	// ���������� ��������� � ������ ������� � ��� 263 (26.3)
	ldi r17,0x02
	mul r16,r17
	mov r16,r0
	ldi r17,0x0A
	add r17,r16
	clr r18

	lds r16,SHAGOVIK_X2_HIGH_RAM  ; ������� ������� ���� �������� ��������� �
	rcall EEPROM_write

	inc r17
	lds r16,SHAGOVIK_X2_LOW_RAM  ; ������ ������� ���� �������� ��������� �
	rcall EEPROM_write
	cbi PORTC,LED_prog
	pop r18
	pop r17

	inc r17
	sts RAM_position_number,r17 ; �������� ��������� �������
	dec r18 
	brne go1 ; ���� r18 �� ����� ����, �� ���������

	// ��� ����� �������� �������� �������� ��������� 
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


// ��������� ������� ���� ��� ������, ������� �������� ��� ��������� ����� ����������������
check_block_prog_ok:

	//ldi r16,0x00
	//sts PROG_FUNC_WORK_STATUS_RAM,r16 ; ���������� ���� � ���� ������ ������ ����. 0 - ������ ���� �� ��������. 1 - ��������.

	SBIS PINA,0 ; ���� ������ PROG ������, �� �� �������
	ret ; ����� �������

	SBIS PINB,1 ; ���� ������ BLOCK ������, �� �� �������
	ret ; ����� �������

	SBIS PINB,2 ; ���� ������ OK ������, �� �� �������
	ret ; ����� �������

	// ���� ��� ��� ������ ���������� ��������, ������ ������ ���� ����������
	ldi r16,0x01
	sts PROG_FUNC_WORK_STATUS_RAM,r16 ; ���������� ������� � ���� ������ ������ ����. 0 - ������ ���� �� ��������. 1 - ��������.
	sts EXIT_FROM_SETTINGS_CLK_FLAG,r16

	// ��������� BLOCK
	CBI PORTC,LED_block
	ldi r16,0b00000011 
	sts access_to_button_flags,r16 ; ��������� ������� ���� ������, ����� ����
	ldi r16,0b01000000
	out GIFR,r16 ; ������� ���� ������������ ����������
	out GICR,r16 ; ��������� ���������� �� INT0, ����� ��������� ��������� ��������
	rcall Clock_start_for_block // �������� ������, ������� ������� ���������� ����� 10 ������

	call blink_prog_led // ������ ����������� ������ PROG

// ��� ���� �������� ������ BLOCK
wait_block_release:
	SBIC PINB,1 ; ���� ������ ��������, �� ���������� ����. �������
	rjmp wait_block_release

// ��� ���� �������� ������ HOME
wait_home_release:
	SBIC PINA,1 ; ���� ������ ��������, �� ���������� ����. �������
	rjmp wait_home_release

// ��� ���� �������� ������ OK
wait_ok_release:
	SBIC PINB,2 ; ���� ������ ��������, �� ���������� ����. �������
	rjmp wait_ok_release

ret

// ������, ������� �������� ����������, ���� ������ �� ���������� � ������� 10 ������. ��� ������ ���� ������� ������ ������ ��� ��������������
Clock_start_for_block:
	push r16
	ldi r16,0x01
	sts SET_BLOCK_CLK_FLAG,r16
	sts EXIT_FROM_SETTINGS_CLK_FLAG,r16 // ������ � ������������ ������ ��� ������ �� ������ ��������
	pop r16
ret

// ��� ���������� ������� ����������
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
// ��������� ������� ����������� �� ���
	lds r20,MILLIS_1
	lds r19,MILLIS_2
	lds r18,MILLIS_3
	lds r17,MILLIS_4

	// ���������� � �������� ����������� �������
	ldi r16,0x01
	add r17,r16
	clr r16
	adc r18,r16
	adc r19,r16
	adc r20,r16

	// ���������� ������� ����������� � ���
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
	// ���������� � �������� ������� �������
	lds r20,SET_BLOCK_CLK_COUNTER_HIGH
	lds r19,SET_BLOCK_CLK_COUNTER_MID
	lds r18,SET_BLOCK_CLK_COUNTER_LOW
	// ��������� ���� ������ ��������
	lds r16,SET_BLOCK_CLK_FLAG

	// ��������� ����.
	cpi r16,0x01
	brne set_block_spx_next1
set_block_flag: // ���� ���� ����� 1
	// ���������� ������� ������� � �������
	ldi r18,0x01
	clr r19
	clr r20
	sts SET_BLOCK_CLK_COUNTER_HIGH,r20
	sts SET_BLOCK_CLK_COUNTER_MID,r19
	sts SET_BLOCK_CLK_COUNTER_LOW,r18
	clr r16
	sts SET_BLOCK_CLK_FLAG,r16 ; ���������� ����
set_block_spx_next1: // ���� ���� ����� 0
	// ��������� ���� ����� ������� �������. ���� ������� ����� 0, �� ������ �����
	clr r16
	CPSE r18,r16
	rjmp set_block_spx_next2 ; �����
	CPSE r19,r16
	rjmp set_block_spx_next2 ; �����
	CPSE r20,r16
	rjmp set_block_spx_next2 ; �����
	rjmp set_block_spx_exit
set_block_spx_next2: // ���� ������� �� ����� 0
	// ���������� 1 � ��������
	ldi r16,0x01
	add r18,r16
	clr r16
	adc r19,r16
	adc r20,r16
	// �������� ������� � ������� ������ (������ �� ������������� ���������� �������)
	ldi r16,SET_BLOCK_CLK_TIME_LOW
	CPSE r18,r16
	rjmp set_block_spx_next3
	ldi r16,SET_BLOCK_CLK_TIME_MID
	CPSE r19,r16
	rjmp set_block_spx_next3
	ldi r16,SET_BLOCK_CLK_TIME_HIGH
	CPSE r20,r16
	rjmp set_block_spx_next3
set_block_final_time: // ���� ������� ����� �������� ����� (������ ������������� �����)
	// �������� ����� �����
	SBI PORTC,LED_block
	ldi r16,0b00000001 
	sts access_to_button_flags,r16 ; ��������� ������� ���� ������
	ldi r16,0b00000000
	out GICR,r16 ; INT0 � ������� ��� ��������� ���������� �� ���� INT0


	// ������ �� �� ���������
	clr r18
	sts SET_BLOCK_CLK_COUNTER_HIGH,r18
	sts SET_BLOCK_CLK_COUNTER_MID,r18
	sts SET_BLOCK_CLK_COUNTER_LOW,r18
	rjmp set_block_spx_exit
set_block_spx_next3: // ���� ������� �� ����� �������� ����� (�� ������ ������������� �����)
	// ������ ��������� �������
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

	// ���������, ��� �� �������� ��� ����� �� ������ ��������, � ���� ��, �� ����� �� �������
	lds r16,PROG_FUNC_WORK_STATUS_RAM // 0 - ������ ���� �� ��������. 1 - ��������.
	cpi r16,0x00 // ���� �� ����� ����, �� ����� �� �������
	breq exit_from_settings_spx_exit

	// ���������� � �������� ������� �������
	lds r20,EXIT_FROM_SETTINGS_CLK_COUNTER_HIGH
	lds r19,EXIT_FROM_SETTINGS_CLK_COUNTER_MID
	lds r18,EXIT_FROM_SETTINGS_CLK_COUNTER_LOW
	// ��������� ���� ������ ��������
	lds r16,EXIT_FROM_SETTINGS_CLK_FLAG

	// ��������� ����.
	cpi r16,0x01
	brne exit_from_settings_spx_next1
exit_from_settings_flag: // ���� ���� ����� 1
	// ���������� ������� ������� � �������
	ldi r18,0x01
	clr r19
	clr r20
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_HIGH,r20
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_MID,r19
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_LOW,r18
	clr r16
	sts EXIT_FROM_SETTINGS_CLK_FLAG,r16 ; ���������� ����
exit_from_settings_spx_next1: // ���� ���� ����� 0
	// ��������� ���� ����� ������� �������. ���� ������� ����� 0, �� ������ �����
	clr r16
	CPSE r18,r16
	rjmp exit_from_settings_spx_next2 ; �����
	CPSE r19,r16
	rjmp exit_from_settings_spx_next2 ; �����
	CPSE r20,r16
	rjmp exit_from_settings_spx_next2 ; �����
	rjmp exit_from_settings_spx_exit
exit_from_settings_spx_next2: // ���� ������� �� ����� 0
	// ���������� 1 � ��������
	ldi r16,0x01
	add r18,r16
	clr r16
	adc r19,r16
	adc r20,r16
	// �������� ������� � ������� ������ (������ �� ������������� ���������� �������)
	ldi r16,EXIT_FROM_SETTINGS_CLK_TIME_LOW
	CPSE r18,r16
	rjmp exit_from_settings_spx_next3
	ldi r16,EXIT_FROM_SETTINGS_CLK_TIME_MID
	CPSE r19,r16
	rjmp exit_from_settings_spx_next3
	ldi r16,EXIT_FROM_SETTINGS_CLK_TIME_HIGH
	CPSE r20,r16
	rjmp exit_from_settings_spx_next3
exit_from_settings_final_time: // ���� ������� ����� �������� ����� (������ ������������� �����)
	// ������� �� ������ ���������, ������� �� ���� ����
	ldi r16,0x00
	sts PROG_FUNC_WORK_STATUS_RAM,r16
	cbi PORTC,LED_prog // ��������� ��������� ��� ������� PROG

	// ������ �� �� ���������
	clr r18
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_HIGH,r18
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_MID,r18
	sts EXIT_FROM_SETTINGS_CLK_COUNTER_LOW,r18
	rjmp exit_from_settings_spx_exit
exit_from_settings_spx_next3: // ���� ������� �� ����� �������� ����� (�� ������ ������������� �����)
	// ������ ��������� �������
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


