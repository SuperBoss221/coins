from machine import UART, Pin
import time
import ustruct
import buzser
import coin
import tm1637
import _thread
coin_total = 0
coin_multi = 10

mydisplay = tm1637.TM1637(clk=Pin(27), dio=Pin(26))
mydisplay.show("LOAD")

buzser.SUCCESS()
#time.sleep(1)
buzser.Start()
#time.sleep(1)
buzser.CLICK()
#time.sleep(1)
buzser.Error()
#time.sleep(1)
relay1 = Pin(7, Pin.OUT, value=0)
mydisplay.number(coin_total)

debounce_delay = 500
timer_direction = 0
running = 0
stepload = 3


def RUNRELAY(ps):
    global timer_direction,running , coin_total
    if coin_total <= 0 :
        mydisplay.number(coin_total)
        buzser.Error()
        return False
    if running == 0 :
        running = 1
        relay1.value(1)
    else :
        running = 0
        relay1.value(0)


def coin_callback(pin):
    global coin_total
    coin_total += 1
    mydisplay.number(coin_total)
    buzser.CLICK()
    print(pin)
    

def delete_callback(pin):
    global coin_total
    if pin.value() == 0 :
        print(pin.value())
        if coin_total <= 0 :
            relay1.value(0)
            running = 0
            coin_total = 0
            mydisplay.number(coin_total)
            buzser.Error()
            return False
        coin_total -= 1
        mydisplay.number(coin_total)
        if coin_total <= 0 :
            coin_total = 0
            relay1.value(0)
            running = 0
            buzser.SUCCESS()
            return True
        #relay1.value(0)
        buzser.CLICK()
        print(pin)
    return True
    
add_coin = machine.Pin(8, machine.Pin.IN)
add_coin.irq(trigger=machine.Pin.IRQ_FALLING, handler=delete_callback)

pin_coin = machine.Pin(22, machine.Pin.IN)
pin_coin.irq(trigger=machine.Pin.IRQ_FALLING, handler=delete_callback)

pin_coin = machine.Pin(21, machine.Pin.IN)
pin_coin.irq(trigger=machine.Pin.IRQ_FALLING, handler=coin_callback)

run_button1 = machine.Pin(20,  mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)
run_button1.irq(trigger=machine.Pin.IRQ_RISING, handler=RUNRELAY)

def th_func():
    global coin_total
    while True:
        if coin_total <= 0 and relay1.value() == 1:
            relay1.value(0)
        time.sleep(1)
        if coin_total >= 1 and relay1.value() == 0 :
            print('Running thread ',coin_total)
            relay1.value(1)
            
_thread.start_new_thread(th_func,())

# --- กำหนดค่าการเชื่อมต่อ UART และ Modbus ---
UART_ID = 0
TX_PIN = 0
RX_PIN = 1
RE_DE_PIN = 2 # ถ้ามีขาควบคุมทิศทาง (RS485 Transceiver)
BAUD_RATE = 9600
SLAVE_ID = 1

# --- กำหนด Holding Registers (Address เริ่มจาก 0) ---
# ขยายขนาดของ holding_registers ให้ครอบคลุมถึง Address 69 (20 + 50 - 1)
# เพื่อรองรับการอ่าน 50 registers จาก Address 20 ของ ESP32
# เราจะใช้ list แทน dictionary เพื่อให้ง่ายต่อการจัดการ address แบบ sequential
holding_registers = [0] * 100 # Index 0 ถึง 69

# กำหนดค่าเริ่มต้นของ Registers ที่ต้องการ
holding_registers[0] = 0   # Error reset/slience
holding_registers[1] = 0   # Start run
holding_registers[2] = 0   # Autorun advance
holding_registers[3] = 0   # Force stop
holding_registers[4] = coin_total   # Coins/money paid
holding_registers[5] = 0   # Selective program

holding_registers[20] = 1  # Run status: 1 (Standby)
holding_registers[21] = 2  # Door status: 2 (Door closed)
holding_registers[22] = 0  # Error status: 0 (Normal)
# เพิ่มค่าเริ่มต้นสำหรับ address อื่นๆ ที่ ESP32 อ่าน (Address 23 ถึง 69)
# ถ้าไม่กำหนด ก็จะเป็น 0 ตามค่าเริ่มต้นของ list
holding_registers[23] = 0 # Run step remain time (min)
holding_registers[24] = 0 # Run step remain time (sec)
holding_registers[25] = 0 # Auto program total remain time (h)
holding_registers[26] = 0 # Auto program total remain time (min)
holding_registers[27] = 0 # Auto program total remain time (sec)
holding_registers[28] = 0 # Current level (cm)
holding_registers[29] = 0 # Current set level
holding_registers[30] = 0 # Current temperature
holding_registers[31] = 0 # Current set temperature
holding_registers[32] = 0 # Current speed
holding_registers[33] = 0 # Current set speed
holding_registers[34] = 0 # Currently running washing program number
holding_registers[35] = 0 # Currently running step number
holding_registers[36] = 0 # Coins required of currently selecting program
holding_registers[37] = 0 # Current coins 
holding_registers[38] = 0 # Total coins recorded
holding_registers[39] = 0 # Current coins in cash box
# Address 40-49 ตามเอกสาร คุณต้องกำหนดค่าเริ่มต้นหรือ Placeholder ไว้ให้ครบ
for i in range(40, 50):
    holding_registers[i] = 0 # Placeholder for other registers up to 49

# ถ้าคุณต้องการให้ ESP32 อ่านถึง address 69 ก็ต้องเตรียม registers ไว้ให้ครบ
# (ในเอกสารมีถึง 49 แต่โค้ด ESP32 ร้องขอ 50 registers จาก 20)
# ดังนั้น Address 50-69 ก็ควรจะมีค่าเริ่มต้นไว้เช่นกัน
for i in range(50, 70): 
    holding_registers[i] = 0 # Placeholder for registers from 50 to 69 (if ESP32 tries to read this far)


# --- CRC16 Calculation (มาตรฐาน Modbus) ---
def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

# --- ฟังก์ชันจัดการ RS485 RE/DE pin (ถ้ามี) ---
def set_rs485_direction(pin, direction):
    if pin:
        pin.value(direction) # 0 for receive, 1 for transmit

# --- Main Modbus Loop ---
def main():
    uart = UART(UART_ID, baudrate=BAUD_RATE, tx=Pin(TX_PIN), rx=Pin(RX_PIN), bits=8, parity=None, stop=1)
    re_de_pin_obj = None
    if RE_DE_PIN is not None:
        re_de_pin_obj = Pin(RE_DE_PIN, Pin.OUT)
        set_rs485_direction(re_de_pin_obj, 0) # ตั้งค่าเริ่มต้นเป็นรับ

    print(f"Pico Modbus RTU Slave (DIY) started with ID {SLAVE_ID} on UART{UART_ID}.")
    print(f"Holding registers size: {len(holding_registers)} (Addresses 0 to {len(holding_registers)-1})")


    buffer = bytearray()
    last_byte_time = time.ticks_ms()
    T3_5 = 3.6 * 1000 / BAUD_RATE # ประมาณ 3.5 เท่าของเวลาบิต (สำหรับ 9600 baud คือประมาณ 3.6 ms)

    while True:
        if uart.any():
            byte = uart.read(1)[0]
            buffer.append(byte)
            last_byte_time = time.ticks_ms()
        
        # ตรวจสอบ End of Frame (silent interval)
        if len(buffer) > 0 and (time.ticks_ms() - last_byte_time) > T3_5:
            request_frame = bytes(buffer)
            buffer = bytearray() # เคลียร์ buffer สำหรับ request ถัดไป

            # --- ประมวลผล Modbus Request ---
            if len(request_frame) < 4: # เฟรมที่สั้นเกินไป
                print(f"Too short frame: {request_frame.hex()}")
                continue

            req_slave_id = request_frame[0]
            req_func_code = request_frame[1]
            req_data = request_frame[1:-2] # func_code + data
            req_crc = ustruct.unpack('<H', request_frame[-2:])[0] # CRC from frame

            calculated_crc = crc16(request_frame[:-2]) # Calculate CRC on entire frame except last two bytes

            if req_slave_id != SLAVE_ID:
                # ไม่ใช่ Slave ID ของเรา
                continue

            if req_crc != calculated_crc:
                print(f"CRC mismatch! Recv: {hex(req_crc)}, Calc: {hex(calculated_crc)}")
                # ไม่ตอบกลับเมื่อ CRC ไม่ตรง
                continue

            response_frame = bytearray()

            if req_func_code == 0x03: # Read Holding Registers
                try:
                    start_address = ustruct.unpack('>H', request_frame[2:4])[0]
                    num_registers = ustruct.unpack('>H', request_frame[4:6])[0]

                    print(f"Read Request (FC 0x03): Addr {start_address}, Count {num_registers}")

                    # ตรวจสอบว่า request ไม่เกินขอบเขตของ holding_registers
                    if start_address + num_registers > len(holding_registers) or num_registers == 0:
                        print(f"Error 0x02: Illegal Data Address for Read (Start: {start_address}, Count: {num_registers})")
                        response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x02]) # Illegal Data Address
                    else:
                        data_bytes = bytearray()
                        for i in range(num_registers):
                            addr = start_address + i
                            data_bytes.extend(ustruct.pack('>H', holding_registers[addr])) # Big-endian
                        
                        response_frame = bytearray([SLAVE_ID, req_func_code, len(data_bytes)])
                        response_frame.extend(data_bytes)
                    
                except Exception as e:
                    print(f"Error processing FC 0x03: {e}")
                    response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x04]) # Slave Device Failure

            elif req_func_code == 0x10: # Write Multiple Registers
                try:
                    start_address = ustruct.unpack('>H', request_frame[2:4])[0]
                    num_registers = ustruct.unpack('>H', request_frame[4:6])[0]
                    byte_count = request_frame[6]
                    
                    print(f"Write Request (FC 0x10): Addr {start_address}, Count {num_registers}, Byte Count {byte_count}")

                    # ตรวจสอบว่า request ไม่เกินขอบเขตและ byte_count ถูกต้อง
                    if byte_count != num_registers * 2 or num_registers == 0 or \
                       start_address + num_registers > len(holding_registers):
                        print(f"Error 0x03: Illegal Data Value or Address for Write (Start: {start_address}, Count: {num_registers}, Byte Count: {byte_count})")
                        response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x03]) # Illegal Data Value
                    else:
                        for i in range(num_registers):
                            addr = start_address + i
                            value = ustruct.unpack('>H', request_frame[7 + (i*2) : 9 + (i*2)])[0]
                            
                            # --- ตรวจสอบ Value Range ก่อนเขียน (สำหรับ Error 0x03) ---
                            is_value_valid = True
                            if addr == 0 and not (0 <= value <= 1): # Error reset/slience
                                is_value_valid = False
                            elif addr in [1, 2, 3] and not (0 <= value <= 1): # Start run, Autorun advance, Force stop
                                is_value_valid = False
                            elif addr == 4 and not (0 <= value <= 65535): # Coins/money paid
                                is_value_valid = False
                            elif addr == 5 and not (1 <= value <= 30): # Selective program (ถ้าเป็น free, ตรวจสอบตามเอกสาร)
                                is_value_valid = False
                            # เพิ่มการตรวจสอบ range สำหรับ address อื่นๆ ถ้ามี
                            
                            if not is_value_valid:
                                buzser.Error()
                                print(f"Error 0x03: Illegal Data Value for Addr {addr}, Value {value}")
                                response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x03]) # Illegal Data Value
                                break # หยุดการประมวลผลและส่ง Error
                            
                            holding_registers[addr] = value
                            print(f"  Writing: Addr {addr}, Value {value}")
                            
                            # --- ตรงนี้ใส่ Logic ควบคุมอุปกรณ์ของคุณ ---
                            if addr == 0 and value == 1:
                                print("  Command: Error Reset/Slience")
                                holding_registers[20] = 1 # Status
                                holding_registers[21] = 2 # Door
                                holding_registers[22] = 0 # Error
                                holding_registers[23] = 0 # Run step remain time (min)
                                holding_registers[24] = 0 # Run step remain time (sec)
                                holding_registers[25] = 0 # Auto program total remain time (h)
                                holding_registers[26] = 0 # Auto program total remain time (min)
                                holding_registers[27] = 0 # Auto program total remain time (sec)
                                holding_registers[28] = 0 # Current level (cm)
                                holding_registers[29] = 16 # Current set level
                                holding_registers[30] = 87 # Current temperature
                                holding_registers[31] = 158 # Current set temperature
                                holding_registers[32] = 0 # Current speed
                                holding_registers[33] = 0 # Current set speed
                                holding_registers[34] = 5 # Currently running washing program number
                                holding_registers[35] = 0 # Currently running step number
                                holding_registers[36] = 4 # Coins required of currently selecting program
                                holding_registers[37] = 0 # Current coins 
                                holding_registers[38] = 880 # Total coins recorded
                                holding_registers[39] = 852 # Current coins in cash box
                                buzser.SUCCESS()
                            elif addr == 1 and value == 1:
                                print("  Command: Start 1")
                                buzser.Start()
                                holding_registers[20] = 3 # ตั้งเป็น Autorun
                                holding_registers[21] = 3 # ตั้งเป็น Autorun
                                holding_registers[23] = 43 # ตั้งเป็น Autorun
                                holding_registers[24] = 60 # ตั้งเป็น Autorun
                                holding_registers[25] = 1 # ตั้งเป็น Autorun
                                holding_registers[26] = 20 # ตั้งเป็น Autorun
                                holding_registers[27] = 30 # ตั้งเป็น Autorun
                                holding_registers[29] = 1 # ตั้งเป็น Autorun
                                holding_registers[30] = 50 # ตั้งเป็น Autorun
                                holding_registers[31] = 80 # ตั้งเป็น Autorun
                            elif addr == 2 :
                                print("  Command: Start 1")
                                buzser.Start()
                                holding_registers[20] = 3 # ตั้งเป็น Autorun
                                holding_registers[21] = 3 # ตั้งเป็น Autorun
                            elif addr == 3 :
                                print("  Command: Force 3")
                                buzser.Error()
                                holding_registers[20] = 1
                                holding_registers[21] = 2
                                holding_registers[36] = 4
                                holding_registers[37] = 0
                                holding_registers[38] = 4
                            elif addr == 4 :
                                print("  Coins: Force 4")
                                buzser.CLICK()
                                holding_registers[37] = value # ตั้งเป็น Idle
                                holding_registers[21] = 3 # ตั้งเป็น Autorun
                            elif addr == 5 :
                                print("Command: Menu ",value)
                                buzser.CLICK()
                                holding_registers[21] = 2
                                holding_registers[34] = value
                            # ------------------------------------------
                        
                        if not response_frame: # ถ้าไม่มี error
                            response_frame = bytearray([SLAVE_ID, req_func_code, 
                                                        request_frame[2], request_frame[3], # Start Address
                                                        request_frame[4], request_frame[5]]) # Number of Registers
                    
                except Exception as e:
                    print(f"Error processing FC 0x10: {e}")
                    buzser.Error()
                    response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x04]) # Slave Device Failure

            else:
                # Unknown Function Code Exception
                print(f"Error 0x01: Illegal Function Code {hex(req_func_code)}")
                response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x01]) # Illegal Function

            # --- ส่ง RespEC64C97C1AF8onse กลับ ---
            if response_frame:
                response_frame.extend(ustruct.pack('<H', crc16(response_frame))) # Add CRC
                set_rs485_direction(re_de_pin_obj, 1) # ตั้งค่าเป็นส่ง
                uart.write(response_frame)
                set_rs485_direction(re_de_pin_obj, 0) # ตั้งค่ากลับเป็นรับ
                print(f"  Sent Response: {response_frame.hex()}") # Debug print

        time.sleep_ms(10) # หน่วงเวลาเล็กน้อย

if __name__ == "__main__":
    main()