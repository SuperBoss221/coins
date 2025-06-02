from machine import UART, Pin
import time
import ustruct # สำหรับแปลงข้อมูลไบต์

# --- กำหนดค่าการเชื่อมต่อ UART และ Modbus ---
UART_ID = 0
TX_PIN = 0
RX_PIN = 1
RE_DE_PIN = 2 # ถ้ามีขาควบคุมทิศทาง
BAUD_RATE = 9600
SLAVE_ID = 1

# --- กำหนด Holding Registers ---
# เราจะใช้ dictionary เพื่อให้จัดการ address ได้ง่ายขึ้น
holding_registers = {
    0: 0,   # Error reset/slience
    1: 0,   # Start run
    2: 0,   # Autorun advance
    3: 0,   # Force stop
    4: 0,   # Coins/money paid
    5: 0,   # Selective program
    20: 1,  # Run status: 1 (Standby)
    21: 2,  # Door status: 2 (Door closed)
    22: 0,  # Error status: 0 (Normal)
    # เพิ่ม address อื่นๆ ที่จำเป็นตามเอกสาร
    34: 0,  # Currently running washing program number
    35: 0,  # Currently running step number
    36: 0,  # Coins required of currently selecting program
    37: 0,  # Current coins
    38: 0,  # Total coins recorded
    39: 0,  # Current coins in cash box
}

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

    buffer = bytearray()
    last_byte_time = time.ticks_ms()
    T3_5 = 3.5 * 1000 / BAUD_RATE # ประมาณ 3.5 เท่าของเวลาบิต (สำหรับ 9600 baud คือประมาณ 3.6 ms)

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
                # ส่ง Exception Response (Optional)
                # response = bytearray([SLAVE_ID, req_func_code | 0x80, 0x04]) # Illegal Data Value
                # response.extend(ustruct.pack('<H', crc16(response)))
                # set_rs485_direction(re_de_pin_obj, 1)
                # uart.write(response)
                # set_rs485_direction(re_de_pin_obj, 0)
                continue

            response_frame = bytearray()

            if req_func_code == 0x03: # Read Holding Registers
                try:
                    start_address = ustruct.unpack('>H', request_frame[2:4])[0]
                    num_registers = ustruct.unpack('>H', request_frame[4:6])[0]

                    if num_registers > 125 or start_address + num_registers > max(holding_registers.keys()) + 1:
                        # Illegal Data Value Exception
                        response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x03]) # Illegal Data Value
                    else:
                        data_bytes = bytearray()
                        for i in range(num_registers):
                            addr = start_address + i
                            if addr in holding_registers:
                                data_bytes.extend(ustruct.pack('>H', holding_registers[addr])) # Big-endian
                            else:
                                # ถ้า address ไม่มีใน holding_registers ให้ส่ง exception
                                response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x02]) # Illegal Data Address
                                break
                        
                        if not response_frame: # ถ้าไม่มี error
                            response_frame = bytearray([SLAVE_ID, req_func_code, len(data_bytes)])
                            response_frame.extend(data_bytes)
                    
                    print(f"Read Request (FC 0x03): Addr {start_address}, Count {num_registers}")

                except Exception as e:
                    print(f"Error processing FC 0x03: {e}")
                    response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x04]) # Slave Device Failure

            elif req_func_code == 0x10: # Write Multiple Registers
                try:
                    start_address = ustruct.unpack('>H', request_frame[2:4])[0]
                    num_registers = ustruct.unpack('>H', request_frame[4:6])[0]
                    byte_count = request_frame[6]
                    
                    if byte_count != num_registers * 2 or num_registers > 123: # max regs for FC10
                        response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x03]) # Illegal Data Value
                    else:
                        for i in range(num_registers):
                            addr = start_address + i
                            value = ustruct.unpack('>H', request_frame[7 + (i*2) : 9 + (i*2)])[0]
                            if addr in holding_registers:
                                holding_registers[addr] = value
                                print(f"Write Request (FC 0x10): Addr {addr}, Value {value}")
                                # --- ตรงนี้ใส่ Logic ควบคุมอุปกรณ์ของคุณ ---
                                if addr == 0:
                                    if value == 1: print("Error Reset/Slience command received")
                                elif addr == 1:
                                    if value == 1: print("Start Run command received")
                                elif addr == 3:
                                    if value == 1: print("Force Stop command received")
                                # ------------------------------------------
                            else:
                                response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x02]) # Illegal Data Address
                                break
                        
                        if not response_frame:
                            response_frame = bytearray([SLAVE_ID, req_func_code, 
                                                        ustruct.unpack('>H', request_frame[2:4])[0], # Start Address
                                                        ustruct.unpack('>H', request_frame[4:6])[0]]) # Number of Registers
                    
                except Exception as e:
                    print(f"Error processing FC 0x10: {e}")
                    response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x04]) # Slave Device Failure

            else:
                # Unknown Function Code Exception
                response_frame = bytearray([SLAVE_ID, req_func_code | 0x80, 0x01]) # Illegal Function

            # --- ส่ง Response กลับ ---
            if response_frame:
                response_frame.extend(ustruct.pack('<H', crc16(response_frame))) # Add CRC
                set_rs485_direction(re_de_pin_obj, 1) # ตั้งค่าเป็นส่ง
                uart.write(response_frame)
                set_rs485_direction(re_de_pin_obj, 0) # ตั้งค่ากลับเป็นรับ
                # print(f"Sent: {response_frame.hex()}") # Debug print

        time.sleep_ms(10) # หน่วงเวลาเล็กน้อย

if __name__ == "__main__":
    main()