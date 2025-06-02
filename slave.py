import machine
import time
import ustruct

# --- Hardware Configuration for Raspberry Pi Pico and RS485 HAT ---
# ตรวจสอบเอกสาร HAT ของคุณเพื่อยืนยัน UART ID และ Pin ที่ใช้
# ตัวอย่าง:
#   - ถ้า HAT ใช้ UART0: TX=GPIO0, RX=GPIO1
#   - ถ้า HAT ใช้ UART1: TX=GPIO4, RX=GPIO5
#   - DE/RE Pin: ตรวจสอบว่า HAT ใช้ GPIO ใดในการควบคุม (เช่น GPIO8 หรืออื่นๆ)
RS485_UART_ID = 0  # เลือก UART ID (0 หรือ 1)
RS485_TX_PIN = 0   # Pin สำหรับ Transmit (TX)
RS485_RX_PIN = 1   # Pin สำหรับ Receive (RX)
RS485_DE_RE_PIN = 2 # Pin สำหรับควบคุม Driver Enable / Receiver Enable (DE/RE)
                     # <<<< คุณต้องปรับค่านี้ให้ตรงกับ HAT ของคุณ!

# --- Modbus RTU Settings ---
MODBUS_BAUDRATE = 9600
MODBUS_DATA_BITS = 8
MODBUS_STOP_BITS = 1
MODBUS_PARITY = None  # None Parity check
MODBUS_SLAVE_ADDRESS = 1 # Pico's Modbus Slave Address (1-247)

# --- ฟังก์ชันสำหรับ CRC16 (ตามมาตรฐาน Modbus RTU) ---
def calculate_crc16(data):
    """Calculates the CRC16 checksum for Modbus RTU."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')

# --- Holding Registers Definition for Washing Machine (Decimal Addresses) ---
# Each key is the Modbus register address (decimal).
# Each value is the current 16-bit content of that register.
# Initialize with default or known states.
HOLDING_REGISTERS = {
    # Device Status & Door Status (Address 0x11D = 285)
    # This register's value will be set by the slave based on its internal state.
    # We will simulate: 1=Standby, 2=Auto run, 3=Idle for Device Status (low bits)
    #                 1=Door open, 2=Door closed, 3=Door locked, 4=Door error, 5=Locking (high bits, or different interpretation)
    # For simplicity, let's make it reflect Device Status first.
    285: 0x0001, # Default: Standby (1)

    # Start/Lock/Unlock Allowed (Address 0x11E = 286)
    # Bit 0: 是否允许启动运行 (Whether start operation is allowed, 0=Not allowed, 1=Allowed)
    # Bit 1: 是否可以上锁或解锁 (Whether can lock or unlock, 0=Not allowed, 1=Allowed)
    286: 0x0003, # Default: Allowed to Start (Bit 0 = 1), Can Lock/Unlock (Bit 1 = 1)

    # Current Running Program Number (Address 0x11F = 287)
    # Value range: 1-3 (for washing machine as per doc table, though 1-10 for program selection)
    287: 0x0000, # No program running initially

    # Remaining Time (Address 0x120 = 288)
    # Doc implies Hours, Minutes, Seconds are at this single address.
    # This is unusual for Modbus. We'll store a single 16-bit value here.
    # Master needs to know how to pack/unpack this. For now, let's make it seconds (up to 65535s)
    288: 0x0000, # Remaining time in seconds (e.g., 60 for 1 minute)

    # Overflow Water Level (Address 0x121 = 289, Low 8 bits, unit: CM)
    # Max Spin Time (Address 0x121 = 289, High 8 bits, unit: min)
    289: 0x0000, # e.g., 0x0A05 (10 min spin, 5cm overflow)

    # Max Working Temperature (Address 0x122 = 290)
    # Read-only (读)
    290: 0x0064, # Default: 100 (example)

    # Max Washing Speed (Address 0x123 = 291)
    # Read-only (读), Unit: rpm
    291: 0x01F4, # Default: 500 rpm (example)

    # Mid Spin Max Speed (Address 0x124 = 292)
    # Read-only (读), Unit: rpm
    292: 0x03E8, # Default: 1000 rpm (example)

    # High Spin Max Speed (Address 0x125 = 293)
    # Read-only (读), Unit: rpm
    293: 0x07D0, # Default: 2000 rpm (example)

    # Program Selection (Address 0x126 = 294)
    # Writable (写), Value range: 1~10
    294: 0x0001, # Default: Program 1 selected

    # Command Action Table (Address 0x127 = 295)
    # Writable (写)
    # Bit 0: 启动运行 (Start run)
    # Bit 1: 停止运行 (Stop run)
    # Bit 2: 读取程序内容 (Read program content)
    # Bit 3: 保存程序内容 (Save program content)
    # Bit 4: 锁门 解锁 (Lock door / Unlock door)
    295: 0x0000, # Default: No command active

    # --- Automatic Program Structure (Complex Registers for Writing Program Content) ---
    # These registers are accessed when writing/reading program content (Function 0x10)
    # They are for defining program steps. Max continuous 120 words.
    # Placeholder for a range of registers
    # (You would typically manage this dynamically or via a dedicated program memory structure)
    #**{i: 0x0000 for i in range(945, 1061)} # Covers program structure registers from 0x381 to 0x424
}

# --- Modbus RTU Slave Class ---
class ModbusRTUSlave:
    def __init__(self, uart_id, tx_pin, rx_pin, de_re_pin, slave_address, holding_registers, crc_func):
        self.uart = machine.UART(uart_id, MODBUS_BAUDRATE, tx=machine.Pin(tx_pin), rx=machine.Pin(rx_pin), # <<--- แก้ไข TypeError แล้ว
                                bits=MODBUS_DATA_BITS, stop=MODBUS_STOP_BITS,
                                parity=MODBUS_PARITY)
        self.de_re_pin = machine.Pin(de_re_pin, machine.Pin.OUT)
        self.de_re_pin.value(0) # Default to Receive mode
        self.slave_address = slave_address
        self.holding_registers = holding_registers
        self.calculate_crc16 = crc_func # <<--- แก้ไข NameError แล้ว
        time.sleep_ms(100)

    def _send_response(self, response_adu):
        """Sends a Modbus ADU response over UART."""
        self.de_re_pin.value(1) # Enable transmit
        self.uart.write(response_adu)
        time.sleep_us(3500) # Wait for transmission to complete
        self.de_re_pin.value(0) # Switch back to receive

    def _handle_request(self, request_adu):
        """Processes an incoming Modbus RTU request."""
        if len(request_adu) < 8: # Minimum ADU length for a valid request (Slave ID + Func Code + Addr + Qty + CRC)
            return None

        # Validate CRC of the received request
        received_crc = int.from_bytes(request_adu[-2:], 'little')
        calculated_crc = int.from_bytes(self.calculate_crc16(request_adu[:-2]), 'little') # <<--- แก้ไข NameError แล้ว
        if received_crc != calculated_crc:
            print("Slave: CRC mismatch in request!")
            return None

        slave_id = request_adu[0]
        function_code = request_adu[1]
        
        if slave_id != self.slave_address:
            return None # Not for this slave

        response_pdu = bytearray()
        exception_code = 0 # 0 means no exception

        try:
            if function_code == 0x03: # Read Holding Registers
                start_address = int.from_bytes(request_adu[2:4], 'big')
                quantity = int.from_bytes(request_adu[4:6], 'big')

                # Validate address and quantity
                if quantity == 0 or quantity > 125: # Max registers for FC 0x03
                    exception_code = 0x03 # Illegal Data Value
                elif not all(addr in self.holding_registers for addr in range(start_address, start_address + quantity)):
                    exception_code = 0x02 # Illegal Data Address
                else:
                    response_pdu.extend([function_code, quantity * 2]) # Func Code, Byte Count
                    for i in range(quantity):
                        reg_value = self.holding_registers.get(start_address + i, 0) # Get value, default to 0
                        response_pdu.extend(reg_value.to_bytes(2, 'big'))

            elif function_code == 0x10: # Write Multiple Registers
                start_address = int.from_bytes(request_adu[2:4], 'big')
                num_registers = int.from_bytes(request_adu[4:6], 'big')
                byte_count = request_adu[6]
                data_bytes = request_adu[7:-2] # Exclude CRC

                # Validate data length
                if byte_count != num_registers * 2:
                    exception_code = 0x03 # Illegal Data Value (Byte count does not match register count)
                # Validate address and quantity
                elif num_registers == 0 or num_registers > 123: # Max registers for FC 0x10
                    exception_code = 0x03 # Illegal Data Value
                elif not all(addr in self.holding_registers for addr in range(start_address, start_address + num_registers)):
                    exception_code = 0x02 # Illegal Data Address
                else:
                    for i in range(num_registers):
                        value = int.from_bytes(data_bytes[i*2:(i+1)*2], 'big')
                        self.holding_registers[start_address + i] = value
                        print(f"Slave: Register {start_address + i} updated to {value}") # For debugging
                    
                    # Normal response for 0x10: Echo func code, start address, and quantity
                    response_pdu.extend(function_code.to_bytes(1, 'big'))
                    response_pdu.extend(start_address.to_bytes(2, 'big'))
                    response_pdu.extend(num_registers.to_bytes(2, 'big'))

            else:
                exception_code = 0x01 # Illegal Function Code

        except Exception as e:
            print(f"Slave: Processing error: {e}")
            exception_code = 0x04 # Slave Device Failure

        if exception_code != 0:
            response_pdu = bytearray([function_code | 0x80, exception_code]) # Exception response

        response_adu = bytearray([self.slave_address])
        response_adu.extend(response_pdu)
        response_adu.extend(self.calculate_crc16(response_adu)) # <<--- แก้ไข NameError แล้ว
        return response_adu

    def run_forever(self):
        print("Modbus RTU Slave running on Pico...")
        print(f"Slave Address: {self.slave_address}")
        print(f"Holding Registers (first 50): {list(self.holding_registers.keys())[:50]}...") # Print a subset for readability
        while True:
            if self.uart.any():
                request_adu = self.uart.read()
                print(f"Slave Rx ({len(request_adu)} bytes): {request_adu.hex()}") # Uncomment for detailed debugging
                response_adu = self._handle_request(request_adu)
                if response_adu:
                    print(f"Slave Tx ({len(response_adu)} bytes): {response_adu.hex()}") # Uncomment for detailed debugging
                    self._send_response(response_adu)
            time.sleep_ms(10) # Small delay to avoid busy-waiting


# --- Main execution for Pico Slave ---
if __name__ == "__main__":
    slave = ModbusRTUSlave(
        RS485_UART_ID,
        RS485_TX_PIN,
        RS485_RX_PIN,
        RS485_DE_RE_PIN,
        MODBUS_SLAVE_ADDRESS,
        HOLDING_REGISTERS,
        calculate_crc16 # <<--- ส่งฟังก์ชัน CRC16 เข้าไปในคอนสตรักเตอร์
    )
    slave.run_forever()

