import logging
import math
import statistics
import serial
import os
import time

from . import bus
from . import filament_switch_sensor

BDP_CHIP_ADDR = 4
BDP_I2C_SPEED = 100000
BDP_REGS = {
    '_version': 0x0,
    '_measure_data': 15,
    'pa_probe_mode': 48,  ## 7= CLOCK_OSR_16384  2=CLOCK_OSR_512
    'raw_data_out': 49,
    'probe_thr': 50,
    'rang': 51,
    'reset_probe': 52,
    'invert_data': 53
}

PIN_MIN_TIME = 0.100
RESEND_HOST_TIME = 0.300 + PIN_MIN_TIME
MAX_SCHEDULE_TIME = 5.0


class BD_Pressure_Advance:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.port = config.get("port")
        self.old_res = ''
        self.thrhold = config.getint('thrhold', 8, minval=1)
        self.usb = None
        self.usb_port = None
        self._baud = None
        
        # 串口初始化（带异常处理）
        if "i2c" in self.port:
            self.i2c = bus.MCU_I2C_from_config(config, BDP_CHIP_ADDR, BDP_I2C_SPEED)
        elif "usb" in self.port:
            self.usb_port = config.get("serial")
            self._baud = config.getint('baud', 38400, minval=2400)
            try:
                self.usb = serial.Serial(self.usb_port, self._baud, timeout=0.1)
                self.usb.reset_input_buffer()
                self.usb.reset_output_buffer()
            except Exception as e:
                logging.error(f"串口初始化失败: {e}")
                self.usb = None

        self.PA_data = []
        self.bd_name = config.get_name().split()[1]
        self.gcode = self.printer.lookup_object('gcode')
        
        # 移除XY电机控制（无需移动，仅保留初始化避免报错）
        self._invert_stepper_x = None
        self.mcu_enable_pin_x = None
        self._invert_stepper_x1 = None
        self.mcu_enable_pin_x1 = None
        self._invert_stepper_y = None
        self.mcu_enable_pin_y = None
        self._invert_stepper_y1 = None
        self.mcu_enable_pin_y1 = None
        
        self.last_state = 0
        self.pressure_base_value = 0  # 基准压力值（仅挤出时使用）
        self.pressure_variation_threshold = 10  # 压力变化阈值（替代XY移动判定）

        # 注册GCODE指令
        self.gcode.register_mux_command("SET_BDPRESSURE", "NAME", self.bd_name,
                                        self.cmd_SET_BDPRESSURE,
                                        desc=self.cmd_SET_BDPRESSURE_help)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler("homing:homing_move_begin", self.handle_homing_move_begin)

    def set_probe_mode(self):
        response = ""
        if self.usb and "usb" == self.port:
            try:
                self.usb.write(f'e;{self.thrhold};'.encode())
                time.sleep(0.05)
            except Exception as e:
                logging.error(f"设置Probe模式失败: {e}")
        elif "i2c" == self.port:
            try:
                response += self.read_register('_version', 15).decode('utf-8')
                self.write_register('pa_probe_mode', 2)
                self.write_register('probe_thr', self.thrhold)
            except Exception as e:
                logging.error(f"I2C设置失败: {e}")

    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.set_probe_mode()
        # 初始化基准压力值
        if self.last_state == 0:
            self.calibrate_base_pressure()

    def handle_homing_move_begin(self, hmove):
        self.set_probe_mode()

    cmd_SET_BDPRESSURE_help = "cmd for BD_PRESSURE sensor,SET_BDPRESSURE NAME=xxx COMMAND=START/STOP/RESET_PROBE/READ/CAL_BASE VALUE=X"

    def cmd_SET_BDPRESSURE(self, gcmd):
        cmd = gcmd.get('COMMAND')
        if 'START' in cmd:
            self.cmd_start(gcmd)
        elif 'STOP' in cmd:
            self.cmd_stop(gcmd)
        elif 'RESET_PROBE' in cmd:
            self.cmd_reset_probe(gcmd)
        elif 'READ' in cmd:
            self.cmd_read(gcmd)
        elif 'CAL_BASE' in cmd:
            self.calibrate_base_pressure()  # 校准基准压力值

    # 新增：校准基准压力值（仅挤出前执行）
    def calibrate_base_pressure(self):
        if self.usb and "usb" == self.port:
            try:
                self.usb.write(';'.encode())
                time.sleep(0.1)
                if self.usb.in_waiting:
                    response = self.usb.read(self.usb.in_waiting).decode('utf-8', errors='ignore').strip()
                    if 'R:' in response:
                        R_v = response.strip().split('R:')[1].split(',')
                        if len(R_v) >= 1:
                            self.pressure_base_value = int(R_v[0])
                            self.gcode.respond_info(f"[{self.bd_name}] 基准压力值校准完成: {self.pressure_base_value}")
            except Exception as e:
                logging.error(f"校准基准压力失败: {e}")
                self.pressure_base_value = 127  # 默认值

    def _resend_current_val(self, eventtime):
        # 移除电机相关的重发逻辑（无需控制XY电机）
        return self.reactor.NEVER

    def enable_pin_init(self, config, stepper_name):
        # 空实现（无需控制XY电机）
        return None, None

    def _set_pin(self, print_time, value, is_resend=False):
        # 空实现（无需控制XY电机）
        pass

    def cmd_start(self, gcmd):
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead')

        self.last_state = 1
        # 重新校准基准压力值
        self.calibrate_base_pressure()

        # 串口操作（仅切换模式，无电机控制）
        response = ""
        if self.usb and "usb" == self.port:
            try:
                self.usb.write('l;D;'.encode())
                time.sleep(0.05)
                if self.usb.in_waiting:
                    response = self.usb.read(self.usb.in_waiting).decode('utf-8', errors='ignore').strip()
            except Exception as e:
                logging.error(f"串口START指令失败: {e}")
        elif "i2c" == self.port:
            try:
                self.write_register('pa_probe_mode', 7)
                self.write_register('raw_data_out', 0)
                response = self.read_register('_version', 15).decode('utf-8')
            except Exception as e:
                logging.error(f"I2C START指令失败: {e}")

        self.gcode.respond_info(f".cmd_start {self.port}: {response}")

    # 核心优化：仅挤出的压力数据处理逻辑
    def pa_data_process(self, gcmd, str_data):
        self.gcode.respond_info(f"{self.bd_name}: {str_data}")
        if 'R:' in str_data and ',' in str_data:
            try:
                R_v = str_data.strip().split('R:')[1].split(',')
                if len(R_v) == 5:
                    res = int(R_v[0])       # 实时压力值
                    lk = int(R_v[1])
                    rk = int(R_v[2])
                    Hk = int(R_v[3])        # 压力变化值1
                    Ha = int(R_v[4].split('\n')[0])  # 压力变化值2
                    val_step = float(gcmd.get('VALUE', 0))
                    
                    # 计算压力偏差（相对于基准值）
                    pressure_diff = abs(res - self.pressure_base_value)
                    pa_val = [val_step, res, lk, rk, Hk, Ha, pressure_diff]
                    self.PA_data.append(pa_val)
                    
                    # 仅挤出时的判定逻辑：压力偏差最小的点为最优PA
                    num = len(self.PA_data)
                    if num >= 15:  # 数据量足够时开始判定
                        # 找到压力偏差最小的点（替代原XY移动的判定逻辑）
                        min_diff = min([p[6] for p in self.PA_data])
                        min_index = [p[6] for p in self.PA_data].index(min_diff)
                        if pressure_diff <= min_diff + 2:  # 偏差稳定时停止
                            self.stop_pa(gcmd)
            except ValueError as e:
                logging.error(f"数据解析失败: {e}")
                return
        elif 'stop' in str_data:
            self.last_state = 0

    def cmd_read(self, gcmd):
        self.bdw_data = ''
        response = ""

        if self.usb and "usb" == self.port:
            try:
                if self.usb.is_open:
                    if self.usb.in_waiting:
                        response = self.usb.read(self.usb.in_waiting).decode('utf-8', errors='ignore').strip()
                    if response:
                        self.old_res = response
                        self.pa_data_process(gcmd, response)
                    else:
                        self.pa_data_process(gcmd, self.old_res)
            except Exception as e:
                logging.error(f"串口读取失败: {e}")
                return False
        elif "i2c" == self.port:
            try:
                response = self.read_register('_measure_data', 32).decode('utf-8').strip('\0')
                self.pa_data_process(gcmd, response)
            except Exception as e:
                logging.error(f"I2C读取失败: {e}")
                return False
        return True

    def read_register(self, reg_name, read_len):
        regs = [BDP_REGS[reg_name]]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params['response'])

    def write_register(self, reg_name, data):
        if type(data) is not list:
            data = [data]
        reg = BDP_REGS[reg_name]
        data.insert(0, reg)
        self.i2c.i2c_write(data)

    def stop_pa(self, gcmd):
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead')

        self.last_state = 0

        if self.usb and "usb" == self.port:
            try:
                self.usb.write('e;D;'.encode())
                time.sleep(0.05)
            except Exception as e:
                logging.error(f"串口STOP指令失败: {e}")
        elif "i2c" == self.port:
            try:
                self.write_register('pa_probe_mode', 2)
                self.write_register('raw_data_out', 0)
            except Exception as e:
                logging.error(f"I2C STOP指令失败: {e}")

    # 核心优化：仅挤出的PA值计算逻辑
    def cmd_stop(self, gcmd):
        self.stop_pa(gcmd)
        if len(self.PA_data) >= 5:
            # 移除前3个不稳定数据
            del self.PA_data[:3]

            if not self.PA_data:
                self.gcode.respond_info("No valid PA calibration data")
                return

            # 找到压力偏差最小的点（仅挤出的核心逻辑）
            min_diff = min([p[6] for p in self.PA_data])
            best_pa = [p for p in self.PA_data if p[6] == min_diff][0]
            
            self.gcode.respond_info(f"[最优PA值] 压力基准值: {self.pressure_base_value}, 实时压力: {best_pa[1]}, 偏差: {best_pa[6]}, PA值: {best_pa[0]:f}")
            set_pa = f'SET_PRESSURE_ADVANCE ADVANCE={best_pa[0]}'
            self.gcode.run_script_from_command(set_pa)
        else:
            self.gcode.respond_info("No PA calibration data or number is <=5")

    def cmd_reset_probe(self, gcmd):
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead')

        if self.usb and "usb" == self.port:
            try:
                self.usb.write('N;'.encode())
                time.sleep(0.05)
                # 重置后重新校准基准压力
                self.calibrate_base_pressure()
            except Exception as e:
                logging.error(f"串口重置指令失败: {e}")
        elif "i2c" == self.port:
            try:
                self.write_register('reset_probe', 1)
                self.calibrate_base_pressure()
            except Exception as e:
                logging.error(f"I2C重置指令失败: {e}")

    def get_status(self, eventtime=None):
        return {
            'state': "START" if self.last_state else "STOP",
            'base_pressure': self.pressure_base_value,
            'data_count': len(self.PA_data)
        }


def load_config_prefix(config):
    return BD_Pressure_Advance(config)
