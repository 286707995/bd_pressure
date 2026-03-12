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
                self.usb = serial.Serial(self.usb_port, self._baud, timeout=0.1)  # 缩短超时
                self.usb.reset_input_buffer()
                self.usb.reset_output_buffer()
            except Exception as e:
                logging.error(f"串口初始化失败: {e}")
                self.usb = None

        self.PA_data = []
        self.bd_name = config.get_name().split()[1]
        self.gcode = self.printer.lookup_object('gcode')
        
        # 电机使能引脚初始化
        self._invert_stepper_x, self.mcu_enable_pin_x = self.enable_pin_init(config, "stepper_x")
        self._invert_stepper_x1, self.mcu_enable_pin_x1 = self.enable_pin_init(config, "stepper_x1")
        self._invert_stepper_y, self.mcu_enable_pin_y = self.enable_pin_init(config, "stepper_y")
        self._invert_stepper_y1, self.mcu_enable_pin_y1 = self.enable_pin_init(config, "stepper_y1")
        
        self.last_state = 0
        self.motor_locked = True  # 电机锁定状态标记
        self.callback_registered = False  # 回调注册标记

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
                time.sleep(0.05)  # 非阻塞延时
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

    def handle_homing_move_begin(self, hmove):
        self.set_probe_mode()

    cmd_SET_BDPRESSURE_help = "cmd for BD_PRESSURE sensor,SET_BDPRESSURE NAME=xxx COMMAND=START/STOP/RESET_PROBE/READ VALUE=X"

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

    def _resend_current_val(self, eventtime):
        if self.last_value == self.shutdown_value:
            self.reactor.unregister_timer(self.resend_timer)
            self.resend_timer = None
            return self.reactor.NEVER

        systime = self.reactor.monotonic()
        print_time = self.mcu_enable_pin_x.get_mcu().estimated_print_time(systime)
        time_diff = (self.last_print_time + self.resend_interval) - print_time
        if time_diff > 0.:
            return systime + time_diff
        self._set_pin(print_time + PIN_MIN_TIME, self.last_value, True)
        return systime + self.resend_interval

    def enable_pin_init(self, config, stepper_name):
        stconfig = config.getsection(stepper_name)
        if stconfig is None:
            return None, None
        enable_pin_s = stconfig.get('enable_pin', None)
        if enable_pin_s is None:
            return None, None
        logging.info(f"init {stepper_name}")
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')

        pin_params = ppins.lookup_pin(enable_pin_s, can_invert=True, can_pullup=True, share_type='stepper_enable')
        mcu_pin_s = pin_params['chip'].setup_pin('digital_out', pin_params)
        _invert_stepper = pin_params['invert']

        self.scale = 1.
        self.last_print_time = 0.
        self.reactor = self.printer.get_reactor()
        self.resend_timer = None
        self.resend_interval = 0.
        max_mcu_duration = config.getfloat('maximum_mcu_duration', 0.,
                                           minval=0.500,
                                           maxval=MAX_SCHEDULE_TIME)
        mcu_pin_s.setup_max_duration(max_mcu_duration)
        if max_mcu_duration:
            config.deprecate('maximum_mcu_duration')
            self.resend_interval = max_mcu_duration - RESEND_HOST_TIME

        # 修正：电机默认锁定（enable_pin为HIGH）
        static_value = not _invert_stepper  # 关键修复：反转逻辑，默认锁定电机
        self.last_value = self.shutdown_value = static_value / self.scale
        mcu_pin_s.setup_start_value(self.last_value, self.shutdown_value)
        return _invert_stepper, mcu_pin_s

    def _set_pin(self, print_time, value, is_resend=False):
        if value == self.last_value and not is_resend:
            return
        print_time = max(print_time, self.last_print_time + PIN_MIN_TIME)
        
        # 只在引脚存在时操作
        if self.mcu_enable_pin_x:
            self.mcu_enable_pin_x.set_digital(print_time, value)
        if self.mcu_enable_pin_y:
            self.mcu_enable_pin_y.set_digital(print_time, value)
        if self.mcu_enable_pin_x1:
            self.mcu_enable_pin_x1.set_digital(print_time, value)
        if self.mcu_enable_pin_y1:
            self.mcu_enable_pin_y1.set_digital(print_time, value)
            
        self.last_value = value
        self.last_print_time = print_time
        
        if self.resend_interval and self.resend_timer is None:
            self.resend_timer = self.reactor.register_timer(
                self._resend_current_val, self.reactor.NOW)


def cmd_start(self, gcmd):
    if self.toolhead is None:
        self.toolhead = self.printer.lookup_object('toolhead')

    # 关键修复1：强制锁定电机（避免误触发）
    self.motor_locked = True
    self.last_state = 1

    # 关键修复2：清空串口缓冲区（避免旧指令干扰）
    response = ""
    if self.usb and "usb" == self.port:
        try:
            # 先清空缓冲区
            self.usb.reset_input_buffer()
            self.usb.reset_output_buffer()
            # 分步发送PA模式指令（确保固件解析）
            self.usb.write('l;'.encode())
            time.sleep(0.1)  # 延长延时，确保固件解析
            self.usb.write('D;'.encode())
            time.sleep(0.1)
            # 读取响应确认模式
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

    def pa_data_process(self, gcmd, str_data):
        self.gcode.respond_info(f"{self.bd_name}: {str_data}")
        if 'R:' in str_data and ',' in str_data:
            R_v = str_data.strip().split('R:')[1].split(',')
            if len(R_v) == 5:
                try:
                    res = int(R_v[0])
                    lk = int(R_v[1])
                    rk = int(R_v[2])
                    Hk = int(R_v[3])
                    Ha = int(R_v[4].split('\n')[0])
                    val_step = float(gcmd.get('VALUE', 0))
                    pa_val = [val_step, res, lk, rk, Hk, Ha]
                    self.PA_data.append(pa_val)
                except ValueError as e:
                    logging.error(f"数据解析失败: {e}")
                    return

                num = len(self.PA_data)
                flag = 1
                if num >= 20:
                    for s_pa in self.PA_data[num - 5:]:
                        if s_pa[4] < 2 or s_pa[5] < 5:
                            flag = 0
                            break
                    if flag == 1:
                        self.stop_pa(gcmd)
        elif 'stop' in str_data:
            self.last_state = 0

    def cmd_read(self, gcmd):
        self.bdw_data = ''
        response = ""

        if self.usb and "usb" == self.port:
            try:
                if self.usb.is_open:
                    # 非阻塞读取
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

        # 恢复电机默认状态
        self.motor_locked = True
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


def cmd_stop(self, gcmd):
    self.stop_pa(gcmd)
    if len(self.PA_data) >= 5:
        # 移除前5个不稳定数据
        del self.PA_data[:5]

        # 关键修复：过滤无效数据（Hk/Ha为0或异常值）
        valid_data = [d for d in self.PA_data if d[4] > 0 and d[5] > 0]  # d[4]=Hk, d[5]=Ha
        if not valid_data:
            self.gcode.respond_info("No valid PA data (Hk/Ha=0)")
            return

        min_s = valid_data[-1]
        min_index = len(valid_data) - 1

        # 查找最小阈值索引
        for index, s_pa in enumerate(reversed(valid_data)):
            if s_pa[4] < 5:
                min_index = len(valid_data) - 1 - index
                break
        if min_index == len(valid_data) - 1:
            for index, s_pa in enumerate(reversed(valid_data)):
                if s_pa[5] < 5:
                    min_index = len(valid_data) - 1 - index
                    break

        if min_index == len(valid_data) - 1:
            self.gcode.respond_info("Calc the best Pressure Advance error! (no Hk/Ha <5)")
            return

        min_r = valid_data[-1]
        for s_pa in valid_data[min_index:]:
            if (min_r[1] + abs(min_r[5])) > (s_pa[1] + abs(s_pa[5])):
                min_r = s_pa
        min_s = min_r

        self.gcode.respond_info(f"Calc the best Pressure Advance: {min_s[0]:f}, {min_s[1]} {min_index}")
        set_pa = f'SET_PRESSURE_ADVANCE ADVANCE={min_s[0]}'
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
            except Exception as e:
                logging.error(f"串口重置指令失败: {e}")
        elif "i2c" == self.port:
            try:
                self.write_register('reset_probe', 1)
            except Exception as e:
                logging.error(f"I2C重置指令失败: {e}")

    def get_status(self, eventtime=None):
        return {
            'state': "START" if self.last_state else "STOP",
            'motor_locked': self.motor_locked
        }


def load_config_prefix(config):
    return BD_Pressure_Advance(config)
