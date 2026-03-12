## klipper 


#### 1. Install software module
```
cd  ~
git clone https://github.com/markniu/bd_pressure.git
chmod 777 ~/bd_pressure/klipper/install.sh
~/bd_pressure/klipper/install.sh
```

#### 2. Configure Klipper

Add [include bd_pressure.cfg] into the printer.cfg , and modify the pins to your actual use in the section [probe] and [bdpressure bd_pa]

#### 3. OrcaSlicer:

1. Disable the Pressure advance in the Material settings.

2. Add the following G-code lines into the beginning of the Start_Gcode in the slicer, then it will do pressure advance calibration with your setting and automatically set the right PA value. 
```
G28                    ; Home all the axis
G1 Z30                 ; move to the poop position
G1 X240 Y240   
PA_CALIBRATE NOZZLE_TEMP=[first_layer_temperature] MAX_VOLUMETRIC=[filament_max_volumetric_speed] ACC_WALL=[outer_wall_acceleration]  TRAVEL_SPEED=[travel_speed]  ACC_TO_DECEL_FACTOR=[accel_to_decel_factor]
```


















import logging
import math
import statistics
import serial
import os
import time
import glob

from . import bus
from . import filament_switch_sensor


BDP_CHIP_ADDR = 4
BDP_I2C_SPEED = 100000  # 修正为I2C标准速度100000
BDP_REGS = {
     '_version' : 0x0,
     '_measure_data' : 15,
      'pa_probe_mode' : 48, ## 7= CLOCK_OSR_16384  2=CLOCK_OSR_512
     'raw_data_out' : 49,
     'probe_thr' : 50,
     'rang' : 51,
     'reset_probe' : 52,
     'invert_data' : 53
}

PIN_MIN_TIME = 0.100
RESEND_HOST_TIME = 0.300 + PIN_MIN_TIME
MAX_SCHEDULE_TIME = 5.0

# 串口配置常量
SERIAL_TIMEOUT = 1.0
SERIAL_WRITE_TIMEOUT = 1.0
RETRY_COUNT = 3  # 串口初始化重试次数
RETRY_DELAY = 0.5  # 重试延时


class BD_Pressure_Advance:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.port = config.get("port")
        self.old_res = ''
        self.thrhold = config.getint('thrhold', 4, minval=1) 
        self.toolhead = None  # 提前初始化toolhead变量
        self.usb = None  # 初始化串口对象为None
        self.usb_port = None
        self._baud = None
        self.gcode = None  # 先初始化gcode属性为None
        self.bd_name = config.get_name().split()[1]  # 提前初始化bd_name
        
        # 1. 先初始化gcode（关键修复：调整初始化顺序）
        self.gcode = self.printer.lookup_object('gcode')
        
        # 2. 初始化步进电机引脚（需要gcode之前初始化）
        self._invert_stepper_x, self.mcu_enable_pin_x = self.enable_pin_init(config,"stepper_x")
        self._invert_stepper_x1, self.mcu_enable_pin_x1 = self.enable_pin_init(config,"stepper_x1")
        self._invert_stepper_y, self.mcu_enable_pin_y = self.enable_pin_init(config,"stepper_y")
        self._invert_stepper_y1, self.mcu_enable_pin_y1 = self.enable_pin_init(config,"stepper_y1")
        self.last_state = 0
        
        # 3. 初始化串口（现在gcode已存在）
        if "i2c" in self.port:  
            self.i2c = bus.MCU_I2C_from_config(config, BDP_CHIP_ADDR, BDP_I2C_SPEED)
        elif "usb" in self.port:
            # 保存串口配置，便于后续重连
            self.usb_port = config.get("serial")
            self._baud = config.getint('baud', 38400, minval=2400) 
            # 初始化串口（带重试机制）
            self._init_usb_serial()
        
        self.PA_data = []    
        
        # 4. 注册GCODE指令（最后注册）
        self.gcode.register_mux_command("SET_BDPRESSURE", "NAME", self.bd_name,
                                   self.cmd_SET_BDPRESSURE,
                                   desc=self.cmd_SET_BDPRESSURE_help)   
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler("homing:homing_move_begin", self.handle_homing_move_begin)
    
    def _log_info(self, message):
        """安全的日志输出函数，兼容gcode未初始化的情况"""
        if self.gcode is not None:
            self.gcode.respond_info(f"[{self.bd_name}] {message}")
        else:
            # 回退到logging模块输出
            logging.info(f"[{self.bd_name}] {message}")
    
    def _init_usb_serial(self):
        """初始化USB串口（带重试和权限检查）"""
        if not self.usb_port or not self._baud:
            self._log_info("串口配置不完整，无法初始化")
            return False
            
        # 检查串口设备是否存在
        if not os.path.exists(self.usb_port):
            # 尝试自动查找FTDI设备
            ftdi_devs = glob.glob('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART*')
            if ftdi_devs:
                self.usb_port = ftdi_devs[0]
                self._log_info(f"自动找到FTDI设备：{self.usb_port}")
            else:
                self._log_info(f"串口设备不存在：{self.usb_port}")
                return False
        
        # 检查串口权限
        try:
            if not os.access(self.usb_port, os.R_OK | os.W_OK):
                self._log_info(f"串口权限不足：{self.usb_port}")
                # 尝试添加权限（需要root）
                os.system(f"chmod 666 {self.usb_port}")
        except Exception as e:
            self._log_info(f"检查串口权限失败：{str(e)}")
        
        # 多次重试初始化串口
        for retry in range(RETRY_COUNT):
            try:
                # 关闭已存在的串口连接
                if self.usb and self.usb.is_open:
                    self.usb.close()
                
                # 初始化串口
                self.usb = serial.Serial(
                    port=self.usb_port,
                    baudrate=self._baud,
                    timeout=SERIAL_TIMEOUT,
                    write_timeout=SERIAL_WRITE_TIMEOUT,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False
                )
                
                # 清空缓存
                self.usb.reset_input_buffer()
                self.usb.reset_output_buffer()
                
                self._log_info(f"串口初始化成功（第{retry+1}次尝试）：{self.usb_port} @ {self._baud}")
                return True
                
            except Exception as e:
                self._log_info(f"串口初始化失败（第{retry+1}次尝试）：{str(e)}")
                time.sleep(RETRY_DELAY)
        
        self._log_info(f"串口初始化最终失败，已重试{RETRY_COUNT}次")
        self.usb = None
        return False
    
    def _ensure_usb_connected(self):
        """确保串口已连接，断开则自动重连"""
        if "i2c" in self.port:
            return True
            
        # 检查串口状态
        if self.usb is None or not self.usb.is_open:
            self._log_info("串口未连接，尝试重连...")
            return self._init_usb_serial()
        
        # 检查串口是否可用
        try:
            # 发送空指令测试串口
            self.usb.write(b';\n')
            self.usb.flush()
            return True
        except Exception as e:
            self._log_info(f"串口连接异常：{str(e)}，尝试重连...")
            return self._init_usb_serial()
    
    def _send_usb_command(self, cmd, desc="指令"):
        """通用USB指令发送函数，带错误处理和响应验证"""
        # 确保串口已连接
        if not self._ensure_usb_connected():
            self._log_info(f"串口未连接，{desc}发送失败")
            return None
        
        try:
            # 清空输入缓存，避免旧数据干扰
            self.usb.reset_input_buffer()
            # 发送指令
            self.usb.write(cmd.encode())
            self.usb.flush()  # 确保指令完全发送
            # 等待传感器响应
            time.sleep(0.2)  # 增加稳定延时
            # 读取响应
            response = ""
            while self.usb.in_waiting > 0:
                response += self.usb.read(self.usb.in_waiting).decode('utf-8', errors='ignore').strip()
                time.sleep(0.05)  # 分段读取，避免数据截断
            self._log_info(f"{desc}发送成功: {cmd.strip()} | 响应: {response}")
            return response
        except Exception as e:
            self._log_info(f"{desc}发送失败: {str(e)}")
            # 重置串口对象，触发下次重连
            self.usb = None
            return None

    def set_probe_mode(self):
        """切换到Probe模式（仅模式切换，不设置阈值）"""
        if "usb" == self.port:
            # 发送Probe模式指令，带换行符
            self._send_usb_command('e;\n', "切换Probe模式")
        elif "i2c" == self.port: 
            self.write_register('pa_probe_mode', 2)
            self.write_register('probe_thr', self.thrhold)

    def set_threshold(self, new_threshold):
        """即时设置阈值核心函数（适配传感器USB指令格式）"""
        # 校验阈值合法性
        if not isinstance(new_threshold, int) or new_threshold < 1:
            self._log_info(f"阈值错误：必须是≥1的整数，当前值：{new_threshold}")
            return False
        
        # 更新本地阈值变量
        self.thrhold = new_threshold
        self._log_info(f"本地阈值th已更新为：{self.thrhold}")
        
        # USB模式：按传感器指令规则下发阈值
        if "usb" == self.port:
            # 步骤1：先切换到Probe模式（设置阈值的前提）
            self._send_usb_command('e;\n', "切换Probe模式(设置阈值前置)")
            
            # 步骤2：发送阈值指令（传感器要求单独发送数字;\\n）
            threshold_cmd = f'{self.thrhold};\n'
            response = self._send_usb_command(threshold_cmd, f"设置阈值{self.thrhold}")
            
            # 验证阈值是否设置成功
            if response and f"THRHOLD_Z: {self.thrhold}" in response:
                self._log_info(f"阈值{self.thrhold}设置成功并验证通过！")
            elif response:
                self._log_info(f"阈值下发成功，但响应未验证: {response}")
            else:
                self._log_info(f"阈值下发后未收到响应，可能设置失败")
                return False
        # I2C模式逻辑不变
        elif "i2c" == self.port:
            self.write_register('probe_thr', self.thrhold)
            self._log_info(f"新阈值已写入I2C寄存器！")
        
        return True

    def _handle_ready(self):
        """Klipper就绪时初始化并下发初始阈值"""
        self.toolhead = self.printer.lookup_object('toolhead')
        # 确保串口已连接
        if "usb" in self.port:
            self._ensure_usb_connected()
        # 先切换Probe模式，再下发初始阈值
        self.set_probe_mode()
        if "usb" in self.port:
            # 显式发送初始阈值
            self.set_threshold(self.thrhold)
        
    def handle_homing_move_begin(self, hmove):
        """归位时重新切换Probe模式并重置阈值"""
        # 确保串口连接
        self._ensure_usb_connected()
        # 切换Probe模式
        self.set_probe_mode()
        # 归位时重新下发当前阈值，避免阈值丢失
        if "usb" == self.port:
            self.set_threshold(self.thrhold)        
        
    # 扩展指令说明，新增SET_THRESHOLD命令
    cmd_SET_BDPRESSURE_help = "cmd for BD_PRESSURE sensor,SET_BDPRESSURE NAME=xxx COMMAND=START/STOP/RESET_PROBE/READ/SET_THRESHOLD VALUE=X"
    def cmd_SET_BDPRESSURE(self, gcmd):
        cmd = gcmd.get('COMMAND')
        
        # 处理阈值设置指令
        if 'SET_THRESHOLD' in cmd:
            new_thr = gcmd.get_int('VALUE', minval=1)
            self.set_threshold(new_thr)
            return
            
        # 原有命令逻辑
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
        
        max_mcu_duration = config.getfloat('maximum_mcu_duration', 0., minval=0.500, maxval=MAX_SCHEDULE_TIME)
        mcu_pin_s.setup_max_duration(max_mcu_duration)
        if max_mcu_duration:
            config.deprecate('maximum_mcu_duration')
            self.resend_interval = max_mcu_duration - RESEND_HOST_TIME
            
        static_value = (_invert_stepper == True)
        self.last_value = self.shutdown_value = static_value / self.scale
        mcu_pin_s.setup_start_value(self.last_value, self.shutdown_value)
        
        return _invert_stepper, mcu_pin_s
                                       
    def _set_pin(self, print_time, value, is_resend=False):
        if value == self.last_value and not is_resend:
            return
        print_time = max(print_time, self.last_print_time + PIN_MIN_TIME)
        self.mcu_enable_pin_x.set_digital(print_time, value)
        self.mcu_enable_pin_y.set_digital(print_time, value)
        
        if self.mcu_enable_pin_x1 is not None:
            self.mcu_enable_pin_x1.set_digital(print_time, value) 
        if self.mcu_enable_pin_y1 is not None:
            self.mcu_enable_pin_y1.set_digital(print_time, value) 
            
        self.last_value = value
        self.last_print_time = print_time
        
        if self.resend_interval and self.resend_timer is None:
            self.resend_timer = self.reactor.register_timer(self._resend_current_val, self.reactor.NOW)        
    
    def cmd_start(self, gcmd):
        """启动PA模式（适配传感器USB指令）"""
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead')
            
        self.toolhead.register_lookahead_callback(
               lambda print_time: self._set_pin(print_time, self._invert_stepper_x==False))  

        self.PA_data = [] 
        self.last_state = 1
        
        if "usb" == self.port:
            # 步骤1：切换到PA模式
            self._send_usb_command('l;\n', "切换PA模式")
            # 步骤2：启用原始数据输出
            self._send_usb_command('d;\n', "启用原始数据输出")
        elif "i2c" == self.port: 
            self.write_register('pa_probe_mode', 7)
            self.write_register('raw_data_out', 0)
            response = self.read_register('_version', 15).decode('utf-8')
            self._log_info(f".cmd_start {self.port}: {response}") 

    def pa_data_process(self, gcmd, str_data):
        self._log_info(f"{str_data}")
        if 'R:' in str_data and ',' in str_data:
            R_v = str_data.strip().split('R:')[1].split(',')
            if len(R_v) == 5:                
                res = int(R_v[0])
                lk = int(R_v[1])
                rk = int(R_v[2])
                Hk = int(R_v[3])
                Ha = int(R_v[4].split('\n')[0])
                val_step = float(gcmd.get('VALUE'))
                pa_val = [val_step, res, lk, rk, Hk, Ha]
                self.PA_data.append(pa_val)
                
            num = len(self.PA_data)
            flag = 1
            if num >= 20:
                for s_pa in self.PA_data[num-5:]:
                    if s_pa[4] < 2 or s_pa[5] < 5:
                        flag = 0
                        break
                if flag == 1:         
                    self.stop_pa(gcmd)
        elif 'stop' in str_data:
            self.last_state = 0
                        
    def cmd_read(self, gcmd):    
        self.bdw_data = ''    
        buffer = bytearray()
        response = ""
        
        # 确保串口连接
        if not self._ensure_usb_connected():
            self._log_info("串口未连接，读取数据失败")
            return False
                
        if "usb" == self.port:
            try:
                # 发送空指令触发传感器返回状态
                self.usb.write(';\n'.encode())
                time.sleep(0.1)
                # 读取所有响应数据
                response = self.usb.read(self.usb.in_waiting or 1024).decode('utf-8', errors='ignore').strip() 
            except Exception as e:
                self._log_info(f"读取数据失败：{str(e)}")
                # 重置串口对象，触发下次重连
                self.usb = None
                return False
                
            if response:
                self.old_res = response
                self.pa_data_process(gcmd, response)
            else:
                self.pa_data_process(gcmd, self.old_res)
        elif "i2c" == self.port:
            response = self.read_register('_measure_data', 32).decode('utf-8').strip('\0')
            self.pa_data_process(gcmd, response)
            
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
        """停止PA模式（切回Probe模式+禁用原始数据）"""
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead')
            
        self.toolhead.register_lookahead_callback(
                lambda print_time: self._set_pin(print_time, self._invert_stepper_x==True))
                
        self.last_state = 0     
        
        if "usb" == self.port:
            # 步骤1：切换回Probe模式
            self._send_usb_command('e;\n', "切换回Probe模式")
            # 步骤2：禁用原始数据输出
            self._send_usb_command('D;\n', "禁用原始数据输出")
            # 步骤3：重新下发当前阈值，确保阈值不丢失
            self.set_threshold(self.thrhold)
        elif "i2c" == self.port: 
            self.write_register('pa_probe_mode', 2)
            self.write_register('probe_thr', self.thrhold)
            self.write_register('raw_data_out', 0)

    def cmd_stop(self, gcmd):
        self.stop_pa(gcmd)     
        
        if len(self.PA_data) >= 5: 
            # 移除前5个数据点
            self.PA_data.pop(0)
            self.PA_data.pop(1)
            self.PA_data.pop(2)
            self.PA_data.pop(3)
            self.PA_data.pop(4)
            
            min_s = self.PA_data[-1]  
            min_index = len(self.PA_data)-1
            
            # 查找最小阈值索引
            for index, s_pa in enumerate(reversed(self.PA_data)):
                if s_pa[4] < 5:
                    min_index = len(self.PA_data)-1 - index
                    break
            if min_index == len(self.PA_data)-1:
                for index, s_pa in enumerate(reversed(self.PA_data)):
                    if s_pa[5] < 5:
                        min_index = len(self.PA_data)-1 - index
                        break   
                        
            if min_index == len(self.PA_data)-1:
                self._log_info("Calc the best Pressure Advance error!")  
                return
                
            min_r = self.PA_data[-1]   
            for s_pa in self.PA_data[min_index:]:
                if (min_r[1] + abs(min_r[5])) > (s_pa[1] + abs(s_pa[5])):
                    min_r = s_pa
            min_s = min_r      
            
            self._log_info(f"Calc the best Pressure Advance: {min_s[0]}, {min_s[1]} {min_index}")  
            set_pa = f'SET_PRESSURE_ADVANCE ADVANCE={min_s[0]}'
            self.gcode.run_script_from_command(set_pa)
        else:
            self._log_info("No PA calibration data or number is <=5") 
         
    def cmd_reset_probe(self, gcmd):
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead')
            
        if "usb" == self.port:
            # 确保串口连接
            if not self._ensure_usb_connected():
                return
            # 步骤1：发送重置指令
            response = self._send_usb_command('N;\n', "重置传感器基准值")
            # 步骤2：重置后重新下发当前阈值，避免阈值回退
            if response:
                self.set_threshold(self.thrhold)
        elif "i2c" == self.port: 
            self.write_register('reset_probe', 1)

    def get_status(self, eventtime=None):
        """返回当前状态和阈值"""
        status = {
            'state': "START" if self.last_state else "STOP",
            'current_threshold': self.thrhold,
            'usb_connected': self.usb is not None and self.usb.is_open if "usb" in self.port else True
        }
        return status        

def load_config_prefix(config):
    return BD_Pressure_Advance(config)








