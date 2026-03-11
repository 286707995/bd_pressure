import logging
import math
import statistics
import serial
import os


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


class BD_Pressure_Advance:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.port = config.get("port")
        self.old_res = ''
        self.thrhold = config.getint('thrhold', 4, minval=1) 
        self.toolhead = None  # 提前初始化toolhead变量
        
        if "i2c" in self.port:  
            self.i2c = bus.MCU_I2C_from_config(config, BDP_CHIP_ADDR, BDP_I2C_SPEED)
        elif "usb" in self.port:
            self.usb_port = config.get("serial")
            self._baud = config.getint('baud', 38400, minval=2400) 
            try:
                self.usb = serial.Serial(self.usb_port, self._baud, timeout=0.5)
                self.usb.reset_input_buffer()
                self.usb.reset_output_buffer()
            except Exception as e:
                logging.error(f"USB串口初始化失败: {str(e)}")
                self.usb = None
        
        self.PA_data = []    
        self.bd_name = config.get_name().split()[1]     
        self.gcode = self.printer.lookup_object('gcode')
        self._invert_stepper_x, self.mcu_enable_pin_x = self.enable_pin_init(config,"stepper_x")
        self._invert_stepper_x1, self.mcu_enable_pin_x1 = self.enable_pin_init(config,"stepper_x1")
        self._invert_stepper_y, self.mcu_enable_pin_y = self.enable_pin_init(config,"stepper_y")
        self._invert_stepper_y1, self.mcu_enable_pin_y1 = self.enable_pin_init(config,"stepper_y1")
        self.last_state = 0
        
        self.gcode.register_mux_command("SET_BDPRESSURE", "NAME", self.bd_name,
                                   self.cmd_SET_BDPRESSURE,
                                   desc=self.cmd_SET_BDPRESSURE_help)   
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler("homing:homing_move_begin", self.handle_homing_move_begin)                                        

    def set_probe_mode(self):
        """切换到Probe模式（仅模式切换，不设置阈值）"""
        if self.usb is None and "usb" in self.port:
            self.gcode.respond_info(f"[{self.bd_name}] USB串口未初始化，无法切换Probe模式")
            return
            
        if "usb" == self.port:
            try:
                # 传感器指令：切换到Probe模式 e;\n
                self.usb.write('e;\n'.encode())
                self.toolhead.dwell(0.1)  # 延时确保模式切换完成
                self.usb.reset_input_buffer()
                self.gcode.respond_info(f"[{self.bd_name}] 已切换到Probe模式")
            except Exception as e:
                self.gcode.respond_info(f"[{self.bd_name}] 切换Probe模式失败: {str(e)}")
        elif "i2c" == self.port: 
            self.write_register('pa_probe_mode', 2)
            self.write_register('probe_thr', self.thrhold)

    def set_threshold(self, new_threshold):
        """即时设置阈值核心函数（适配传感器USB指令格式）"""
        # 校验阈值合法性
        if not isinstance(new_threshold, int) or new_threshold < 1:
            self.gcode.respond_info(f"[{self.bd_name}] 阈值错误：必须是≥1的整数，当前值：{new_threshold}")
            return False
        
        # 更新本地阈值变量
        self.thrhold = new_threshold
        self.gcode.respond_info(f"[{self.bd_name}] 本地阈值th已更新为：{self.thrhold}")
        
        # USB模式：按传感器指令规则下发阈值
        if "usb" == self.port:
            if self.usb is None:
                self.gcode.respond_info(f"[{self.bd_name}] USB串口未初始化，无法下发阈值")
                return False
                
            try:
                # 步骤1：先切换到Probe模式（设置阈值的前提）
                self.usb.write('e;\n'.encode())
                self.toolhead.dwell(0.1)
                
                # 步骤2：单独发送阈值指令（传感器要求单独发送数字;\\n）
                self.usb.write(f'{self.thrhold};\n'.encode())
                self.toolhead.dwell(0.05)  # 延长延时确保传感器接收
                
                # 清空串口缓存，避免指令堆积
                self.usb.reset_input_buffer()
                self.gcode.respond_info(f"[{self.bd_name}] 新阈值{self.thrhold}已按官方格式下发！")
            except Exception as e:
                self.gcode.respond_info(f"[{self.bd_name}] 阈值下发失败：{str(e)}")
                return False
        # I2C模式逻辑不变
        elif "i2c" == self.port:
            self.write_register('probe_thr', self.thrhold)
            self.gcode.respond_info(f"[{self.bd_name}] 新阈值已写入I2C寄存器！")
        
        return True

    def _handle_ready(self):
        """Klipper就绪时初始化并下发初始阈值"""
        self.toolhead = self.printer.lookup_object('toolhead')
        # 先切换Probe模式，再下发初始阈值
        self.set_probe_mode()
        if "usb" == self.port:
            try:
                self.usb.write(f'{self.thrhold};\n'.encode())
                self.toolhead.dwell(0.2)
                self.gcode.respond_info(f"[{self.bd_name}] 初始阈值{self.thrhold}已下发")
            except Exception as e:
                self.gcode.respond_info(f"[{self.bd_name}] 初始阈值下发失败: {str(e)}")
        
    def handle_homing_move_begin(self, hmove):
        """归位时重新切换Probe模式"""
        self.set_probe_mode()        
        
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
        response = ""
        
        if "usb" == self.port:
            if self.usb is None:
                self.gcode.respond_info(f"[{self.bd_name}] USB串口未初始化，无法启动PA模式")
                return
                
            try:
                # 步骤1：切换到PA模式 l;\n
                self.usb.write('l;\n'.encode())
                self.toolhead.dwell(0.4)
                self.usb.reset_input_buffer()
                self.usb.reset_output_buffer()
                
                # 步骤2：再次发送PA模式指令确保切换成功
                self.usb.write('l;\n'.encode())
                self.toolhead.dwell(0.4)
                
                # 步骤3：启用原始数据输出 d;\n（传感器要求）
                self.usb.write('d;\n'.encode())
                self.toolhead.dwell(0.4) 
                
                # 读取传感器响应
                response += self.usb.readline().decode('utf-8').strip()
                while self.usb.in_waiting:
                    self.usb.read(self.usb.in_waiting)
                    
                self.gcode.respond_info(f"[{self.bd_name}] PA模式启动成功，传感器响应：{response}")
            except Exception as e:
                self.gcode.respond_info(f"[{self.bd_name}] PA模式启动失败：{str(e)}")
        elif "i2c" == self.port: 
            self.write_register('pa_probe_mode', 7)
            self.write_register('raw_data_out', 0)
            response += self.read_register('_version', 15).decode('utf-8')
            
        self.gcode.respond_info(f".cmd_start {self.port}: {response}") 

    def pa_data_process(self, gcmd, str_data):
        self.gcode.respond_info(f"{self.bd_name}: {str_data}")
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
        
        if "usb" == self.port:
            if self.usb is None or not self.usb.is_open:
                self.gcode.respond_info(f"[{self.bd_name}] USB串口未打开，无法读取数据")
                return False
                
            self.usb.timeout = 1
            try:
                response = self.usb.read(self.usb.in_waiting or 1).decode('utf-8').strip() 
            except Exception as e:
                self.gcode.respond_info(f"[{self.bd_name}] 读取数据失败：{str(e)}")
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
        response = ""
        
        if "usb" == self.port:
            if self.usb is None:
                self.gcode.respond_info(f"[{self.bd_name}] USB串口未初始化，无法停止PA模式")
                return
                
            try:
                # 步骤1：切换回Probe模式 e;\n
                self.usb.write('e;\n'.encode())
                self.toolhead.dwell(0.1)
                
                # 步骤2：发送当前阈值（确保传感器保留最新阈值）
                self.usb.write(f'{self.thrhold};\n'.encode())
                self.toolhead.dwell(0.1)
                
                # 步骤3：禁用原始数据输出 D;\n
                self.usb.write('D;\n'.encode())
                self.toolhead.dwell(0.4)
                
                response += self.usb.readline().decode('utf-8').strip()
            except Exception as e:
                self.gcode.respond_info(f"[{self.bd_name}] 停止PA模式失败：{str(e)}")
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
                self.gcode.respond_info("Calc the best Pressure Advance error!")  
                return
                
            min_r = self.PA_data[-1]   
            for s_pa in self.PA_data[min_index:]:
                if (min_r[1] + abs(min_r[5])) > (s_pa[1] + abs(s_pa[5])):
                    min_r = s_pa
            min_s = min_r      
            
            self.gcode.respond_info(f"Calc the best Pressure Advance: {min_s[0]}, {min_s[1]} {min_index}")  
            set_pa = f'SET_PRESSURE_ADVANCE ADVANCE={min_s[0]}'
            self.gcode.run_script_from_command(set_pa)
        else:
            self.gcode.respond_info("No PA calibration data or number is <=5") 
         
    def cmd_reset_probe(self, gcmd):
        if self.toolhead is None:
            self.toolhead = self.printer.lookup_object('toolhead')
            
        response = ""
        if "usb" == self.port:
            if self.usb is None:
                self.gcode.respond_info(f"[{self.bd_name}] USB串口未初始化，无法重置传感器")
                return
                
            try:
                self.usb.write('N;\n'.encode())  # 重置指令加换行符
                self.toolhead.dwell(0.2)
                response += self.usb.readline().decode('utf-8').strip()
                self.gcode.respond_info(f"[{self.bd_name}] 传感器重置成功，响应：{response}")
            except Exception as e:
                self.gcode.respond_info(f"[{self.bd_name}] 重置传感器失败：{str(e)}")
        elif "i2c" == self.port: 
            self.write_register('reset_probe', 1)

    def get_status(self, eventtime=None):
        """返回当前状态和阈值"""
        status = {
            'state': "START" if self.last_state else "STOP",
            'current_threshold': self.thrhold
        }
        return status        

def load_config_prefix(config):
    return BD_Pressure_Advance(config)
