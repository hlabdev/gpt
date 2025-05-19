#!/usr/bin/env python3
"""
CSP Test Mode - Wecon VD3E with Absolute Move, Multi-axis Support, and Tkinter GUI
"""
import pysoem
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk
import logging

logging.basicConfig(
    filename="controller.log",
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)

class CSPAxis:
    """Represents a single axis and provides motion-control helpers."""
    def __init__(self, slave):
        self.slave = slave
        self.target_position = 0
        self.actual_position = 0
        self.running = True
        self.jog_direction = 0
        self.jog_step = 10000
        self.control_word = 0x0006  # Initial state: Ready to switch on
        # Add profile parameters for motion control
        self.profile_velocity = 1000     # Default velocity (encoder counts/sec)
        self.profile_acceleration = 1000  # Default acceleration (encoder counts/sec^2)
        self.profile_deceleration = 1000  # Default deceleration (encoder counts/sec^2)
        # เพิ่มค่า encoder resolution สำหรับแปลงเป็น RPM
        self.encoder_resolution = 10000  # จำนวน encoder counts ต่อ 1 รอบ
        # เพิ่มตัวแปรเก็บสถานะว่า Servo เปิดอยู่หรือไม่
        self.servo_enabled = False
        # เพิ่มตัวแปรเก็บค่า offset ตำแหน่ง สำหรับกรณีที่ไม่สามารถตั้งค่า offset ใน drive ได้
        self.position_offset = 0
        # เพิ่มตัวแปรเก็บค่าความเร็วจริงที่อ่านได้จากมอเตอร์
        self.actual_velocity = 0
        # เพิ่มตัวคูณปรับสเกลสำหรับค่าความเร็วจริง
        self.velocity_scale_factor = 0.001  # ค่าเริ่มต้น ปรับให้เหมาะกับไดรฟ์ของคุณ
        # เพิ่มตัวแปรสำหรับการ scaling หน่วยที่แสดงในแอพพลิเคชัน
        self.app_units_per_rev = 3600  # จำนวน units ในแอพพลิเคชันต่อ 1 รอบ (ตามที่ต้องการ)
        self.motor_direction_inverted = False  # ตัวแปรเก็บสถานะการกลับทิศทางมอเตอร์
        # เพิ่มตัวแปรเก็บค่าโหมดการหยุด (0 = Free Stop, 1 = Zero-Speed Stop)
        self.stop_mode = 0  # ค่าเริ่มต้นคือ Free Stop

    def send_pdo(self):
        try:
            # คำนวณตำแหน่งเป้าหมายจริง โดยไม่ปรับค่า offset ในกรณี JOG
            if self.jog_direction != 0:
                # กรณี JOG ให้ใช้ตำแหน่งเป้าหมายปัจจุบัน + จำนวนก้าวของ JOG
                self.target_position += self.jog_direction * self.jog_step
                # ไม่มีการปรับด้วย offset เพราะจะทำให้การ JOG ผิดพลาด
                pos_to_send = self.target_position
            else:
                # กรณีอื่นๆ ไม่ต้องปรับค่า target_position เพราะจะถูกตั้งค่าจากภายนอกอยู่แล้ว
                pos_to_send = self.target_position
                
            pos_offset = 0  # PDO offset นี้จะถูกตั้งค่าที่ drive จึงไม่จำเป็นต้องใช้
            op_mode = 8
            outdata = struct.pack("<HiHb", self.control_word, pos_to_send, pos_offset, op_mode)
            out_buf = bytearray(self.slave.output)
            out_buf[:len(outdata)] = outdata
            self.slave.output = bytes(out_buf)
        except TypeError:
            print("[Warning] slave.output is immutable; skipping PDO write")

    def read_status(self):
        """อ่านสถานะและตำแหน่งปัจจุบัน พร้อมปรับค่าตามค่า offset ซอฟต์แวร์"""
        try:
            status = struct.unpack("<H", self.slave.sdo_read(0x6041, 0, 2))[0]
            self.actual_position = struct.unpack("<i", self.slave.sdo_read(0x6064, 0, 4))[0]
            
            # อ่านค่าความเร็วจริง (Actual Velocity)
            try:
                raw_velocity = struct.unpack("<i", self.slave.sdo_read(0x606C, 0, 4))[0]
                # ปรับค่าด้วยตัวคูณเพื่อให้ได้ค่าที่ถูกต้อง
                self.actual_velocity = int(raw_velocity * self.velocity_scale_factor)
            except Exception as e:
                print(f"Warning: Could not read actual velocity: {e}")
                self.actual_velocity = 0
                
            # หักลบด้วยค่า offset ที่บันทึกไว้ในโปรแกรม (ถ้ามี)
            if hasattr(self, 'position_offset') and self.position_offset != 0:
                # ปรับค่าตำแหน่งโดยใช้ค่า offset ที่บันทึกไว้
                adjusted_position = self.actual_position - self.position_offset
                return status, adjusted_position
                
            return status, self.actual_position
        except Exception as e:
            print(f"Error reading status: {e}")
            return 0, 0
            
    def get_actual_velocity_rpm(self):
        """แปลงค่าความเร็วจริงเป็น RPM"""
        return self.counts_to_rpm(self.actual_velocity)

    def move_absolute(self, position):
        try:
            # Check servo status
            try:
                status = struct.unpack("<H", self.slave.sdo_read(0x6041, 0, 2))[0]
                if (status & 0x006F) != 0x0027:  # Check if operation enabled
                    print("Warning: Servo is not enabled. Please use Servo On button first.")
                    return
            except Exception as e:
                print(f"Error checking servo status: {e}")
                return
            
            # Set motion parameters
            self.set_motion_parameters()
            
            # Set new target position
            self.target_position = position
            self.slave.sdo_write(0x607A, 0, struct.pack("<i", position))
            time.sleep(0.1)
            
            # Enable operation with new setpoint (bit 4 set)
            self.control_word = 0x000F | 0x0010  # 0x001F - Operation enabled + new setpoint
            self.slave.sdo_write(0x6040, 0, struct.pack("<H", self.control_word))
            time.sleep(0.1)
            
            # Toggle bit 4 to acknowledge setpoint
            self.control_word = 0x000F  # Operation enabled, clear new setpoint bit
            self.slave.sdo_write(0x6040, 0, struct.pack("<H", self.control_word))
            
            # Also update via PDO for immediate effect
            outdata = struct.pack("<HiHb", self.control_word, self.target_position, 0, 8)
            out_buf = bytearray(self.slave.output)
            out_buf[:len(outdata)] = outdata
            self.slave.output = bytes(out_buf)
        except Exception as e:
            print(f"Error in move_absolute: {e}")
            
    def set_motion_parameters(self):
        """Set profile velocity, acceleration and deceleration on the drive"""
        try:
            # Set profile velocity (0x6081)
            self.slave.sdo_write(0x6081, 0, struct.pack("<i", self.profile_velocity))
            
            # Set profile acceleration (0x6083)
            self.slave.sdo_write(0x6083, 0, struct.pack("<i", self.profile_acceleration))
            
            # Set profile deceleration (0x6084)
            self.slave.sdo_write(0x6084, 0, struct.pack("<i", self.profile_deceleration))
        except Exception as e:
            print(f"Error setting motion parameters: {e}")
            
    def set_profile_velocity(self, velocity):
        """Set profile velocity (speed) parameter"""
        try:
            self.profile_velocity = velocity
            if update_base:
                self.base_profile_velocity = velocity
            self.slave.sdo_write(0x6081, 0, struct.pack("<i", velocity))
            return True
        except Exception as e:
            print(f"Error setting profile velocity: {e}")
            return False
            
    def set_profile_acceleration(self, acceleration):
        """Set profile acceleration parameter"""
        try:
            self.profile_acceleration = acceleration
            self.slave.sdo_write(0x6083, 0, struct.pack("<i", acceleration))
            return True
        except Exception as e:
            print(f"Error setting profile acceleration: {e}")
            return False
            
    def set_profile_deceleration(self, deceleration):
        """Set profile deceleration parameter"""
        try:
            self.profile_deceleration = deceleration
            self.slave.sdo_write(0x6084, 0, struct.pack("<i", deceleration))
            return True
        except Exception as e:
            print(f"Error setting profile deceleration: {e}")
            return False

    def emergency_stop(self):
        """หยุดฉุกเฉินทันที - ล็อกเพลา (Zero-Speed Stop)"""
        try:
            # ตั้ง Stop Mode เป็น Zero-Speed Stop
            try:
                self.slave.sdo_write(0x2000, 0x05, struct.pack("<h", 1))
                time.sleep(0.05)
            except Exception as e:
                print(f"Warning: Cannot set stop mode: {e}")
            # Quick Stop (ล็อกเพลา)
            self.control_word = 0x0002
            self.slave.sdo_write(0x6040, 0, struct.pack("<H", self.control_word))
            self.jog_direction = 0
            time.sleep(0.1)
        except Exception as e:
            print(f"Error in emergency_stop: {e}")
            
    def quick_stop(self):
        """Quick stop (หยุดทันที, Lock เพลา, Servo ยัง ON)"""
        try:
            # ตั้ง Stop Mode เป็น Zero-Speed Stop (ล็อกเพลา)
            try:
                self.slave.sdo_write(0x2000, 0x05, struct.pack("<h", 1))
                time.sleep(0.01)
                print("[quick_stop] Set stop mode to Zero-Speed Stop (0x2000:05 = 1)")
            except Exception as e:
                print(f"[quick_stop] Warning: Cannot set stop mode: {e}")
            # สั่ง Operation Enabled ก่อน (0x000F)
            self.control_word = 0x000F
            self.slave.sdo_write(0x6040, 0, struct.pack("<H", self.control_word))
            time.sleep(0.01)
            # สั่ง Quick Stop (clear bit 2) → 0x000B
            self.control_word = 0x000B
            self.slave.sdo_write(0x6040, 0, struct.pack("<H", self.control_word))
            self.jog_direction = 0
            self.send_pdo()
            time.sleep(0.05)
            print("[quick_stop] Quick stop command sent (0x000B, lock shaft, servo ON)")
            return True
        except Exception as e:
            print(f"[quick_stop] Error in quick_stop: {e}")
            return False

    def set_stop_mode(self, mode):
        """ตั้งค่าโหมดการหยุด (พารามิเตอร์ 2000-05)
        
        Parameters:
        ----------
        mode : int
            0 = Free Stop (หยุดและปล่อยเพลาหมุนอิสระ)
            1 = Zero-Speed Stop (หยุดและล็อกเพลา)
        """
        try:
            if mode not in [0, 1]:
                print(f"Invalid stop mode: {mode}, using default (0 = Free Stop)")
                mode = 0
                
            # บันทึกค่าโหมดการหยุดลงตัวแปรในคลาส
            self.stop_mode = mode
            
            # ตั้งค่าพารามิเตอร์ 2000-05 ด้วย SDO write
            self.slave.sdo_write(0x2000, 0x05, struct.pack("<h", mode))
            
            # แสดงผลค่าที่ตั้ง
            mode_name = "Free Stop (ไม่ล็อกเพลา)" if mode == 0 else "Zero-Speed Stop (ล็อกเพลา)"
            print(f"Set stop mode to: {mode} - {mode_name}")
            return True
        except Exception as e:
            print(f"Error setting stop mode: {e}")
            return False

    # ฟังก์ชันแปลงความเร็วจาก counts/sec เป็น RPM
    def counts_to_rpm(self, counts_per_sec):
        """Convert encoder counts per second to RPM"""
        if not hasattr(self, 'encoder_resolution') or self.encoder_resolution == 0:
            return 0
        # คำนวณ RPM = (counts/sec × 60) ÷ (encoder_resolution)
        rpm = (counts_per_sec * 60) / self.encoder_resolution
        return rpm

    # เพิ่มฟังก์ชัน rpm_to_counts
    def rpm_to_counts(self, rpm):
        """Convert RPM to encoder counts per second"""
        if not hasattr(self, 'encoder_resolution') or self.encoder_resolution == 0:
            return 0
        # คำนวณ counts/sec = (rpm × encoder_resolution) ÷ 60
        counts_per_sec = (rpm * self.encoder_resolution) / 60
        return int(counts_per_sec)
    
    def encoder_to_app_units(self, encoder_counts):
        """แปลงค่าจาก encoder counts เป็นหน่วยที่แสดงในแอพพลิเคชัน"""
        # สูตร: app_units = encoder_counts * (app_units_per_rev / encoder_resolution)
        app_units = encoder_counts * (self.app_units_per_rev / self.encoder_resolution)
        # ปรับทิศทางถ้าตั้งค่าเป็น inverted
        if self.motor_direction_inverted:
            app_units = -app_units
        return int(app_units)  # ส่งค่ากลับเป็นจำนวนเต็ม
    
    def app_to_encoder_units(self, app_units):
        """แปลงค่าจากหน่วยที่แสดงในแอพพลิเคชันเป็น encoder counts"""
        # ปรับทิศทางถ้าตั้งค่าเป็น inverted
        if self.motor_direction_inverted:
            app_units = -app_units
        # สูตร: encoder_counts = app_units * (encoder_resolution / app_units_per_rev)
        encoder_counts = app_units * (self.encoder_resolution / self.app_units_per_rev)
        return int(encoder_counts)  # ส่งค่ากลับเป็นจำนวนเต็ม

class CSPController:
    """Manages EtherCAT communication and the Tkinter GUI interface."""
    def __init__(self):
        self.master = pysoem.Master()
        self.axes = []
        self.running = False
        self.gui = None
        self.log_messages = []  # เก็บประวัติ log
        self.log_text = None    # จะถูกกำหนดใน launch_gui
        # เพิ่มค่าเริ่มต้นสำหรับความเร็ว JOG
        self.default_jog_speed = 10  # ค่าเริ่มต้น JOG speed
        # เพิ่มตัวแปรเก็บสถานะการเชื่อมต่อ
        self.is_connected = False
        self.connection_status_label = None

    def connect(self, adapter_name=None):
        adapters = pysoem.find_adapters()
        if not adapters:
            print("No adapters found")
            self.is_connected = False
            return False

        try:
            self.master.open(adapter_name or adapters[0].name)
            if self.master.config_init() <= 0:
                self.is_connected = False
                return False
            self.master.config_map()
            for s in self.master.slaves:
                axis = CSPAxis(s)
                self.axes.append(axis)
            self.master.state = pysoem.OP_STATE
            self.master.write_state()
            connection_result = self.master.state_check(pysoem.OP_STATE, 5000000)
            self.is_connected = connection_result
            return connection_result
        except Exception as e:
            print(f"Connection failed: {e}")
            self.is_connected = False
            return False

    # เพิ่มฟังก์ชันตรวจสอบสถานะการเชื่อมต่อ
    def check_connection_status(self):
        """ตรวจสอบสถานะการเชื่อมต่อ EtherCAT"""
        try:
            if not hasattr(self, 'master') or self.master is None:
                self.is_connected = False
                return False
                
            # ตรวจสอบว่ามี slaves และสามารถเข้าถึงได้หรือไม่
            slaves_ok = len(self.axes) > 0 and all(hasattr(axis, 'slave') for axis in self.axes)
            
            # ตรวจสอบการเชื่อมต่ออย่างง่ายโดยการอ่านค่าสถานะของ slave ตัวแรก
            if slaves_ok and len(self.axes) > 0:
                try:
                    # ลองอ่านค่าสถานะ (จะทำให้เกิด exception ถ้าไม่สามารถเชื่อมต่อได้)
                    axis = self.axes[0]
                    status = struct.unpack("<H", axis.slave.sdo_read(0x6041, 0, 2))[0]
                    # ถ้าอ่านค่าได้โดยไม่เกิด exception แสดงว่ายังเชื่อมต่ออยู่
                    self.is_connected = True
                    print(f"Connection check successful: status=0x{status:04X}")
                    return True
                except Exception as e:
                    print(f"Connection check failed: {e}")
                    self.is_connected = False
                    return False
            else:
                print(f"Connection check failed: No valid slaves found")
                self.is_connected = False
                return False
                
        except Exception as e:
            print(f"Error checking connection: {e}")
            self.is_connected = False
            return False
            
    # อัพเดทสถานะการเชื่อมต่อบน GUI
    def update_connection_status_display(self):
        """อัพเดทการแสดงสถานะการเชื่อมต่อบน GUI"""
        if not self.running:
            return
            
        # ตรวจสอบสถานะการเชื่อมต่อ
        print(f"[{time.strftime('%H:%M:%S')}] Checking connection status...")
        connection_status = self.check_connection_status()
        
        # อัพเดท GUI ถ้ามี label สำหรับแสดงสถานะการเชื่อมต่อ
        if hasattr(self, 'connection_status_label') and self.connection_status_label:
            try:
                if connection_status:
                    print(f"[{time.strftime('%H:%M:%S')}] Connection check: CONNECTED")
                    self.connection_status_label.config(text="CONNECTED", foreground="white", background="green")
                else:
                    print(f"[{time.strftime('%H:%M:%S')}] Connection check: DISCONNECTED")
                    self.connection_status_label.config(text="DISCONNECTED", foreground="white", background="red")
            except Exception as e:
                print(f"[{time.strftime('%H:%M:%S')}] Error updating connection status label: {e}")
                
        # ตั้งเวลาให้ตรวจสอบอีกครั้งหลังจากผ่านไป 5 วินาที
        if self.gui:
            self.gui.after(5000, self.update_connection_status_display)
    
    # เพิ่มฟังก์ชันอัพเดทข้อมูลการเชื่อมต่อในแท็บ Settings
    def update_connection_info(self):
        """อัพเดทข้อมูลการเชื่อมต่อในแท็บ Settings"""
        if not self.running:
            return
            
        try:
            # อัพเดทข้อมูล adapter
            if hasattr(self, 'adapter_label'):
                if hasattr(self, 'master') and self.master:
                    try:
                        adapter_name = self.master.ifname
                        if adapter_name:
                            self.adapter_label.config(text=adapter_name)
                        else:
                            self.adapter_label.config(text="Not connected")
                    except:
                        self.adapter_label.config(text="Not connected")
                else:
                    self.adapter_label.config(text="Not connected")
                    
            # อัพเดทจำนวน slaves
            if hasattr(self, 'slaves_label'):
                if hasattr(self, 'master') and self.master and hasattr(self.master, 'slaves'):
                    try:
                        num_slaves = len(self.master.slaves)
                        self.slaves_label.config(text=str(num_slaves))
                    except:
                        self.slaves_label.config(text="0")
                else:
                    self.slaves_label.config(text="0")
                    
            # อัพเดทสถานะการเชื่อมต่อ
            if hasattr(self, 'state_label'):
                if hasattr(self, 'master') and self.master:
                    try:
                        state = self.master.state
                        if state == pysoem.OP_STATE:
                            self.state_label.config(text="Operational (OP)", foreground="green")
                        elif state == pysoem.PREOP_STATE:
                            self.state_label.config(text="Pre-Operational (PREOP)", foreground="orange")
                        elif state == pysoem.SAFEOP_STATE:
                            self.state_label.config(text="Safe-Operational (SAFEOP)", foreground="blue")
                        elif state == pysoem.INIT_STATE:
                            self.state_label.config(text="Initialized (INIT)", foreground="red")
                        else:
                            self.state_label.config(text=f"Unknown ({state})", foreground="red")
                    except:
                        self.state_label.config(text="Unknown", foreground="red")
                else:
                    self.state_label.config(text="Not connected", foreground="red")
                    
            # ตั้งเวลาให้อัพเดทอีกครั้งหลังจากผ่านไป 5 วินาที
            if self.gui:
                self.gui.after(5000, self.update_connection_info)
                
        except Exception as e:
            print(f"Error updating connection info: {e}")
            # ตั้งเวลาให้ลองอัพเดทอีกครั้ง
            if self.gui:
                self.gui.after(5000, self.update_connection_info)

    def launch_gui(self):
        """เริ่มต้น GUI สำหรับควบคุมมอเตอร์"""
        print("Starting to launch GUI...")
        self.gui = tk.Tk()
        self.gui.title("CSP Axis Controller")
        self.gui.configure(bg="#f0f0f0")
        
        # กำหนดขนาดหน้าต่างเริ่มต้นให้เหมาะสม
        self.gui.geometry("1100x800")
        
        self.status_labels = []
        self.error_labels = []

        # ตั้งค่าการตอบสนองต่อการปิดหน้าต่าง - จะเรียกเมธอด stop ของคลาส
        self.gui.protocol("WM_DELETE_WINDOW", self.on_close)
        print("GUI window created successfully")

        # สร้าง style สำหรับ ttk widgets
        style = ttk.Style()
        style.configure("Title.TLabel", font=("Helvetica", 16, "bold"), background="#f0f0f0")
        style.configure("Header.TLabel", font=("Helvetica", 12, "bold"), background="#f0f0f0")
        style.configure("Info.TLabel", background="#f0f0f0")
        style.configure("Danger.TButton", foreground="white", background="red")
        style.configure("Exit.TButton", foreground="white", background="#d9534f")
        style.configure("StatusBar.TFrame", background="#f8f9fa")
        style.configure("StatusItem.TLabel", font=("Helvetica", 9), background="#f8f9fa")
        style.configure("StatusValue.TLabel", font=("Helvetica", 9, "bold"), background="#f8f9fa")

        # หัวข้อ
        title = ttk.Label(self.gui, text="EtherCAT CSP Multi-Axis Controller", style="Title.TLabel")
        title.grid(row=0, column=0, columnspan=12, pady=(20, 20), sticky="n")

        # สร้าง Notebook สำหรับแต่ละแกน
        notebook = ttk.Notebook(self.gui)
        notebook.grid(row=1, column=0, columnspan=12, padx=10, pady=10, sticky="nsew")
        
        # สร้าง Footer Status Bar
        footer_frame = ttk.Frame(self.gui, style="StatusBar.TFrame", relief=tk.GROOVE, borderwidth=1)
        footer_frame.grid(row=2, column=0, columnspan=12, sticky="ew", padx=0, pady=0)
        
        # จัดการให้ footer อยู่ติดด้านล่างเสมอ
        self.gui.grid_rowconfigure(1, weight=1)  # ให้ notebook ขยายตัวได้
        self.gui.grid_columnconfigure(0, weight=1)  # ให้คอลัมน์ขยายตัวได้
        
        # แบ่ง footer เป็นส่วนต่างๆ
        # 1. สถานะการเชื่อมต่อ
        conn_frame = ttk.Frame(footer_frame, style="StatusBar.TFrame")
        conn_frame.pack(side=tk.LEFT, padx=5, pady=2)
        ttk.Label(conn_frame, text="Connection:", style="StatusItem.TLabel").pack(side=tk.LEFT)
        self.connection_status_label = ttk.Label(conn_frame, text="CHECKING...", foreground="white", background="#f0ad4e", 
                                                width=12, anchor="center")
        self.connection_status_label.pack(side=tk.LEFT, padx=2)
        
        # Separator
        ttk.Separator(footer_frame, orient="vertical").pack(side=tk.LEFT, fill="y", padx=5, pady=2)
        
        # 2. สถานะ Axis
        axis_frame = ttk.Frame(footer_frame, style="StatusBar.TFrame")
        axis_frame.pack(side=tk.LEFT, padx=5, pady=2)
        ttk.Label(axis_frame, text="Position:", style="StatusItem.TLabel").pack(side=tk.LEFT)
        self.footer_position_label = ttk.Label(axis_frame, text="0", style="StatusValue.TLabel", width=10)
        self.footer_position_label.pack(side=tk.LEFT, padx=2)
        
        # Separator
        ttk.Separator(footer_frame, orient="vertical").pack(side=tk.LEFT, fill="y", padx=5, pady=2)
        
        # 3. สถานะ Servo
        servo_frame = ttk.Frame(footer_frame, style="StatusBar.TFrame")
        servo_frame.pack(side=tk.LEFT, padx=5, pady=2)
        ttk.Label(servo_frame, text="Servo:", style="StatusItem.TLabel").pack(side=tk.LEFT)
        self.footer_servo_status = ttk.Label(servo_frame, text="OFF", foreground="white", background="red", 
                                           width=4, anchor="center")
        self.footer_servo_status.pack(side=tk.LEFT, padx=2)
        
        # Separator
        ttk.Separator(footer_frame, orient="vertical").pack(side=tk.LEFT, fill="y", padx=5, pady=2)
        
        # 4. ความเร็ว
        speed_frame = ttk.Frame(footer_frame, style="StatusBar.TFrame")
        speed_frame.pack(side=tk.LEFT, padx=5, pady=2)
        ttk.Label(speed_frame, text="Speed:", style="StatusItem.TLabel").pack(side=tk.LEFT)
        # Increase width so both set and actual speed values remain visible
        self.footer_speed_label = ttk.Label(speed_frame, text="0 (0.0 RPM)", style="StatusValue.TLabel", width=30)
        self.footer_speed_label.pack(side=tk.LEFT, padx=2)
        
        # Separator
        ttk.Separator(footer_frame, orient="vertical").pack(side=tk.LEFT, fill="y", padx=5, pady=2)
        
        # 5. รหัสข้อผิดพลาด
        error_frame = ttk.Frame(footer_frame, style="StatusBar.TFrame")
        error_frame.pack(side=tk.LEFT, padx=5, pady=2)
        ttk.Label(error_frame, text="Error:", style="StatusItem.TLabel").pack(side=tk.LEFT)
        self.footer_error_label = ttk.Label(error_frame, text="0x0000", style="StatusValue.TLabel")
        self.footer_error_label.pack(side=tk.LEFT, padx=2)
        
        # Separator
        ttk.Separator(footer_frame, orient="vertical").pack(side=tk.LEFT, fill="y", padx=5, pady=2)
        
        # 6. เวลา (Timestamp)
        time_frame = ttk.Frame(footer_frame, style="StatusBar.TFrame")
        time_frame.pack(side=tk.RIGHT, padx=5, pady=2)
        self.footer_time_label = ttk.Label(time_frame, text=time.strftime("%H:%M:%S"), style="StatusValue.TLabel")
        self.footer_time_label.pack(side=tk.RIGHT)
        ttk.Label(time_frame, text="Time:", style="StatusItem.TLabel").pack(side=tk.RIGHT, padx=2)
        
        # เริ่มต้นการอัพเดทเวลา
        self.update_footer_time()
        
        # สร้าง Tab สำหรับ Settings
        settings_tab = ttk.Frame(notebook)
        notebook.add(settings_tab, text="Settings")
        
        # สร้างหน้า Settings
        settings_frame = ttk.LabelFrame(settings_tab, text="Motor Settings", padding=10)
        settings_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # ส่วนตั้งค่า Encoder Resolution และการ Scale
        enc_frame = ttk.LabelFrame(settings_frame, text="Encoder and Scaling Settings")
        enc_frame.pack(fill='x', padx=10, pady=10)
        
        # ส่วนตั้งค่า Motor Type (Rotary/Linear)
        motor_type_frame = ttk.Frame(enc_frame)
        motor_type_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(motor_type_frame, text="Motor Type:", font=("Helvetica", 12)).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        # วิตเจ็ต Radio Button สำหรับเลือกประเภทมอเตอร์
        motor_type_var = tk.StringVar(value="Rotary")
        ttk.Radiobutton(motor_type_frame, text="Rotary", variable=motor_type_var, value="Rotary").grid(row=0, column=1, padx=10, pady=5)
        ttk.Radiobutton(motor_type_frame, text="Linear", variable=motor_type_var, value="Linear").grid(row=0, column=2, padx=10, pady=5)
        
        # ส่วนตั้งค่า Encoder Resolution
        encoder_frame = ttk.Frame(enc_frame)
        encoder_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(encoder_frame, text="Encoder Resolution (counts/rev):", font=("Helvetica", 12)).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        # สร้างช่องกรอกค่า Encoder Resolution
        self.encoder_resolution_var = tk.StringVar(value="3600")
        encoder_entry = ttk.Entry(encoder_frame, textvariable=self.encoder_resolution_var, width=10)
        encoder_entry.grid(row=0, column=1, padx=5, pady=5)
        
        # ส่วนตั้งค่า Application Units
        app_units_frame = ttk.Frame(enc_frame)
        app_units_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(app_units_frame, text="Application Units (units/rev):", font=("Helvetica", 12)).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        # สร้างช่องกรอกค่า Application Units
        self.app_units_var = tk.StringVar(value="3600")
        app_units_entry = ttk.Entry(app_units_frame, textvariable=self.app_units_var, width=10)
        app_units_entry.grid(row=0, column=1, padx=5, pady=5)
        
        # ส่วนตั้งค่า Invert Direction
        invert_frame = ttk.Frame(enc_frame)
        invert_frame.pack(fill='x', padx=10, pady=5)
        
        # Checkbox สำหรับกลับทิศทางมอเตอร์
        self.invert_direction_var = tk.BooleanVar(value=False)
        invert_checkbox = ttk.Checkbutton(invert_frame, text="Invert motor direction", variable=self.invert_direction_var)
        invert_checkbox.pack(side=tk.LEFT, padx=5, pady=5)
        
        # ปุ่มตั้งค่าทั้งหมด
        apply_scaling_btn = ttk.Button(
            enc_frame, 
            text="Apply All Scaling Settings", 
            command=self.apply_scaling_settings
        )
        apply_scaling_btn.pack(side=tk.RIGHT, padx=10, pady=10)
        
                # คำอธิบาย        ttk.Label(            enc_frame,             text="Set the encoder resolution and application units per revolution.\nExample: 3600 encoder counts = 3600 application units",            font=("Helvetica", 9)        ).pack(padx=10, pady=5, anchor=tk.W)                # ส่วนตั้งค่า Encoder Resolution ที่มีอยู่เดิม (อาจย้ายหรือรวมกับส่วนใหม่)        # ... existing code ...                # เพิ่มส่วนตั้งค่าโหมดการหยุด        stop_mode_frame = ttk.LabelFrame(settings_frame, text="Stop Mode Settings", padding=10)        stop_mode_frame.pack(fill='x', padx=10, pady=10)                # คำอธิบาย        ttk.Label(            stop_mode_frame,            text="Select the emergency stop behavior for VD3E servo drive (Parameter 2000-05):",            font=("Helvetica", 10)        ).pack(padx=10, pady=5, anchor=tk.W)                # Radio buttons สำหรับเลือกโหมดการหยุด        self.stop_mode_var = tk.IntVar(value=0)  # ค่าเริ่มต้นคือ 0 (Free Stop)                mode_frame = ttk.Frame(stop_mode_frame)        mode_frame.pack(fill='x', padx=10, pady=5)                # ตัวเลือกสำหรับ Free Stop        free_stop_radio = ttk.Radiobutton(            mode_frame,            text="Free Stop (ไม่ล็อกเพลา: หยุดช้า แรงกระแทกน้อย)",            variable=self.stop_mode_var,            value=0        )        free_stop_radio.grid(row=0, column=0, padx=5, pady=2, sticky="w")                # ตัวเลือกสำหรับ Zero-Speed Stop        zero_stop_radio = ttk.Radiobutton(            mode_frame,            text="Zero-Speed Stop (ล็อกเพลา: หยุดเร็ว ป้องกันการหมุนหลังหยุด)",            variable=self.stop_mode_var,            value=1        )        zero_stop_radio.grid(row=1, column=0, padx=5, pady=2, sticky="w")                # ปุ่มตั้งค่าโหมดการหยุด        apply_stop_mode_btn = ttk.Button(            stop_mode_frame,             text="Apply Stop Mode",             command=self.apply_stop_mode        )        apply_stop_mode_btn.pack(side=tk.RIGHT, padx=10, pady=10)                # เพิ่มส่วนการตั้งค่าการเชื่อมต่อและแสดงข้อมูลอุปกรณ์ในหน้า Settings
        connection_settings_frame = ttk.LabelFrame(settings_frame, text="Connection Settings", padding=10)
        connection_settings_frame.pack(fill='x', padx=10, pady=10)
        
        # แสดงข้อมูลอุปกรณ์ที่เชื่อมต่ออยู่
        ttk.Label(connection_settings_frame, text="Connection Information:", font=("Helvetica", 12, "bold")).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        # สร้าง Frame สำหรับแสดงข้อมูลอุปกรณ์
        device_info_frame = ttk.Frame(connection_settings_frame)
        device_info_frame.grid(row=1, column=0, padx=5, pady=5, sticky="w")
        
        # แสดงข้อมูล Adapter
        ttk.Label(device_info_frame, text="Network Adapter:", font=("Helvetica", 10)).grid(row=0, column=0, padx=5, pady=2, sticky="w")
        self.adapter_label = ttk.Label(device_info_frame, text="Not connected", font=("Helvetica", 10))
        self.adapter_label.grid(row=0, column=1, padx=5, pady=2, sticky="w")
        
        # แสดงจำนวน Slaves
        ttk.Label(device_info_frame, text="Connected Slaves:", font=("Helvetica", 10)).grid(row=1, column=0, padx=5, pady=2, sticky="w")
        self.slaves_label = ttk.Label(device_info_frame, text="0", font=("Helvetica", 10))
        self.slaves_label.grid(row=1, column=1, padx=5, pady=2, sticky="w")
        
        # แสดงสถานะการเชื่อมต่อ
        ttk.Label(device_info_frame, text="Connection State:", font=("Helvetica", 10)).grid(row=2, column=0, padx=5, pady=2, sticky="w")
        self.state_label = ttk.Label(device_info_frame, text="Unknown", font=("Helvetica", 10))
        self.state_label.grid(row=2, column=1, padx=5, pady=2, sticky="w")
        
        # เพิ่มปุ่มเชื่อมต่อใหม่ในหน้า Settings
        reconnect_frame = ttk.Frame(connection_settings_frame)
        reconnect_frame.grid(row=2, column=0, padx=5, pady=10, sticky="w")
        
        ttk.Label(reconnect_frame, text="Hardware Connection:", font=("Helvetica", 12)).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        reconnect_btn = ttk.Button(
            reconnect_frame,
            text="Reconnect to Hardware",
            command=self.reconnect_hardware
        )
        reconnect_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # สร้าง Tab สำหรับ Log
        log_tab = ttk.Frame(notebook)
        notebook.add(log_tab, text="Operation Log")
        
        # ส่วนแสดง Log ในแท็บใหม่
        log_frame = ttk.LabelFrame(log_tab, text="System Log", padding=10)
        log_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # สร้าง Text widget และ Scrollbar สำหรับ log
        self.log_text = tk.Text(log_frame, height=20, width=80, bg="#f8f8f8", fg="#333333")
        self.log_text.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.config(yscrollcommand=scrollbar.set)
        
        # ทำให้ Text เป็น read-only
        self.log_text.config(state=tk.DISABLED)
        
        # สร้าง Frame สำหรับปุ่ม log controls
        log_btn_frame = ttk.Frame(log_frame)
        log_btn_frame.grid(row=0, column=2, padx=5, pady=5, sticky="ns")
        
        # ปุ่มล้าง Log
        clear_log_btn = ttk.Button(log_btn_frame, text="Clear Log", 
                                command=lambda: [self.clear_log(), self.add_to_log("Log cleared")])
        clear_log_btn.grid(row=0, column=0, padx=2, pady=2)
        
        # ปุ่ม Copy Log
        copy_log_btn = ttk.Button(log_btn_frame, text="Copy Log", 
                                command=self.copy_log_to_clipboard)
        copy_log_btn.grid(row=1, column=0, padx=2, pady=2)
        
        # อัพเดท log ด้วยข้อความเริ่มต้น
        self.add_to_log("System initialized")
        
        # ทำให้ log frame ขยายเมื่อมีการปรับขนาดหน้าต่าง
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # เริ่มต้น try-except block สำหรับการสร้าง Tab แกน
        try:
            # สร้าง Tab สำหรับแต่ละแกน
            for i, axis in enumerate(self.axes):
                # สร้าง Tab
                tab = ttk.Frame(notebook)
                notebook.add(tab, text=f"Axis {i}")
                
                # สร้าง Scrollbar
                main_canvas = tk.Canvas(tab, width=1050, height=700)
                main_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)
                
                scrollbar = ttk.Scrollbar(tab, orient=tk.VERTICAL, command=main_canvas.yview)
                scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
                
                main_canvas.configure(yscrollcommand=scrollbar.set)
                
                # สร้างกรอบส่วนประกอบหลัก
                main_frame = ttk.Frame(main_canvas)
                
                # กำหนดขนาดของ main_frame ให้เหมาะกับเนื้อหา
                main_canvas.create_window((0, 0), window=main_frame, anchor="nw")
                
                # อัพเดทขนาดของ scrollregion เมื่อขนาดของ main_frame เปลี่ยนแปลง
                def _configure_frame(event):
                    # อัพเดทขนาดของ scrollregion ให้เท่ากับขนาดของ main_frame
                    main_canvas.configure(scrollregion=main_canvas.bbox("all"))
                
                main_frame.bind("<Configure>", _configure_frame)
                
                # เพิ่มการรองรับ mousewheel scrolling
                def _on_mousewheel(event):
                    main_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
                    
                main_canvas.bind_all("<MouseWheel>", _on_mousewheel)
                
                # สร้างเลย์เอาต์เป็น 2 คอลัมน์
                left_column = ttk.Frame(main_frame)
                left_column.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
                
                right_column = ttk.Frame(main_frame)
                right_column.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
                
                # แถวที่ 0: ปุ่มหยุดฉุกเฉิน, ปุ่ม Servo On, ปุ่ม Reset
                row1_frame = ttk.Frame(left_column)
                row1_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
                
                # สร้างปุ่ม Stop (ค่อยๆ ลดความเร็ว) เป็นสีเหลือง-ส้ม
                emergency_btn = tk.Button(
                    row1_frame, 
                    text="EMERGENCY\nSTOP", 
                    bg="#8B0000", 
                    fg="white", 
                    font=("Helvetica", 12, "bold"),
                    width=15, 
                    height=3, 
                    relief=tk.RAISED,
                    command=lambda ax=axis: self.emergency_stop_only(ax)
                )
                emergency_btn.grid(row=0, column=0, padx=10, pady=5)
                
                # สร้างปุ่ม Servo On เป็นสีเขียว
                servo_on_btn = tk.Button(
                    row1_frame, 
                    text="SERVO\nON", 
                    bg="#28a745", 
                    fg="white", 
                    font=("Helvetica", 12, "bold"),
                    width=15, 
                    height=3, 
                    relief=tk.RAISED,
                    command=lambda ax=axis: self.servo_on_axis(ax)
                )
                servo_on_btn.grid(row=0, column=1, padx=10, pady=5)
                
                # สร้างปุ่ม Reset เป็นสีฟ้า
                reset_btn = tk.Button(
                    row1_frame, 
                    text="RESET\nERROR", 
                    bg="#2f74d0", 
                    fg="white", 
                    font=("Helvetica", 12, "bold"),
                    width=15, 
                    height=3, 
                    relief=tk.RAISED,
                    command=lambda ax=axis: self.reset_error(ax)
                )
                reset_btn.grid(row=0, column=2, padx=10, pady=5)
                
                # สร้างปุ่ม Set Zero คู่กับ Reset (ย้ายมาจากที่อื่น)
                set_zero_btn = tk.Button(
                    row1_frame, 
                    text="SET\nZERO", 
                    bg="#17a2b8", 
                    fg="white", 
                    font=("Helvetica", 12, "bold"),
                    width=15, 
                    height=3, 
                    relief=tk.RAISED,
                    command=lambda ax=axis: self.set_zero(ax)
                )
                set_zero_btn.grid(row=0, column=3, padx=10, pady=5)
                
                # เพิ่ม Speed Control Box ด้านบนของแท็บและใต้ปุ่ม Servo
                speed_control_box = ttk.LabelFrame(left_column, text="Speed Control")
                speed_control_box.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
                
                # ใช้ speed control panel ใหม่ที่รองรับ override แบบ realtime
                speed_panel = self.create_speed_control_panel(speed_control_box, axis)
                speed_panel.pack(fill="x", padx=10, pady=5)
                
                # เพิ่มคำอธิบายว่า Speed Control นี้ใช้ได้กับทุกการเคลื่อนที่
                ttk.Label(speed_control_box, 
                         text="This speed control applies to all movement types (JOG, Absolute, Relative)",
                         font=("Helvetica", 9, "italic")).pack(padx=10, pady=5, anchor=tk.W)
                
                # สร้าง Notebook หลักสำหรับแท็บควบคุมทั้งหมด
                control_notebook = ttk.Notebook(left_column)
                control_notebook.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
                
                # สร้าง Tab Motion Control
                motion_control_tab = ttk.Frame(control_notebook)
                control_notebook.add(motion_control_tab, text="Motion Control")
                
                # สร้างเนื้อหาใน Motion Control Tab
                motion_content = ttk.Frame(motion_control_tab)
                motion_content.pack(fill="both", expand=True, padx=10, pady=10)
                
                # ส่วนควบคุม Velocity Slider
                velocity_frame = ttk.LabelFrame(motion_content, text="Velocity Control")
                velocity_frame.pack(fill="x", padx=10, pady=5)
                
                velocity_slider_frame = ttk.Frame(velocity_frame)
                velocity_slider_frame.pack(fill="x", padx=10, pady=5)
                
                ttk.Label(velocity_slider_frame, text="Velocity:", font=("Helvetica", 12)).grid(row=0, column=0, padx=5, pady=5, sticky="w")
                
                speed_var = tk.IntVar(value=0)
                speed_label = ttk.Label(velocity_slider_frame, text="0 (0 RPM)", width=12, font=("Helvetica", 12, "bold"))
                speed_label.grid(row=0, column=2, padx=5, pady=5, sticky="e")
                
                speed_slider = ttk.Scale(
                    velocity_slider_frame, 
                    from_=-100, 
                    to=100, 
                    orient=tk.HORIZONTAL, 
                    variable=speed_var, 
                    length=300
                )
                speed_slider.grid(row=0, column=1, padx=10, pady=5, sticky="ew")
                
                # ปุ่มควบคุมการเคลื่อนที่
                direction_frame = ttk.LabelFrame(motion_content, text="Direction Control")
                direction_frame.pack(fill="x", padx=10, pady=10)
                
                button_frame = ttk.Frame(direction_frame)
                button_frame.pack(fill="x", padx=10, pady=10)
                
                # ปุ่ม Backward (ทิศทางลบ)
                backward_btn = tk.Button(
                    button_frame, 
                    text="◀ BACKWARD", 
                    bg="#6c757d", 
                    fg="white", 
                    font=("Helvetica", 12, "bold"),
                    width=12, 
                    height=2,
                    command=lambda ax=axis: self.set_backward_motion(ax, speed_var)
                )
                backward_btn.grid(row=0, column=0, padx=5, pady=5)
                
                # ปุ่ม Stop (แทนที่ Stop ทั่วไป) - หยุดฉุกเฉินและเปิด servo กลับ
                emergency_btn = tk.Button(
                    button_frame, 
                    text="STOP", 
                    bg="#8B0000",
                    fg="white", 
                    font=("Helvetica", 12, "bold"),
                    width=15, 
                    height=2,
                    command=lambda ax=axis: self.free_stop_and_servo_on(ax)
                )
                emergency_btn.grid(row=0, column=1, padx=5, pady=5)
                
                # ปุ่ม Forward (ทิศทางบวก)
                forward_btn = tk.Button(
                    button_frame, 
                    text="FORWARD ▶", 
                    bg="#6c757d", 
                    fg="white", 
                    font=("Helvetica", 12, "bold"),
                    width=12, 
                    height=2,
                    command=lambda ax=axis: self.set_forward_motion(ax, speed_var)
                )
                forward_btn.grid(row=0, column=2, padx=5, pady=5)
                
                # # The old CONTROLLED STOP button has been removed
                
                # สร้าง Tab JOG Control
                jog_control_tab = ttk.Frame(control_notebook)
                control_notebook.add(jog_control_tab, text="JOG Control")
                
                # สร้างเนื้อหาใน JOG Control Tab
                jog_content = ttk.LabelFrame(jog_control_tab, text="JOG Control (Press and Hold)")
                jog_content.pack(fill="both", expand=True, padx=10, pady=10)
                
                # เพิ่มป้ายอธิบายการใช้งาน JOG
                ttk.Label(jog_content, text="Press and hold the JOG buttons below to move the motor", font=("Helvetica", 9, "italic")).pack(padx=10, pady=5)
                
                # สร้างแถวสำหรับปุ่ม JOG
                jog_buttons_frame = ttk.Frame(jog_content)
                jog_buttons_frame.pack(fill="x", padx=10, pady=10)
                
                # สร้างปุ่ม JOG- (ทิศทางลบ ขนาดใหญ่)
                jog_neg_btn = tk.Button(
                    jog_buttons_frame,
                    text="◀◀ JOG BACKWARD",
                    bg="#4b89dc",
                    fg="white",
                    font=("Helvetica", 14, "bold"),
                    width=18,
                    height=3,
                    relief=tk.RAISED,
                    activebackground="#3a6ab2",
                    activeforeground="white"
                )
                # กำหนดการกดและปล่อยปุ่ม
                try:
                    jog_neg_btn.bind("<ButtonPress>", lambda e, ax=axis: self.start_jog_negative_with_speed(ax))
                    jog_neg_btn.bind("<ButtonRelease>", lambda e, ax=axis: self.stop_jog_with_ramp(ax))
                    print(f"Successfully bound events to JOG BACKWARD button")
                except Exception as e:
                    print(f"Error binding JOG BACKWARD events: {e}")
                jog_neg_btn.grid(row=0, column=0, padx=10, pady=5)
                
                # ปุ่ม JOG+ (ทิศทางบวก ขนาดใหญ่)
                jog_pos_btn = tk.Button(
                    jog_buttons_frame,
                    text="JOG FORWARD ▶▶",
                    bg="#4b89dc",
                    fg="white",
                    font=("Helvetica", 14, "bold"),
                    width=18,
                    height=3,
                    relief=tk.RAISED,
                    activebackground="#3a6ab2",
                    activeforeground="white"
                )
                # กำหนดการกดและปล่อยปุ่ม
                try:
                    jog_pos_btn.bind("<ButtonPress>", lambda e, ax=axis: self.start_jog_positive_with_speed(ax))
                    jog_pos_btn.bind("<ButtonRelease>", lambda e, ax=axis: self.stop_jog_with_ramp(ax))
                    print(f"Successfully bound events to JOG FORWARD button")
                except Exception as e:
                    print(f"Error binding JOG FORWARD events: {e}")
                jog_pos_btn.grid(row=0, column=1, padx=10, pady=5)
                
                # เพิ่มแถวสำหรับปุ่มฟังก์ชันพิเศษเกี่ยวกับตำแหน่ง
                position_control_frame = ttk.Frame(jog_content)
                position_control_frame.pack(fill="x", padx=10, pady=10)
                
                # เพิ่มป้ายคำอธิบาย
                # ttk.Label(position_control_frame, text="Position Functions:", font=("Helvetica", 10, "bold")).pack(side=tk.TOP, anchor="w", padx=5, pady=5)
                
                # # สร้างปุ่ม Set Zero ขนาดใหญ่
                # set_zero_big_btn = tk.Button(
                #     position_control_frame,
                #     text="SET CURRENT POSITION TO ZERO",
                #     bg="#17a2b8",
                #     fg="white",
                #     font=("Helvetica", 12, "bold"),
                #     width=30,
                #     height=2,
                #     relief=tk.RAISED,
                #     command=lambda ax=axis: self.set_zero(ax)
                # )
                # set_zero_big_btn.pack(side=tk.TOP, pady=5)
                
                # สร้าง Tab Absolute Move
                abs_move_tab = ttk.Frame(control_notebook)
                control_notebook.add(abs_move_tab, text="Absolute Move")
                
                # สร้างเนื้อหาใน Absolute Move Tab
                abs_move_frame = self.create_absolute_move_tab(abs_move_tab, axis)
                
                # สร้าง Tab Relative Move
                rel_move_tab = ttk.Frame(control_notebook)
                control_notebook.add(rel_move_tab, text="Relative Move")
                
                # สร้างเนื้อหาใน Relative Move Tab
                rel_move_frame = self.create_relative_move_tab(rel_move_tab, axis)
                
                # เพิ่มการอัพเดทสถานะสำหรับ tabs
                def update_move_tabs_status():
                    if not self.running:
                        return
                        
                    try:
                        # อ่านตำแหน่งปัจจุบัน
                        status, pos = axis.read_status()
                        
                        # อัพเดทตำแหน่งใน Absolute Move tab
                        if hasattr(axis, 'abs_move_refs') and 'current_pos' in axis.abs_move_refs:
                            axis.abs_move_refs['current_pos'].config(text=str(pos))
                            
                        # อัพเดทตำแหน่งใน Relative Move tab
                        if hasattr(axis, 'rel_move_refs') and 'current_pos' in axis.rel_move_refs:
                            axis.rel_move_refs['current_pos'].config(text=str(pos))
                            
                    except Exception as e:
                        print(f"Error updating move tabs: {e}")
                        
                    # กำหนดให้อัพเดทอีกครั้งหลังจากผ่านไป 0.5 วินาที
                    self.gui.after(500, update_move_tabs_status)
                    
                # เริ่มการอัพเดทสถานะ
                self.gui.after(500, update_move_tabs_status)
                
                # ทำให้ left_column ขยายตัวตามขนาดของหน้าต่าง
                left_column.columnconfigure(0, weight=1)
                left_column.rowconfigure(3, weight=1)  # ให้ control_notebook ขยายตัวเต็มพื้นที่
                
                # เก็บ reference ของ labels เพื่ออัพเดทข้อมูล
                axis.gui_refs = {
                    'servo_button': servo_on_btn,
                    'jog_pos_btn': jog_pos_btn,
                    'jog_neg_btn': jog_neg_btn,
                }
                
                # ลบ right_column เพราะย้ายทุกอย่างไปอยู่ในแท็บแล้ว
                
                print(f"Tab organization created successfully for axis {i}")
                
                # เพิ่มฟังก์ชัน update_axis_status ที่หายไป
                def update_axis_status(axis_id=0):  # กำหนดให้ใช้ axis_id=0 เสมอเพราะมีเพียง 1 axis
                    if not self.running:
                        return
                    print(f"Updating status for axis {axis_id}...")    
                    try:
                        # ตรวจสอบว่า axis_id ไม่เกินขนาดของลิสต์
                        if axis_id >= len(self.axes):
                            print(f"Error: axis_id {axis_id} out of range (0-{len(self.axes)-1})")
                            return
                            
                        # ดึงข้อมูลสถานะของ axis
                        axis = self.axes[axis_id]
                        
                        # ถ้าไม่มีการเก็บ reference ของ GUI elements ไว้ ให้ข้าม
                        if not hasattr(axis, 'gui_refs'):
                            print(f"Warning: axis {axis_id} has no gui_refs")
                            return
                        
                        # อัพเดทสถานะปุ่ม Servo
                        if axis.servo_enabled:
                            print(f"Axis {axis_id} servo is ENABLED")
                            if 'servo_button' in axis.gui_refs:
                                axis.gui_refs['servo_button'].config(text="SERVO\nOFF", bg="#dc3545")
                            
                            # อัพเดท footer servo status
                            if hasattr(self, 'footer_servo_status'):
                                self.footer_servo_status.config(text="ON", foreground="white", background="green")
                        else:
                            print(f"Axis {axis_id} servo is DISABLED")
                            if 'servo_button' in axis.gui_refs:
                                axis.gui_refs['servo_button'].config(text="SERVO\nON", bg="#28a745")
                            
                            # อัพเดท footer servo status
                            if hasattr(self, 'footer_servo_status'):
                                self.footer_servo_status.config(text="OFF", foreground="white", background="red")
                        
                        # อ่านสถานะปัจจุบัน
                        try:
                            status, pos = axis.read_status()
                            
                            # อัพเดท footer position
                            if hasattr(self, 'footer_position_label'):
                                self.footer_position_label.config(text=str(pos))
                            
                            # แสดงความเร็วที่ตั้งค่าไว้
                            rpm = axis.counts_to_rpm(axis.profile_velocity)

                            # แสดงความเร็วจริงที่อ่านได้จากมอเตอร์
                            actual_rpm = axis.get_actual_velocity_rpm()

                            # แสดงทั้งความเร็วที่ตั้งค่าและความเร็วจริง
                            velocity_text = (
                                f"Set: {axis.profile_velocity} | "
                                f"Actual: {axis.actual_velocity} ({actual_rpm:.1f} RPM)"
                            )
                            
                            # อัพเดท footer speed
                            if hasattr(self, 'footer_speed_label'):
                                self.footer_speed_label.config(text=velocity_text)
                            
                            # อัพเดทโหมดการทำงาน
                            try:
                                # อ่านโหมดการทำงานปัจจุบัน
                                op_mode = struct.unpack("b", axis.slave.sdo_read(0x6061, 0, 1))[0]
                                mode_name = "Unknown"
                                
                                # แปลรหัสโหมดเป็นข้อความ
                                if op_mode == 8:
                                    mode_name = "CSP"
                                elif op_mode == 9:
                                    mode_name = "CSV"
                                elif op_mode == 10:
                                    mode_name = "CST"
                                elif op_mode == 1:
                                    mode_name = "PP"
                                elif op_mode == 3:
                                    mode_name = "PV"
                                elif op_mode == 4:
                                    mode_name = "PT"
                                elif op_mode == 6:
                                    mode_name = "HM"
                                
                                # สร้างข้อความแสดงโหมด
                                mode_text = f"{mode_name} ({op_mode})"
                                
                                # อัพเดทใน footer
                                if hasattr(self, 'footer_mode_label'):
                                    self.footer_mode_label.config(text=mode_text)
                                    
                            except Exception as e:
                                print(f"Error reading operation mode: {e}")
                                # ใช้ค่าเริ่มต้น CSP Mode
                                if hasattr(self, 'footer_mode_label'):
                                    self.footer_mode_label.config(text="CSP (8)")
                                    
                            # Highlight buttons while jogging
                            if axis.jog_direction > 0 and 'jog_pos_btn' in axis.gui_refs:
                                axis.gui_refs['jog_pos_btn'].config(relief=tk.SUNKEN, bg="#3a6ab2")
                            elif axis.jog_direction < 0 and 'jog_neg_btn' in axis.gui_refs:
                                axis.gui_refs['jog_neg_btn'].config(relief=tk.SUNKEN, bg="#3a6ab2")
                            elif axis.jog_direction == 0:
                                if 'jog_pos_btn' in axis.gui_refs:
                                    axis.gui_refs['jog_pos_btn'].config(relief=tk.RAISED, bg="#4b89dc")
                                if 'jog_neg_btn' in axis.gui_refs:
                                    axis.gui_refs['jog_neg_btn'].config(relief=tk.RAISED, bg="#4b89dc")
                            
                            # อ่านและแสดงรหัสข้อผิดพลาด
                            try:
                                err = struct.unpack("<H", axis.slave.sdo_read(0x603F, 0, 2))[0]
                                err_text = f"0x{err:04X}"
                                if err != 0:
                                    # อัพเดท footer error
                                    if hasattr(self, 'footer_error_label'):
                                        self.footer_error_label.config(text=err_text, foreground="red")
                                else:
                                    # อัพเดท footer error
                                    if hasattr(self, 'footer_error_label'):
                                        self.footer_error_label.config(text=err_text, foreground="black")
                            except Exception:
                                pass
                            
                        except Exception as e:
                            print(f"Error reading status: {e}")
                            
                    except Exception as e:
                        print(f"Error updating axis status: {e}")
                        
                    # กำหนดให้อัพเดทอีกครั้งหลังจากผ่านไป 1 วินาที
                    self.gui.after(1000, update_axis_status, axis_id)
                
                # เริ่มการอัพเดทสถานะโดยใช้ค่า index ที่ถูกต้อง
                self.gui.after(1000, lambda: update_axis_status(0))  # เรียกใช้กับ axis_id=0 เสมอ
                
                # พิมพ์ข้อความแสดงสถานะการสร้าง tab
                print(f"Tab for axis {i} created successfully - starting status updates with fixed index 0")
                
        except Exception as e:
            error_msg = f"Error in launch_gui: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            
        try:
            # แก้ไขส่วนนี้
            # ทำการตรวจสอบสถานะการเชื่อมต่อทันที
            print(f"[{time.strftime('%H:%M:%S')}] Initial connection status check...")
            connection_status = self.check_connection_status()
            if hasattr(self, 'connection_status_label') and self.connection_status_label:
                if connection_status:
                    print(f"[{time.strftime('%H:%M:%S')}] Initial connection status: CONNECTED")
                    self.connection_status_label.config(text="CONNECTED", foreground="white", background="green")
                else:
                    print(f"[{time.strftime('%H:%M:%S')}] Initial connection status: DISCONNECTED")
                    self.connection_status_label.config(text="DISCONNECTED", foreground="white", background="red")
            
            # ตั้งเวลาให้ตรวจสอบความต่อเนื่องหลังจากนี้
            self.gui.after(5000, self.update_connection_status_display)
            
            # อัพเดทข้อมูลการเชื่อมต่อในแท็บ Settings
            self.update_connection_info()
            
            # เริ่มต้นรอบ mainloop ของ GUI (ทำเพียงครั้งเดียว)
            self.gui.mainloop()
        except Exception as e:
            error_msg = f"Error in launch_gui: {e}"
            print(error_msg)
            self.add_to_log(error_msg)

    def reconnect_hardware(self):
        """พยายามเชื่อมต่อกับฮาร์ดแวร์อีกครั้ง"""
        try:
            # แสดงข้อความกำลังเชื่อมต่อ
            self.add_to_log("Attempting to reconnect to EtherCAT hardware...")
            
            # อัพเดทสถานะการเชื่อมต่อให้แสดงว่ากำลังเชื่อมต่อ
            if hasattr(self, 'connection_status_label') and self.connection_status_label:
                self.connection_status_label.config(text="CONNECTING...", foreground="white", background="#f0ad4e")
                
            # ช่วยให้ GUI ได้อัพเดท
            if self.gui:
                self.gui.update_idletasks()
            
            # หยุดการทำงานปัจจุบัน
            self.running = False
            time.sleep(0.2)
            
            # ล้างรายการ axes เดิม
            self.axes = []
            
            # ปิดการเชื่อมต่อเดิม
            try:
                self.master.state = pysoem.INIT_STATE
                self.master.write_state()
                time.sleep(0.1)
                self.master.close()
            except Exception as e:
                self.add_to_log(f"Error closing previous connection: {e}")
            
            # เริ่มต้นเชื่อมต่อใหม่
            connection_success = self.connect("\\Device\\NPF_{D11C2F0E-5365-4007-B7C6-9C025AAF0799}")
            
            if connection_success:
                self.setup_all_axes()
                self.start_loop()
                self.add_to_log("Successfully reconnected to EtherCAT hardware")
                
                # อัพเดทสถานะการเชื่อมต่อใหม่ทันที
                if hasattr(self, 'connection_status_label') and self.connection_status_label:
                    self.connection_status_label.config(text="CONNECTED", foreground="white", background="green")
                    
                # อัพเดทข้อมูลการเชื่อมต่อในแท็บ Settings
                self.update_connection_info()
                
                return True
            else:
                self.add_to_log("Failed to reconnect to EtherCAT hardware")
                
                # อัพเดทสถานะการเชื่อมต่อว่าไม่สำเร็จ
                if hasattr(self, 'connection_status_label') and self.connection_status_label:
                    self.connection_status_label.config(text="DISCONNECTED", foreground="white", background="red")
                    
                return False
                
        except Exception as e:
            error_msg = f"Error during reconnection: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            
            # อัพเดทสถานะการเชื่อมต่อว่าไม่สำเร็จ
            if hasattr(self, 'connection_status_label') and self.connection_status_label:
                self.connection_status_label.config(text="ERROR", foreground="white", background="red")
                
            return False

    def set_jog(self, axis, direction):
        """Set jog direction for axis"""
        try:
            # Check if servo is enabled
            status = struct.unpack("<H", axis.slave.sdo_read(0x6041, 0, 2))[0]
            if (status & 0x006F) != 0x0027:  # Check if operation enabled
                self.add_to_log(f"Warning: Servo is not enabled. Please use Servo On button first.")
                return
            
            # เมื่อสั่งหยุด (direction = 0) ให้ค่อยๆ ลดความเร็ว
            if direction == 0 and axis.jog_direction != 0:
                # บันทึกทิศทางเดิมไว้เพื่อรู้ว่ากำลังลดความเร็วในทิศทางไหน
                old_direction = axis.jog_direction
                current_velocity = axis.profile_velocity
                
                self.add_to_log(f"Controlled stop initiated from speed {current_velocity}")
                
                # ลดความเร็วลงเป็นขั้นๆ จนถึง 0
                steps = 10  # จำนวนขั้นที่จะค่อยๆ ลด
                for i in range(steps, 0, -1):
                    reduced_velocity = current_velocity * i / steps
                    axis.set_profile_velocity(int(reduced_velocity))
                    
                    # ยังคงเคลื่อนที่ในทิศทางเดิมแต่ช้าลง
                    axis.jog_direction = old_direction
                    
                    # ส่งค่าความเร็วใหม่ไปที่มอเตอร์
                    axis.send_pdo()
                    time.sleep(0.05)  # เว้นช่วงสั้นๆ ระหว่างการลดความเร็ว
                
                # หยุดสมบูรณ์
                axis.jog_direction = 0
                self.add_to_log(f"Controlled stop completed")
                return
            
            # Apply current motion parameters
            axis.set_motion_parameters()
            
            # Set jog direction
            axis.jog_direction = direction
            
            # Log the jog command with speed information
            direction_text = "stop" if direction == 0 else f"direction {direction}"
            self.add_to_log(f"Jog set to {direction_text} (Speed: {axis.profile_velocity})")
            
            # If starting to jog, set appropriate control word
            if direction != 0:
                axis.control_word = 0x001F  # Operation enabled + new setpoint
            
        except Exception as e:
            error_msg = f"Error in set_jog: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
    
    def smooth_stop(self, axis):
        """เรียกใช้ฟังก์ชัน quick_stop ของ axis และบันทึกลงใน log"""
        try:
            self.add_to_log("STOP button pressed - Smooth stop initiated")
            if axis.quick_stop():
                self.add_to_log("Smooth stop completed successfully - Servo still ON")
                return True
            else:
                self.add_to_log("Smooth stop failed")
                return False
        except Exception as e:
            error_msg = f"Error in smooth_stop: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def step_move(self, axis, step):
        """Move axis by relative step"""
        try:
            # Check if servo is enabled
            status = struct.unpack("<H", axis.slave.sdo_read(0x6041, 0, 2))[0]
            if (status & 0x006F) != 0x0027:  # Check if operation enabled
                self.add_to_log(f"Warning: Servo is not enabled. Please use Servo On button first.")
                return
            
            # Set motion parameters before moving
            axis.set_motion_parameters()
            
            # Calculate target position (current + step)
            target = axis.actual_position + step
            
            # Log the step move with motion parameters
            self.add_to_log(f"Step move by {step} from {axis.actual_position} to {target} (V={axis.profile_velocity})")
            
            # Execute the move
            axis.move_absolute(target)
        except Exception as e:
            error_msg = f"Error in step_move: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
    
    def reset_all_errors(self):
        """Reset errors for all axes"""
        success_count = 0
        for i, axis in enumerate(self.axes):
            try:
                if self.reset_error(axis):
                    success_count += 1
            except Exception as e:
                error_msg = f"Error resetting axis {i}: {e}"
                print(error_msg)
                self.add_to_log(error_msg)
        self.add_to_log(f"Reset all errors: {success_count}/{len(self.axes)} successful")
    
    def stop_all_movement(self):
        """Stop movement for all axes"""
        for i, axis in enumerate(self.axes):
            try:
                axis.jog_direction = 0
            except Exception as e:
                error_msg = f"Error stopping axis {i}: {e}"
                print(error_msg)
                self.add_to_log(error_msg)
        self.add_to_log("All movement stopped")

    def set_velocity(self, axis, velocity):
        """Set profile velocity for an axis"""
        try:
            if axis.set_profile_velocity(velocity):
                self.add_to_log(f"Set velocity to {velocity}")
                return True
            return False
        except Exception as e:
            error_msg = f"Error setting velocity: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def set_velocity_override(self, axis, percentage):
        """Apply a velocity override as a percentage of the base speed."""
        try:
            if axis.override_velocity(percentage):
                self.add_to_log(f"Velocity override: {percentage:.0f}% -> {axis.profile_velocity} counts/sec")
                return True
            return False
        except Exception as e:
            error_msg = f"Error applying velocity override: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
            
    def set_acceleration(self, axis, acceleration):
        """Set profile acceleration for an axis"""
        try:
            if axis.set_profile_acceleration(acceleration):
                self.add_to_log(f"Set acceleration to {acceleration}")
                return True
            return False
        except Exception as e:
            error_msg = f"Error setting acceleration: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
            
    def set_deceleration(self, axis, deceleration):
        """Set profile deceleration for an axis"""
        try:
            if axis.set_profile_deceleration(deceleration):
                self.add_to_log(f"Set deceleration to {deceleration}")
                return True
            return False
        except Exception as e:
            error_msg = f"Error setting deceleration: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def set_velocity_override(self, axis, percentage):
        """Change the axis speed by applying a percentage override."""
        try:
            if axis.override_velocity(percentage):
                self.add_to_log(f"Velocity override set to {percentage}%")
                return True
            return False
        except Exception as e:
            error_msg = f"Error setting velocity override: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
            
    def set_all_motion_parameters(self, velocity, acceleration, deceleration):
        """Set motion parameters for all axes"""
        try:
            success_count = 0
            for i, axis in enumerate(self.axes):
                try:
                    # Set velocity
                    axis.set_profile_velocity(velocity)
                    
                    # Set acceleration
                    axis.set_profile_acceleration(acceleration)
                    
                    # Set deceleration
                    axis.set_profile_deceleration(deceleration)
                    
                    success_count += 1
                except Exception as e:
                    error_msg = f"Error setting parameters for axis {i}: {e}"
                    print(error_msg)
                    self.add_to_log(error_msg)
                    
            self.add_to_log(f"Set motion parameters for {success_count}/{len(self.axes)} axes")
            return success_count == len(self.axes)
        except Exception as e:
            error_msg = f"Error in set_all_motion_parameters: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def set_zero(self, axis):
        """ตั้งค่าตำแหน่งปัจจุบันเป็นตำแหน่งศูนย์ (ใช้วิธีซอฟต์แวร์ล้วนๆ)"""
        try:
            # ตรวจสอบเงื่อนไขที่อาจทำให้เกิด Er.36
            risk_factor = 0  # นับจำนวนปัจจัยเสี่ยงที่อาจทำให้เกิด Er.36
            warning_messages = []

            # 1. ตรวจสอบว่า Servo เปิดอยู่หรือไม่
            if axis.servo_enabled:
                risk_factor += 1
                warning_messages.append("- Servo กำลังเปิด (ON) อยู่")
                
            # 2. ตรวจสอบว่ามอเตอร์กำลังเคลื่อนที่อยู่หรือไม่
            if axis.jog_direction != 0:
                risk_factor += 2  # อันตรายมากกว่าเงื่อนไขอื่น
                warning_messages.append("- มอเตอร์กำลังเคลื่อนที่อยู่ (JOG)")
                
            # 3. ตรวจสอบว่ามีความแตกต่างของตำแหน่งมากเกินไปหรือไม่
            try:
                _, current_pos = axis.read_status()
                if abs(current_pos - axis.target_position) > 5000:  # ค่าต่างกันมากกว่า 5000 counts
                    risk_factor += 1
                    warning_messages.append(f"- ตำแหน่งปัจจุบัน ({current_pos}) ต่างจากตำแหน่งเป้าหมาย ({axis.target_position}) มาก")
            except:
                pass
                
            # แสดงคำเตือนถ้ามีความเสี่ยง
            if risk_factor > 0:
                # เตรียมข้อความแจ้งเตือน
                warning_title = "⚠️ คำเตือน: อาจทำให้เกิด Er.36"
                warning_text = "การตั้งค่าตำแหน่งศูนย์ในขณะนี้มีความเสี่ยงที่จะเกิด Error Er.36\n\n"
                warning_text += "พบความเสี่ยง:\n" + "\n".join(warning_messages) + "\n\n"
                
                if risk_factor >= 2:
                    warning_text += "⚠️ ความเสี่ยงสูง: ขอแนะนำให้หยุดมอเตอร์และปิด Servo ก่อนตั้งค่าตำแหน่งศูนย์\n\n"
                else:
                    warning_text += "⚠️ ความเสี่ยงปานกลาง: ควรพิจารณาปิด Servo ก่อนตั้งค่าตำแหน่งศูนย์\n\n"
                
                warning_text += "ต้องการดำเนินการต่อหรือไม่?"
                
                # แสดงกล่องข้อความเตือน
                from tkinter import messagebox
                proceed = messagebox.askyesno(warning_title, warning_text)
                
                if not proceed:
                    self.add_to_log("ยกเลิกการตั้งค่าตำแหน่งศูนย์")
                    return False
                
                self.add_to_log("ผู้ใช้เลือกดำเนินการตั้งค่าตำแหน่งศูนย์ต่อแม้จะมีความเสี่ยง")
            
            self.add_to_log("กำลังตั้งค่าตำแหน่งปัจจุบันเป็นศูนย์ (วิธีซอฟต์แวร์)...")
            
            # อ่านตำแหน่งปัจจุบันด้วยวิธีอ่านอย่างเดียว (ไม่มีการเขียนค่าใดๆ ลงในฮาร์ดแวร์)
            current_pos = 0
            try:
                # ข้อสำคัญ: อ่านค่าเท่านั้น ไม่มีการเขียนค่ากลับไปที่ไดร์ฟ
                status, pos = axis.read_status()
                current_pos = pos
                self.add_to_log(f"อ่านตำแหน่งปัจจุบัน: {current_pos}")
            except Exception as e:
                self.add_to_log(f"ไม่สามารถอ่านตำแหน่งปัจจุบัน: {e}")
                # ใช้ค่าตำแหน่งปัจจุบันที่เก็บไว้ใน object
                current_pos = axis.actual_position
                self.add_to_log(f"ใช้ค่าตำแหน่งจากหน่วยความจำแทน: {current_pos}")
            
            # บันทึกค่าตำแหน่งก่อนเปลี่ยน
            old_target = axis.target_position
            
            # เก็บค่า offset ไว้ในตัวแปรของโปรแกรมเท่านั้น - ไม่มีการเขียนค่าลงฮาร์ดแวร์
            axis.position_offset = current_pos
            self.add_to_log(f"บันทึกค่า offset = {current_pos} ในหน่วยความจำเรียบร้อย")
            
            # อัพเดท UI แสดงผลทันที
            if hasattr(axis, 'gui_refs') and 'position' in axis.gui_refs:
                axis.gui_refs['position'].config(text="0")
                self.add_to_log("อัพเดทการแสดงผลเป็นตำแหน่ง 0 เรียบร้อย")
            
            # อัพเดทตำแหน่งใน footer ด้วย (ถ้ามี)
            if hasattr(self, 'footer_position_label'):
                self.footer_position_label.config(text="0")
            
            # สำคัญ: ต้องปรับค่า target_position ด้วยเพื่อไม่ให้มอเตอร์เคลื่อนที่
            # ตั้งค่า target_position เป็นตำแหน่งปัจจุบัน เพื่อให้ไม่มีการเคลื่อนที่
            if axis.servo_enabled:
                try:
                    # ไม่ตั้งค่าเป็น 0 เพราะจะทำให้มอเตอร์เคลื่อนที่ แต่ปรับตามค่า offset แทน
                    axis.target_position = current_pos
                    self.add_to_log(f"ปรับ target_position เป็นตำแหน่งปัจจุบัน ({current_pos}) เพื่อป้องกันการเคลื่อนที่")
                    
                    # ส่งค่า PDO ที่มีค่าตำแหน่งปัจจุบันเพื่อป้องกัน Er.36
                    outdata = struct.pack("<HiHb", axis.control_word, current_pos, 0, 8)
                    out_buf = bytearray(axis.slave.output)
                    out_buf[:len(outdata)] = outdata
                    axis.slave.output = bytes(out_buf)
                    self.add_to_log("ส่งค่าตำแหน่งปัจจุบันไปยัง drive ผ่าน PDO")
                except Exception as e:
                    self.add_to_log(f"ไม่สามารถอัพเดทตำแหน่งเป้าหมาย: {e}")
            
            self.add_to_log("ตั้งค่าตำแหน่งปัจจุบันเป็นศูนย์สำเร็จ (วิธีซอฟต์แวร์)")
            
            # แสดงข้อความแนะนำเพิ่มเติมหากมีความเสี่ยงแต่ทำสำเร็จแล้ว
            if risk_factor > 0:
                from tkinter import messagebox
                messagebox.showinfo(
                    "ตั้งค่าตำแหน่งศูนย์สำเร็จ", 
                    "ตั้งค่าตำแหน่งศูนย์เรียบร้อยแล้ว\n\n"
                    "คำแนะนำ: ควรปิด Servo ก่อนตั้งค่าตำแหน่งศูนย์\n"
                    "เพื่อป้องกันการเกิด Error Er.36 ในครั้งต่อไป"
                )
                
            return True
            
        except Exception as e:
            error_msg = f"เกิดข้อผิดพลาดในการตั้งค่าตำแหน่งศูนย์: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def apply_encoder_resolution(self):
        """Apply encoder resolution to all axes"""
        try:
            resolution = int(self.encoder_resolution_var.get())
            for axis in self.axes:
                axis.encoder_resolution = resolution
                self.add_to_log(f"Set encoder resolution to {resolution} for axis {self.axes.index(axis)}")
            self.add_to_log(f"Encoder resolution applied to {len(self.axes)} axes")
            return True
        except Exception as e:
            error_msg = f"Error applying encoder resolution: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def set_forward_motion(self, axis, speed_var):
        """ตั้งค่าการเคลื่อนที่ไปข้างหน้า โดยตรวจสอบความเร็วให้ถูกต้อง"""
        try:
            # ตรวจสอบค่าความเร็วจาก Slider
            slider_value = speed_var.get()
            
            # ค่าความเร็วในตัวแปร slider_value อาจเป็น 0 
            # แต่ถ้าคนเลื่อนแท่งเลื่อนไปที่ศูนย์แล้วกดปุ่ม ให้ตั้งค่าเป็นความเร็วขั้นต่ำ
            if slider_value == 0:
                # ตั้งค่าความเร็วขั้นต่ำถ้าไม่ได้ตั้งค่าไว้
                axis.set_profile_velocity(100)  # ความเร็วต่ำแต่ไม่เป็น 0
                self.add_to_log(f"Setting minimum speed (100) for forward motion")
            
            # ส่งคำสั่งเคลื่อนที่ไปข้างหน้า
            self.set_jog(axis, 1)
            
        except Exception as e:
            error_msg = f"Error in forward motion: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            
    def set_backward_motion(self, axis, speed_var):
        """ตั้งค่าการเคลื่อนที่ไปข้างหลัง โดยตรวจสอบความเร็วให้ถูกต้อง"""
        try:
            # ตรวจสอบค่าความเร็วจาก Slider
            slider_value = speed_var.get()
            
            # ค่าความเร็วในตัวแปร slider_value อาจเป็น 0 
            # แต่ถ้าคนเลื่อนแท่งเลื่อนไปที่ศูนย์แล้วกดปุ่ม ให้ตั้งค่าเป็นความเร็วขั้นต่ำ
            if slider_value == 0:
                # ตั้งค่าความเร็วขั้นต่ำถ้าไม่ได้ตั้งค่าไว้
                axis.set_profile_velocity(100)  # ความเร็วต่ำแต่ไม่เป็น 0
                self.add_to_log(f"Setting minimum speed (100) for backward motion")
            
            # ส่งคำสั่งเคลื่อนที่ไปข้างหลัง
            self.set_jog(axis, -1)
            
        except Exception as e:
            error_msg = f"Error in backward motion: {e}"
            print(error_msg)
            self.add_to_log(error_msg)

    def add_to_log(self, message):
        """Add a message to the log display"""
        try:
            timestamp = time.strftime("[%H:%M:%S]", time.localtime())
            log_message = f"{timestamp} {message}"
            logging.info(message)
            
            # เก็บข้อความไว้ในรายการ log
            self.log_messages.append(log_message)
            
            # แสดงผลใน log widget ถ้ามี
            if hasattr(self, 'log_text') and self.log_text:
                try:
                    # ถ้า GUI ถูกทำลายไปแล้ว อาจจะเกิด TclError
                    # บันทึกค่า state เดิม
                    current_state = self.log_text.cget("state")
                    
                    # เปิด state ให้แก้ไขได้ชั่วคราว
                    self.log_text.config(state=tk.NORMAL)
                    
                    # เพิ่มข้อความใหม่
                    self.log_text.insert(tk.END, log_message + "\n")
                    
                    # เลื่อนไปที่บรรทัดล่าสุด
                    self.log_text.see(tk.END)
                    
                    # กลับไปใช้ state เดิม
                    self.log_text.config(state=current_state)
                except Exception as e:
                    # เกิดปัญหาในการอัพเดท log_text อาจเพราะ GUI ถูกทำลายไปแล้ว
                    # แสดงในคอนโซลแทน
                    print(log_message)
                    print(f"Error updating log in GUI: {e}")
            
            # ถ้าไม่มี GUI ให้แสดงในคอนโซลด้วย
            else:
                print(log_message)
                
        except Exception as e:
            print(f"Error adding to log: {e}")
            print(f"Original message: {message}")
            
    def clear_log(self):
        """Clear the log display"""
        try:
            if hasattr(self, 'log_text') and self.log_text:
                # บันทึกค่า state เดิม
                current_state = self.log_text.cget("state")
                
                # เปิด state ให้แก้ไขได้ชั่วคราว
                self.log_text.config(state=tk.NORMAL)
                
                # ล้างข้อความทั้งหมด
                self.log_text.delete(1.0, tk.END)
                
                # กลับไปใช้ state เดิม
                self.log_text.config(state=current_state)
                
            # ล้างรายการประวัติ log ด้วย
            self.log_messages.clear()
            
        except Exception as e:
            print(f"Error clearing log: {e}")

    def set_jog_speed(self, axis, speed_value):
        """ตั้งค่าความเร็ว JOG สำหรับแกน"""
        try:
            # ตั้งค่าความเร็วสำหรับ JOG
            axis.set_profile_velocity(speed_value)
            
            self.add_to_log(f"JOG speed set to {speed_value} counts/sec ({axis.counts_to_rpm(speed_value):.1f} RPM)")
            return True
            
        except Exception as e:
            error_msg = f"Error setting JOG speed: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
    
    def start_jog_positive_with_speed(self, axis):
        """เริ่มการ JOG ในทิศทางบวกด้วยความเร็วที่กำหนด"""
        try:
            # ตรวจสอบว่า servo เปิดอยู่หรือไม่
            if not axis.servo_enabled:
                self.add_to_log("Cannot JOG: Servo is OFF. Please enable servo first.")
                return False
                
            # อ่านความเร็ว JOG ที่ตั้งไว้
            jog_speed = axis.profile_velocity
            
            # ถ้าไม่ได้ตั้งค่าความเร็ว ให้ใช้ค่าเริ่มต้น
            if jog_speed <= 0:
                jog_speed = self.default_jog_speed
                axis.profile_velocity = jog_speed
                
            # บันทึก log
            self.add_to_log(f"Starting JOG+ at {jog_speed} counts/sec")
            
            # ตั้งค่าพารามิเตอร์การเคลื่อนที่
            axis.set_motion_parameters()
            
            # เริ่ม JOG ในทิศทางบวก
            axis.jog_direction = 1
            
            # เน้นความรวดเร็วในการตอบสนอง ส่ง PDO ทันที
            axis.send_pdo()
            
            # ไฮไลต์ปุ่มให้รู้ว่ากำลัง JOG
            if hasattr(axis, 'gui_refs') and 'jog_pos_btn' in axis.gui_refs:
                axis.gui_refs['jog_pos_btn'].config(relief=tk.SUNKEN)
                
            return True
                
        except Exception as e:
            error_msg = f"Error starting JOG+: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
            
    def start_jog_negative_with_speed(self, axis):
        """เริ่มการ JOG ในทิศทางลบด้วยความเร็วที่กำหนด"""
        try:
            # ตรวจสอบว่า servo เปิดอยู่หรือไม่
            if not axis.servo_enabled:
                self.add_to_log("Cannot JOG: Servo is OFF. Please enable servo first.")
                return False
                
            # อ่านความเร็ว JOG ที่ตั้งไว้
            jog_speed = axis.profile_velocity
            
            # ถ้าไม่ได้ตั้งค่าความเร็ว ให้ใช้ค่าเริ่มต้น
            if jog_speed <= 0:
                jog_speed = self.default_jog_speed
                axis.profile_velocity = jog_speed
                
            # บันทึก log
            self.add_to_log(f"Starting JOG- at {jog_speed} counts/sec")
            
            # ตั้งค่าพารามิเตอร์การเคลื่อนที่
            axis.set_motion_parameters()
            
            # เริ่ม JOG ในทิศทางลบ
            axis.jog_direction = -1
            
            # เน้นความรวดเร็วในการตอบสนอง ส่ง PDO ทันที
            axis.send_pdo()
            
            # ไฮไลต์ปุ่มให้รู้ว่ากำลัง JOG
            if hasattr(axis, 'gui_refs') and 'jog_neg_btn' in axis.gui_refs:
                axis.gui_refs['jog_neg_btn'].config(relief=tk.SUNKEN)
                
            return True
                
        except Exception as e:
            error_msg = f"Error starting JOG-: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
            
    def stop_jog_with_ramp(self, axis):
        """หยุดการ JOG แบบค่อยๆ ลดความเร็ว"""
        try:
            # ตรวจสอบว่ากำลัง JOG อยู่หรือไม่
            if axis.jog_direction == 0:
                return True  # ไม่ได้กำลัง JOG อยู่
                
            # อ่านค่าความเร็วและทิศทางปัจจุบัน
            current_direction = axis.jog_direction
            current_velocity = axis.profile_velocity
            
            self.add_to_log(f"Stopping JOG with ramp-down from {current_velocity} counts/sec")
            
            # ค่อยๆ ลดความเร็วลงเป็นขั้นๆ
            steps = 5  # จำนวนขั้นในการลดความเร็ว
            for i in range(steps, 0, -1):
                # คำนวณความเร็วที่ลดลงตามสัดส่วน
                reduced_velocity = int(current_velocity * i / steps)
                
                # ตั้งค่าความเร็วที่ลดลง
                axis.set_profile_velocity(reduced_velocity)
                
                # ยังคงเคลื่อนที่ในทิศทางเดิมแต่ช้าลง
                axis.jog_direction = current_direction
                
                # ส่งค่าไปยังมอเตอร์
                axis.send_pdo()
                
                # เว้นช่วงเล็กน้อยระหว่างแต่ละขั้น
                time.sleep(0.02)  # 20ms ช่วงสั้นๆ แต่มองเห็นการเปลี่ยนแปลง
            
            # หยุดสมบูรณ์
            axis.jog_direction = 0
            axis.set_profile_velocity(current_velocity)  # คืนค่าความเร็วเดิม
            
            # ส่ง PDO อีกครั้งเพื่อยืนยันการหยุด
            axis.send_pdo()
            
            # คืนสภาพปุ่ม JOG ทั้งสองปุ่ม
            if hasattr(axis, 'gui_refs'):
                if 'jog_pos_btn' in axis.gui_refs:
                    axis.gui_refs['jog_pos_btn'].config(relief=tk.RAISED)
                if 'jog_neg_btn' in axis.gui_refs:
                    axis.gui_refs['jog_neg_btn'].config(relief=tk.RAISED)
            
            self.add_to_log("JOG stopped successfully")
            return True
            
        except Exception as e:
            error_msg = f"Error stopping JOG: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            
            # พยายามหยุดฉุกเฉินในกรณีที่มีข้อผิดพลาด
            try:
                axis.jog_direction = 0
                axis.send_pdo()
            except:
                pass
                
            return False

    def create_speed_control_panel(self, parent_frame, axis):
        """Create a new speed control panel with presets and real-time override."""
        try:
            panel = ttk.Frame(parent_frame)

            # Preset speed buttons
            presets = [500, 1000, 5000, 10000]
            preset_frame = ttk.Frame(panel)
            preset_frame.pack(fill="x", padx=5, pady=5)
            ttk.Label(preset_frame, text="Presets:").pack(side=tk.LEFT, padx=5)
            for spd in presets:
                ttk.Button(
                    preset_frame,
                    text=str(spd),
                    command=lambda v=spd: self.set_velocity(axis, v),
                    width=6,
                ).pack(side=tk.LEFT, padx=2)

            # Custom speed entry
            custom_frame = ttk.Frame(panel)
            custom_frame.pack(fill="x", padx=5, pady=5)
            ttk.Label(custom_frame, text="Custom speed:").pack(side=tk.LEFT, padx=5)
            custom_var = tk.StringVar()
            ttk.Entry(custom_frame, textvariable=custom_var, width=8).pack(side=tk.LEFT)
            ttk.Button(
                custom_frame,
                text="Set",
                command=lambda: self.set_velocity(axis, int(custom_var.get() or 0)),
            ).pack(side=tk.LEFT, padx=2)

            # Override slider
            override_frame = ttk.Frame(panel)
            override_frame.pack(fill="x", padx=5, pady=5)
            ttk.Label(override_frame, text="Override %:").pack(side=tk.LEFT, padx=5)
            ov_var = tk.DoubleVar(value=100)

            def on_override(val):
                try:
                    percent = float(val)
                    self.set_velocity_override(axis, percent)
                except ValueError:
                    pass

            ttk.Scale(
                override_frame,
                from_=10,
                to=200,
                orient=tk.HORIZONTAL,
                variable=ov_var,
                command=on_override,
                length=200,
            ).pack(side=tk.LEFT, padx=5)
            ttk.Label(override_frame, textvariable=ov_var).pack(side=tk.LEFT)

            return panel
        except Exception as e:
            self.add_to_log(f"Error creating speed control: {e}")
            empty = ttk.Frame(parent_frame)
            ttk.Label(empty, text="Speed control error").pack()
            return empty

    def create_jog_speed_selector(self, parent_frame, axis):
        """สร้าง UI สำหรับเลือกความเร็ว JOG ที่สามารถนำไปใช้ซ้ำได้
        
        Args:
            parent_frame: Frame ที่จะใส่ตัวเลือกความเร็ว JOG
            axis: แกนที่จะตั้งค่าความเร็ว
            
        Returns:
            (frame, label): tuple ของ frame และ label ที่แสดงความเร็วปัจจุบัน
        """
        try:
            # สร้าง frame สำหรับตัวเลือกความเร็ว JOG
            jog_selector_frame = ttk.Frame(parent_frame)
            
            # ส่วนหัวตัวเลือกความเร็ว
            header_frame = ttk.Frame(jog_selector_frame)
            header_frame.pack(fill="x", padx=5, pady=5)
            
            ttk.Label(header_frame, text="JOG Speed:", font=("Helvetica", 12, "bold")).pack(side=tk.LEFT, padx=5)
            
            # สร้างตัวแปรสำหรับเก็บความเร็ว JOG ที่เลือก
            jog_speed_var = tk.StringVar(value="Medium")
            
            # เก็บค่านี้ใน axis เพื่อให้ใช้ซ้ำได้
            if not hasattr(axis, 'jog_speed_vars'):
                axis.jog_speed_vars = []
            axis.jog_speed_vars.append(jog_speed_var)
            
            # สร้าง frame สำหรับปุ่มเลือกความเร็ว
            buttons_frame = ttk.Frame(jog_selector_frame)
            buttons_frame.pack(fill="x", padx=5, pady=2)
            
            # สร้างปุ่มตัวเลือกความเร็ว
            jog_speeds = [
                ("Very Slow", 200, "#d1e7f7"),   # สีฟ้าอ่อนมาก
                ("Slow", 1000, "#b8daff"),       # สีฟ้าอ่อน
                ("Medium", 5000, "#8ac4ff"),     # สีฟ้ากลาง
                ("Fast", 20000, "#5a9bea"),      # สีฟ้าเข้ม
                ("Very Fast", 50000, "#3275d1")  # สีฟ้าเข้มมาก
            ]
            
            for i, (speed_name, speed_value, color) in enumerate(jog_speeds):
                speed_btn = tk.Radiobutton(
                    buttons_frame,
                    text=speed_name,
                    value=speed_name,
                    variable=jog_speed_var,
                    indicatoron=0,
                    width=10,
                    pady=5,
                    bg=color,
                    fg="black" if i < 3 else "white",  # สีตัวอักษรดำหรือขาวขึ้นอยู่กับสีพื้นหลัง
                    selectcolor="#4b89dc",  # สีเมื่อเลือก
                    command=lambda ax=axis, val=speed_value: self.set_jog_speed(ax, val)
                )
                speed_btn.grid(row=0, column=i, padx=1, pady=2)
                
                # ถ้าเป็น Medium speed ให้เลือกเป็นค่าเริ่มต้น
                if speed_name == "Medium":
                    speed_btn.select()
                    # ตั้งค่า jog speed เริ่มต้น
                    self.set_jog_speed(axis, speed_value)
            
            # เพิ่มปุ่มลัดสำหรับความเร็วต่ำมากๆ
            precise_speeds_frame = ttk.LabelFrame(jog_selector_frame, text="Precise RPM Controls")
            precise_speeds_frame.pack(fill="x", padx=5, pady=5)
            
            rpm_buttons_frame = ttk.Frame(precise_speeds_frame)
            rpm_buttons_frame.pack(fill="x", padx=5, pady=5)
            
            # ปุ่มลัดสำหรับตั้งค่าความเร็ว 1 RPM
            one_rpm_btn = tk.Button(
                rpm_buttons_frame,
                text="1 RPM",
                bg="#FF9800",
                fg="black",
                font=("Helvetica", 10, "bold"),
                width=8,
                height=2,
                command=lambda: self.set_speed_in_rpm(axis, 1.0)
            )
            one_rpm_btn.pack(side=tk.LEFT, padx=5, pady=5)
            
            # ปุ่มลัดสำหรับตั้งค่าความเร็ว 2 RPM
            two_rpm_btn = tk.Button(
                rpm_buttons_frame,
                text="2 RPM",
                bg="#FF9800",
                fg="black",
                font=("Helvetica", 10, "bold"),
                width=8,
                height=2,
                command=lambda: self.set_speed_in_rpm(axis, 2.0)
            )
            two_rpm_btn.pack(side=tk.LEFT, padx=5, pady=5)
            
            # ปุ่มลัดสำหรับตั้งค่าความเร็ว 5 RPM
            five_rpm_btn = tk.Button(
                rpm_buttons_frame,
                text="5 RPM",
                bg="#FF9800",
                fg="black",
                font=("Helvetica", 10, "bold"),
                width=8,
                height=2,
                command=lambda: self.set_speed_in_rpm(axis, 5.0)
            )
            five_rpm_btn.pack(side=tk.LEFT, padx=5, pady=5)
            
            # ปุ่มลัดสำหรับตั้งค่าความเร็ว 10 RPM
            ten_rpm_btn = tk.Button(
                rpm_buttons_frame,
                text="10 RPM",
                bg="#FF9800",
                fg="black",
                font=("Helvetica", 10, "bold"),
                width=8,
                height=2,
                command=lambda: self.set_speed_in_rpm(axis, 10.0)
            )
            ten_rpm_btn.pack(side=tk.LEFT, padx=5, pady=5)
            
            # เพิ่มช่องสำหรับกรอกความเร็วเป็น RPM
            custom_rpm_frame = ttk.Frame(precise_speeds_frame)
            custom_rpm_frame.pack(fill="x", padx=5, pady=5)
            
            # ช่องกรอกความเร็วเองในหน่วย RPM
            ttk.Label(custom_rpm_frame, text="Custom Speed:", font=("Helvetica", 10, "bold")).pack(side=tk.LEFT, padx=5)
            custom_rpm_var = tk.StringVar()
            custom_rpm_entry = ttk.Entry(custom_rpm_frame, textvariable=custom_rpm_var, width=10)
            custom_rpm_entry.pack(side=tk.LEFT, padx=5)
            
            # เพิ่มหน่วย
            ttk.Label(custom_rpm_frame, text="RPM", font=("Helvetica", 9)).pack(side=tk.LEFT, padx=2)
            
            # ปุ่ม Apply สำหรับใช้ความเร็วที่กรอกเอง (RPM)
            apply_rpm_btn = ttk.Button(
                custom_rpm_frame,
                text="Apply",
                command=lambda: self.apply_custom_rpm(axis, custom_rpm_var)
            )
            apply_rpm_btn.pack(side=tk.LEFT, padx=5)
            
            # เพิ่มช่องสำหรับกรอกความเร็วเอง (counts/sec)
            custom_speed_frame = ttk.Frame(jog_selector_frame)
            custom_speed_frame.pack(fill="x", padx=5, pady=5)
            
            ttk.Label(custom_speed_frame, text="Raw Counts Speed:", font=("Helvetica", 10)).pack(side=tk.LEFT, padx=5)
            
            # ช่องกรอกความเร็วเองในหน่วย counts/sec
            custom_speed_var = tk.StringVar()
            custom_speed_entry = ttk.Entry(custom_speed_frame, textvariable=custom_speed_var, width=10)
            custom_speed_entry.pack(side=tk.LEFT, padx=5)
            
            # เพิ่มหน่วย
            ttk.Label(custom_speed_frame, text="counts/sec", font=("Helvetica", 9)).pack(side=tk.LEFT, padx=2)
            
            # ปุ่ม Apply สำหรับใช้ความเร็วที่กรอกเอง
            apply_custom_btn = ttk.Button(
                custom_speed_frame,
                text="Apply",
                command=lambda: self.apply_custom_speed(axis, custom_speed_var)
            )
            apply_custom_btn.pack(side=tk.LEFT, padx=5)
            
            # สร้าง label แสดงความเร็ว JOG ปัจจุบัน
            speed_display_frame = ttk.Frame(jog_selector_frame)
            speed_display_frame.pack(fill="x", padx=5, pady=2)
            
            ttk.Label(speed_display_frame, text="Current JOG Speed:", font=("Helvetica", 10)).pack(side=tk.LEFT, padx=5)
            current_speed_label = ttk.Label(speed_display_frame, text="5000 counts/sec (30.0 RPM)", font=("Helvetica", 10, "bold"))
            current_speed_label.pack(side=tk.LEFT, padx=5)
            
            # เพิ่มส่วนแสดงความเร็วจริง
            actual_speed_frame = ttk.Frame(jog_selector_frame)
            actual_speed_frame.pack(fill="x", padx=5, pady=2)
            
            ttk.Label(actual_speed_frame, text="Actual Speed:", font=("Helvetica", 10)).pack(side=tk.LEFT, padx=5)
            actual_speed_label = ttk.Label(actual_speed_frame, text="0 counts/sec (0.0 RPM)", font=("Helvetica", 10, "bold"), foreground="green")
            actual_speed_label.pack(side=tk.LEFT, padx=5)
            
            # เก็บ reference ของ label ไว้อัพเดท
            if not hasattr(axis, 'gui_refs'):
                axis.gui_refs = {}
            axis.gui_refs['actual_speed_label'] = actual_speed_label
            
            # เพิ่มฟังก์ชันอัพเดทความเร็วจริง
            def update_actual_speed():
                if not self.running:
                    return
                    
                try:
                    # อ่านค่าความเร็วจริง
                    status, _ = axis.read_status()  # อ่านสถานะและความเร็วจริง
                    
                    # แสดงความเร็วจริง
                    actual_speed = axis.actual_velocity
                    actual_rpm = axis.get_actual_velocity_rpm()
                    
                    # อัพเดท label
                    actual_speed_label.config(
                        text=f"{actual_speed} counts/sec ({actual_rpm:.1f} RPM)",
                        foreground="green" if abs(actual_speed) > 0 else "black"
                    )
                    
                except Exception as e:
                    print(f"Error updating actual speed: {e}")
                    
                # อัพเดททุก 0.5 วินาที
                if self.gui:
                    self.gui.after(500, update_actual_speed)
                    
            # เริ่มอัพเดทความเร็วจริง
            if self.gui:
                self.gui.after(1000, update_actual_speed)
            
            # คำอธิบายเพิ่มเติม
            note_frame = ttk.Frame(jog_selector_frame)
            note_frame.pack(fill="x", padx=5, pady=2)
            ttk.Label(note_frame, text="ปุ่ม 1 RPM จะทำให้มอเตอร์หมุนที่ความเร็ว 1 รอบต่อนาที", font=("Helvetica", 9, "italic")).pack(side=tk.LEFT, padx=5)
            
            # เพิ่มส่วนตั้งค่าตัวคูณปรับสเกลความเร็ว
            scale_frame = ttk.LabelFrame(jog_selector_frame, text="Velocity Scaling Factor")
            scale_frame.pack(fill="x", padx=5, pady=5)
            
            scale_input_frame = ttk.Frame(scale_frame)
            scale_input_frame.pack(fill="x", padx=5, pady=5)
            
            ttk.Label(scale_input_frame, text="Scale Factor:", font=("Helvetica", 10)).pack(side=tk.LEFT, padx=5)
            
            # ช่องกรอกค่าตัวคูณปรับสเกล
            scale_factor_var = tk.StringVar(value=str(axis.velocity_scale_factor))
            scale_factor_entry = ttk.Entry(scale_input_frame, textvariable=scale_factor_var, width=10)
            scale_factor_entry.pack(side=tk.LEFT, padx=5)
            
            # ปุ่ม Apply สำหรับใช้ตัวคูณที่กรอก
            apply_scale_btn = ttk.Button(
                scale_input_frame,
                text="Apply",
                command=lambda: self.apply_velocity_scale_factor(axis, scale_factor_var)
            )
            apply_scale_btn.pack(side=tk.LEFT, padx=5)
            
            # คำอธิบายการใช้งาน
            ttk.Label(scale_frame, 
                    text="Adjust this value if the actual speed reading is incorrect. Use smaller values for high numbers.",
                    font=("Helvetica", 9, "italic")).pack(anchor="w", padx=5, pady=2)
            
            return jog_selector_frame, current_speed_label
            
        except Exception as e:
            error_msg = f"Error creating JOG speed selector: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            
            # สร้าง frame เปล่าเพื่อไม่ให้โปรแกรมพัง
            empty_frame = ttk.Frame(parent_frame)
            empty_label = ttk.Label(empty_frame, text="Error loading JOG speed selector")
            empty_label.pack()
            return empty_frame, empty_label

    def apply_custom_speed(self, axis, speed_var):
        """ใช้ความเร็วที่ผู้ใช้กรอกเอง"""
        try:
            # รับค่าความเร็วจากช่องกรอก
            speed_str = speed_var.get().strip()
            
            # ตรวจสอบว่ามีค่าหรือไม่
            if not speed_str:
                self.add_to_log("กรุณากรอกค่าความเร็ว")
                return False
                
            # พยายามแปลงเป็นตัวเลข
            try:
                speed_value = int(speed_str)
            except ValueError:
                self.add_to_log("ค่าความเร็วต้องเป็นตัวเลขเท่านั้น")
                return False
                
            # ตรวจสอบว่าเป็นค่าที่ถูกต้อง (มากกว่า 0)
            if speed_value <= 0:
                self.add_to_log("ค่าความเร็วต้องมากกว่า 0")
                return False
                
            # ตั้งค่าความเร็วใหม่
            self.set_jog_speed(axis, speed_value)
            
            # แสดงข้อความว่าตั้งค่าสำเร็จ
            self.add_to_log(f"ตั้งค่าความเร็วเป็น {speed_value} counts/sec เรียบร้อยแล้ว")
            
            # รีเซ็ตค่าใน radio buttons (ไม่มีตัวเลือกไหนถูกเลือก)
            if hasattr(axis, 'jog_speed_vars'):
                for var in axis.jog_speed_vars:
                    var.set("")
                    
            return True
            
        except Exception as e:
            error_msg = f"เกิดข้อผิดพลาดในการตั้งค่าความเร็ว: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def apply_velocity_scale_factor(self, axis, scale_var):
        """ตั้งค่าตัวคูณปรับสเกลความเร็ว"""
        try:
            scale_str = scale_var.get().strip()
            
            # ตรวจสอบว่ามีค่าหรือไม่
            if not scale_str:
                self.add_to_log("กรุณากรอกค่าตัวคูณปรับสเกล")
                return False
                
            # พยายามแปลงเป็นตัวเลข
            try:
                scale_value = float(scale_str)
            except ValueError:
                self.add_to_log("ค่าตัวคูณต้องเป็นตัวเลขเท่านั้น")
                return False
                
            # ตรวจสอบว่าเป็นค่าที่ถูกต้อง (มากกว่า 0)
            if scale_value <= 0:
                self.add_to_log("ค่าตัวคูณต้องมากกว่า 0")
                return False
                
            # ตั้งค่าตัวคูณใหม่
            axis.velocity_scale_factor = scale_value
            
            # แสดงข้อความว่าตั้งค่าสำเร็จ
            self.add_to_log(f"ตั้งค่าตัวคูณปรับสเกลความเร็วเป็น {scale_value} เรียบร้อยแล้ว")
            return True
                
        except Exception as e:
            error_msg = f"เกิดข้อผิดพลาดในการตั้งค่าตัวคูณปรับสเกล: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def create_absolute_move_tab(self, parent_frame, axis):
        """สร้างแท็บสำหรับการเคลื่อนที่แบบ Absolute Position"""
        try:
            # สร้างเฟรมหลัก
            abs_frame = ttk.LabelFrame(parent_frame, text="Absolute Position Move")
            abs_frame.pack(fill="both", expand=True, padx=10, pady=10)
            
            # สร้างเฟรมย่อยสำหรับข้อมูลตำแหน่ง
            pos_frame = ttk.Frame(abs_frame)
            pos_frame.pack(fill="x", padx=10, pady=10)
            
            # แสดงตำแหน่งปัจจุบัน
            ttk.Label(pos_frame, text="Current Position:", font=("Helvetica", 12, "bold")).grid(row=0, column=0, padx=5, pady=5, sticky="w")
            current_pos_label = ttk.Label(pos_frame, text="0", font=("Helvetica", 12))
            current_pos_label.grid(row=0, column=1, padx=5, pady=5, sticky="w")
            
            # ช่องสำหรับกรอกตำแหน่งเป้าหมาย
            ttk.Label(pos_frame, text="Target Position:", font=("Helvetica", 12, "bold")).grid(row=1, column=0, padx=5, pady=5, sticky="w")
            target_pos_var = tk.StringVar()
            target_pos_entry = ttk.Entry(pos_frame, textvariable=target_pos_var, width=15, font=("Helvetica", 12))
            target_pos_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")
            
            # ปุ่มเพิ่ม/ลดค่า
            inc_btns_frame = ttk.Frame(pos_frame)
            inc_btns_frame.grid(row=1, column=2, padx=5, pady=5, sticky="w")
            
            def update_target_pos(amount):
                try:
                    current = int(target_pos_var.get() or "0")
                    target_pos_var.set(str(current + amount))
                except ValueError:
                    target_pos_var.set(str(amount))
            
            ttk.Button(inc_btns_frame, text="-1000", command=lambda: update_target_pos(-1000)).pack(side=tk.LEFT, padx=2)
            ttk.Button(inc_btns_frame, text="-100", command=lambda: update_target_pos(-100)).pack(side=tk.LEFT, padx=2)
            ttk.Button(inc_btns_frame, text="-10", command=lambda: update_target_pos(-10)).pack(side=tk.LEFT, padx=2)
            ttk.Button(inc_btns_frame, text="+10", command=lambda: update_target_pos(10)).pack(side=tk.LEFT, padx=2)
            ttk.Button(inc_btns_frame, text="+100", command=lambda: update_target_pos(100)).pack(side=tk.LEFT, padx=2)
            ttk.Button(inc_btns_frame, text="+1000", command=lambda: update_target_pos(1000)).pack(side=tk.LEFT, padx=2)
            
            # ปุ่มเลือกตำแหน่งที่ใช้บ่อย
            common_pos_frame = ttk.LabelFrame(abs_frame, text="Common Positions")
            common_pos_frame.pack(fill="x", padx=10, pady=5)
            
            common_btn_frame = ttk.Frame(common_pos_frame)
            common_btn_frame.pack(fill="x", padx=10, pady=5)
            
            positions = [
                ("Zero", 0),
                ("Position 1", 10000),
                ("Position 2", 20000),
                ("Position 3", 50000),
                ("Position 4", 100000),
            ]
            
            for i, (name, pos) in enumerate(positions):
                ttk.Button(
                    common_btn_frame, 
                    text=f"{name} ({pos})",
                    command=lambda p=pos: target_pos_var.set(str(p))
                ).grid(row=0, column=i, padx=5, pady=5)
            
            # ลบส่วนควบคุมความเร็วออกเพราะใช้ global speed control แทน
            # เพิ่มคำอธิบายว่าใช้ Speed Control ด้านบน
            note_frame = ttk.Frame(abs_frame)
            note_frame.pack(fill="x", padx=10, pady=5)
            ttk.Label(note_frame, text="Note: Use the Speed Control box at the top of the screen to set movement speed", 
                     font=("Helvetica", 9, "italic")).pack(padx=10, pady=5, anchor=tk.W)
            
            # ปุ่มสั่งการเคลื่อนที่
            move_frame = ttk.Frame(abs_frame)
            move_frame.pack(fill="x", padx=10, pady=10)
            
            # ปุ่ม Set as Target
            set_target_btn = tk.Button(
                move_frame,
                text="SET AS TARGET",
                bg="#4b89dc",
                fg="white",
                font=("Helvetica", 12, "bold"),
                width=15,
                height=2,
                relief=tk.RAISED,
                command=lambda: self.set_absolute_target(axis, target_pos_var)
            )
            set_target_btn.pack(side=tk.LEFT, padx=10, pady=10)
            
            # ปุ่ม Move Now
            move_now_btn = tk.Button(
                move_frame,
                text="MOVE NOW",
                bg="#28a745",
                fg="white",
                font=("Helvetica", 12, "bold"),
                width=15,
                height=2,
                relief=tk.RAISED,
                command=lambda: self.move_to_absolute_position(axis, target_pos_var)
            )
            move_now_btn.pack(side=tk.LEFT, padx=10, pady=10)
            
            # ปุ่ม STOP
            stop_btn = tk.Button(
                move_frame,
                text="STOP",
                bg="#dc3545",
                fg="white",
                font=("Helvetica", 12, "bold"),
                width=15,
                height=2,
                relief=tk.RAISED,
                command=lambda: self.smooth_stop(axis)
            )
            stop_btn.pack(side=tk.LEFT, padx=10, pady=10)
            
            # เก็บ reference เพื่ออัพเดทข้อมูล
            if not hasattr(axis, 'abs_move_refs'):
                axis.abs_move_refs = {}
            
            axis.abs_move_refs = {
                'current_pos': current_pos_label,
                'target_pos': target_pos_var
            }
            
            return abs_frame
            
        except Exception as e:
            error_msg = f"Error creating Absolute Move tab: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            
            # สร้าง frame เปล่าเพื่อไม่ให้โปรแกรมพัง
            empty_frame = ttk.Frame(parent_frame)
            ttk.Label(empty_frame, text="Error loading Absolute Move tab").pack()
            return empty_frame
    
    def create_relative_move_tab(self, parent_frame, axis):
        """สร้างแท็บสำหรับการเคลื่อนที่แบบ Relative Move"""
        try:
            # สร้างเฟรมหลัก
            rel_frame = ttk.LabelFrame(parent_frame, text="Relative Move")
            rel_frame.pack(fill="both", expand=True, padx=10, pady=10)
            
            # สร้างเฟรมย่อยสำหรับข้อมูลตำแหน่ง
            pos_frame = ttk.Frame(rel_frame)
            pos_frame.pack(fill="x", padx=10, pady=10)
            
            # แสดงตำแหน่งปัจจุบัน
            ttk.Label(pos_frame, text="Current Position:", font=("Helvetica", 12, "bold")).grid(row=0, column=0, padx=5, pady=5, sticky="w")
            current_pos_label = ttk.Label(pos_frame, text="0", font=("Helvetica", 12))
            current_pos_label.grid(row=0, column=1, padx=5, pady=5, sticky="w")
            
            # ช่องสำหรับกรอกระยะทางที่ต้องการเคลื่อนที่
            ttk.Label(pos_frame, text="Move Distance:", font=("Helvetica", 12, "bold")).grid(row=1, column=0, padx=5, pady=5, sticky="w")
            distance_var = tk.StringVar()
            distance_entry = ttk.Entry(pos_frame, textvariable=distance_var, width=15, font=("Helvetica", 12))
            distance_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")
            
            # ปุ่มเลือกระยะห่างที่ใช้บ่อย
            common_dist_frame = ttk.LabelFrame(rel_frame, text="Common Distances")
            common_dist_frame.pack(fill="x", padx=10, pady=5)
            
            # แถวแรก - ค่าลบ (ถอยหลัง)
            neg_btn_frame = ttk.Frame(common_dist_frame)
            neg_btn_frame.pack(fill="x", padx=10, pady=5)
            
            ttk.Label(neg_btn_frame, text="BACKWARD:", font=("Helvetica", 10, "bold")).pack(side=tk.LEFT, padx=5)
            
            for dist in [-50000, -10000, -5000, -1000, -500, -100, -10]:
                ttk.Button(
                    neg_btn_frame, 
                    text=str(dist),
                    command=lambda d=dist: distance_var.set(str(d))
                ).pack(side=tk.LEFT, padx=2)
            
            # แถวที่สอง - ค่าบวก (เดินหน้า)
            pos_btn_frame = ttk.Frame(common_dist_frame)
            pos_btn_frame.pack(fill="x", padx=10, pady=5)
            
            ttk.Label(pos_btn_frame, text="FORWARD:", font=("Helvetica", 10, "bold")).pack(side=tk.LEFT, padx=5)
            
            for dist in [10, 100, 500, 1000, 5000, 10000, 50000]:
                ttk.Button(
                    pos_btn_frame, 
                    text=str(dist),
                    command=lambda d=dist: distance_var.set(str(d))
                ).pack(side=tk.LEFT, padx=2)
            
            # ลบส่วนควบคุมความเร็วออกเพราะใช้ global speed control แทน
            # เพิ่มคำอธิบายว่าใช้ Speed Control ด้านบน
            note_frame = ttk.Frame(rel_frame)
            note_frame.pack(fill="x", padx=10, pady=5)
            ttk.Label(note_frame, text="Note: Use the Speed Control box at the top of the screen to set movement speed", 
                     font=("Helvetica", 9, "italic")).pack(padx=10, pady=5, anchor=tk.W)
            
            # ปุ่มสั่งการเคลื่อนที่
            move_frame = ttk.Frame(rel_frame)
            move_frame.pack(fill="x", padx=10, pady=10)
            
                        # ปุ่ม Move Backward (กรณีที่ไม่ต้องการกรอกค่าเอง)            move_back_btn = tk.Button(                move_frame,                text="◀ MOVE BACKWARD",                bg="#6c757d",                fg="white",                font=("Helvetica", 12, "bold"),                width=15,                height=2,                relief=tk.RAISED,                # command=lambda: self.move_relative_distance(axis, -1000)                command=lambda: self.move_relative_distance(axis, -int(distance_var.get()))            )
            move_back_btn = tk.Button(
                move_frame,
                text="◀ MOVE BACKWARD",
                bg="#6c757d",
                fg="white",
                font=("Helvetica", 12, "bold"),
                width=15,
                height=2,
                relief=tk.RAISED,
                # command=lambda: self.move_relative_distance(axis, -1000)
                command=lambda: self.move_relative_distance(axis, -int(distance_var.get()))
            )
            move_back_btn.pack(side=tk.LEFT, padx=5, pady=10)
            
            # ปุ่ม Move Exactly
            move_exact_btn = tk.Button(
                move_frame,
                text="MOVE EXACTLY",
                bg="#28a745",
                fg="white",
                font=("Helvetica", 12, "bold"),
                width=15,
                height=2,
                relief=tk.RAISED,
                command=lambda: self.move_relative_distance_from_input(axis, distance_var)
            )
            move_exact_btn.pack(side=tk.LEFT, padx=5, pady=10)
            
            # ปุ่ม Move Forward (กรณีที่ไม่ต้องการกรอกค่าเอง)
            move_fwd_btn = tk.Button(
                move_frame,
                text="MOVE FORWARD ▶",
                bg="#6c757d",
                fg="white",
                font=("Helvetica", 12, "bold"),
                width=15,
                height=2,
                relief=tk.RAISED,
                # command=lambda: self.move_relative_distance(axis, 1000)
                command=lambda: self.move_relative_distance(axis, int(distance_var.get()))
            )
            move_fwd_btn.pack(side=tk.LEFT, padx=5, pady=10)
            
            # เก็บ reference เพื่ออัพเดทข้อมูล
            if not hasattr(axis, 'rel_move_refs'):
                axis.rel_move_refs = {}
            
            axis.rel_move_refs = {
                'current_pos': current_pos_label,
                'distance': distance_var
            }
            
            return rel_frame
            
        except Exception as e:
            error_msg = f"Error creating Relative Move tab: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            
            # สร้าง frame เปล่าเพื่อไม่ให้โปรแกรมพัง
            empty_frame = ttk.Frame(parent_frame)
            ttk.Label(empty_frame, text="Error loading Relative Move tab").pack()
            return empty_frame
    
    def set_absolute_target(self, axis, target_pos_var):
        """ตั้งค่าตำแหน่งเป้าหมายโดยไม่เคลื่อนที่ทันที"""
        try:
            # ตรวจสอบค่า target_pos
            try:
                target_pos = int(target_pos_var.get())
            except ValueError:
                self.add_to_log("ค่าตำแหน่งเป้าหมายไม่ถูกต้อง กรุณาระบุเป็นตัวเลขเท่านั้น")
                return False
            
            # ตรวจสอบสถานะ servo
            if not axis.servo_enabled:
                self.add_to_log("ไม่สามารถตั้งค่าตำแหน่งเป้าหมายได้: Servo ยังไม่เปิด")
                return False
            
            # ตั้งค่าตำแหน่งเป้าหมาย
            axis.target_position = target_pos
            
            self.add_to_log(f"ตั้งค่าตำแหน่งเป้าหมายเป็น {target_pos} เรียบร้อยแล้ว")
            return True
            
        except Exception as e:
            error_msg = f"Error setting absolute target: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
    
    def move_to_absolute_position(self, axis, target_pos_var):
        """เคลื่อนที่ไปยังตำแหน่งที่กำหนดแบบ Absolute"""
        try:
            # ตรวจสอบค่า target_pos
            try:
                target_pos = int(target_pos_var.get())
            except ValueError:
                self.add_to_log("ค่าตำแหน่งเป้าหมายไม่ถูกต้อง กรุณาระบุเป็นตัวเลขเท่านั้น")
                return False
            
            self.add_to_log(f"กำลังเคลื่อนที่ไปยังตำแหน่ง: {target_pos}")
            
            # ใช้ฟังก์ชัน move_absolute ที่มีอยู่แล้ว
            axis.move_absolute(target_pos)
            
            return True
            
        except Exception as e:
            error_msg = f"Error moving to absolute position: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
    
    def move_relative_distance(self, axis, distance):
        """เคลื่อนที่ไปจากตำแหน่งปัจจุบันเป็นระยะที่กำหนด"""
        try:
            # ตรวจสอบสถานะ servo
            if not axis.servo_enabled:
                self.add_to_log("ไม่สามารถเคลื่อนที่ได้: Servo ยังไม่เปิด")
                return False
            
            # อ่านตำแหน่งปัจจุบัน
            try:
                status, current_pos = axis.read_status()
                target_pos = current_pos + distance
                
                self.add_to_log(f"กำลังเคลื่อนที่จากตำแหน่ง {current_pos} ไปอีก {distance} หน่วย (ตำแหน่งใหม่: {target_pos})")
                
                # ใช้ฟังก์ชัน move_absolute ที่มีอยู่แล้ว
                axis.move_absolute(target_pos)
                
                return True
                
            except Exception as e:
                self.add_to_log(f"ไม่สามารถอ่านตำแหน่งปัจจุบันได้: {e}")
                return False
            
        except Exception as e:
            error_msg = f"Error in relative move: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
    
    def move_relative_distance_from_input(self, axis, distance_var):
        """เคลื่อนที่ไปจากตำแหน่งปัจจุบันเป็นระยะที่กรอกไว้"""
        try:
            # ตรวจสอบค่า distance
            try:
                distance = int(distance_var.get())
            except ValueError:
                self.add_to_log("ค่าระยะทางไม่ถูกต้อง กรุณาระบุเป็นตัวเลขเท่านั้น")
                return False
            
            # เรียกใช้ฟังก์ชัน move_relative_distance
            return self.move_relative_distance(axis, distance)
            
        except Exception as e:
            error_msg = f"Error in relative move: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def setup_all_axes(self):
        for axis in self.axes:
            try:
                # Reset error first
                axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x0080))  # Reset
                time.sleep(0.1)
                
                # Clear error
                axis.slave.sdo_write(0x603F, 0, struct.pack("<H", 0x0000))  # Clear error
                time.sleep(0.1)
                
                # Shutdown state
                axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x0006))
                time.sleep(0.1)
                
                # Set operation mode to CSP
                axis.slave.sdo_write(0x6060, 0, struct.pack("b", 8))
                time.sleep(0.05)
                
                # Try to read encoder resolution from drive if supported
                try:
                    # สำหรับ Wecon VD3E ค่า encoder resolution อาจอยู่ที่ object index อื่น
                    # ตัวอย่างเช่น Maxsine มี object id 0x2383 sub-index 5
                    # ตรวจสอบคู่มือของ drive เพื่อหา object index ที่ถูกต้อง
                    # encoder_res = struct.unpack("<i", axis.slave.sdo_read(0x2383, 5, 4))[0]
                    # if encoder_res > 0:
                    #    axis.encoder_resolution = encoder_res
                    self.add_to_log(f"Using encoder resolution: {axis.encoder_resolution} counts/rev")
                    self.add_to_log(f"Application units scaling: {axis.app_units_per_rev} units/rev")
                except Exception as e:
                    self.add_to_log(f"Could not read encoder resolution: {e}")
                
                # Set initial motion parameters
                axis.set_motion_parameters()
                print(f"Set initial motion parameters: V={axis.profile_velocity}, A={axis.profile_acceleration}, D={axis.profile_deceleration}")
                
                # Read current position
                pos = struct.unpack("<i", axis.slave.sdo_read(0x6064, 0, 4))[0]
                axis.target_position = pos
                axis.control_word = 0x0006  # Shutdown state
                
                print(f"Axis setup completed successfully (Servo Off)")
            except Exception as e:
                print(f"Axis setup failed: {e}")

    def start_loop(self):
        self.running = True
        def loop():
            while self.running:
                for axis in self.axes:
                    axis.send_pdo()
                self.master.send_processdata()
                self.master.receive_processdata(1000)
                time.sleep(0.001)
        threading.Thread(target=loop, daemon=True).start()

    def stop(self):
        """Safely stop the controller and close all connections"""
        try:
            self.add_to_log("Shutting down system...")
            print("Stopping controller...")
            
            # Stop process loop first
            self.running = False
            time.sleep(0.3)  # Give more time for threads to finish
            
            # ปิด servo และหยุดการเคลื่อนที่ทั้งหมดก่อน
            for axis in self.axes:
                try:
                    # หยุดการเคลื่อนที่ทั้งหมดก่อน
                    if axis.jog_direction != 0:
                        axis.jog_direction = 0
                    
                    # ถ้า servo ยังเปิดอยู่ ให้ปิดอย่างถูกต้อง
                    if hasattr(axis, 'servo_enabled') and axis.servo_enabled:
                        try:
                            # ลดความเร็วเป็น 0 ก่อน
                            axis.set_profile_velocity(0)
                            time.sleep(0.1)
                            
                            # ปิด servo อย่างปลอดภัย
                            axis.control_word = 0x0006  # Shutdown state
                            axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x0006))
                            time.sleep(0.2)
                            axis.servo_enabled = False
                        except Exception as e:
                            print(f"Error during servo disable: {e}")
                    
                    # สำรองการตั้งค่า control word เป็น 0x0006 (Shutdown) อีกครั้ง
                    try:
                        axis.control_word = 0x0006
                        axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x0006))
                    except Exception as e:
                        print(f"Error setting control word: {e}")
                
                except Exception as e:
                    print(f"Error disabling axis: {e}")
            
            # รอให้คำสั่งทั้งหมดเสร็จสมบูรณ์
            time.sleep(0.3)
            
            # Change EtherCAT state to PRE-OP first, then to INIT for safer shutdown
            try:
                # ลดสถานะเป็น PREOP ก่อน
                self.add_to_log("Changing EtherCAT state to PREOP...")
                self.master.state = pysoem.PREOP_STATE
                self.master.write_state()
                time.sleep(0.2)
                
                # จากนั้นเปลี่ยนเป็น INIT
                self.add_to_log("Changing EtherCAT state to INIT...")
                self.master.state = pysoem.INIT_STATE
                self.master.write_state()
                time.sleep(0.2)
            except Exception as e:
                error_msg = f"Error changing EtherCAT state: {e}"
                print(error_msg)
                self.add_to_log(error_msg)
            
            # Close EtherCAT master
            try:
                self.add_to_log("Closing EtherCAT master...")
                self.master.close()
            except Exception as e:
                error_msg = f"Error closing EtherCAT master: {e}"
                print(error_msg)
                self.add_to_log(error_msg)
            
            # Close GUI if it exists
            if self.gui:
                self.add_to_log("Application closed properly")
                # Schedule destroy after all pending events
                self.gui.after(100, self.gui.quit)
                self.gui.after(200, self.gui.destroy)
                
            print("Controller stopped successfully")
            
        except Exception as e:
            error_msg = f"Error during shutdown: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            if self.gui:
                self.gui.destroy()

    def on_close(self):
        """จัดการการปิดหน้าต่าง GUI"""
        try:
            # หยุดการทำงานของระบบด้วยเมธอด stop() ซึ่งจะจัดการปิดการเชื่อมต่อ EtherCAT อย่างถูกต้อง
            self.add_to_log("Window closing - shutting down system...")
            
            # ตรวจสอบและหยุดการเคลื่อนที่ทุกแกนก่อน
            for axis in self.axes:
                try:
                    if axis.jog_direction != 0:
                        axis.jog_direction = 0
                        # ถ้า servo ยังเปิดอยู่ ให้ปิดด้วย
                        if axis.servo_enabled:
                            self.servo_off_axis(axis)
                except Exception as e:
                    print(f"Error stopping axis during shutdown: {e}")
            
            # เรียกใช้เมธอด stop() เพื่อจัดการปิดระบบอย่างถูกต้อง
            # ซึ่งจะจัดการปรับสถานะ EtherCAT เป็น INIT และปิดการเชื่อมต่อ
            self.stop()
            
        except Exception as e:
            print(f"Error during closing: {e}")
            # พยายามปิดหน้าต่างอย่างปลอดภัย
            if self.gui:
                self.gui.destroy()
                
    def servo_off_axis(self, axis):
        """Disable servo on a single axis (Servo OFF)"""
        try:
            self.add_to_log("Servo OFF initiated...")
            
            # หยุดการเคลื่อนที่ก่อน
            if axis.jog_direction != 0:
                # ถ้ากำลังเคลื่อนที่อยู่ ให้หยุดก่อน
                self.add_to_log("Stopping motion before servo off...")
                axis.jog_direction = 0
                axis.set_profile_velocity(0)
                time.sleep(0.3)
            
            # ตั้งค่า control word เป็น Disable Operation (bit 1 = 1, bit 2 = 1, bit 3 = 0)
            axis.control_word = 0x0007  # Bits 1, 2 set, bit 3 clear
            axis.slave.sdo_write(0x6040, 0, struct.pack("<H", axis.control_word))
            time.sleep(0.3)
            
            # ตรวจสอบสถานะ
            status = struct.unpack("<H", axis.slave.sdo_read(0x6041, 0, 2))[0]
            self.add_to_log(f"Status after disable operation: 0x{status:04X}")
            
            # ตั้งค่าสถานะ servo เป็น disabled
            axis.servo_enabled = False
            self.add_to_log("Servo successfully disabled (OFF)")
            return True
        except Exception as e:
            error_msg = f"Error in servo_off_axis: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
            
    def servo_on_axis(self, axis):
        """Toggle servo state (ON/OFF) on a single axis"""
        try:
            # ถ้า servo เปิดอยู่แล้ว ให้ปิด (Servo OFF)
            if axis.servo_enabled:
                print("Servo already ON, turning OFF...")
                return self.servo_off_axis(axis)
                
            print("Starting servo_on_axis procedure...")
            # ถ้า servo ปิดอยู่ ให้เปิด (Servo ON)
            # Reset any errors first
            self.add_to_log("Resetting errors before servo on...")
            axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x0080))  # Reset
            time.sleep(0.3)  # เพิ่มการรอ
            
            # Clear Position Offset ก่อน Servo On
            self.add_to_log("Clearing position offset...")
            try:
                axis.slave.sdo_write(0x60B0, 0, struct.pack("<i", 0))
                time.sleep(0.3)
            except Exception as e:
                self.add_to_log(f"Warning: Could not clear position offset: {e}")
            
            # First read current position to avoid Er.36
            try:
                # อ่านตำแหน่งปัจจุบันหลายครั้งเพื่อความแน่นอน
                current_pos_readings = []
                for _ in range(3):
                    pos = struct.unpack("<i", axis.slave.sdo_read(0x6064, 0, 4))[0]
                    current_pos_readings.append(pos)
                    time.sleep(0.1)
                
                # ใช้ค่าล่าสุด
                current_pos = current_pos_readings[-1]
                self.add_to_log(f"Current position before servo on: {current_pos}")
                
                # Set this as target position to avoid position jump
                axis.target_position = current_pos
                axis.slave.sdo_write(0x607A, 0, struct.pack("<i", current_pos))
                time.sleep(0.3)  # เพิ่มการรอ
                
                # อ่านค่าที่ตั้งไว้เพื่อตรวจสอบ
                target_check = struct.unpack("<i", axis.slave.sdo_read(0x607A, 0, 4))[0]
                self.add_to_log(f"Target position set to: {target_check}")
                
            except Exception as e:
                error_msg = f"Error reading/setting position: {e}"
                print(error_msg)
                self.add_to_log(error_msg)
                return False
            
            # Shutdown state
            self.add_to_log("Setting drive to Shutdown state...")
            axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x0006))
            time.sleep(0.3)
            
            # ตรวจสอบสถานะหลังจาก Shutdown
            status = struct.unpack("<H", axis.slave.sdo_read(0x6041, 0, 2))[0]
            self.add_to_log(f"Status after shutdown: 0x{status:04X}")
            
            # Verify operation mode is CSP (mode 8)
            op_mode = struct.unpack("b", axis.slave.sdo_read(0x6061, 0, 1))[0]
            if op_mode != 8:
                self.add_to_log(f"Setting CSP mode (current mode: {op_mode})")
                axis.slave.sdo_write(0x6060, 0, struct.pack("b", 8))
                time.sleep(0.3)
                
                # ตรวจสอบว่าโหมดถูกตั้งค่าถูกต้อง
                op_mode_check = struct.unpack("b", axis.slave.sdo_read(0x6061, 0, 1))[0]
                self.add_to_log(f"Operation mode is now: {op_mode_check}")
            
            # Set motion parameters - ตั้งค่าความเร็วต่ำเพื่อความปลอดภัย
            old_vel = axis.profile_velocity
            old_acc = axis.profile_acceleration
            old_dec = axis.profile_deceleration
            
            # ตั้งค่าความเร็วต่ำชั่วคราว
            axis.profile_velocity = 500
            axis.profile_acceleration = 500
            axis.profile_deceleration = 500
            axis.set_motion_parameters()
            self.add_to_log(f"Set temporary low motion parameters: V={axis.profile_velocity}, A={axis.profile_acceleration}")
            
            # Reconfirm target position is set to current position
            self.add_to_log("Reconfirming target position...")
            axis.slave.sdo_write(0x607A, 0, struct.pack("<i", current_pos))
            time.sleep(0.3)
            
            # Switch on
            self.add_to_log("Switching on drive...")
            axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x0007))
            time.sleep(0.5)  # เพิ่มการรอนานขึ้น
            
            # ตรวจสอบสถานะหลังจาก Switch On
            status = struct.unpack("<H", axis.slave.sdo_read(0x6041, 0, 2))[0]
            self.add_to_log(f"Status after switch on: 0x{status:04X}")
            
            # Enable operation
            self.add_to_log("Enabling operation...")
            axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x000F))
            time.sleep(0.5)  # เพิ่มการรอนานขึ้น
            
            # ตรวจสอบสถานะหลังจาก Enable Operation
            status = struct.unpack("<H", axis.slave.sdo_read(0x6041, 0, 2))[0]
            self.add_to_log(f"Status after enable: 0x{status:04X}")
            
            # ตรวจสอบค่า Error ว่ามีหรือไม่
            err = struct.unpack("<H", axis.slave.sdo_read(0x603F, 0, 2))[0]
            if err != 0:
                self.add_to_log(f"Warning: Error code 0x{err:04X} detected after enable")
            
            # Update control word
            axis.control_word = 0x000F
            
            # Force sync via PDO to ensure target position is correct
            self.add_to_log("Sending target position via PDO...")
            outdata = struct.pack("<HiHb", axis.control_word, axis.target_position, 0, 8)
            out_buf = bytearray(axis.slave.output)
            out_buf[:len(outdata)] = outdata
            axis.slave.output = bytes(out_buf)
            
            # คืนค่าพารามิเตอร์การเคลื่อนที่เดิม
            axis.profile_velocity = old_vel
            axis.profile_acceleration = old_acc
            axis.profile_deceleration = old_dec
            axis.set_motion_parameters()
            self.add_to_log(f"Restored motion parameters: V={axis.profile_velocity}, A={axis.profile_acceleration}")
            
            # อัพเดทสถานะ servo
            axis.servo_enabled = True
            self.add_to_log(f"Servo enabled on axis at position {current_pos}")
            print(f">>> SERVO ENABLED - servo_enabled set to {axis.servo_enabled}")
            
            # อัพเดทสถานะ GUI ทันที
            if hasattr(axis, 'gui_refs') and 'servo_status' in axis.gui_refs:
                axis.gui_refs['servo_status'].config(text="ON", foreground="white", background="green")
                if 'servo_button' in axis.gui_refs:
                    axis.gui_refs['servo_button'].config(text="SERVO\nOFF", bg="#dc3545")
                print(">>> GUI servo status updated immediately")
            
            return True
        except Exception as e:
            error_msg = f"Error enabling servo: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
            
    def reset_error(self, axis):
        """Reset error for a specific axis"""
        try:
            # Reset error
            try:
                axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x0080))  # Reset
                time.sleep(0.1)
            except Exception as e:
                print(f"Warning: Could not write reset command: {e}")
                
            # Try to clear error with a different method
            try:
                # Clear error
                axis.slave.sdo_write(0x603F, 0, struct.pack("<H", 0x0000))  # Clear error
                time.sleep(0.1)
            except Exception as e:
                print(f"Warning: Could not clear error register: {e}")
            
            # Try to set shutdown state regardless of previous steps
            try:
                # Shutdown
                axis.slave.sdo_write(0x6040, 0, struct.pack("<H", 0x0006))
                time.sleep(0.1)
                
                # Set control word to Shutdown state
                axis.control_word = 0x0006
            except Exception as e:
                print(f"Warning: Could not set shutdown state: {e}")
            
            self.add_to_log(f"Reset error for axis attempted. Use Servo On to enable operation.")
            return True
        except Exception as e:
            error_msg = f"Error reset failed: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False
            
    def servo_on_all(self):
        """Enable operation on all axes (servo on)"""
        try:
            success_count = 0
            for i, axis in enumerate(self.axes):
                try:
                    if self.servo_on_axis(axis):
                        success_count += 1
                except Exception as e:
                    error_msg = f"Error enabling servo on axis {i}: {e}"
                    print(error_msg)
                    self.add_to_log(error_msg)
                    
            self.add_to_log(f"Servo enabled on {success_count}/{len(self.axes)} axes")
        except Exception as e:
            error_msg = f"Error in servo_on_all: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            
    def copy_log_to_clipboard(self):
        """Copy all log contents to clipboard"""
        try:
            if hasattr(self, 'log_text') and self.log_text:
                log_content = self.log_text.get(1.0, tk.END)
                
                # Clear and set clipboard content
                self.gui.clipboard_clear()
                self.gui.clipboard_append(log_content)
                
                self.add_to_log("Log copied to clipboard")
        except Exception as e:
            error_msg = f"Error copying log to clipboard: {e}"
            print(error_msg)
            self.add_to_log(error_msg)

    def update_footer_time(self):
        """อัพเดทเวลาใน footer"""
        if not self.running:
            return
        
        try:
            self.footer_time_label.config(text=time.strftime("%H:%M:%S"))
        except Exception as e:
            print(f"Error updating footer time: {e}")
            
        # อัพเดททุก 1 วินาที
        if self.gui:
            self.gui.after(1000, self.update_footer_time)
            
    def set_speed_in_rpm(self, axis, rpm_value):
        """ตั้งค่าความเร็วโดยระบุเป็น RPM"""
        try:
            # แปลงค่า RPM เป็น counts/sec
            counts_per_sec = axis.rpm_to_counts(rpm_value)
            
            # ตรวจสอบว่าค่าที่ได้ถูกต้อง
            if counts_per_sec <= 0:
                self.add_to_log(f"ค่า RPM ที่ระบุไม่ถูกต้อง ({rpm_value})")
                return False
                
            # ตั้งค่าความเร็วใหม่
            return self.set_jog_speed(axis, counts_per_sec)
            
        except Exception as e:
            error_msg = f"เกิดข้อผิดพลาดในการตั้งค่าความเร็ว RPM: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def apply_custom_rpm(self, axis, rpm_var):
        """ใช้ความเร็วที่ผู้ใช้กรอกเองในหน่วย RPM"""
        try:
            # รับค่าความเร็วจากช่องกรอก
            rpm_str = rpm_var.get().strip()
            
            # ตรวจสอบว่ามีค่าหรือไม่
            if not rpm_str:
                self.add_to_log("กรุณากรอกค่าความเร็ว RPM")
                return False
                
            # พยายามแปลงเป็นตัวเลข
            try:
                rpm_value = float(rpm_str)
            except ValueError:
                self.add_to_log("ค่าความเร็ว RPM ต้องเป็นตัวเลขเท่านั้น")
                return False
                
            # ตรวจสอบว่าเป็นค่าที่ถูกต้อง (มากกว่า 0)
            if rpm_value <= 0:
                self.add_to_log("ค่าความเร็ว RPM ต้องมากกว่า 0")
                return False
                
            # เรียกใช้ฟังก์ชันตั้งค่าความเร็วเป็น RPM
            result = self.set_speed_in_rpm(axis, rpm_value)
            
            # รีเซ็ตค่าใน radio buttons (ไม่มีตัวเลือกไหนถูกเลือก)
            if result and hasattr(axis, 'jog_speed_vars'):
                for var in axis.jog_speed_vars:
                    var.set("")
                    
            return result
            
        except Exception as e:
            error_msg = f"เกิดข้อผิดพลาดในการตั้งค่าความเร็ว RPM: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def apply_scaling_settings(self):
        """ตั้งค่าการ scaling ทั้งหมดสำหรับทุกแกน"""
        try:
            # รับค่า encoder resolution
            try:
                encoder_res = int(self.encoder_resolution_var.get())
                if encoder_res <= 0:
                    self.add_to_log("ค่า Encoder Resolution ต้องมากกว่า 0")
                    return False
            except ValueError:
                self.add_to_log("ค่า Encoder Resolution ต้องเป็นตัวเลขเท่านั้น")
                return False
            
            # รับค่า application units
            try:
                app_units = int(self.app_units_var.get())
                if app_units <= 0:
                    self.add_to_log("ค่า Application Units ต้องมากกว่า 0")
                    return False
            except ValueError:
                self.add_to_log("ค่า Application Units ต้องเป็นตัวเลขเท่านั้น")
                return False
            
            # รับค่า invert direction
            invert_direction = self.invert_direction_var.get()
            
            # ตั้งค่าทั้งหมดสำหรับทุกแกน
            success_count = 0
            for i, axis in enumerate(self.axes):
                try:
                    # ตั้งค่า encoder resolution
                    axis.encoder_resolution = encoder_res
                    
                    # ตั้งค่า application units
                    axis.app_units_per_rev = app_units
                    
                    # ตั้งค่า direction
                    axis.motor_direction_inverted = invert_direction
                    
                    success_count += 1
                except Exception as e:
                    error_msg = f"Error setting scaling for axis {i}: {e}"
                    print(error_msg)
                    self.add_to_log(error_msg)
            
            # แสดงค่า scaling factor
            scaling_factor = encoder_res / app_units
            self.add_to_log(f"Applied scaling settings to {success_count}/{len(self.axes)} axes")
            self.add_to_log(f"Encoder resolution: {encoder_res} counts/rev")
            self.add_to_log(f"Application units: {app_units} units/rev")
            self.add_to_log(f"Scaling factor: {scaling_factor:.4f} (counts per unit)")
            self.add_to_log(f"Direction inverted: {invert_direction}")
            
            return True
        except Exception as e:
            error_msg = f"Error applying scaling settings: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def apply_stop_mode(self):
        """ตั้งค่าโหมดการหยุดสำหรับทุกแกน"""
        try:
            # รับค่าโหมดการหยุดจาก radio button
            stop_mode = self.stop_mode_var.get()
            
            # ชื่อโหมดสำหรับ log
            mode_name = "Free Stop (ไม่ล็อกเพลา)" if stop_mode == 0 else "Zero-Speed Stop (ล็อกเพลา)"
            
            # ตั้งค่าให้กับทุกแกน
            success_count = 0
            for i, axis in enumerate(self.axes):
                try:
                    # บันทึกค่าในตัวแปรของแกน
                    axis.stop_mode = stop_mode
                    
                    # ตั้งค่าไปยังไดรฟ์โดยตรง
                    axis.set_stop_mode(stop_mode)
                    success_count += 1
                except Exception as e:
                    error_msg = f"Error setting stop mode for axis {i}: {e}"
                    print(error_msg)
                    self.add_to_log(error_msg)
            
            # บันทึก log
            self.add_to_log(f"Applied stop mode to {success_count}/{len(self.axes)} axes")
            self.add_to_log(f"Stop mode: {stop_mode} - {mode_name}")
            
            return True
        except Exception as e:
            error_msg = f"Error applying stop mode: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def emergency_stop_with_servo_on(self, axis):
        """หยุดฉุกเฉินทันทีแล้วเปิด servo กลับเมื่อมอเตอร์หยุด"""
        try:
            # เรียกใช้ emergency_stop เพื่อหยุดมอเตอร์ทันที
            self.add_to_log("Emergency stop initiated...")
            axis.emergency_stop()
            
            # รอให้มอเตอร์หยุดสนิท
            time.sleep(1.0)  # รอ 1 วินาที
            
            # เปิด servo กลับ
            self.add_to_log("Emergency stop complete, turning servo back on...")
            self.servo_on_axis(axis)
            
            self.add_to_log("Emergency stop with servo recovery completed")
            return True
        except Exception as e:
            error_msg = f"Error in emergency stop with recovery: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def emergency_stop_only(self, axis):
        """หยุดฉุกเฉินทันที (Quick Stop: Servo ยัง ON)"""
        try:
            self.add_to_log("EMERGENCY STOP button pressed - Emergency quick stop initiated")
            if axis.quick_stop():
                self.add_to_log("Emergency quick stop completed successfully - Servo still ON")
                return True
            else:
                self.add_to_log("Emergency quick stop failed")
                return False
        except Exception as e:
            error_msg = f"Error in emergency_stop_only: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

    def free_stop_and_servo_on(self, axis):
        """Free Stop (Servo Off) แล้วรอจน actual velocity = 0 แล้ว Servo On"""
        try:
            self.add_to_log("Free Stop initiated (Servo Off, will auto Servo On when stopped)...")
            axis.emergency_stop()
            axis.servo_enabled = False
            # รอจน actual velocity = 0
            for _ in range(100):  # รอสูงสุด 10 วินาที (100 x 0.1)
                status, _ = axis.read_status()
                if abs(axis.actual_velocity) < 1:
                    break
                time.sleep(0.1)
            self.add_to_log("Axis stopped, turning Servo ON...")
            self.servo_on_axis(axis)
            return True
        except Exception as e:
            error_msg = f"Error in free_stop_and_servo_on: {e}"
            print(error_msg)
            self.add_to_log(error_msg)
            return False

if __name__ == "__main__":
    controller = CSPController()
    
    # แยกการเชื่อมต่อและการแสดง UI ออกจากกัน
    # ถึงแม้ว่าการเชื่อมต่อจะล้มเหลว UI ก็จะยังแสดงและแจ้งเตือนผู้ใช้
    try:
        print("[" + time.strftime("%H:%M:%S") + "] Attempting to connect to EtherCAT hardware...")
        connection_success = controller.connect("\\Device\\NPF_{D11C2F0E-5365-4007-B7C6-9C025AAF0799}")
        if connection_success:
            controller.setup_all_axes()
            controller.start_loop()
            # เริ่ม UI เสมอแม้มีปัญหากับ EtherCAT
            controller.add_to_log("Connected to EtherCAT hardware successfully")
            controller.is_connected = True
            print("[" + time.strftime("%H:%M:%S") + "] Connected to EtherCAT hardware successfully")
        else:
            # แม้เชื่อมต่อไม่สำเร็จก็ยังเปิด UI
            print("[" + time.strftime("%H:%M:%S") + "] Failed to connect to EtherCAT hardware")
            controller.add_to_log("WARNING: Failed to connect to EtherCAT hardware - UI started in offline mode")
            controller.is_connected = False
    except Exception as e:
        # จับข้อผิดพลาดทั้งหมดเพื่อให้ UI ยังเปิดได้
        error_msg = f"ERROR: Connection to EtherCAT failed: {e}"
        print("[" + time.strftime("%H:%M:%S") + "] " + error_msg)
        controller.add_to_log(error_msg)
        controller.is_connected = False
    
    # เปิด UI เสมอไม่ว่าจะเชื่อมต่อสำเร็จหรือไม่
    try:
        # เริ่ม GUI และรันในลูป - จะหยุดการทำงานเมื่อหน้าต่างถูกปิด
        controller.launch_gui()
        
        # ไม่ต้องเรียก update_connection_status_display ที่นี่อีกต่อไป
        # เพราะได้ย้ายไปเรียกใน launch_gui แล้ว
        
        # ไม่ต้องเรียก stop() ที่นี่แล้ว เพราะมีการเรียกในเมธอด on_close ของ GUI แล้ว
    except Exception as e:
        print(f"Error in GUI: {e}")
        try:
            # แต่ถ้าเกิดข้อผิดพลาดในการเริ่ม GUI ยังต้องเรียก stop เพื่อหยุดการทำงานอย่างปลอดภัย
            controller.stop()
        except Exception as cleanup_error:
            print(f"Error during cleanup: {cleanup_error}")
            # ถ้าไม่สามารถเรียก stop() ได้ ให้พยายามปิดการเชื่อมต่อตามที่ทำได้
            try:
                # หยุดรอบการทำงาน
                controller.running = False
                time.sleep(0.3)
                
                # พยายามทำให้ slave อยู่ในสถานะ INIT
                if hasattr(controller, 'master') and controller.master:
                    controller.master.state = pysoem.INIT_STATE
                    controller.master.write_state()
                    time.sleep(0.2)
                    controller.master.close()
            except Exception as final_error:
                print(f"Final cleanup error: {final_error}")
    
    print("Application terminated")
