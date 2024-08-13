import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QHBoxLayout,
    QTabWidget,
    QWidget,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QComboBox,
    QGridLayout,
    QScrollArea,
)
from PyQt5.QtCore import pyqtSlot, QTimer, QDateTime


class SerialControl(QMainWindow):
    def __init__(self):
        super().__init__()
        self.profile_count = 4
        self.initUI()

    def initUI(self):
        # Main layout
        self.setWindowTitle("Arduino-AD9958 Control")
        self.setGeometry(100, 100, 1200, 800)  # Increase the size for better layout

        # Main vertical layout
        main_vertical_layout = QVBoxLayout()

        # Device selection layout
        device_layout = QHBoxLayout()

        self.master_device_list = QComboBox()
        self.master_device_list.addItem("Select Master Device")
        self.refresh_master_devices()
        device_layout.addWidget(QLabel("Master Device:"))
        device_layout.addWidget(self.master_device_list)

        self.connect_master_btn = QPushButton("Connect")
        self.connect_master_btn.clicked.connect(self.connect_master_device)
        device_layout.addWidget(self.connect_master_btn)

        self.slave_device_list = QComboBox()
        self.slave_device_list.addItem("Select Slave Device")
        device_layout.addWidget(QLabel("Slave Device:"))
        device_layout.addWidget(self.slave_device_list)

        self.connect_slave_btn = QPushButton("Connect")
        self.connect_slave_btn.clicked.connect(self.connect_slave_device)
        device_layout.addWidget(self.connect_slave_btn)

        main_vertical_layout.addLayout(device_layout)

        # Horizontal layout for log box and right-side panels
        main_horizontal_layout = QHBoxLayout()

        # Log box
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        main_horizontal_layout.addWidget(
            self.log_box, 3
        )  # Give more space to the log box (stretch factor 3)

        # Right-hand side layout for tabs
        right_layout = QVBoxLayout()

        # Tabs for Channel 0, Channel 1, and Profiles
        self.tab_widget = QTabWidget()
        self.channel_0_tab = QWidget()
        self.channel_1_tab = QWidget()
        self.profile_tab = QWidget()

        self.create_channel_tab(self.channel_0_tab, 0)
        self.create_channel_tab(self.channel_1_tab, 1)
        self.create_profile_tab(self.profile_tab)

        self.tab_widget.addTab(self.channel_0_tab, "Channel 0")
        self.tab_widget.addTab(self.channel_1_tab, "Channel 1")
        self.tab_widget.addTab(self.profile_tab, "Profiles")

        right_layout.addWidget(self.tab_widget)

        main_horizontal_layout.addLayout(
            right_layout, 2
        )  # Adjust the ratio to give more space to the log box (stretch factor 2)

        # Add the horizontal layout to the main vertical layout
        main_vertical_layout.addLayout(main_horizontal_layout)

        # Set main layout
        main_widget = QWidget()
        main_widget.setLayout(main_vertical_layout)
        self.setCentralWidget(main_widget)

    def refresh_master_devices(self):
        ports = serial.tools.list_ports.comports()
        self.master_device_list.clear()
        self.master_device_list.addItem("Select Master Device")
        for port in ports:
            self.master_device_list.addItem(port.device)

    def connect_master_device(self):
        selected_port = self.master_device_list.currentText()
        if selected_port != "Select Master Device":
            try:
                self.master_serial = serial.Serial(selected_port, 115200, timeout=1)
                self.log_entry(f"Connected to master device: {selected_port}")
                self.request_slave_devices()
            except Exception as e:
                self.log_entry(f"Failed to connect to master device: {str(e)}")
                self.master_serial = None

    def request_slave_devices(self):
        try:
            self.master_serial.write(b"REQUEST_SLAVES\n")
            slave_devices = self.master_serial.readline().decode().strip().split(",")
            self.slave_device_list.clear()
            self.slave_device_list.addItem("Select Slave Device")
            for device in slave_devices:
                self.slave_device_list.addItem(device)
            self.log_entry(f"Slave devices: {', '.join(slave_devices)}")
        except Exception as e:
            self.log_entry(f"Failed to request slave devices: {str(e)}")

    def connect_slave_device(self):
        selected_slave = self.slave_device_list.currentText()
        if selected_slave != "Select Slave Device":
            try:
                self.master_serial.write(f"CONNECT_SLAVE {selected_slave}\n".encode())
                response = self.master_serial.readline().decode().strip()
                self.log_entry(f"Slave device response: {response}")
            except Exception as e:
                self.log_entry(f"Failed to connect to slave device: {str(e)}")

    def create_channel_tab(self, tab, channel):
        layout = QVBoxLayout()
        grid_layout = QGridLayout()

        # Frequency
        freq_label = QLabel("Frequency:")
        grid_layout.addWidget(freq_label, 0, 0)

        self.freq_input = QLineEdit()
        grid_layout.addWidget(self.freq_input, 0, 1)

        freq_write_btn = QPushButton("Write")
        freq_write_btn.clicked.connect(lambda: self.write_value(channel, "frequency"))
        grid_layout.addWidget(freq_write_btn, 0, 2)

        # Amplitude
        amp_label = QLabel("Amplitude:")
        grid_layout.addWidget(amp_label, 1, 0)

        self.amp_input = QLineEdit()
        grid_layout.addWidget(self.amp_input, 1, 1)

        amp_write_btn = QPushButton("Write")
        amp_write_btn.clicked.connect(lambda: self.write_value(channel, "amplitude"))
        grid_layout.addWidget(amp_write_btn, 1, 2)

        # Phase
        phase_label = QLabel("Phase:")
        grid_layout.addWidget(phase_label, 2, 0)

        self.phase_input = QLineEdit()
        grid_layout.addWidget(self.phase_input, 2, 1)

        phase_write_btn = QPushButton("Write")
        phase_write_btn.clicked.connect(lambda: self.write_value(channel, "phase"))
        grid_layout.addWidget(phase_write_btn, 2, 2)

        layout.addLayout(grid_layout)
        tab.setLayout(layout)

    def create_profile_tab(self, tab):
        layout = QVBoxLayout()
        self.profile_layout = QGridLayout()
        self.profile_tabs = QWidget()
        self.profile_scroll_area = QScrollArea()
        self.profile_scroll_area.setWidgetResizable(True)

        # Add headers
        self.profile_layout.addWidget(QLabel("Profile"), 0, 0)
        self.profile_layout.addWidget(QLabel("Frequency"), 0, 1)
        self.profile_layout.addWidget(QLabel("Amplitude"), 0, 2)
        self.profile_layout.addWidget(QLabel("Channel"), 0, 3)

        self.profiles = []
        self.add_profile_rows(self.profile_count)

        add_profile_btn = QPushButton("Add Profile")
        add_profile_btn.clicked.connect(self.add_profile)
        layout.addWidget(add_profile_btn)

        read_profiles_btn = QPushButton("Read Profiles")
        read_profiles_btn.clicked.connect(self.read_profiles)
        layout.addWidget(read_profiles_btn)

        write_profiles_btn = QPushButton("Write Profiles")
        write_profiles_btn.clicked.connect(self.write_profiles)
        layout.addWidget(write_profiles_btn)

        self.profile_tabs.setLayout(self.profile_layout)
        self.profile_scroll_area.setWidget(self.profile_tabs)
        layout.addWidget(self.profile_scroll_area)

        tab.setLayout(layout)

    def add_profile_rows(self, count):
        start_row = len(self.profiles) + 1
        for i in range(count):
            row_num = start_row + i
            profile_label = QLabel(f"{row_num}")
            freq_edit = QLineEdit()
            amp_edit = QLineEdit()
            channel_edit = QLineEdit()

            self.profile_layout.addWidget(profile_label, row_num, 0)
            self.profile_layout.addWidget(freq_edit, row_num, 1)
            self.profile_layout.addWidget(amp_edit, row_num, 2)
            self.profile_layout.addWidget(channel_edit, row_num, 3)

            self.profiles.append((freq_edit, amp_edit, channel_edit))

    def add_profile(self):
        if self.profile_count < 8:
            self.profile_count += 1
            self.add_profile_rows(1)

    def log_entry(self, message):
        timestamp = QDateTime.currentDateTime().toString("HH:mm:ss")
        self.log_box.append(f"{timestamp} - {message}")

    def read_serial_messages(self):
        if self.master_serial and self.master_serial.isOpen():
            try:
                while self.master_serial.in_waiting:
                    message = self.master_serial.readline().decode().strip()
                    self.log_entry(message)
            except Exception as e:
                self.log_entry(f"Error reading from serial port: {str(e)}")

    def read_profiles(self):
        try:
            self.master_serial.write(b"READ PROFILES\n")
            response = self.master_serial.readline().decode().strip()
            profile_data = response.split(";")
            for i, profile in enumerate(profile_data):
                freq, amp, channel = profile.split(",")
                self.profiles[i][0].setText(freq)
                self.profiles[i][1].setText(amp)
                self.profiles[i][2].setText(channel)
            self.log_entry(f"Read profiles: {response}")
        except Exception as e:
            self.log_entry(f"Failed to read profiles: {str(e)}")

    def write_profiles(self):
        try:
            profile_data = []
            for i in range(self.profile_count):
                freq = self.profiles[i][0].text()
                amp = self.profiles[i][1].text()
                channel = self.profiles[i][2].text()
                profile_data.append(f"{freq},{amp},{channel}")
            command = f"WRITE PROFILES {self.profile_count} {';'.join(profile_data)}\n"
            self.master_serial.write(command.encode())
            response = self.master_serial.readline().decode().strip()
            self.log_entry(f"Wrote profiles: {response}")
        except Exception as e:
            self.log_entry(f"Failed to write profiles: {str(e)}")

    def write_value(self, channel, value_type):
        try:
            command = ""
            if value_type == "frequency":
                value = self.freq_input.text()
                command = f"C0{channel:01d};F0{value.zfill(4)}"
            elif value_type == "amplitude":
                value = self.amp_input.text()
                command = f"C0{channel:01d};A0{value}"
            elif value_type == "phase":
                value = self.phase_input.text()
                command = f"C0{channel:01d};P0{value.zfill(3)}"
            elif value_type == "profile_freq":
                value = self.freq_input.text()
                profile_index = 0  # Assume this is provided somewhere
                command = f"M{profile_index}{value.zfill(4)}"
            elif value_type == "profile_amp":
                value = self.amp_input.text()
                profile_index = 0  # Assume this is provided somewhere
                command = f"N{profile_index}{value}"
            elif value_type == "profile_channel":
                value = self.channel_input.text()
                profile_index = 0  # Assume this is provided somewhere
                command = f"B{profile_index}{value}"

            if self.master_serial:
                self.master_serial.write(f"{command}".encode())
                response = self.master_serial.readline().decode().strip()
                self.log_entry(f"Write {value_type} to channel {channel}: {response}")
            else:
                self.log_entry("No master serial connection.")
        except Exception as e:
            self.log_entry(
                f"Failed to write {value_type} to channel {channel}: {str(e)}"
            )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = SerialControl()
    ex.show()
    sys.exit(app.exec_())
