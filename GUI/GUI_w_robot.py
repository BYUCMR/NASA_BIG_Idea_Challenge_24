import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox, QLineEdit, QMessageBox, QLabel, QTextEdit
from PyQt5.QtCore import Qt, QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pygame
from Robot import *
import time
import serial

'''
use pyserial
baud 115200
set up serial device use com...
get data
ser.write(data.encode())
send every 5-10 ms
'''

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = plt.figure(figsize=(width, height), dpi=dpi)
        self.ax = fig.add_subplot(111, projection='3d')
        super(MplCanvas, self).__init__(fig)

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle('Robot Control Interface')
        self.setGeometry(100, 100, 800, 600)
        self.node_list = []
        self.num_points = 6
        self.robot_array = np.array([])

        self.robot_is_created = False
        self.movement_vector = [0,0,0]
        self.view = (0,0)
        self.elev = 30
        self.azim = -60

        # Main widget
        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        self.layout = QHBoxLayout(self.main_widget)

        # Matplotlib canvas
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        self.layout.addWidget(self.canvas)

        # Control panel
        self.control_panel = QVBoxLayout()
        self.layout.addLayout(self.control_panel)

        # Control panel
        self.control_panel_widget = QWidget()  # Create a widget to hold the control panel
        self.control_panel_widget.setFixedWidth(250)  # Set the control panel width to 100
        self.control_panel = QVBoxLayout(self.control_panel_widget)
        self.layout.addWidget(self.control_panel_widget)  # A


      
        self.button1 = QPushButton('Octahedron')
        self.button2 = QPushButton('Solar')
        self.button3 = QPushButton('Crane')

        # Connect buttons to the same method with different parameters
        self.button1.clicked.connect(lambda: self.robot_preset(1))
        self.button2.clicked.connect(lambda: self.robot_preset(2))
        self.button3.clicked.connect(lambda: self.robot_preset(3))

        # Create layouts
        button_layout = QVBoxLayout()
        button_layout.addWidget(self.button1)
        button_layout.addWidget(self.button2)
        button_layout.addWidget(self.button3)
        self.control_panel.addLayout(button_layout)

        # # Create robot button
        # self.btn_create_robot = QPushButton('Create Robot')
        # self.btn_create_robot.clicked.connect(self.create_robot)
        # self.control_panel.addWidget(self.btn_create_robot)
       
        self.support_nodes = QLabel(f"Support Nodes: (1,2,3)")
        self.support_nodes.setFixedHeight(15)
        self.control_panel.addWidget(self.support_nodes)

        # Node selection dropdown
        self.node_dropdown = QComboBox()
        self.control_panel.addWidget(self.node_dropdown)
        self.node_dropdown.currentIndexChanged.connect(self.on_change_node)

        # Movement buttons
        self.btn_up = QPushButton('Up')
        self.btn_up.clicked.connect(lambda: self.move_node('up'))
        self.control_panel.addWidget(self.btn_up)

        self.btn_down = QPushButton('Down')
        self.btn_down.clicked.connect(lambda: self.move_node('down'))
        self.control_panel.addWidget(self.btn_down)

        self.btn_left = QPushButton('Left')
        self.btn_left.clicked.connect(lambda: self.move_node('left'))
        self.control_panel.addWidget(self.btn_left)

        self.btn_right = QPushButton('Right')
        self.btn_right.clicked.connect(lambda: self.move_node('right'))
        self.control_panel.addWidget(self.btn_right)

        self.btn_forward = QPushButton('Forward')
        self.btn_forward.clicked.connect(lambda: self.move_node('forward'))
        self.control_panel.addWidget(self.btn_forward)

        self.btn_backward = QPushButton('Backward')
        self.btn_backward.clicked.connect(lambda: self.move_node('backward'))
        self.control_panel.addWidget(self.btn_backward)

        self.start_button = QPushButton("Start Export", self)
        self.start_button.clicked.connect(self.start_export)
        self.control_panel.addWidget(self.start_button)

        self.stop_button = QPushButton("Stop Export", self)
        self.stop_button.clicked.connect(self.stop_export)
        self.control_panel.addWidget(self.stop_button)

        self.interval_label = QLabel("Time Interval (ms):", self)
        self.control_panel.addWidget(self.interval_label)
        self.interval_input = QLineEdit(self)
        self.interval_input.setText("1000")  # Default 1 second
        self.control_panel.addWidget(self.interval_input)

        self.status_text = QTextEdit(self)
        self.status_text.setReadOnly(True)
        self.control_panel.addWidget(self.status_text)

        # Set layout
        container = QWidget()
        container.setLayout(self.layout)
        self.setCentralWidget(container)

        # Initialize serial communication
        self.serial_port = None
        self.data_timer = QTimer(self)
        self.data_timer.timeout.connect(self.send_data)

        self.data = b'No robot created\n'
       
        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            self.joystick = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_joystick)
        self.timer.start(100) 

    def start_export(self):
        try:
            self.serial_port = serial.Serial('COM4', 115200, timeout=1) 
            self.status_text.append("Serial port opened successfully")

            # Start sending data at desired intervals
            interval = int(self.interval_input.text())  # Get the interval from the input field
            self.data_timer.start(interval)
            self.status_text.append(f"Export started. Sending data every {interval} ms.")
        except Exception as e:
            self.status_text.append(f"Error opening serial port: {str(e)}")

    def stop_export(self):
        # Stop sending data and close the serial port
        self.timer.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.status_text.append("Serial port closed. Export stopped.")

    def send_data(self):
        if self.robot_is_created:
            data_string = ",".join([str(int(x * 1000)) for x in self.robot.node_positions_along_tube[-1,:]])
            self.data = data_string.encode('utf-8')

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(self.data)
            except Exception as e:
                self.status_text.append(f"Error sending data: {str(e)}")

    def on_change_node(self):
        self.update_canvas()
        if not self.robot_is_created:
            return
        if self.node_dropdown.currentIndex() in self.robot.support:
            self.show_warning()
            self.node_dropdown.setCurrentIndex(4)

    def show_warning(self):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setWindowTitle("Warning")
        msg.setText("Cannot move support node")
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def robot_preset(self, robot):
        if robot == 1:
            self.create_robot(1)
        elif robot == 2:
            self.create_robot(2)

    def create_robot(self, type):
        self.robot_is_created = False
        self.robot = Robot(type)
        self.data = self.robot.node_positions_along_tube[-1,:]
        self.node_list = [f"Node {i+1}" for i in range(np.shape(self.robot.pos)[0])]   
        self.node_dropdown.clear()
        self.node_dropdown.addItems(self.node_list)
        self.node_dropdown.setCurrentIndex(4)
        self.robot_is_created = True
        self.update_canvas()

    def update_canvas(self):
        if not self.robot_is_created:
            return  
        
        self.canvas.ax.clear()

        self.plot_robot()
        # self.canvas.ax.view_init(elev=self.elev, azim=self.azim)
        
        self.canvas.draw()

    def move_node(self, direction):
        index = self.node_dropdown.currentIndex()
        increment_step = 0.1
        if index >= 0:
            if direction == 'up':
                self.robot.move(index, [0,0,increment_step])
            elif direction == 'down':
                self.robot.move(index, [0,0,-increment_step])
            elif direction == 'left':
                self.robot.move(index, [-increment_step,0,0])
            elif direction == 'right':
                self.robot.move(index, [increment_step,0,0])
            elif direction == 'forward':
                self.robot.move(index, [0,increment_step,0])
            elif direction == 'backward':
                self.robot.move(index, [0,-increment_step,0])
            self.update_canvas()

    def read_joystick(self):
        if not self.robot_is_created:
            return
        if self.joystick:
            pygame.event.pump()
            axis_x = self.joystick.get_axis(1)
            axis_y = self.joystick.get_axis(0)
            axis_z = self.joystick.get_axis(2)

            view = self.joystick.get_hat(0)
            pan_constant = 2
            # self.elev = self.elev + view[1] * pan_constant
            # self.azim = self.azim + view[0] * pan_constant
            
            increment_step = 0.1

            index = self.node_dropdown.currentIndex()
            move_vector = []
            if index >= 0:
                if abs(axis_x) >= 0.5:
                    move_vector.append(axis_x*increment_step)
                else:
                    move_vector.append(0)
                if abs(axis_y) >= 0.5:
                    move_vector.append(axis_y*increment_step)
                else:
                    move_vector.append(0)
                if abs(axis_z) >= 0.5:
                    move_vector.append(axis_z*increment_step)
                else:
                    move_vector.append(0)
            if not move_vector == [0,0,0]:
                self.robot.move(index, move_vector)
            self.movement_vector = move_vector
            self.update_canvas()

    def plot_robot(self):

        x,y,z,edges = self.robot.plot_robot()
        # Plot the points
        index = self.node_dropdown.currentIndex()
        self.canvas.ax.scatter(x, y, z, color='b', s=50, label='Points')
        self.canvas.ax.scatter(x[index], y[index], z[index], color='r', s=75)

        for i in range(len(x)):
            self.canvas.ax.text(x[i], y[i], z[i]-0.1, f'{i+1}', color='black', fontsize=12, ha='center', va='center')

        self.canvas.ax.quiver(x[index],y[index],z[index], self.movement_vector[0], self.movement_vector[1], self.movement_vector[2], color='b')
        

        # Plot the edges
        triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
        for idx, edge in enumerate(edges):
            p1, p2 = edge
            self.canvas.ax.plot([x[p1], x[p2]], [y[p1], y[p2]], [z[p1], z[p2]], triangle_colors[idx//3], lw=1.5)

        # Add labels
        self.canvas.ax.set_xlabel('X')
        self.canvas.ax.set_ylabel('Y')
        self.canvas.ax.set_zlabel('Z')

        # Add grid and legend
        self.canvas.ax.grid(True)
        self.canvas.ax.set_aspect('equal')




app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
