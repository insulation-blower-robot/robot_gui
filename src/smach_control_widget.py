import os
import rospy
import rospkg
import roslaunch
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import pyqtSignal, QTimer
from mission_manager.srv import Trigger, TriggerResponse
from mission_manager.msg import Context
from mission_manager.node_manager import NodeManager
from cavity_detection_api.api import move_roi, update_roi, split_roi, mark_target
from cavity_detection_msgs.msg import RoiList, Roi


class SmachControlWidget(QWidget):

    context_updated = pyqtSignal(str)

    def __init__(self):
        super(SmachControlWidget, self).__init__()
        self.setObjectName('SmachControlWidget')
        self.context = None
        self.nodes = None
        self.current_target = None
        self.target_list = []
        self.target_dict = {}
        layout = QVBoxLayout()
        self.context_updated.connect(self._update_context_label)

        # Create buttons
        self.btn_start_robot = QPushButton("START ROBOT")
        self.btn_start_exploring = QPushButton("START EXPLORING")
        self.btn_done_exploring = QPushButton("DONE EXPLORING")
        self.btn_goto_target = QPushButton("GO TO TARGET")
        self.btn_start_blowing = QPushButton("START BLOWING")
        self.btn_done_blowing = QPushButton("DONE BLOWING")
        self.btn_estop = QPushButton("MANUAL STOP")

        # Create radio buttons for exploration method
        exp = QGroupBox("Exploration")
        exp_layout = QVBoxLayout()
        self.exp_teleop = QRadioButton("Teleop")
        self.exp_auto = QRadioButton("Autonomous")
        exp_layout.addWidget(self.exp_teleop)
        exp_layout.addWidget(self.exp_auto)
        self.exp_teleop.toggled.connect(self.update_exp_method)
        self.exp_auto.toggled.connect(self.update_exp_method)
        exp_layout.addWidget(self.btn_start_exploring)
        exp_layout.addWidget(self.btn_done_exploring)
        exp.setLayout(exp_layout)
        exp.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)

        target = QGroupBox("Target")
        tar_layout = QVBoxLayout()
        self.target_dropdown = QComboBox()
        tar_layout.addWidget(self.target_dropdown)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_dropdown)
        self.timer.start(1000)         
        self.target_dropdown.currentIndexChanged.connect(self.on_target_selected)
        
        move = QGroupBox("Move Target")
        move_layout = QFormLayout()
        self.spin_x = QDoubleSpinBox()
        self.spin_y = QDoubleSpinBox()
        self.spin_theta = QDoubleSpinBox()
        self.btn_move = QPushButton("Move")
        move_layout.addRow("x: ", self.spin_x)
        move_layout.addRow("y: ", self.spin_y)
        move_layout.addRow("θ: ", self.spin_theta)
        move_layout.addRow(self.btn_move)
        move.setLayout(move_layout)

        update = QGroupBox("Update Target Attributes")
        update_layout = QFormLayout()
        self.spin_length = QDoubleSpinBox()
        self.spin_height = QDoubleSpinBox()
        self.spin_spacing = QDoubleSpinBox()
        self.spin_num_cavities = QSpinBox()
        self.btn_update = QPushButton("Update")
        update_layout.addRow("Length: ", self.spin_length) 
        update_layout.addRow("Height: ", self.spin_height)
        update_layout.addRow("Spacing: ", self.spin_spacing)
        update_layout.addRow("Cavities: ", self.spin_num_cavities)
        update_layout.addRow(self.btn_update)
        update.setLayout(update_layout)

        # Split controls
        split = QGroupBox("Split Target")
        split_layout = QFormLayout()  
        self.spin_from = QSpinBox()
        self.spin_to = QSpinBox()
        self.btn_split = QPushButton("Split Target")
        split_layout.addRow("From: ", self.spin_from)
        split_layout.addRow("To: ", self.spin_to)
        split_layout.addRow(self.btn_split)  
        split.setLayout(split_layout) 

        # Go to target button
        tar_layout.addWidget(move)        
        tar_layout.addWidget(update)        
        tar_layout.addWidget(split)
        tar_layout.addWidget(self.btn_goto_target)
        target.setLayout(tar_layout)

        # Create radio buttons for the fill method
        fill = QGroupBox("Fill Method")
        fill_layout = QVBoxLayout()
        self.fill_cont = QRadioButton("Continuous")
        self.fill_disc = QRadioButton("Discrete")
        self.fill_cont.toggled.connect(self.update_fill_method)
        self.fill_disc.toggled.connect(self.update_fill_method)
        fill_layout.addWidget(self.fill_cont)
        fill_layout.addWidget(self.fill_disc)
        fill.setLayout(fill_layout)
        fill.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)

        # Create a label for the status
        self.context_label = QLabel()
        # Add buttons to layout
        layout.addWidget(self.context_label)
        layout.addWidget(exp)
        layout.addWidget(target)
        layout.addWidget(fill)
        layout.addWidget(self.btn_start_blowing)
        layout.addWidget(self.btn_done_blowing)
        layout.addWidget(self.btn_estop)
        self.setLayout(layout)

        # Connect buttons (adjust names if using .ui file)
        self.btn_start_robot.clicked.connect(lambda: self.start())
        self.btn_start_exploring.clicked.connect(lambda: self._send_trigger('START EXPLORING'))
        self.btn_done_exploring.clicked.connect(lambda: self._send_trigger('CONFIRM EXPLORING'))
        self.btn_goto_target.clicked.connect(lambda: self._send_trigger('CONFIRM TARGET', self.context.current_target))
        self.btn_start_blowing.clicked.connect(lambda: self._send_trigger('CONFIRM BLOWING', self.context.current_target))
        self.btn_done_blowing.clicked.connect(lambda: self._send_trigger('CONFIRM FILLED', self.context.current_target))
        self.btn_estop.clicked.connect(lambda: self._send_trigger('MANUAL STOP'))

        self.btn_move.clicked.connect(lambda: self._move_target())
        self.btn_update.clicked.connect(lambda: self._update_target())
        self.btn_split.clicked.connect(lambda: self._split_target())

        # Subscribe to the state machine context topic
        rospy.Subscriber('/smach_context', Context, self._context_callback)
        rospy.Subscriber('/cavity_detection/target_list', RoiList, self._targets_callback)
        self.smach_trigger_service = None

        # Setup the roslaunch object during initialization
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file_path = roslaunch.rlutil.resolve_launch_arguments(["teleop_twist_joy", "teleop.launch"])[0]
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])

        self.start()

    def start(self):
        self.btn_start_robot.setEnabled(False)
        service_name = '/smach_gui_confirmation'
        rospy.loginfo(f"Waiting for SMACH control service: {service_name}")
        rospy.wait_for_service(service_name, timeout=30.0)
        self.smach_trigger_service = rospy.ServiceProxy(service_name, Trigger)
        rospy.loginfo(f"Connected to SMACH control service: {service_name}")
        self.launch.start()
        rospy.loginfo("Robot manager started")

    def _update_context_label(self, text):
        self.context_label.setText(text)

    def _context_callback(self, msg):
        self.context = msg
        self.context_updated.emit(f"Current State: {self.context.current_state}")
    
    def _targets_callback(self, msg):
        self.target_list = []
        self.target_dict = {}
        for roi in msg.list:
            self.target_list.append(roi.id)
            self.target_dict[roi.id] = roi


    def _send_trigger(self, button, target=""):
        response = self.smach_trigger_service.call(button, target)
        rospy.loginfo(response.message)
    
    def _move_target(self):
        dx = self.spin_x.value()
        dy = self.spin_y.value()
        dtheta = self.spin_theta.value()
        move_roi(self.current_target, dtheta, dx, dy)

    def _update_target(self):
        length = self.spin_length.value()
        height = self.spin_height.value()
        spacing = self.spin_spacing.value()
        num_cavities = self.spin_num_cavities.value()
        update_roi(self.current_target, length, height, spacing, num_cavities)
    
    def _split_target(self):
        start = self.spin_from.value()
        end = self.spin_to.value()
        split_roi(self.current_target, start, end)

    def shutdown_widget(self):
        # Clean up resources, service proxies etc.
        if self.smach_trigger_service:
            self.smach_trigger_service.close()
        rospy.loginfo("Shutting down SmachControlWidget")

    def update_dropdown(self):
        current_items = [self.target_dropdown.itemText(i) for i in range(self.target_dropdown.count())]
        if current_items != self.target_list:
            self.target_dropdown.clear()
            self.target_dropdown.addItems(self.target_list)
    
    def on_target_selected(self, index):
        if index < 0:
            return  
        self.current_target = self.target_dropdown.currentText().strip()
        assert isinstance(self.current_target, str)
        assert self.current_target.isascii()            
        print(f"Selected target: {self.current_target}")
        mark_target(self.current_target)
        roi = self.target_dict.get(self.current_target)
        if roi:
            self.spin_x.setValue(0.0)
            self.spin_y.setValue(0.0)
            self.spin_theta.setValue(0.0)
            self.spin_length.setValue(roi.length)
            self.spin_height.setValue(roi.depth)
            self.spin_spacing.setValue(roi.cavity_width)
            self.spin_num_cavities.setValue(roi.num_cavities)

    def update_exp_method(self):
        if self.exp_teleop.isChecked():
            rospy.set_param("/exploration/method", "teleop")
        elif self.exp_auto.isChecked():
            rospy.set_param("/exploration/method", "autonomous")

    def update_fill_method(self):
        if self.fill_cont.isChecked():
            rospy.set_param("/fill_speed_controller/strategy", "continuous")
        elif self.fill_disc.isChecked():
            rospy.set_param("/fill_speed_controller/strategy", "discrete")

class SmachControlPlugin(Plugin):
    """
    RQT Plugin main class
    """
    def __init__(self, context):
        super(SmachControlPlugin, self).__init__(context)
        self.setObjectName('SmachControlPlugin')
        if not rospy.core.is_initialized():
            rospy.init_node('robot_gui_node', anonymous=True, disable_signals=True)
        self._widget = SmachControlWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_widget()