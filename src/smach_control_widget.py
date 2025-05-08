import os
import rospy
import rospkg
import roslaunch
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QLabel, QGroupBox, QRadioButton, QSizePolicy
from python_qt_binding.QtCore import pyqtSignal
from mission_manager.srv import Trigger, TriggerResponse
from mission_manager.msg import Context
from mission_manager.node_manager import NodeManager


class SmachControlWidget(QWidget):

    context_updated = pyqtSignal(str)

    def __init__(self):
        super(SmachControlWidget, self).__init__()
        self.setObjectName('SmachControlWidget')
        self.context = None
        self.nodes = None
        layout = QVBoxLayout()
        self.context_updated.connect(self._update_context_label)

        # Create buttons
        self.btn_start_robot = QPushButton("START ROBOT")
        self.btn_start_exploring = QPushButton("START EXPLORING")
        self.btn_done_exploring = QPushButton("DONE EXPLORING")
        self.btn_goto_target = QPushButton("GO TO TARGET")
        self.btn_start_blowing = QPushButton("START BLOWING")
        self.btn_done_blowing = QPushButton("DONE BLOWING")
        self.btn_estop = QPushButton("EMERGENCY STOP")

        # Create radio buttons for exploration method
        exp = QGroupBox("Exploration Method")
        exp_layout = QVBoxLayout()
        self.exp_teleop = QRadioButton("Teleop")
        self.exp_auto = QRadioButton("Autonomous")
        self.exp_teleop.toggled.connect(self.update_exp)
        self.exp_auto.toggled.connect(self.update_exp)
        exp_layout.addWidget(self.exp_teleop)
        exp_layout.addWidget(self.exp_auto)
        exp.setLayout(exp_layout)
        exp.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        # Create radio buttons for the fill method
        fill = QGroupBox("Fill Method")
        fill_layout = QVBoxLayout()
        self.fill_cont = QRadioButton("Continuous")
        self.fill_disc = QRadioButton("Discrete")
        self.fill_cont.toggled.connect(self.update_fill)
        self.fill_disc.toggled.connect(self.update_fill)
        fill_layout.addWidget(self.fill_cont)
        fill_layout.addWidget(self.fill_disc)
        fill.setLayout(fill_layout)
        fill.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)

        # Create a label for the status
        self.context_label = QLabel()
        # Add buttons to layout
        layout.addWidget(self.context_label)
        layout.addWidget(exp)
        layout.addWidget(self.btn_start_robot)
        layout.addWidget(self.btn_start_exploring)
        layout.addWidget(self.btn_done_exploring)
        layout.addWidget(self.btn_goto_target)
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
        self.btn_estop.clicked.connect(lambda: self._send_trigger('ESTOP'))

        # Subscribe to the state machine context topic
        rospy.Subscriber('/smach_context', Context, self._context_callback)
        self.smach_trigger_service = None

                # Setup the roslaunch object during initialization
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file_path = roslaunch.rlutil.resolve_launch_arguments(["teleop_twist_joy", "teleop.launch"])[0]
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])

        self.start()

    def start(self):
        # Start the node manager
        # self.nodes = NodeManager("gui")
        # self.nodes.start_all()
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

    def _send_trigger(self, button, target=""):
        response = self.smach_trigger_service.call(button, target)
        rospy.loginfo(response.message)    

    def shutdown_widget(self):
        # Clean up resources, service proxies etc.
        if self.smach_trigger_service:
            self.smach_trigger_service.close()
        rospy.loginfo("Shutting down SmachControlWidget")

    def update_exp(self):
        if self.exp_teleop.isChecked():
            rospy.set_param("/exploration/method", "teleop")
        elif self.exp_auto.isChecked():
            rospy.set_param("/exploration/method", "autonomous")

    def update_fill(self):
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

        self._widget = SmachControlWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_widget()