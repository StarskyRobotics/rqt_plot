#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from collections import defaultdict

import time
import threading

import rospkg
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QDateTime, pyqtSignal
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget, QFileDialog

import rospy
import rosbag

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers
from std_msgs.msg import Bool

from . rosplot import ROSData, RosPlotException
from .data_plot import DataPlot

def _get_all_topics(bag):
    if bag is None:
        return []
    return bag.all_topics_types

def _get_topic_type(topic, bag):
    """
    subroutine for getting the topic type
    (nearly identical to rostopic._get_topic_type, except it returns rest of name instead of fn)

    :returns: topic type, real topic name, and rest of name referenced
      if the topic points to a field within a topic, e.g. /rosout/msg, ``str, str, str``
    """
    topics = _get_all_topics(bag)
    for t, ttype in topics:
        if t == topic:
            return ttype, t, ""
    for t, ttype in topics:
        if topic.startswith(t + '/'):
            return ttype, t, topic[len(t):]

    return None, None, None

    # matches = [(t, t_type) for t, t_type in val if t == topic or topic.startswith(t + '/')]
    # if matches:
    #     t, t_type = matches[0]
    #     if t_type == roslib.names.ANYTYPE:
    #         return None, None, None
    #     if t_type == topic:
    #         return t_type, None
    #     return t_type, t, topic[len(t):]
    # else:
    #     return None, None, None


def get_topic_type(topic, bag):
    """
    Get the topic type (nearly identical to rostopic.get_topic_type, except it doesn't return a fn)

    :returns: topic type, real topic name, and rest of name referenced
      if the \a topic points to a field within a topic, e.g. /rosout/msg, ``str, str, str``
    """
    topic_type, real_topic, rest = _get_topic_type(topic, bag)
    if topic_type:
        return topic_type, real_topic, rest
    else:
        return None, None, None


def update_topics(self, bag):
    self.model().clear()
    if bag is None:
        return
    topic_list = _get_all_topics(bag)
    for topic_path, topic_type in topic_list:
        topic_name = topic_path.strip('/')
        message_class = roslib.message.get_message_class(topic_type)
        if message_class is None:
            qWarning('TopicCompleter.update_topics(): could not get message class for topic type "%s" on topic "%s"' % (
            topic_type, topic_path))
            continue
        message_instance = message_class()
        self.model().add_message(message_instance, topic_name, topic_type, topic_path)

def get_plot_fields(topic_name, bag):
    topic_type, real_topic, _ = get_topic_type(topic_name, bag)
    if topic_type is None:
        message = "topic %s does not exist" % ( topic_name )
        return [], message
    field_name = topic_name[len(real_topic)+1:]

    slot_type, is_array, array_size = roslib.msgs.parse_type(topic_type)
    field_class = roslib.message.get_message_class(slot_type)

    fields = [f for f in field_name.split('/') if f]

    for field in fields:
        # parse the field name for an array index
        try:
            field, _, field_index = roslib.msgs.parse_type(field)
        except roslib.msgs.MsgSpecException:
            message = "invalid field %s in topic %s" % ( field, real_topic )
            return [], message

        if field not in getattr(field_class, '__slots__', []):
            message = "no field %s in topic %s" % ( field_name, real_topic )
            return [], message
        slot_type = field_class._slot_types[field_class.__slots__.index(field)]
        slot_type, slot_is_array, array_size = roslib.msgs.parse_type(slot_type)
        is_array = slot_is_array and field_index is None

        field_class = topic_helpers.get_type_class(slot_type)

    if field_class in (int, float, bool):
        topic_kind = 'boolean' if field_class == bool else 'numeric'
        if is_array:
            if array_size is not None:
                message = "topic %s is fixed-size %s array" % ( topic_name, topic_kind )
                return [ "%s[%d]" % (topic_name, i) for i in range(array_size) ], message
            else:
                message = "topic %s is variable-size %s array" % ( topic_name, topic_kind )
                return [], message
        else:
            message = "topic %s is %s" % ( topic_name, topic_kind )
            return [ topic_name ], message
    else:
        if not roslib.msgs.is_valid_constant_type(slot_type):
            numeric_fields = []
            for i, slot in enumerate(field_class.__slots__):
                slot_type = field_class._slot_types[i]
                slot_type, is_array, array_size = roslib.msgs.parse_type(slot_type)
                slot_class = topic_helpers.get_type_class(slot_type)
                if slot_class in (int, float) and not is_array:
                    numeric_fields.append(slot)
            message = ""
            if len(numeric_fields) > 0:
                message = "%d plottable fields in %s" % ( len(numeric_fields), topic_name )
            else:
                message = "No plottable fields in %s" % ( topic_name )
            return [ "%s/%s" % (topic_name, f) for f in numeric_fields ], message
        else:
            message = "Topic %s is not numeric" % ( topic_name )
            return [], message

def is_plottable(topic_name, bag):
    fields, message = get_plot_fields(topic_name, bag)
    return len(fields) > 0, message

class PlotWidget(QWidget):
    _redraw_interval = 200

    load_done_sig = pyqtSignal()
    draw_done_sig = pyqtSignal()
    status_sig = pyqtSignal(str)

    def __init__(self, initial_topics=None, initial_bag=None, start_paused=False):
        super(PlotWidget, self).__init__()
        self.setObjectName('PlotWidget')

        self._initial_topics = initial_topics

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_plotbag'), 'resource', 'plot.ui')
        loadUi(ui_file, self)
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('list-add'))
        self.remove_topic_button.setIcon(QIcon.fromTheme('list-remove'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = None

        self.subscribe_topic_button.setEnabled(True)
        if start_paused:
            self.pause_button.setChecked(True)

        self._topic_completer = TopicCompleter(self.topic_edit)
        update_topics(self._topic_completer, None)
        self.topic_edit.setCompleter(self._topic_completer)

        self._start_time = rospy.get_time()
        self._rosdata = {}
        self._remove_topic_menu = QMenu()

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)

        self.load_done_sig.connect(self.load_done)
        self.draw_done_sig.connect(self.draw_done)
        self.status_sig.connect(self._set_status)

        self._bag = None
        self.set_status()

        if initial_bag is not None and len(initial_bag):
            print("Loading initial bag from commandline")
            self.bag_edit.setText(initial_bag)
            self.on_load_button_clicked()

    def set_status(self, msg=""):
        self.status_sig.emit(msg)

    def _set_status(self, msg=""):
        self.status_label.setText(msg)
        if msg == "ERR": msg = ""
        self.subscribe_topic_button.setEnabled(msg == "")
        self.redraw_button.setEnabled(msg == "")
        self.load_button.setEnabled(msg == "")


    def switch_data_plot_widget(self, data_plot):
        self.enable_timer(enabled=False)

        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()

        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(self.autoscroll_checkbox.isChecked())
        if self.autoscale_checkbox.isChecked():
            self.data_plot.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        else:
            self.data_plot.set_autoscale(y=False)

        # setup drag 'n drop
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent

    def update_topics(self):
        if self._initial_topics:
            print("Plotting initial topics from commandline")
            autodraw = self.autodraw_checkbox.isChecked()
            if len(self._initial_topics) > 1 and autodraw:
                self.autodraw_checkbox.setChecked(False)

            for topic_name in self._initial_topics:
                self.add_topic(topic_name)

            if len(self._initial_topics) > 1 and autodraw:
                self.autodraw_checkbox.setChecked(True)
                self.on_redraw_button_clicked()

            self._initial_topics = None


        else:
            for topic_name, rosdata in self._rosdata.items():
                data_x, data_y = rosdata.next()
                self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)

        self._subscribed_topics_changed()

    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        # get topic name
        if not event.mimeData().hasText():
            if not hasattr(event.source(), 'selectedItems') or len(event.source().selectedItems()) == 0:
                qWarning('Plot.dragEnterEvent(): not hasattr(event.source(), selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = event.source().selectedItems()[0]
            topic_name = item.data(0, Qt.UserRole)
            if topic_name == None:
                qWarning('Plot.dragEnterEvent(): not hasattr(item, ros_topic_name_)')
                return
        else:
            topic_name = str(event.mimeData().text())

        # check for plottable field type
        plottable, message = is_plottable(topic_name, self._bag)
        if plottable:
            event.acceptProposedAction()
        else:
            qWarning('Plot.dragEnterEvent(): rejecting: "%s"' % (message))

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.add_topic(topic_name)

    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            update_topics(self._topic_completer, self._bag)

        plottable, message = is_plottable(topic_name, self._bag)
        self.subscribe_topic_button.setEnabled(plottable)
        self.subscribe_topic_button.setToolTip(message)

    @Slot()
    def on_topic_edit_returnPressed(self):
        if self.subscribe_topic_button.isEnabled():
            self.add_topic(str(self.topic_edit.text()))

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.add_topic(str(self.topic_edit.text()))

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    @Slot(bool)
    def on_autoscroll_checkbox_clicked(self, checked):
        self.data_plot.autoscroll(checked)
        if checked:
            self.data_plot.redraw()

    @Slot(bool)
    def on_autoscale_checkbox_clicked(self, checked):
        if checked:
            self.data_plot.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
            self.data_plot.redraw()
        else:
            self.data_plot.set_autoscale(y=False)

    @Slot()
    def on_clear_button_clicked(self):
        self.clear_plot()

    @Slot()
    def on_browse_button_clicked(self):
        self.bag_edit.setText(QFileDialog.getOpenFileName()[0])

    @Slot()
    def on_load_button_clicked(self):
        thread = threading.Thread(target=self._load, args=())
        thread.daemon = True
        thread.start()
        self.set_status("LOADING")

    def _load(self):
        bagfile = self.bag_edit.text()
        try:
            rospy.loginfo("Loading bag file %s" % bagfile)
            self._bag = rosbag.Bag(bagfile)
            self.set_status("INDEXING")
            rospy.loginfo("Bag loaded, indexing topics")
            self._bag.all_topics_types = [(k, v.msg_type) for k, v in self._bag.get_type_and_topic_info()[1].items()]
            self._bag.all_topics = [k for k, v in self._bag.all_topics_types]
            rospy.loginfo("%d Topics indexed" % len(self._bag.all_topics))
            self.load_done_sig.emit()
        except Exception as e:
            rospy.logerr("Unable to load bag! "+str(e))
            self.set_status("ERR")

    def load_done(self):
        self.set_status("")
        self.start_time_edit.setDateTime(QDateTime.fromMSecsSinceEpoch(self._bag.get_start_time() * 1000))
        self.end_time_edit.setDateTime(QDateTime.fromMSecsSinceEpoch(self._bag.get_end_time() * 1000))
        self._start_time = self.get_times()[0]

        self.update_topics()

    @Slot()
    def on_redraw_button_clicked(self):
        self.clear_plot()
        thread = threading.Thread(target=self._redraw, args=())
        thread.daemon = True
        thread.start()
        self.set_status("REDRAWING")

    def _redraw(self):
        start, end = self.get_times()
        self._start_time = start
        real_topics = defaultdict(list)
        for topic, data in self._rosdata.items():
            real_topics[data.real_topic] += [data]
            data.start_time = start
            data.start = start
            data.end = end

        for topic, msg, t in self._bag.read_messages(start_time=rospy.Time(start), end_time=rospy.Time(end),
                                               topics=real_topics.keys()):
            for data in real_topics[topic]:
                data.add_item(msg, t)

        self.draw_done_sig.emit()

    def draw_done(self):
        self.set_status()

    def get_times(self):
        """ Returns time inputs in unix time"""
        start = time.mktime(self.start_time_edit.dateTime().toPyDateTime().timetuple())
        end = time.mktime(self.end_time_edit.dateTime().toPyDateTime().timetuple())
        return start, end

    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    if data_x or data_y:
                        self.data_plot.update_values(topic_name, data_x, data_y)
                        needs_redraw = True
                except RosPlotException as e:
                    qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
            if needs_redraw:
                self.data_plot.redraw()

    def _subscribed_topics_changed(self):
        self._update_remove_topic_menu()
        if not self.pause_button.isChecked():
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._rosdata)
        self.data_plot.redraw()

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._rosdata.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        if len(self._rosdata) > 1:
            all_action = QAction('All', self._remove_topic_menu)
            all_action.triggered.connect(self.clean_up_subscribers)
            self._remove_topic_menu.addAction(all_action)

        self.remove_topic_button.setMenu(self._remove_topic_menu)

    def add_topic(self, topic_name):
        topics_changed = False
        fields = get_plot_fields(topic_name, self._bag)[0]
        autodraw = self.autodraw_checkbox.isChecked()
        if len(fields) > 1 and autodraw:
            self.autodraw_checkbox.setChecked(False)

        for topic_name in fields:
            if topic_name in self._rosdata:
                qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
                continue
            start, end = self.get_times()
            self._rosdata[topic_name] = ROSData(topic_name, self._start_time, start, end)
            if self.autodraw_checkbox.isChecked():
                self.load_topic_data(topic_name)
            else:
                self._rosdata[topic_name].get_topic_info(self._bag)

            if self._rosdata[topic_name].error is not None:
                qWarning(str(self._rosdata[topic_name].error))
                del self._rosdata[topic_name]
            else:
                data_x, data_y = self._rosdata[topic_name].next()
                self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)
                topics_changed = True

        if topics_changed:
            self._subscribed_topics_changed()

        if len(fields) > 1 and autodraw:
            self.autodraw_checkbox.setChecked(True)
            self.on_redraw_button_clicked()

    def load_topic_data(self, topic_name):
        self.set_status("READING")
        thread = threading.Thread(target=self._load_topic_data, args=(topic_name,))
        thread.daemon = True
        thread.start()

    def _load_topic_data(self, topic_name):
        self._rosdata[topic_name].load_data(self._bag)
        self.draw_done_sig.emit()

    def remove_topic(self, topic_name):
        self._rosdata[topic_name].close()
        del self._rosdata[topic_name]
        self.data_plot.remove_curve(topic_name)

        self._subscribed_topics_changed()

    def clear_plot(self):
        for topic_name, _ in self._rosdata.items():
            self.data_plot.clear_values(topic_name)
        self.data_plot.redraw()

    def clean_up_subscribers(self):
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
            self.data_plot.remove_curve(topic_name)
        self._rosdata = {}

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
