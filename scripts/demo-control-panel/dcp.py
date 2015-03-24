#!/usr/bin/env python

import rospy
import gtk
import gtk.glade
import pygtk
import thread
import threading
from std_msgs.msg import String


class DemoController:
    def __init__(self):
        rospy.init_node('demo_controller')
        
        self.pubCommand = rospy.Publisher("/demo_command", String, latch=True, queue_size=10)
        gtk.gdk.threads_init()
        thread.start_new_thread(self.spinner, ())
    
    def sendCommand(self, strCommand):
        strString = String()
        strString.data = strCommand
        
        self.pubCommand.publish(strString)
    
    def spinner(self):
        rospy.spin()


class DemoControlPanelUI:
    def __init__(self, ctrlController):
        self.glade = gtk.Builder()
        self.gladefile = "ui/ui.glade"
        self.glade.add_from_file(self.gladefile)
        
        self.ctrlController = ctrlController
        self.prepareUI()
    
    def start(self):
        self.ctrlController.sendCommand("tell :content dcpstart :receiver all :sender dcp")
        self.showInitializerWindow()
        
        gtk.main()
    
    def initialize(self):
        self.showMainWindow()
    
    def prepareUI(self):
        self.wndInitializer = self.glade.get_object("wndInitializer")
        
        if self.wndInitializer:
            self.wndInitializer.set_keep_above(True)
            self.wndInitializer.connect("delete_event", self.deleteEvent)
            self.wndInitializer.connect("destroy", self.destroy)
            self.wndInitializer.set_title("Choose ROS Master Hosts")
            
            self.glade.get_object("btnInitializerQuit").connect("clicked", self.destroy);
            self.glade.get_object("btnInitialize").connect("clicked", self.initialize);
            self.renderer = gtk.CellRendererText()
            
            self.glade.get_object("ddHost1").append_text("http://localhost:11311");
            self.glade.get_object("ddHost1").append_text("http://pr2a:11311");
            self.glade.get_object("ddHost1").append_text("http://leela:11311");
            self.glade.get_object("ddHost2").append_text("http://localhost:11311");
            self.glade.get_object("ddHost2").append_text("http://pr2a:11311");
            self.glade.get_object("ddHost2").append_text("http://leela:11311");
        
        self.wndMain = self.glade.get_object("wndMain")
        
        if self.wndMain:
            self.wndMain.set_keep_above(True)
            self.wndMain.connect("delete_event", self.deleteEvent)
            self.wndMain.connect("destroy", self.destroy)
            self.wndMain.set_title("Demo Control Panel - RoboHow Review Year 3")
            self.wndMain.resize(800, 600)
            
            self.glade.get_object("btnQuit").connect("clicked", self.destroy)
            self.glade.get_object("btnContinue").connect("clicked", self.actionContinue)
    
    def actionContinue(self, widget):
        self.ctrlController.sendCommand("tell :content continue :receiver all :sender dcp")
    
    def deleteEvent(self, widget, event, data=None):
        return False
    
    def destroy(self, widget, data=None):
        self.quit()
    
    def quit(self):
        self.ctrlController.sendCommand("tell :content dcpquit :receiver all :sender dcp")
        rospy.sleep(0.1)
        gtk.main_quit()
    
    def showMainWindow(self):
        self.wndMain.show_all()
    
    def showInitializerWindow(self):
        self.wndInitializer.show_all()


if __name__ == "__main__":
    ctrlController = DemoController()
    
    dcpUI = DemoControlPanelUI(ctrlController)
    dcpUI.start()

