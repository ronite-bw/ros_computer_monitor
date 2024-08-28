import rclpy
from rclpy.node import Node
from computer_msgs.msg import Process, CPUUsage
from std_msgs.msg import Float32, UInt64

import os
import pathlib
import psutil

class ROSTop(Node):

    def __init__(self):
        super().__init__('cpu_monitor')
        self.processes = {}

        self.poll()

        self.declare_parameter('poll_frequency', 0.2)
        self.declare_parameter('pub_frequency', 1.0)

        freq = self.get_parameter('poll_frequency').value
        self.poll_timer = self.create_timer(1.0 / freq, self.poll)

        freq = self.get_parameter('pub_frequency').value
        self.pub_timer = self.create_timer(1.0 / freq, self.publish)
        
        self.declare_parameter('ignored_nodes', [''])
        self.ignored_node_names = self.get_parameter('ignored_nodes').value
        print(self.ignored_node_names)
        for n in self.ignored_node_names:
            print("\t\tWill ignore node {}".format(n))

        self.total_cpu_publisher =  self.create_publisher(Float32, "cpu_monitor/total_cpu", 20)
        self.total_free_mem_publisher =  self.create_publisher(UInt64, "cpu_monitor/total_free_mem", 20)
        
        # core frequency
        self.core_publishers = []
        self.freq_publishers = []
        for i in range(psutil.cpu_count()):
            self.core_publishers.append(self.create_publisher(Float32, "cpu_monitor/core" + str(i), 20))
            self.freq_publishers.append(self.create_publisher(Float32, "cpu_monitor/freq" + str(i), 20))


    def is_ros(self, process):
        if '__node' in ' '.join(process.info['cmdline']):
            return True
        return False


    def poll(self, event=None):
        for p in psutil.process_iter(['name', 'cmdline', 'exe']):
            if self.is_ros(p):
                self.processes[p.pid] = p

    def publish(self, event=None):
        #Publish the total CPU used
        total_cpu_msg = Float32()
        total_cpu_msg.data = psutil.cpu_percent()
        self.total_cpu_publisher.publish(total_cpu_msg)

        #Publish free memory
        vm = psutil.virtual_memory()
        total_free_mem_msg = UInt64()
        total_free_mem_msg.data = getattr(vm, 'free')
        self.total_free_mem_publisher.publish(total_free_mem_msg)
        
        #Publish cpu percent and freq per core
        cpu_percent_per_core = psutil.cpu_percent(percpu=True)
        cpu_freq_per_core = psutil.cpu_freq(percpu=True)
        cpu_percent_per_core_msg = Float32()
        cpu_freq_per_core_msg = Float32()
        
        for i in range(psutil.cpu_count()):
            cpu_percent_per_core_msg.data = cpu_percent_per_core[i]
            cpu_freq_per_core_msg.data = cpu_freq_per_core[i][0]
            self.core_publishers[i].publish(cpu_percent_per_core_msg)
            if cpu_freq_per_core is not None and len(cpu_freq_per_core):
                self.freq_publishers[i].publish(cpu_freq_per_core_msg)

        #Publish process info with a different topic
        for pid, process in list(self.processes.items()):
            try:
                pub_process = Process()
                pub_process.pid = pid
                pub_process.command = ' '.join(process.cmdline())
                for field in ['name', 'cpu_percent', 'memory_percent', 'num_threads']:
                    setattr(pub_process, field, getattr(process, field)())
                if pub_process.name == 'ros_top' or pub_process.name in self.ignored_node_names:
                    continue
                topic_name = '/cpu_monitor/' + pub_process.name + '/cpu'
                temp_pub = self.create_publisher(Float32, topic_name, 20)
                cpu_percent_msg = Float32()
                cpu_percent_msg.data = pub_process.cpu_percent
                temp_pub.publish(cpu_percent_msg)
            except psutil.NoSuchProcess:
                del self.processes[pid]

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ROSTop())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
