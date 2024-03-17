#!/usr/bin/env python
import can
import rospy
import pubsub.json_message

class CAN_Handler:
    def __init__(self, baudrate):
        rospy.init_node('CAN_Handler', anonymous=True)

        self.pub = pubsub.json_message.Publisher('ethernet.send', queue_size=10)
        pubsub.json_message.Subscriber('can.send', self.message_listener)
        
        
        self.bus = can.interface.Bus(bustype="socketcan", channel="can0", bitrate=baudrate)

    def message_listener(self, message):
        msg = can.Message(arbitration_id=message["address"], data=message["data"], is_extended_id=False)
        
        try:
            self.bus.send(msg, timeout=0.01)
        except Exception as e:
            print("Message not sent:", [e, msg])

    def run(pub):
        msg = can.Bus().recv(0)
        
        if msg is not None:
            data = msg.data
            data_array = [int.from_bytes(byte, byteorder='big') for byte in data]
            
            message={"type": "CAN", "address": msg.arbitration_id, "data": data_array}
            pub.publish(message)

if __name__ == '__main__':
    try:
        can_handler = CAN_Handler(250000)
        rate_param = rospy.get_param('~rate', 10.0)
        rate = rospy.Rate(rate_param)

        while not rospy.is_shutdown():
            can_handler.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass