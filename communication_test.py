import time
import random
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect

class tester:
    i = 0

    def __init__(self):
        self.drone = mavutil.mavlink_connection('localhost:14445',
                                           baud=57600,
                                           source_system=1,
                                           source_component=1)

    # Find is enable to connect a drone by heartbeating...
    def is_enable(self) -> bool:
        # Just wait until not connect for 5 seconds.
        return self.drone.wait_heartbeat(timeout=5)

    # Send test fake gps msg
    # Fake gps is just random values.
    def send_test_msg(self):
        text = f"count: { self.i }, lat {"{:.4f}".format(random.uniform(35,36))} lng | {"{:.4f}".format(random.uniform(127,129))}"

        # create STATUSTEXT msg for mavlink
        message = dialect.MAVLink_statustext_message(
            severity=dialect.MAV_SEVERITY_INFO,
            text=text.encode("utf-8")
            )

        # send message to MAVLINK
        self.drone.mav.send(message)

        # sleep a bit
        time.sleep(5)
        # is connected?
        self.drone.wait_heartbeat()
        self.i += 1

    def get_msg(self):
        msg = self.drone.recv_match(type='STATUSTEXT', blocking=True)
        if msg is not None:
            print(f"msg : {msg.text}")

if __name__ == "__main__":
    enable = True
    tester = tester()

    print("start the msg...")
    try:
        while True:
            enable = tester.is_enable()

            if enable:
                tester.send_test_msg()
                tester.get_msg()
            else:
                print("MAVLink doesn't connected")
                exit()

    except KeyboardInterrupt:
        print("shutdown pymavlink")