from clearpath_config.common.types.accessory import Accessory, IndexedAccessory
from clearpath_config.common.types.ip import IP
from clearpath_config.common.types.port import Port
from clearpath_config.common.types.file import File
from clearpath_config.common.utils.dictionary import extend_flat_dict
from clearpath_config.sensors.types.sensor import BaseSensor
from clearpath_config.sensors.types.lidars_2d import BaseLidar2D
from clearpath_config.sensors.types.lidars_2d import HokuyoUST


import rclpy
from rclpy.node import Node

from clearpath_config.sensors.types.gps import Ublox
from clearpath_config.sensors.types.gps import ArdusimpleRTKLite


def print_ublox_params(ublox_obj):
    print(f"Ublox Parameters:")
    print(f"  Frame ID: {ublox_obj.frame_id}")
    print(f"  Device: {ublox_obj.device}")
    print(f"  UART Baudrate: {ublox_obj.uart1_baudrate}")
    print(f"  Msg_Rate: {ublox_obj.msg_rate}")
    print(f"  Dynamic Model: {ublox_obj.dynamic_model}")
    print(f"  Enable PPP: {ublox_obj.enable_ppp}")
    print(f"  TMODE3: {ublox_obj.tmode3}")
    print(f"  SV_IN Reset: {ublox_obj.sv_in_reset}")
    print(f"  SV_IN Min Duration: {ublox_obj.sv_in_min_dur}")
    print(f"  SV_IN Accuracy Limit: {ublox_obj.sv_in_acc_limit}")
    print(f"  INF All: {ublox_obj.inf_all}")
    print(f"  Publish All: {ublox_obj.publish_all}")
    print(f"  Publish Nav All: {ublox_obj.publish_nav_all}")
    print()
    
def print_base_lidar_params(lidar_obj):
    print(f"Lidar Parameters:")
    print(f"  Index: {lidar_obj.idx}")
    print(f"  Topic {lidar_obj.topic}")
    print(f"  IP Address: {lidar_obj.ip}")
    print(f"  Port: {lidar_obj.port}")
    print(f"  Min Angle: {lidar_obj.min_angle}")
    print(f"  Max Angle: {lidar_obj.max_angle}")
    print()

   
def main():
    
    #print(Ublox.mro())
    
        
    # Instantiate Ublox class
    ublox_obj = Ublox(
        idx=1,
        name="UbloxGPS",
        topic="fix",
        frame_id="gps_frame",
        device="/dev/ttyUSB0",
        uart1_baudrate=38400,
        msg_rate=4.0,
        dynamic_model="portable",
        enable_ppp=False,
        tmode3=0,
        sv_in_reset=False,
        sv_in_min_dur=300,
        sv_in_acc_limit=3.0,
        inf_all=True,
        publish_all=False,
        publish_nav_all=True
    )
    # Print Ublox object parameters
    print_ublox_params(ublox_obj)
    
    # Instantiate BaseLidar2D Class
    base_lidar_obj = BaseLidar2D(
        idx=2,
        name="BaseLidar",
        topic="scan",
        frame_id="base_lidar_frame",
        ip="192.168.1.22",
        port="2",
        min_angle=0.0,
        max_angle=3.14       
    )
    print_base_lidar_params(base_lidar_obj)
    
    hokuyo_obj = HokuyoUST(
        idx=3,
        name="Hokuyo",
        topic="scan",
        frame_id="base_lidar_frame",
        ip="192.168.1.33",
        port="3",
        min_angle=0.0,
        max_angle=3.14       
    )
    print_base_lidar_params(hokuyo_obj)

    
    
    # Instantiate ArdusimpleRTKLite class
    rtk_obj = ArdusimpleRTKLite(
        idx=2,
        name="RTKLiteGPS",
        topic="fix",
        frame_id="gps_rtk_frame",
        device="/dev/ttyUSB1",
        uart1_baudrate=38400,
        msg_rate=4.0,
        dynamic_model="portable",
        enable_ppp=False,
        tmode3=0,
        sv_in_reset=False,
        sv_in_min_dur=300,
        sv_in_acc_limit=3.0,
        inf_all=True,
        publish_all=False,
        publish_nav_all=True
    )

    # Print ArduSimpleRTKLite object parameters
    print_ublox_params(rtk_obj)


if __name__ == "__main__":
    main()