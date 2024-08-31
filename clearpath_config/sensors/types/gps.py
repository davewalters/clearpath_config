# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from clearpath_config.common.types.accessory import Accessory
from clearpath_config.common.types.ip import IP
from clearpath_config.common.types.port import Port
from clearpath_config.common.types.file import File
from clearpath_config.common.utils.dictionary import extend_flat_dict
from clearpath_config.sensors.types.sensor import BaseSensor
from typing import List

import rclpy
from rclpy.node import Node

class BaseGPS(BaseSensor):
    SENSOR_TYPE = "gps"
    SENSOR_MODEL = "base"
    TOPIC = "fix"

    FRAME_ID = "link"

    class ROS_PARAMETER_KEYS:
        FRAME_ID = "node_name.frame_id"

    class TOPICS:
        FIX = "fix"
        NAME = {
            FIX: "fix",
        }
        RATE = {
            FIX: 60,
        }

    def __init__(
            self,
            idx: int = None,
            name: str = None,
            topic: str = TOPIC,
            frame_id: str = FRAME_ID,
            urdf_enabled: bool = BaseSensor.URDF_ENABLED,
            launch_enabled: bool = BaseSensor.LAUNCH_ENABLED,
            ros_parameters: str = BaseSensor.ROS_PARAMETERS,
            ros_parameters_template: dict = BaseSensor.ROS_PARAMETERS_TEMPLATE,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY
            ) -> None:
        # Frame ID
        self.frame_id: str = self.FRAME_ID
        self.set_frame_id(frame_id)
        # ROS Parameters Template
        template = {
            self.ROS_PARAMETER_KEYS.FRAME_ID: BaseGPS.frame_id,
        }
        ros_parameters_template = extend_flat_dict(template, ros_parameters_template)
        super().__init__(
            idx,
            name,
            topic,
            urdf_enabled,
            launch_enabled,
            ros_parameters,
            ros_parameters_template,
            parent,
            xyz,
            rpy
        )

    @classmethod
    def get_frame_id_from_idx(cls, idx: int) -> str:
        return "%s_%s" % (
            cls.get_name_from_idx(idx),
            cls.FRAME_ID
        )

    @classmethod
    def get_ip_from_idx(cls, idx: int) -> str:
        ip = cls.IP_ADDRESS.split('.')
        network_id = ip[0:3]
        host_id = int(ip[-1]) + idx
        return '.'.join(network_id) + '.' + str(host_id)

    def set_idx(self, idx: int) -> None:
        # Set Base: Name and Topic
        super().set_idx(idx)
        # Set Frame ID
        self.set_frame_id(self.get_frame_id_from_idx(idx))

    @property
    def frame_id(self) -> str:
        return self._frame_id

    @frame_id.setter
    def frame_id(self, link: str) -> None:
        Accessory.assert_valid_link(link)
        self._frame_id = link

    def get_frame_id(self) -> str:
        return self.frame_id

    def set_frame_id(self, link: str) -> None:
        self.frame_id = link


class SwiftNavDuro(BaseGPS):
    SENSOR_MODEL = "swiftnav_duro"

    FRAME_ID = "link"
    IP_ADDRESS = "192.168.131.30"
    IP_PORT = 55555

    class ROS_PARAMETER_KEYS:
        FRAME_ID = "duro_node.imu_frame_id"
        GPS_FRAME = "duro_node.gps_receiver_frame_id"
        IP_ADDRESS = "duro_node.ip_address"
        IP_PORT = "duro_node.port"

    class TOPICS:
        FIX = "fix"
        NAME = {
            FIX: "fix",
        }
        RATE = {
            FIX: 60,
        }

    def __init__(
            self,
            idx: int = None,
            name: str = None,
            topic: str = BaseGPS.TOPIC,
            frame_id: str = FRAME_ID,
            ip: str = IP_ADDRESS,
            port: int = IP_PORT,
            urdf_enabled: bool = BaseSensor.URDF_ENABLED,
            launch_enabled: bool = BaseSensor.LAUNCH_ENABLED,
            ros_parameters: str = BaseSensor.ROS_PARAMETERS,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY
            ) -> None:
        # IP Address
        self.ip: IP = IP(self.IP_ADDRESS)
        self.set_ip(ip)
        # IP Port
        self.port: Port = Port(self.IP_PORT)
        self.set_port(port)
        # ROS Parameter Template
        ros_parameters_template = {
            self.ROS_PARAMETER_KEYS.IP_ADDRESS: SwiftNavDuro.ip,
            self.ROS_PARAMETER_KEYS.IP_PORT: SwiftNavDuro.port,
            self.ROS_PARAMETER_KEYS.FRAME_ID: SwiftNavDuro.frame_id,
            self.ROS_PARAMETER_KEYS.GPS_FRAME: SwiftNavDuro.frame_id,
        }
        super().__init__(
            idx,
            name,
            topic,
            frame_id,
            urdf_enabled,
            launch_enabled,
            ros_parameters,
            ros_parameters_template,
            parent,
            xyz,
            rpy
        )

    @property
    def ip(self) -> str:
        return str(self._ip)

    @ip.setter
    def ip(self, ip: str) -> None:
        self._ip = IP(str(ip))

    def get_ip(self) -> str:
        return str(self.ip)

    def set_ip(self, ip: str) -> None:
        self.ip = ip

    @property
    def port(self) -> int:
        return int(self._port)

    @port.setter
    def port(self, port: int) -> None:
        self._port = Port(int(port))

    def get_port(self) -> int:
        return int(self.port)

    def set_port(self, port: int) -> None:
        self.port = port


class NMEA(BaseGPS):
    SENSOR_MODEL = "nmea_gps"

    FRAME_ID = "link"
    PORT = "/dev/ttyACM0"
    BAUD = 115200

    class ROS_PARAMETER_KEYS:
        FRAME_ID = "nmea_navsat_driver.frame_id"
        PORT = "nmea_navsat_driver.port"
        BAUD = "nmea_navsat_driver.baud"

    class TOPICS:
        FIX = "fix"
        NAME = {
            FIX: "fix",
        }
        RATE = {
            FIX: 60,
        }

    def __init__(
            self,
            idx: int = None,
            name: str = None,
            topic: str = BaseGPS.TOPIC,
            frame_id: str = FRAME_ID,
            port: str = PORT,
            baud: int = BAUD,
            urdf_enabled: bool = BaseSensor.URDF_ENABLED,
            launch_enabled: bool = BaseSensor.LAUNCH_ENABLED,
            ros_parameters: str = BaseSensor.ROS_PARAMETERS,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY
            ) -> None:
        # Port
        self.port = port
        # Baud
        self.baud = baud
        # ROS Paramater Template
        ros_parameters_template = {
            self.ROS_PARAMETER_KEYS.PORT: NMEA.port,
            self.ROS_PARAMETER_KEYS.BAUD: NMEA.baud
        }
        super().__init__(
            idx,
            name,
            topic,
            frame_id,
            urdf_enabled,
            launch_enabled,
            ros_parameters,
            ros_parameters_template,
            parent,
            xyz,
            rpy
        )

    @property
    def port(self) -> str:
        return str(self._port)

    @port.setter
    def port(self, file: str) -> str:
        self._port = File(str(file))

    @property
    def baud(self) -> int:
        return self._baud

    @baud.setter
    def baud(self, baud: int) -> None:
        assert isinstance(baud, int), ("Baud must be of type 'int'.")
        assert baud >= 0, ("Baud must be positive integer.")
        self._baud = baud


class Ublox(BaseGPS):
            
    SENSOR_MODEL = "ublox_gps"

    FRAME_ID = "link"
    DEVICE = "/dev/ttyUSB0"
    UART1_BAUDRATE = 38400
    RATE = 4.0
    DYNAMIC_MODEL = "portable"
    ENABLE_PPP = False
    TMODE3 = 0
    SV_IN_RESET = False
    SV_IN_MIN_DUR = 300
    SV_IN_ACC_LIMIT = 3.0
    INF_ALL = True
    PUBLISH_ALL = False
    PUBLISH_NAV_ALL = True
    
    VALID_DYNAMIC_MODELS = {"portable", "stationary", "pedestrian", "automotive",
                            "sea", "airborne1", "airborne2", "airborne4", "wristwatch",}
    VALID_TMODE3 = {0, 1, 2,}
    

    class ROS_PARAMETER_KEYS:
        FRAME_ID = "ublox_gps.frame_id"
        DEVICE = "ublox_gps.device"
        UART1_BAUDRATE = "ublox_gps.uart1.baudrate"
        RATE = "ublox_gps.rate"
        DYNAMIC_MODEL = "ublox_gps.dynamic_model"
        ENABLE_PPP = "ublox_gps.enable_ppp"
        TMODE3 = "ublox_gps.tmode3"
        SV_IN_RESET = "ublox_gps.sv_in.reset"
        SV_IN_MIN_DUR = "ublox_gps.sv_in.min_dur"
        SV_IN_ACC_LIMIT = "ublox_gps.sv_in.acc_limit"
        INF_ALL = "ublox_gps.inf.all"
        PUBLISH_ALL = "ublox_gps.publish.all"
        PUBLISH_NAV_ALL = "ublox_gps.publish.nav.all"


    class TOPICS:
        FIX = "fix"
        NAME = {
            FIX: "fix",
        }
        RATE = {
            FIX: 4,
        }
        
    def __init__(
            self,
            idx: int = None,
            name: str = None,
            topic: str = BaseGPS.TOPIC,
            frame_id: str = FRAME_ID,
            urdf_enabled: bool = BaseSensor.URDF_ENABLED,
            launch_enabled: bool = BaseSensor.LAUNCH_ENABLED,
            ros_parameters: dict = BaseSensor.ROS_PARAMETERS,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY,
            device: str = DEVICE,
            uart1_baudrate: int = UART1_BAUDRATE,
            rate: float = RATE,
            dynamic_model: str = DYNAMIC_MODEL,
            enable_ppp: bool = ENABLE_PPP,
            tmode3: int = TMODE3,
            sv_in_reset: bool = SV_IN_RESET,
            sv_in_min_dur: int = SV_IN_MIN_DUR,
            sv_in_acc_limit: float = SV_IN_ACC_LIMIT,
            inf_all: bool = INF_ALL,
            publish_all: bool = PUBLISH_ALL,
            publish_nav_all: bool = PUBLISH_NAV_ALL,
            ) -> None:
        # Device
        self.device = device
        # Baud
        self.uart1_baudrate = uart1_baudrate
        # Publishing Rate [Hz]
        self.rate = rate
        # Dynamic Model
        self.dynamic_model = dynamic_model
        # Precise Positioning
        self.enable_ppp = enable_ppp
        # Timing Mode
        self.tmode3 = tmode3
        # Survey-in reset
        self.sv_in_reset = sv_in_reset
        # Survey-in min duration
        self.sv_in_min_dur = sv_in_min_dur
        # Survey-in accuracy limit
        self.sv_in_acc_limit = sv_in_acc_limit
        # Publish all info messages to console
        self.inf_all = inf_all
        # Publish all ublox msgs as ros msgs
        self.publish_all = publish_all
        # Publish all nav_msgs
        self.publish_nav_all = publish_nav_all
               
        # ROS Parameter Template
        ros_parameters_template = {
            self.ROS_PARAMETER_KEYS.DEVICE: Ublox.device,
            self.ROS_PARAMETER_KEYS.UART1_BAUDRATE: Ublox.uart1_baudrate,
            self.ROS_PARAMETER_KEYS.RATE: Ublox.rate,
            self.ROS_PARAMETER_KEYS.DYNAMIC_MODEL: Ublox.dynamic_model,
            self.ROS_PARAMETER_KEYS.ENABLE_PPP: Ublox.enable_ppp,
            self.ROS_PARAMETER_KEYS.TMODE3: Ublox.tmode3,
            self.ROS_PARAMETER_KEYS.SV_IN_RESET: Ublox.sv_in_reset,
            self.ROS_PARAMETER_KEYS.SV_IN_MIN_DUR: Ublox.sv_in_min_dur,
            self.ROS_PARAMETER_KEYS.SV_IN_ACC_LIMIT: Ublox.sv_in_acc_limit,
            self.ROS_PARAMETER_KEYS.INF_ALL: Ublox.inf_all,
            self.ROS_PARAMETER_KEYS.PUBLISH_ALL: Ublox.publish_all,
            self.ROS_PARAMETER_KEYS.PUBLISH_NAV_ALL: Ublox.publish_nav_all,
        }
        
        #print(f"gps.py: ros_parameters_template: {ros_parameters_template}")
        #input()
        
        super().__init__(
            idx,
            name,
            topic,
            frame_id,
            urdf_enabled,
            launch_enabled,
            ros_parameters,
            ros_parameters_template,
            parent,
            xyz,
            rpy
        )

    @property
    def device(self) -> str:
        return str(self._device)

    @device.setter
    def device(self, file: str) -> str:
        self._device = File(str(file))

    @property
    def uart1_baudrate(self) -> int:
        return self._uart1_baudrate

    @uart1_baudrate.setter
    def uart1_baudrate(self, uart1_baudrate: int) -> None:
        assert isinstance(uart1_baudrate, int), ("uart1_baudrate must be of type 'int'.")
        assert uart1_baudrate >= 0, ("uart1_baudrate must be positive integer.")
        self._uart1_baudrate = uart1_baudrate
    
    @property
    def rate(self) -> float:
        return self._rate

    @rate.setter
    def rate(self, rate: float) -> None:
        assert isinstance(rate, float), ("rate must be of type 'float'.")
        assert rate >= 0, ("rate must be positive float.")
        self._rate = rate
        
    @property
    def dynamic_model(self) -> str:
        return str(self._dynamic_model)

    @dynamic_model.setter
    def dynamic_model(self, file: str) -> str:
        assert file in self.VALID_DYNAMIC_MODELS, (
            f"dynamic model must be one of {self.VALID_DYNAMIC_MODELS}.")
        self._dynamic_model = File(str(file))
        
    @property
    def enable_ppp(self) -> bool:
        return self._enable_ppp
        
    @enable_ppp.setter
    def enable_ppp(self, enable_ppp: bool) -> None:
        assert isinstance(enable_ppp, bool), ("enable_ppp must be of type 'bool'.")
        self._enable_ppp = enable_ppp   
    
    @property
    def tmode3(self) -> int:
        return self._tmode3

    @tmode3.setter
    def tmode3(self, tmode3: int) -> None:
        assert tmode3 in self.VALID_TMODE3, (f"tmode3 must be one of {self.VALID_TMODE3}.")
        self._tmode3 = tmode3   
        
    @property
    def sv_in_reset(self) -> bool:
        return self._sv_in_reset

    @sv_in_reset.setter
    def sv_in_reset(self, sv_in_reset: bool) -> None:
        assert isinstance(sv_in_reset, bool), ("sv_in_reset must be of type 'bool'.")
        self._sv_in_reset = sv_in_reset
        
    @property
    def sv_in_min_dur(self) -> int:
        return self._sv_in_min_dur

    @sv_in_min_dur.setter
    def sv_in_min_dur(self, sv_in_min_dur: int) -> None:
        assert sv_in_min_dur >= 0, ("sv_in_min_dur must be >= 0.")
        self._sv_in_min_dur = sv_in_min_dur
    
    @property
    def sv_in_acc_limit(self) -> float:
        return self._sv_in_acc_limit

    @sv_in_acc_limit.setter
    def sv_in_acc_limit(self, sv_in_acc_limit: float) -> None:
        assert isinstance(sv_in_acc_limit, float), ("sv_in_acc_limit must be of type 'float'.")
        assert sv_in_acc_limit >= 0, ("sv_in_acc_limit must be >= 0.0.")
        self._sv_in_acc_limit = sv_in_acc_limit   
         
    @property
    def inf_all(self) -> bool:
        return self._inf_all
        
    @inf_all.setter
    def inf_all(self, inf_all: bool) -> None:
        assert isinstance(inf_all, bool), ("inf_all must be of type 'bool'.")
        self._inf_all = inf_all    
    
    @property
    def publish_all(self) -> bool:
        return self._publish_all
        
    @publish_all.setter
    def publish_all(self, publish_all: bool) -> None:
        assert isinstance(publish_all, bool), ("publish_all must be of type 'bool'.")
        self._publish_all = publish_all    
        
    @property
    def publish_nav_all(self) -> bool:
        return self._publish_nav_all
        
    @publish_nav_all.setter
    def publish_nav_all(self, publish_nav_all: bool) -> None:
        assert isinstance(publish_nav_all, bool), ("publish_nav_all must be of type 'bool'.")
        self._publish_nav_all = publish_nav_all    


class Garmin18x(NMEA):
    SENSOR_MODEL = "garmin_18x"

    FRAME_ID = "link"
    PORT = "/dev/ttyACM0"
    BAUD = 115200

    def __init__(
            self,
            idx: int = None,
            name: str = None,
            topic: str = BaseGPS.TOPIC,
            frame_id: str = FRAME_ID,
            port: str = PORT,
            baud: int = BAUD,
            urdf_enabled: bool = BaseSensor.URDF_ENABLED,
            launch_enabled: bool = BaseSensor.LAUNCH_ENABLED,
            ros_parameters: str = BaseSensor.ROS_PARAMETERS,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY) -> None:
        super().__init__(
            idx,
            name,
            topic,
            frame_id,
            port,
            baud,
            urdf_enabled,
            launch_enabled,
            ros_parameters,
            parent,
            xyz,
            rpy
        )


class NovatelSmart6(NMEA):
    SENSOR_MODEL = "novatel_smart6"

    FRAME_ID = "link"
    PORT = "/dev/ttyACM0"
    BAUD = 115200

    def __init__(
            self,
            idx: int = None,
            name: str = None,
            topic: str = BaseGPS.TOPIC,
            frame_id: str = FRAME_ID,
            port: str = PORT,
            baud: int = BAUD,
            urdf_enabled: bool = BaseSensor.URDF_ENABLED,
            launch_enabled: bool = BaseSensor.LAUNCH_ENABLED,
            ros_parameters: str = BaseSensor.ROS_PARAMETERS,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY) -> None:
        super().__init__(
            idx,
            name,
            topic,
            frame_id,
            port,
            baud,
            urdf_enabled,
            launch_enabled,
            ros_parameters,
            parent,
            xyz,
            rpy
        )


class NovatelSmart7(NMEA):
    SENSOR_MODEL = "novatel_smart7"

    FRAME_ID = "link"
    PORT = "/dev/ttyACM0"
    BAUD = 115200

    def __init__(
            self,
            idx: int = None,
            name: str = None,
            topic: str = BaseGPS.TOPIC,
            frame_id: str = FRAME_ID,
            port: str = PORT,
            baud: int = BAUD,
            urdf_enabled: bool = BaseSensor.URDF_ENABLED,
            launch_enabled: bool = BaseSensor.LAUNCH_ENABLED,
            ros_parameters: str = BaseSensor.ROS_PARAMETERS,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY) -> None:
        super().__init__(
            idx,
            name,
            topic,
            frame_id,
            port,
            baud,
            urdf_enabled,
            launch_enabled,
            ros_parameters,
            parent,
            xyz,
            rpy
        )

class ArduSimpleRTKLite(Ublox):
    SENSOR_MODEL = "ardusimple_RTKLite"
    FRAME_ID = "link"
  
    def __init__(
            self,
            idx: int = None,
            name: str = None,
            topic: str = BaseGPS.TOPIC,
            frame_id: str = FRAME_ID,
            urdf_enabled: bool = BaseSensor.URDF_ENABLED,
            launch_enabled: bool = BaseSensor.LAUNCH_ENABLED,
            ros_parameters: dict = Ublox.ROS_PARAMETERS,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY,
            device: str = Ublox.DEVICE,
            uart1_baudrate: int = Ublox.UART1_BAUDRATE,
            rate: float = Ublox.RATE,
            dynamic_model = Ublox.DYNAMIC_MODEL,
            enable_ppp = Ublox.ENABLE_PPP,
            tmode3 = Ublox.TMODE3,
            sv_in_reset = Ublox.SV_IN_RESET,
            sv_in_min_dur = Ublox.SV_IN_MIN_DUR,
            sv_in_acc_limit = Ublox.SV_IN_ACC_LIMIT,
            inf_all = Ublox.INF_ALL,
            publish_all = Ublox.PUBLISH_ALL,
            publish_nav_all = Ublox.PUBLISH_NAV_ALL,
            ) -> None: 
        super().__init__(
            idx,
            name,
            topic,
            frame_id,
            urdf_enabled,
            launch_enabled,
            ros_parameters,
            parent,
            xyz,
            rpy,
            device,
            uart1_baudrate,
            rate,
            dynamic_model,
            enable_ppp,
            tmode3,
            sv_in_reset,
            sv_in_min_dur,
            sv_in_acc_limit,
            inf_all,
            publish_all,
            publish_nav_all,
        )
