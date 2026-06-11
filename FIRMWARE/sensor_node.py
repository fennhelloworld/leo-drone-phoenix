#!/usr/bin/env python3
"""
LeoDrone Phoenix — 传感器节点
BME280 (温湿度气压) + ICM-42688-P (IMU) 传感器读取

支持真实硬件和仿真模式
"""

import time
import math
import logging
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Optional

logger = logging.getLogger("SensorNode")


@dataclass
class SensorReading:
    """传感器读数"""
    timestamp: float = 0.0
    # IMU
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # BME280
    temperature: float = 25.0
    humidity: float = 50.0
    pressure: float = 1013.25
    altitude: float = 0.0
    # GPS (simulated)
    lat: float = 39.9042
    lon: float = 116.4074
    alt: float = 50.0
    fix_type: int = 3
    num_satellites: int = 10
    hdop: float = 1.0


class BME280Driver:
    """BME280 温湿度气压传感器驱动"""

    # BME280 calibration constants (typical)
    DIG_T1 = 28198
    DIG_T2 = 26346
    DIG_T3 = 50
    DIG_P1 = 37491
    DIG_P2 = -10642
    DIG_P3 = 3024
    DIG_P4 = 6821
    DIG_P5 = 18
    DIG_P6 = -82
    DIG_P7 = 89
    DIG_P8 = 4916
    DIG_P9 = -4170

    def __init__(self, address: int = 0x76, bus: int = 1):
        self.address = address
        self.bus = bus
        self._device = None
        self._t_fine = 0

    def initialize(self) -> bool:
        """Initialize the BME280 sensor"""
        try:
            import smbus2
            self._device = smbus2.SMBus(self.bus)
            # Check chip ID
            chip_id = self._device.read_byte_data(self.address, 0xD0)
            if chip_id != 0x60:
                logger.warning(f"BME280: Unexpected chip ID 0x{chip_id:02X}")
            # Set oversampling and mode
            self._device.write_byte_data(self.address, 0xF2, 0x01)  # Humidity oversampling x1
            self._device.write_byte_data(self.address, 0xF4, 0x27)  # Temp+Press oversampling x1, normal mode
            self._device.write_byte_data(self.address, 0xF5, 0xA0)  # Standby 1000ms, filter off
            return True
        except Exception as e:
            logger.warning(f"BME280 init failed: {e}, using simulated data")
            self._device = None
            return False

    def read(self) -> Dict[str, float]:
        """Read temperature, humidity, and pressure"""
        if self._device is None:
            return self._simulated_read()

        try:
            import smbus2
            # Read raw data
            data = self._device.read_i2c_block_data(self.address, 0xF7, 8)
            raw_pres = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
            raw_temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
            raw_hum = (data[6] << 8) | data[7]

            # Compensate
            temp = self._compensate_temp(raw_temp)
            pressure = self._compensate_pressure(raw_pres)
            humidity = self._compensate_humidity(raw_hum)
            altitude = self._pressure_to_altitude(pressure)

            return {
                'temperature': temp,
                'humidity': humidity,
                'pressure': pressure,
                'altitude': altitude
            }
        except Exception as e:
            logger.error(f"BME280 read error: {e}")
            return self._simulated_read()

    def _compensate_temp(self, raw_temp: int) -> float:
        """Compensate temperature reading"""
        var1 = (((raw_temp >> 3) - (self.DIG_T1 << 1)) * self.DIG_T2) >> 11
        var2 = (((((raw_temp >> 4) - self.DIG_T1) * ((raw_temp >> 4) - self.DIG_T1)) >> 12) * self.DIG_T3) >> 14
        self._t_fine = var1 + var2
        return ((self._t_fine * 5 + 128) >> 8) / 100.0

    def _compensate_pressure(self, raw_pres: int) -> float:
        """Compensate pressure reading"""
        var1 = (self._t_fine >> 1) - 64000
        var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * self.DIG_P6) >> 2
        var2 = var2 + ((var1 * self.DIG_P5) << 1)
        var2 = (var2 >> 2) + (self.DIG_P4 << 16)
        var1 = (((self.DIG_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((self.DIG_P2 * var1) >> 1)) >> 18
        var1 = ((32768 + var1) * self.DIG_P1) >> 15
        if var1 == 0:
            return 0
        p = ((1048576 - raw_pres) - (var2 >> 12)) * 3125
        p = (p // var1) * 2 if p < 0x80000000 else (p * 2) // var1
        var1 = (self.DIG_P9 * (((p >> 3) * (p >> 3)) >> 13)) >> 12
        var2 = ((p >> 2) * self.DIG_P8) >> 13
        p = p + ((var1 + var2 + self.DIG_P7) >> 4)
        return p / 100.0

    def _compensate_humidity(self, raw_hum: int) -> float:
        """Compensate humidity reading"""
        v_x1_u32r = (self._t_fine - (76800 << 10))  # Simplified
        v_x1_u32r = ((((raw_hum << 14) - (78688 << 10)) * 100) >> 10) * 32768
        # Simplified compensation
        humidity = raw_hum / 1024.0 * 100.0
        return max(0.0, min(100.0, humidity))

    @staticmethod
    def _pressure_to_altitude(pressure_hpa: float) -> float:
        """Convert pressure to altitude using barometric formula"""
        return 44330.0 * (1.0 - pow(pressure_hpa / 1013.25, 0.1903))

    def _simulated_read(self) -> Dict[str, float]:
        """Generate simulated BME280 data"""
        t = time.time()
        return {
            'temperature': 25.0 + 2.0 * math.sin(t * 0.1) + np.random.normal(0, 0.1),
            'humidity': 50.0 + 10.0 * math.sin(t * 0.05) + np.random.normal(0, 1),
            'pressure': 1013.25 + 5.0 * math.sin(t * 0.02) + np.random.normal(0, 0.1),
            'altitude': 50.0 + 2.0 * math.sin(t * 0.1) + np.random.normal(0, 0.5)
        }


class ICM42688Driver:
    """ICM-42688-P IMU 驱动"""

    # Registers
    REG_DEVICE_CONFIG = 0x11
    REG_PWR_MGMT0 = 0x4E
    REG_ACCEL_CONFIG = 0x21
    REG_GYRO_CONFIG = 0x20

    def __init__(self, address: int = 0x68, bus: int = 1,
                 accel_range: int = 16, gyro_range: int = 2000):
        self.address = address
        self.bus = bus
        self._device = None
        self.accel_range = accel_range
        self.gyro_range = gyro_range

        # Scale factors
        self.accel_scale = accel_range / 32768.0
        self.gyro_scale = math.radians(gyro_range) / 32768.0

        # Simulation state
        self._sim_time = 0.0

    def initialize(self) -> bool:
        """Initialize the ICM-42688-P"""
        try:
            import smbus2
            self._device = smbus2.SMBus(self.bus)
            # Soft reset
            self._device.write_byte_data(self.address, self.REG_DEVICE_CONFIG, 0x01)
            time.sleep(0.01)
            # Enable accel + gyro in low-noise mode
            self._device.write_byte_data(self.address, self.REG_PWR_MGMT0, 0x0F)
            time.sleep(0.05)
            return True
        except Exception as e:
            logger.warning(f"ICM-42688 init failed: {e}, using simulated data")
            self._device = None
            return False

    def read(self) -> Dict[str, np.ndarray]:
        """Read accelerometer and gyroscope data"""
        if self._device is None:
            return self._simulated_read()

        try:
            import smbus2
            # Read accel (0x1F-0x24) and gyro (0x25-0x2A)
            data = self._device.read_i2c_block_data(self.address, 0x1F, 12)
            ax = (data[0] << 8) | data[1]
            ay = (data[2] << 8) | data[3]
            az = (data[4] << 8) | data[5]
            gx = (data[6] << 8) | data[7]
            gy = (data[8] << 8) | data[9]
            gz = (data[10] << 8) | data[11]

            # Convert to signed
            ax = ax - 65536 if ax > 32767 else ax
            ay = ay - 65536 if ay > 32767 else ay
            az = az - 65536 if az > 32767 else az
            gx = gx - 65536 if gx > 32767 else gx
            gy = gy - 65536 if gy > 32767 else gy
            gz = gz - 65536 if gz > 32767 else gz

            return {
                'accel': np.array([ax, ay, az]) * self.accel_scale,
                'gyro': np.array([gx, gy, gz]) * self.gyro_scale
            }
        except Exception as e:
            logger.error(f"ICM-42688 read error: {e}")
            return self._simulated_read()

    def _simulated_read(self) -> Dict[str, np.ndarray]:
        """Generate simulated IMU data"""
        self._sim_time += 0.005  # 200Hz
        t = self._sim_time

        # Simulate gentle hovering motion
        ax = 0.1 * math.sin(t * 0.5) + np.random.normal(0, 0.02)
        ay = 0.1 * math.cos(t * 0.7) + np.random.normal(0, 0.02)
        az = 9.81 + 0.05 * math.sin(t * 0.3) + np.random.normal(0, 0.01)

        gx = 0.01 * math.sin(t * 0.4) + np.random.normal(0, 0.005)
        gy = 0.01 * math.cos(t * 0.6) + np.random.normal(0, 0.005)
        gz = 0.005 * math.sin(t * 0.2) + np.random.normal(0, 0.005)

        return {
            'accel': np.array([ax, ay, az]),
            'gyro': np.array([gx, gy, gz])
        }


class SimulatedGPS:
    """仿真 GPS (M10N)"""

    def __init__(self, base_lat: float = 39.9042, base_lon: float = 116.4074,
                 base_alt: float = 50.0):
        self.base_lat = base_lat
        self.base_lon = base_lon
        self.base_alt = base_alt
        self._sim_time = 0.0

    def read(self) -> Dict:
        """Read simulated GPS data"""
        self._sim_time += 0.1  # 10Hz
        t = self._sim_time

        # Simulate small drift
        lat_noise = np.random.normal(0, 2.5e-7)  # ~2.5m
        lon_noise = np.random.normal(0, 2.5e-7)
        alt_noise = np.random.normal(0, 2.5)

        return {
            'lat': self.base_lat + 0.00001 * math.sin(t * 0.1) + lat_noise,
            'lon': self.base_lon + 0.00001 * math.cos(t * 0.1) + lon_noise,
            'alt': self.base_alt + 5.0 * math.sin(t * 0.05) + alt_noise,
            'fix_type': 3,
            'num_satellites': np.random.randint(8, 14),
            'hdop': np.random.uniform(0.5, 1.5)
        }


class SensorNode:
    """传感器节点 — 统一管理所有传感器"""

    def __init__(self, simulation: bool = True):
        self.simulation = simulation
        self._running = False

        # Initialize drivers
        self.bme280 = BME280Driver()
        self.icm42688 = ICM42688Driver()
        self.gps = SimulatedGPS()

        # Try to initialize real hardware
        if not simulation:
            self.bme280.initialize()
            self.icm42688.initialize()

        # Latest readings
        self._latest = SensorReading()
        self._lock = None

    def start(self):
        """Start the sensor node"""
        self._running = True
        logger.info(f"Sensor node started ({'simulation' if self.simulation else 'real'})")

    def stop(self):
        """Stop the sensor node"""
        self._running = False
        logger.info("Sensor node stopped")

    def read_all(self) -> Dict:
        """Read all sensors and return combined data"""
        # IMU
        imu_data = self.icm42688.read()
        # BME280
        env_data = self.bme280.read()
        # GPS
        gps_data = self.gps.read()

        # Combine
        reading = {
            'timestamp': time.time(),
            'accel': imu_data['accel'],
            'gyro': imu_data['gyro'],
            'temperature': env_data['temperature'],
            'humidity': env_data['humidity'],
            'pressure': env_data['pressure'],
            'altitude': env_data['altitude'],
            'lat': gps_data['lat'],
            'lon': gps_data['lon'],
            'alt': gps_data['alt'],
            'fix_type': gps_data['fix_type'],
            'num_satellites': gps_data['num_satellites'],
            'hdop': gps_data['hdop']
        }

        # Update latest
        self._latest = SensorReading(
            timestamp=reading['timestamp'],
            accel=imu_data['accel'],
            gyro=imu_data['gyro'],
            temperature=env_data['temperature'],
            humidity=env_data['humidity'],
            pressure=env_data['pressure'],
            altitude=env_data['altitude'],
            lat=gps_data['lat'],
            lon=gps_data['lon'],
            alt=gps_data['alt'],
            fix_type=gps_data['fix_type'],
            num_satellites=gps_data['num_satellites'],
            hdop=gps_data['hdop']
        )

        return reading

    @property
    def latest(self) -> SensorReading:
        """Get the latest sensor reading"""
        return self._latest

    def get_wind_estimate(self) -> Dict[str, float]:
        """Estimate wind speed from pressure and IMU data"""
        # Simple wind estimation from pressure changes
        pressure_change_rate = 0.0  # Would track over time
        wind_speed = abs(pressure_change_rate) * 10.0  # Simplified
        wind_direction = 0.0  # Would use GPS drift

        return {
            'wind_speed': wind_speed + np.random.normal(0, 0.5),
            'wind_direction': wind_direction + np.random.normal(0, 5),
            'gust_speed': wind_speed * 1.5 + np.random.normal(0, 1.0)
        }


# ---------------------------------------------------------------------------
# CLI Test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    node = SensorNode(simulation=True)
    node.start()

    print("Reading sensors for 5 seconds...")
    for i in range(50):
        data = node.read_all()
        print(f"[{i:3d}] T={data['temperature']:.1f}°C  "
              f"H={data['humidity']:.1f}%  "
              f"P={data['pressure']:.1f}hPa  "
              f"Alt={data['altitude']:.1f}m  "
              f"Accel=[{data['accel'][0]:.2f},{data['accel'][1]:.2f},{data['accel'][2]:.2f}]")
        time.sleep(0.1)

    node.stop()
