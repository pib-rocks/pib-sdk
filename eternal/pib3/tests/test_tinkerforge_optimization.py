import unittest
from unittest.mock import MagicMock, patch
import sys

# Mock tinkerforge modules before importing pib3
sys.modules["tinkerforge"] = MagicMock()
sys.modules["tinkerforge.ip_connection"] = MagicMock()
sys.modules["tinkerforge.bricklet_servo_v2"] = MagicMock()

from pib3.backends.robot import RealRobotBackend

class TestTinkerforgeOptimization(unittest.TestCase):
    def setUp(self):
        self.mock_ipcon = MagicMock()
        self.mock_servo = MagicMock()
        
        # Create backend without auto-connect
        self.backend = RealRobotBackend(motor_mode="direct")
        # Manually set up internal state for testing
        self.backend._tinkerforge_conn = self.mock_ipcon
        self.backend._tinkerforge_motor_map = {"test_motor": ("UID123", 0)}
        self.backend._tinkerforge_servos = {"UID123": self.mock_servo}
        self.backend._servo_enabled_cache = {}
            
    def test_configuration_expansion(self):
        """Test that configuration sets period and degree range."""
        self.backend.configure_servo_channel(
            "test_motor",
            period=20000,
            degree_min=-10000,
            degree_max=10000
        )
        
        # Verify new configuration calls
        self.mock_servo.set_period.assert_called_with(0, 20000)
        self.mock_servo.set_degree.assert_called_with(0, -10000, 10000)
        
    def test_enable_caching(self):
        """Test that set_enable is not called repeatedly."""
        # First call should enable
        self.backend._set_motor_direct("test_motor", 0)
        self.mock_servo.set_enable.assert_called_with(0, True)
        self.mock_servo.set_enable.reset_mock()
        
        # Second call should NOT enable (cached)
        self.backend._set_motor_direct("test_motor", 1000)
        self.mock_servo.set_enable.assert_not_called()
        
        # Third call should NOT enable
        self.backend._set_motor_direct("test_motor", 2000)
        self.mock_servo.set_enable.assert_not_called()
        
    def test_cache_clearing(self):
        """Test that cache is cleared on disconnect."""
        # Enable first
        self.backend._set_motor_direct("test_motor", 0)
        self.mock_servo.set_enable.assert_called_with(0, True)
        self.mock_servo.set_enable.reset_mock()

        # Disconnect and reconnect (simulated by clearing and re-init)
        self.backend._disconnect_tinkerforge()
        # Provide motor_mapping so reconnect uses it instead of auto-discovery
        self.backend._low_latency_config.motor_mapping = {"test_motor": ("UID123", 0)}
        self.backend._connect_tinkerforge()
        # Mock re-init of servo object since disconnect clears it
        with patch("tinkerforge.bricklet_servo_v2.BrickletServoV2", return_value=self.mock_servo):
             self.backend._init_servo_bricklets()

        # Should enable again after reconnect
        self.backend._set_motor_direct("test_motor", 1000)
        self.mock_servo.set_enable.assert_called_with(0, True)


if __name__ == "__main__":
    unittest.main()
