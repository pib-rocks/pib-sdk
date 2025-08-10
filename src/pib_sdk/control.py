from typing import Optional, Any, Dict, Callable
import roslibpy

class Write:
    """
    A helper class to send motor settings over ROSBridge to the /motor_settings topic.

    The user specifies `position_deg` in degrees (−90 to 90), which is internally
    mapped to `position` units (×100: −9000 to 9000).

    Example:
        writer = Write(host='localhost', port=9090)
        writer.send(
            motor_name='joint1',
            position_deg=15.5,
            velocity=100,
            turned_on=True
        )
        writer.close()
    """

    def __init__(
        self,
        host: str = 'localhost',
        port: int = 9090,
        topic_name: str = '/motor_settings',
        message_type: str = 'datatypes/MotorSettings',
    ):
        # Connect to ROSBridge
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        # Publisher for MotorSettings messages
        self.topic = roslibpy.Topic(self.ros, topic_name, message_type)

    def send(
        self,
        motor_name: str,
        position_deg: float,
        velocity: Optional[int] = None,
        acceleration: Optional[int] = None,
        deceleration: Optional[int] = None,
        turned_on: Optional[bool] = None,
        pulse_width_min: Optional[int] = None,
        pulse_width_max: Optional[int] = None,
        rotation_range_min: Optional[int] = None,
        rotation_range_max: Optional[int] = None,
        period: Optional[int] = None,
        visible: Optional[bool] = None,
        invert: Optional[bool] = None,
        **kwargs: Any
    ) -> None:
        """
        Publish a MotorSettings message with the provided parameters.

        Args:
            motor_name: name of the motor/link
            position_deg: target angle in degrees (−90 to 90)
            velocity, acceleration, ... : optional motor parameters

        Internally, `position` = int(position_deg * 100) maps −90..90° to −9000..9000.
        """
        # Validate input range
        if not -90.0 <= position_deg <= 90.0:
            raise ValueError(f"position_deg must be between -90 and 90 (got {position_deg})")
        # Convert degrees to internal units
        position = int(position_deg * 100)

        # Base message
        msg: Dict[str, Any] = {
            'motor_name': motor_name,
            'position': position
        }
        # Dynamically add any optional parameter that has a non-None value
        params = {
            k: v
            for k, v in locals().items()
            if k not in (
                'self', 'kwargs', 'msg', 'motor_name', 'position_deg', 'position'
            ) and v is not None
        }
        msg.update(params)
        # Include custom fields
        msg.update(kwargs)
        # Publish
        self.topic.publish(roslibpy.Message(msg))

    def close(self) -> None:
        """
        Cleanly close the ROSBridge connection.
        """
        self.topic.unadvertise()
        self.ros.terminate()

# One-liner convenience function for writing

def write(
    motor_name: str,
    position_deg: float,
    *,
    host: str = 'localhost',
    port: int = 9090,
    topic_name: str = '/motor_settings',
    message_type: str = 'datatypes/MotorSettings',
    **settings: Any
) -> None:
    """
    One-call motor settings write.

    `position_deg` is required (−90 to 90°); other settings are optional.

    Example:
        write(
            'joint1',
            position_deg=15.5,
            velocity=100,
            turned_on=True
        )
    """
    writer = Write(
        host=host,
        port=port,
        topic_name=topic_name,
        message_type=message_type,
    )
    writer.send(motor_name=motor_name, position_deg=position_deg, **settings)
    writer.close()


class Read:
    """
    A helper class to subscribe to /motor_settings and receive position and current values.

    Example:
        def callback(data):
            motor = data.get('motor_name')
            pos = data.get('position')
            curr = data.get('current')
            print(f"{motor}: pos={pos}, current={curr}")

        reader = Read(callback)
        # run until done
        reader.close()
    """

    def __init__(
        self,
        callback: Callable[[Dict[str, Any]], None],
        host: str = 'localhost',
        port: int = 9090,
        topic_name: str = '/motor_settings',
        message_type: str = 'datatypes/MotorSettings',
    ):
        # Connect to ROSBridge
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        # Subscriber for MotorSettings messages
        self.topic = roslibpy.Topic(self.ros, topic_name, message_type)
        # Subscribe
        self.topic.subscribe(lambda msg: callback(msg))

    def close(self) -> None:
        """
        Unsubscribe and close the ROSBridge connection.
        """
        self.topic.unsubscribe()
        self.ros.terminate()

# One-liner convenience function for streaming (callback-based)

def read_stream(
    callback: Callable[[Dict[str, Any]], None],
    *,
    host: str = 'localhost',
    port: int = 9090,
    topic_name: str = '/motor_settings',
    message_type: str = 'datatypes/MotorSettings',
) -> Read:
    """
    Start a continuous subscription to /motor_settings; the callback receives each message dict.

    Example:
        def cb(data): print(data)
        reader = read_stream(cb)
    """
    return Read(callback, host=host, port=port, topic_name=topic_name, message_type=message_type)


# Ultra-simple one-shot read helper
import threading

def read(
    motor_name: str,
    *,
    host: str = 'localhost',
    port: int = 9090,
    topic_name: str = '/motor_settings',
    message_type: str = 'datatypes/MotorSettings',
    timeout: float = 3.0,
) -> Dict[str, Any]:
    """
    Read a single message for `motor_name` from /motor_settings and return it as a dict.

    Usage:
        data = read('joint1')
        # data contains keys like: motor_name, position (int), position_deg (float), current (float)

    Notes:
        - Blocks until a matching message arrives or `timeout` elapses (then raises TimeoutError).
        - Adds `position_deg` = position / 100.0 for convenience when `position` is present.
    """
    ros = roslibpy.Ros(host=host, port=port)
    ros.run()
    topic = roslibpy.Topic(ros, topic_name, message_type)

    result: Dict[str, Any] = {}
    evt = threading.Event()

    def _cb(msg: Dict[str, Any]) -> None:
        if msg.get('motor_name') == motor_name:
            result.update(msg)
            evt.set()

    topic.subscribe(_cb)
    ok = evt.wait(timeout)
    topic.unsubscribe()
    ros.terminate()

    if not ok:
        raise TimeoutError(f"No /motor_settings message for '{motor_name}' within {timeout} seconds")

    pos = result.get('position')
    if isinstance(pos, (int, float)):
        result['position_deg'] = float(pos) / 100.0
    return result
