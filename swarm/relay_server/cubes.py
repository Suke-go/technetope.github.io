from toio import *

class CubeStatus:

  def __init__(self, cube, cube_id):
    self.cube = cube
    self.cube_id = cube_id
    self.name = cube.name
    self.x = None
    self.y = None
    self.angle = None
    self.on_mat = False  # キューブがマット上にいるかどうかを保持

  def update(self, x, y, angle):
    self.x = x
    self.y = y
    self.angle = angle

  def set_on_mat(self, on_mat):
    # true: マット上、 false: マット外
    self.on_mat = on_mat


# 通知ハンドラ
def create_notification_handler(cube_status: CubeStatus, on_update=None):

  def handler(payload: bytearray, handler_info):
    id_info = IdInformation.is_my_data(payload)
    if isinstance(id_info, PositionId):
      x = id_info.center.point.x
      y = id_info.center.point.y
      angle = id_info.center.angle
      cube_status.update(x, y, angle)
      cube_status.set_on_mat(True)
      if on_update:
        on_update()
    elif isinstance(id_info, PositionIdMissed):
      cube_status.set_on_mat(False)
      if on_update:
        on_update()
      # print(f"Position: Missed for {cube_status.name} in notification.")
    else:
      # print(f"No valid position data for {cube_status.name} in notification.")
      pass

  return handler

async def scan_cubes(ids_to_find):
    """Scan for cubes based on given IDs."""
    dev_list = []
    for cube_id in ids_to_find:
        devs = await BLEScanner.scan_with_id(cube_id={cube_id})
        dev_list += devs
    return dev_list

async def connect_cube(dev):
    """Connect to a single cube from the scanned device."""
    cube = ToioCoreCube(dev.interface)
    try:
        await cube.connect()
        cube.name = dev.name
        return cube
    except Exception as e:
        print(f"Failed to connect to device {dev.name}: {e}")
        return None

async def read_battery_level(cube):
    """Read and return the battery level of a single cube."""
    batt_info = await cube.api.battery.read()
    return batt_info.battery_level

async def register_notification_handler(cube, cube_status, on_update=None):
    """Register notification handler for a single cube."""
    handler = create_notification_handler(cube_status, on_update)
    await cube.api.id_information.register_notification_handler(handler)

async def set_led(cube, r, g, b):
  await cube.api.indicator.turn_on(
    IndicatorParam(duration_ms=0, color=Color(r=r, g=g, b=b)))

async def set_motor(cube, left, right):
    """Set motor control for a cube."""
    await cube.api.motor.motor_control(left, right)

async def disconnect_cube(cube_status):
    res = await cube_status.cube.disconnect()
    return res
