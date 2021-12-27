import rospy
import trio

# FlightServices
from clover import srv
from std_srvs.srv import Trigger

from clover_swarm.async_ros import AsyncService

#



class FlightServices:
    get_telemetry = AsyncService("get_telemetry", srv.GetTelemetry)
    navigate = AsyncService("navigate", srv.Navigate)
    navigate_global = AsyncService("navigate_global", srv.NavigateGlobal)
    set_position = AsyncService("set_position", srv.SetPosition)
    set_velocity = AsyncService("set_velocity", srv.SetVelocity)
    set_attitude = AsyncService("set_attitude", srv.SetAttitude)
    set_rates = AsyncService("set_rates", srv.SetRates)
    land = AsyncService("land", Trigger)

    async def connect(self, timeout=None):
        async with trio.open_nursery() as nursery:
            nursery.start_soon(self.get_telemetry.connect, timeout)


class LedServices:
    pass
