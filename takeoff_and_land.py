import asyncio
from mavsdk import System
from mavsdk import OffboardError, VelocityBodyYawspeed

async def run():
    
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode errored with code {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Taking off")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -0.5, 0.0))

    await asyncio.sleep(4)

    print("-- Turning myself...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 30.0))
    await asyncio.sleep(15)

    print("-- Landing")
    await drone.action.land()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
