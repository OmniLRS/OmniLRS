import asyncio

from asyncio_for_robotics.zenoh.session import auto_session


async def main():
    pub = auto_session().declare_publisher("OmniLRS/Terrain/RandomizeRocks")

    try:
        pub.put("10")
    finally:
        pub.undeclare()


asyncio.run(main())
