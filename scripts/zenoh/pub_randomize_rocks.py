__author__ = "Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import asyncio

from asyncio_for_robotics.zenoh.session import auto_session


async def main():
    pub = auto_session().declare_publisher("OmniLRS/terrain/randomize_rocks")

    try:
        pub.put("10")
    finally:
        pub.undeclare()


asyncio.run(main())
