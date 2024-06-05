import airbot
import argparse
import pickle
import functools
import asyncio
import time
import websockets
import math


def diff(joints_1, joints_2):
    return sum([abs(j1 - j2) for j1, j2 in zip(joints_1, joints_2)])


def blend(joints_1, joints_2, ratio):
    return [j1 * ratio + j2 * (1 - ratio) for j1, j2 in zip(joints_1, joints_2)]


async def handler(robots, args, websocket):
    robot_left, robot_right = robots
    sync_start_left = sync_start_right = time.time()
    robot_left.set_max_current([100, 100, 100, 100, 100, 100])
    robot_right.set_max_current([100, 100, 100, 100, 100, 100])
    syncing_left = syncing_right = False
    await websocket.send("Welcome to the WebSocket server!")
    async for message in websocket:
        try:
            info = pickle.loads(message)

            left_target = info["left"]
            if diff(info["left"], robot_left.get_current_joint_q()) < 6 * math.pi * 0.1:
                if syncing_left:
                    print("Left resynced!")
                syncing_left = False
                left_target = info["left"]
            else:
                now = time.time()
                if not syncing_left:
                    print("Resyncing left...")
                    syncing_left = True
                    sync_start_left = time.time()
                left_target = blend(
                    info["left"],
                    robot_left.get_current_joint_q(),
                    now - sync_start_left,
                )
            if args.mit:
                robot_left.set_target_joint_mit(
                    left_target,
                    [0, 0, 0, 0, 0, 0],
                    [100, 100, 100, 10, 10, 10],
                    [1, 1, 1, 0.2, 0.2, 0.2],
                )
            else:
                robot_left.set_target_joint_q(left_target, use_planning=False, vel=4)
            right_target = info["right"]
            if (
                diff(info["right"], robot_right.get_current_joint_q())
                < 6 * math.pi * 0.1
            ):
                if syncing_right:
                    print("Right resynced!")
                syncing_right = False
                right_target = info["right"]
            else:
                now = time.time()
                if not syncing_right:
                    print("Resyncing right...")
                    syncing_right = True
                    sync_start_right = time.time()
                right_target = blend(
                    info["right"],
                    robot_right.get_current_joint_q(),
                    now - sync_start_right,
                )
            if args.mit:
                robot_right.set_target_joint_mit(
                    right_target,
                    [0, 0, 0, 0, 0, 0],
                    [100, 100, 100, 10, 10, 10],
                    [1, 1, 1, 0.2, 0.2, 0.2],
                )
            else:
                robot_right.set_target_joint_q(right_target, use_planning=False, vel=4)

            robot_left.set_target_end(info["left_end"])
            robot_right.set_target_end(info["right_end"])
        except Exception as e:
            print(type(e), e)


async def main(args):
    robot_left = airbot.create_agent(can_interface=args.left, end_mode="gripper")
    robot_right = airbot.create_agent(can_interface=args.right, end_mode="gripper")
    f = functools.partial(handler, (robot_left, robot_right), args)
    async with websockets.serve(
        f, args.host, int(args.port), ping_timeout=None, ping_interval=None
    ):
        await asyncio.Future()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send data to the airbot")
    parser.add_argument("-l", "--left", help="Sender CAN Interface", default="can0")
    parser.add_argument("-r", "--right", help="Sender CAN Interface", default="can1")
    parser.add_argument(
        "-H", "--host", help="Websocket listening host", default="localhost"
    )
    parser.add_argument("-p", "--port", help="Websocket listening port", default=18188)
    parser.add_argument("--mit", action="store_true", help="Use MIT mode")
    args = parser.parse_args()
    asyncio.run(main(args))
