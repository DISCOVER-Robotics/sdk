import cv2
import numpy as np
import multiprocessing as mp
import json
import os

from .hdf5er import save_one_episode


class CppData2Python(object):
    def __init__(self, root_dir, task_name, json_name, camera_names, episode_num):
        """
        default video type is "avi"; if you want to change it, you can use "set_video_type" method
        """
        self.root_dir = root_dir
        self.json_name = json_name + ".json"
        self.camera_names = camera_names
        if not isinstance(episode_num, int):
            self.start_episode = episode_num[0]
            self.end_episode = episode_num[1]
            self.episode_num = self.end_episode - self.start_episode + 1
        else:
            self.episode_num = episode_num
            self.start_episode = 0
            self.end_episode = episode_num - 1
        if root_dir[-1] != "/":
            root_dir += "/"
        if task_name != "":
            self.task_path = root_dir + task_name + "/"
        else:
            self.task_path = root_dir
        # optional record
        self.other_record = {"vel": False, "eff": False, "base": False}
        self.states_num, self.episode_len = self.get_data_info()
        print(f"states_num: {self.states_num}, episode_len: {self.episode_len}")
        print("Start to init data")
        self.data = {
            "/observations/qpos": [
                np.zeros(self.states_num) for _ in range(self.episode_len)
            ],
            "/action": [np.zeros(self.states_num) for _ in range(self.episode_len)],
        }
        for name in self.camera_names:
            self.data["/observations/images/" + name] = [
                np.zeros((480, 640, 3)) for _ in range(self.episode_len)
            ]
        self.video_type = "avi"
        print("Init data done.")

    def set_other_record(self, names):
        """设置额外的记录，如速度、力矩、底盘等数据"""
        for name in names:
            if name == "vel":
                self.data["/observations/qvel"] = [
                    np.zeros(self.states_num) for _ in range(self.episode_len)
                ]
                self.other_record["vel"] = True
            elif name == "eff":
                self.data["/observations/effort"] = [
                    np.zeros(self.states_num) for _ in range(self.episode_len)
                ]
                self.other_record["eff"] = True
            elif name == "base":
                self.data["/base_action"] = [
                    np.zeros(2) for _ in range(self.episode_len)
                ]
                self.other_record["base"] = True

    def get_data_info(self) -> tuple:
        """通过读取第一个episode的json文件，获取数据的信息"""
        episode = self.start_episode
        file_path = self.task_path + f"{episode}/" + self.json_name
        with open(file_path) as f_obj:
            raw_json_data = json.load(f_obj)
        states_num = len(raw_json_data["/observations/pos_f"][0]) + 1
        episode_len = len(raw_json_data["/observations/pos_f"])
        return states_num, episode_len

    def read_json_data(self, episode):
        """读取json文件中的数据，并存储到self.data中"""
        file_path = self.task_path + f"{episode}/" + self.json_name
        with open(file_path) as f_obj:
            data = json.load(f_obj)
        assert len(data["/observations/pos_f"]) == self.episode_len
        assert len(data["/observations/pos_f"][0]) == self.states_num - 1
        if self.states_num == 7:
            pos_f = np.array(data["/observations/pos_f"])
            endpos_f = np.array(data["/observations/endpos_f"])
            pos_t = np.array(data["/observations/pos_t"])
            endpos_t = np.array(data["/observations/endpos_t"])
            self.data["/observations/qpos"] = np.hstack(
                (pos_f, endpos_f.reshape(-1, 1))
            )
            self.data["/action"] = np.hstack((pos_t, endpos_t.reshape(-1, 1)))
            assert self.data["/observations/qpos"].shape == (
                self.episode_len,
                7,
            ), f"qpos shape: {self.data['/observations/qpos'].shape} not equal to ({self.episode_len}, 7)"
            if self.other_record["vel"]:
                vel_f = np.array(data["/observations/vel_f"])
                endvel_f = np.array(data["/observations/endvel_f"])
                self.data["/observations/qvel"] = np.hstack(
                    (vel_f, endvel_f.reshape(-1, 1))
                )
            if self.other_record["eff"]:
                eff_f = np.array(data["/observations/eff_f"])
                endeff_f = np.array(data["/observations/endeff_f"])
                self.data["/observations/effort"] = np.hstack(
                    (eff_f, endeff_f.reshape(-1, 1))
                )
            if self.other_record["base"]:
                self.data["/base_action"] = data["/base_action"]

    def read_video_images(self, episode):
        """读取视频文件中的图像，并存储到self.data中"""
        for camera_name in self.camera_names:
            video_path = (
                self.task_path + f"{episode}/" + camera_name + f".{self.video_type}"
            )
            cap = cv2.VideoCapture(video_path)
            if not cap.isOpened():
                raise Exception("Video not opened.")
            frame_count = 0
            while cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    self.data[f"/observations/images/{camera_name}"][
                        frame_count
                    ] = frame
                    frame_count += 1
                else:
                    break
            cap.release()

    def save_one(self, episode: int, dir=None):
        """Save data to hdf5 for each episode to the same folder as the raw data."""
        # for episode in range(self.start_episode, self.end_episode + 1):
        self.read_json_data(episode)
        self.read_video_images(episode)
        if dir is not None:
            self.task_path = dir
        save_one_episode(
            self.data,
            self.camera_names,
            self.task_path,
            f"episode_{episode}",
            overwrite=True,
            no_base=not self.other_record["base"],
            no_effort=not self.other_record["eff"],
            no_velocity=not self.other_record["vel"],
            compress=True if self.video_type == "avi" else False,
            states_num=self.states_num,
        )

    def save(self, nproc=4, dir=None):
        """Save all data to hdf5."""
        if nproc == 1:
            for episode in range(self.start_episode, self.end_episode + 1):
                self.save_one(episode)
        else:
            with mp.Pool(nproc) as pool:
                pool.map(self.save_one, range(self.start_episode, self.end_episode + 1))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser("convert cpp data to python data")
    parser.add_argument(
        "-rd",
        "--root_dir",
        type=str,
        default="/home/ghz/Work/airbot_play/arm-control/build/demonstrations/",
    )
    parser.add_argument("-tn", "--task_name", type=str, default="test_task")
    parser.add_argument("-jn", "--json_name", type=str, default="records")
    parser.add_argument(
        "-cn", "--camera_names", type=str, default="cam_high,cam_low,cam_left_wrist"
    )
    parser.add_argument("-se", "--start_episode", type=int, default=1, help="inclusive")
    parser.add_argument("-ee", "--end_episode", type=int, default=1, help="inclusive")
    parser.add_argument("-vt", "--video_type", type=str, default="avi")
    parser.add_argument(
        "-sd", "--save_dir", type=str, default=f"{os.getcwd()}/hdf5_data/"
    )
    parser.add_argument(
        "-or", "--other_record", type=str, default="", help="e.g. vel,eff,base"
    )
    parser.add_argument(
        "-nat",
        "--not_add_task_name",
        action="store_true",
        help="not auto add task name for more custom save dir",
    )
    args, unknown = parser.parse_known_args()
    # data path
    root_dir = args.root_dir
    task_name = args.task_name
    json_name = args.json_name
    video_type = args.video_type
    hdf5_dir = args.save_dir if args.save_dir != "" else root_dir + task_name + "/"
    # convert camera names to list
    camera_names = args.camera_names.split(",")
    episode_num = (args.start_episode, args.end_episode)
    # init
    datar = CppData2Python(root_dir, task_name, json_name, camera_names, episode_num)
    datar.video_type = video_type
    if args.other_record != "":
        datar.set_other_record(args.other_record.split(","))
    if not args.not_add_task_name:
        if hdf5_dir[-1] != "/":
            hdf5_dir += "/"
        hdf5_dir += task_name + "/"
    datar.save(8, hdf5_dir)
