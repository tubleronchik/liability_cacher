#!/usr/bin/env python3
import json
import base64

from ipfs_common.msg import Multihash

from ipfs_common.ipfs_rosbag import IpfsRosBag
from helpers.ipfs import ipfs_download

class RobonomicsModel:
    def __init__(self, ipfs_hash: str):
        self.model_hash = ipfs_hash
        self.model_data = self._download_model()

    def _download_model(self) -> str:
        try:
            data = ipfs_download(self.model_hash)
            self.valid = True
            self.rosbag_scheme = json.loads(data)["rosbag_scheme"]
        except:
            data = ""
            self.valid = False

        print(f"Data: {data}\nValid: {self.valid}")
        return data

    def objective(self, objective_hash: Multihash) -> str:
        data = {}
        if self.valid:
            bag = IpfsRosBag(multihash = objective_hash)

            for kbag, vbag in bag.messages.items():
                print(f"{kbag} -> {vbag}")
                ttype = self._topic_type(kbag)

                if ttype == "String":
                    data[kbag] = []
                    for v in vbag:
                        data[kbag].append(v.data)
                elif ttype == "IPFSBin":
                    data[kbag] = []
                    for v in vbag:
                        data[kbag].append(self._ipfs_bin(v.data))
                elif ttype == "IPFSStr":
                    data[kbag] = []
                    for v in vbag:
                        data[kbag].append(self._ipfs_str(v.data))

        print(data)

        return json.dumps(data)

    def result(self, result_hash: Multihash) -> str:
        data = {}
        if self.valid:
            pass

        return json.dumps(data)

    def _topic_type(self, topic: str) -> str:
        for s in self.rosbag_scheme:
            if topic.endswith(s["suffix"]):
                return s["data_type"]

    def _ipfs_str(self, ipfs_hash: str) -> str:
        try:
            data = ipfs_download(ipfs_hash)
        except TimeoutError as e:
            data = ""

        return data

    def _ipfs_bin(self, ipfs_hash: str) -> str:
        try:
            data = ipfs_download(ipfs_hash, "rb")
            data = base64.b64encode(data).decode('utf-8')
        except TimeoutError as e:
            data = ""

        return data

