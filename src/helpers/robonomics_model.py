#!/usr/bin/env python3
import json
import base64

from ipfs_common.msg import Multihash

from ipfs_common.ipfs_rosbag import IpfsRosBag
from helpers.models import Multihash as M
from helpers.ipfs import ipfs_download

class RobonomicsModel:
    def __init__(self, ipfs_hash: str):
        self.model_hash = ipfs_hash
        data = self._download_model()

        self.multihash = M(hash = self.model_hash, data = data)

    def _download_model(self) -> str:
        try:
            data = ipfs_download(self.model_hash)
            self.valid = True
            self.rosbag_scheme = json.loads(data)["rosbag_scheme"]
        except TimeoutError as e:
            data = ""
            self.valid = False

        return data

    def objective(self, objective_hash: Multihash) -> M:
        if self.valid:
            data = {}
            bag = IpfsRosBag(multihash = objective_hash)

            for kbag, vbag in bag.messages:
                ttype = self._topic_type(kbag)

                if ttype == "String":
                    data[kbag] = vbag
                elif ttype == "IPFSBin":
                    data[kbag] = []
                    for v in vbag:
                        data[kbag].append(self._ipfs_bin(v))
                elif ttype == "IPFSStr":
                    data[kbag] = []
                    for v in vbag:
                        data[kbag].append(self._ipfs_str(v))

            obj = M(hash = objective_hash, data = json.dumps(data))
        else:
            obj = M(hash = objective_hash, data = "")

        return obj

    def result(self, result_hash: Multihash) -> M:
        if self.valid:
            pass
        else:
            obj = M(hash = result_hash, data = "")

        return obj

    def _topic_type(self, topic: str) -> str:
        for s in self.rosbag_scheme:
            if topic.endswith(s["/suffix"]):
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
            data = base64.b64encode(data)
        except TimeoutError as e:
            data = ""

        return data

