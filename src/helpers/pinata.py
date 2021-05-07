import os
from tempfile import NamedTemporaryFile
from rosbag import Bag
import requests
import rospy

def pinata_rosbag(ipfs_hash: str) -> str:

    data = requests.get(f'https://gateway.pinata.cloud/ipfs/{ipfs_hash}')
    rospy.loginfo(data.status_code)
    if data.status_code != 200:
        rospy.loginfo(f'Error downloading rosbag! {data.content}')
        return

    else:
        tmpfile = NamedTemporaryFile(delete=False)
        tmpfile.write(data.content)
        bag = Bag(tmpfile.name)
        
        messages = {}
        for topic, msg, timestamp in bag.read_messages():
            if topic not in messages:
                messages[topic] = [msg]
            else:
                messages[topic].append(msg)

        os.unlink(tmpfile.name)
    return (messages, bag)


def pinata_download(hash: str, mode: str = "r") -> str:
    
    res = requests.get(f'https://gateway.pinata.cloud/ipfs/{hash}')

    if res.status_code != 200:
        rospy.loginfo(f'Error while downloading from Pinata: {res.content}')

    else:
        tmpfile = NamedTemporaryFile(delete=False)
        tmpfile.write(res.content)
        with open(tmpfile.name, mode) as f:
            return f.read()
