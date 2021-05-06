import os
from tempfile import NamedTemporaryFile
from rosbag import Bag
import requests
import rospy

def pinata_download(ipfs_hash: str, mode: str = "r") -> str:

    try:
        data = requests.get(f'https://gateway.pinata.cloud/ipfs/{ipfs_hash}')
    except Exception as e:
        rospy.loginfo(f'downloading error: {e}')
    
    tmpfile = NamedTemporaryFile(delete=False)
    tmpfile.write(data.content)
    bag = Bag(tmpfile.name)
    topics_list = bag.get_type_and_topic_info()[1].keys()
    messages = {}

    for topic, msg, timestamp in bag.read_messages():
        if topic not in messages:
            messages[topic] = [msg]
        else:
            messages[topic].append(msg)

    os.unlink(tmpfile.name)
    return (messages, bag)