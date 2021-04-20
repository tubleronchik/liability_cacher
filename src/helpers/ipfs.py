import rospy
import os
import ipfshttpclient
#from urllib3.util.timeout import Timeout
from urllib.parse import urlparse
from tempfile import gettempdir


IPFS_PROVIDER = rospy.get_param("/liability/listener/ipfs_http_provider", "/ip4/127.0.0.1/tcp/5001/http")

def build_client():
    rospy.loginfo("Build IPFS client: %s", IPFS_PROVIDER)
    #no_read_timeout = Timeout(read=None)
    return ipfshttpclient.connect(IPFS_PROVIDER, session=True)

ipfs_client = build_client()

def ipfs_download(ipfs_hash: str, mode: str = "r") -> str:
    tempdir = gettempdir()
    os.chdir(tempdir)

    rospy.loginfo(f"Downloading {ipfs_hash}")
    ipfs_client.get(ipfs_hash, timeout=60)
    with open(ipfs_hash, mode) as f:
        return f.read()

