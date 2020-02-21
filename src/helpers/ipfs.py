import rospy
import os
import ipfshttpclient
from urllib3.util.timeout import Timeout
from tempfile import gettempdir


IPFS_PROVIDER = rospy.get_param("/liability/listener/ipfs_http_provider")

def build_client(provider_endpoint):
    rospy.loginfo("Build IPFS client: %s", provider_endpoint)
    ipfs_url = urlparse(provider_endpoint)
    ipfs_netloc = ipfs_url.netloc.split(':')
    no_read_timeout = Timeout(read=None)
    return ipfshttpclient.connect("/dns/{0}/tcp/{1}/{2}".format(ipfs_netloc[0], ipfs_netloc[1], ipfs_url.scheme),
                                  session=True, timeout=no_read_timeout)

ipfs_client = build_client(IPFS_PROVIDER)

def ipfs_download(ipfs_hash: str) -> str:
    tempdir = gettempdir()
    os.chdir(tempdir)

    rospy.loginfo(f"Downloading {ipfs_hash}")
    ipfs_client.get(ipfs_hash, timeout=30)

    with open(ipfs_hash) as f:
        return f.read()

