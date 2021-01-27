#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
from heron_msgs.msg import Drive
from sensor_msgs.msg import JointState
from ocean_lib.ocean.ocean import Ocean
from ocean_lib.data_provider.data_service_provider import DataServiceProvider
from ocean_utils.agreements.service_factory import ServiceDescriptor
from ocean_lib.web3_internal.wallet import Wallet
from ocean_lib.models.btoken import BToken #BToken is ERC20
import subprocess
import os
import time
import ipfshttpclient
from time import sleep
import yaml

class WaterDrone:
    def __init__(self):
        rospy.init_node("robonomics_listener", anonymous=False)
        self.ipfsclient = ipfshttpclient.connect()
        config_path = rospy.get_param("~config")
        try:
            with open(config_path) as f:
                content =  f.read()
                self.config = yaml.load(content, Loader=yaml.FullLoader)
        except Exception as e:
            while True:
                rospy.logerr("Configuration file is broken or not readable!")
                rospy.logerr(e)
                rospy.sleep(5)
        #print(self.config)
        self.read_data = False
        self.file = None
        self.pub = rospy.Publisher("cmd_drive", Drive, queue_size=10)

    def listener(self, data):
        if self.read_data:
            self.read_data = False
            self.file.write("\n")
            self.file.write(str(data))
    
    def create_datatoken(self):
        config = {'network' : f"https://{self.config['ocean']['network']}.infura.io/v3/{self.config['ocean']['infura']}",
                'metadataStoreUri' : f"https://aquarius.{self.config['ocean']['network']}.oceanprotocol.com",
                'providerUri' : f"https://provider.{self.config['ocean']['network']}.oceanprotocol.com"}
        ocean = Ocean(config)
        alice_wallet = Wallet(ocean.web3, private_key=self.config["ocean"]["test_key"])
        print("create datatoken: begin")
        data_token = ocean.create_data_token('DataToken1', 'DT1', alice_wallet, blob=ocean.config.metadata_store_url)
        token_address = data_token.address
        print("create datatoken: done")
        print(f"token_address = '{token_address}'")
        date_created = "2021-01-26T10:55:11Z"
        metadata =  {
            "main": {
                "type": "dataset", "name": "Water_drone", "author": "LoSk", 
                "license": "CC0: Public Domain", "dateCreated": date_created, 
                "files": [{"index": 0, "contentType": "text/text",
                "url": f"https://gateway.ipfs.io/ipfs/{self.ipfs_hash['Hash']}"}]}
        }
        print(metadata)
        service_attributes = {
                "main": {
                    "name": "dataAssetAccessServiceAgreement",
                    "creator": alice_wallet.address,
                    "timeout": 3600 * 24,
                    "datePublished": date_created,
                    "cost": 1.0, # <don't change, this is obsolete>
                }
            }
        print("sleep 30 s")
        time.sleep(30)
        service_endpoint = DataServiceProvider.get_url(ocean.config)
        download_service = ServiceDescriptor.access_service_descriptor(service_attributes, service_endpoint)
        asset = ocean.assets.create(metadata, alice_wallet, service_descriptors=[download_service], data_token_address=token_address)
        assert token_address == asset.data_token_address
        did = asset.did  # did contains the datatoken address
        print(f"did = '{did}'") 
        data_token.mint_tokens(alice_wallet.address, 100.0, alice_wallet)
        OCEAN_token = BToken(ocean.OCEAN_address)
        assert OCEAN_token.balanceOf(alice_wallet.address) > 0, "need Rinkeby OCEAN"
        print("sleep 40 s")
        sleep(40)
        pool = ocean.pool.create(token_address, data_token_amount=100.0, OCEAN_amount=10.0, from_wallet=alice_wallet)
        pool_address = pool.address
        print(f"pool_address = '{pool_address}'")


    def work(self):
        rate = rospy.Rate(1)
        self.file = open("telemetry", "w")
        print("Work started")
        for i in range(10):
            drive_data = Drive()
            drive_data.left = 0.8
            drive_data.right = 5
            self.pub.publish(drive_data)
            self.read_data = True
            if self.read_data:
                rospy.Subscriber("/joint_states",JointState, self.listener)
            rate.sleep()
        self.file.close()
        drive_data.left = 0
        drive_data.right = 0
        self.pub.publish(drive_data)
        print("Work finished")
        self.ipfs_hash = self.ipfsclient.add("telemetry")
        command = f"echo \"Hash: {self.ipfs_hash['Hash']}\" | {self.config['robonomics']['path']}robonomics io write datalog -s {self.config['robonomics']['dron_key']}"
        print(command)
        send_datalog = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
        print("Data sent to IPFS")
        self.create_datatoken()

    def spin(self):
        print("waiting for payment")
        programm = f"{self.config['robonomics']['path']}robonomics io read launch"
        process = subprocess.Popen(programm, shell=True, stdout=subprocess.PIPE)
        output = process.stdout.readline()
        #print(output.decode().strip())
        #print("5CfpnddQw98i8sAU6eFC7ogRAVYRQJsEJgtub7qvLwH7i9rc >> 5Fh6CVAELDUdGP9wWmjJD5VVSJvB8uU9Wj5Afh9RTzKiR6hS : true")
        if output.decode().strip() == "5CfpnddQw98i8sAU6eFC7ogRAVYRQJsEJgtub7qvLwH7i9rc >> 5Fh6CVAELDUdGP9wWmjJD5VVSJvB8uU9Wj5Afh9RTzKiR6hS : true":
            print("Work paid")
            self.work()
        else:
            print("Wrong payment")
        #rospy.spin()

drone = WaterDrone()
drone.spin()