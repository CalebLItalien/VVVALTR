#!/usr/bin/env python3

"""
    Simple tracker class to aid getting a single tracked location
"""

import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import qtm
import math
import numpy as np

class QtmTracker():

    def __init__(self, ip):
        self.loop = asyncio.get_event_loop()
        self.connection = None
        self.xml_string = None
        self.eulers = None
        self.ip = ip

        self.loop.run_until_complete(self.__connect_to_qtm(self.ip))

    async def __connect_to_qtm(self, ip):

        while self.connection is None:
            self.connection = await qtm.connect(ip)

            if self.connection is None:
                print("Failed to connect: waiting 5s")
                await asyncio.sleep(30)

            self.xml_string = await self.connection.get_parameters(parameters=["6d"])

    def ensure_connected(self):
        if not self.connection.has_transport():
            self.__connect_to_qtm(self.ip)

    # TODO: write function
    def get_translation(self, before, after):
        # complete translation or just scalar distance?
        pass

    # TODO: write function
    def get_rotation(self, before, after):
        pass

    def get_current_pos(self):
        self.eulers = None
        self.loop.run_until_complete(self.__live_stream_pos())
        return self.eulers
        # return self.loop.run_until_complete(self.live_stream_pos())


    def __on_packet(self, packet):
        info, bodies = packet.get_6d()
        # print("Framenumber: {} - Body count: {}".format(packet.framenumber, info.body_count)
        # )

        for index,(position, rotation) in enumerate(bodies):
            if index == 5:
                rotationArray = np.array(rotation.matrix)
                rotationArray.resize((3,3))
                self.eulers = self.__rotationMatrixToEulerAngles(rotationArray)
                # print('eulers', self.eulers)

    async def __live_stream_pos(self):

        # xml_string = await self.connection.get_parameters(parameters=["6d"])

        body_index = self.__create_body_index(self.xml_string)

        keyval_pairs = [(body_index[bodyname],bodyname) for bodyname in body_index.keys()]
        keyval_pairs.sort() #fortunately sorts of first item of tuples
        bodynames = [name for (_,name) in keyval_pairs]
        wanted_body = "L-frame"

        await self.connection.stream_frames(components=["6d"], on_packet=self.__on_packet)
        await self.connection.stream_frames_stop()


    def __create_body_index(self, xml_string):
        """ Extract a name to index dictionary from 6dof settings xml """
        xml = ET.fromstring(xml_string)

        body_to_index = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            body_to_index[body.text.strip()] = index

        return body_to_index

    def __rotationMatrixToEulerAngles(self, R) :

        assert(self.__isRotationMatrix(R))

        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

        singular = sy < 1e-6

        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

    def __isRotationMatrix(self, R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6


def main():
    tracker = QtmTracker("10.76.30.91")
    should_continue = "y"
    while should_continue != "n":
        print(tracker.get_current_pos())
        should_continue = input("Go again - y or n: ")

if __name__ == "__main__":
    main()