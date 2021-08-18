#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from concurrent import futures
import logging

import grpc

import adm2ctrl_pb2
import adm2ctrl_pb2_grpc

import threading


def DtoCServer():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    adm2ctrl_pb2_grpc.add_DtoCServicer_to_server(DtoC(), server)
    server.add_insecure_port('localhost:20208')
    server.start()
    server.wait_for_termination()


threads = []
thd_ctrl = threading.Thread(target=DtoCServer)
threads.append(thd_ctrl)