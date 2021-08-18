"""The Python implementation of the GRPC adm2vis.Visual server for data test."""


from concurrent import futures
import logging

import grpc

import adm2ctrl_pb2
import adm2ctrl_pb2_grpc

import threading


CTRL_RCV_ADDR = 'localhost:20208'


class DtoC(adm2ctrl_pb2_grpc.DtoCServicer):

    def ExecTasks(self, request, context):
        print("control server received: " + str(request.tasks))
        return adm2ctrl_pb2.TasksReply(tasks_flag="tasks received")
    
    def ExecAttack(self, request, context):
        print("control server received: " + str(request.attack_pairs))
        return adm2ctrl_pb2.AttackReply(attack_flag="attack pairs received")


def DtoCServer():
    global CTRL_RCV_ADDR
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    adm2ctrl_pb2_grpc.add_DtoCServicer_to_server(DtoC(), server)
    server.add_insecure_port(CTRL_RCV_ADDR)
    server.start()
    server.wait_for_termination()


threads = []
thd_ctrl = threading.Thread(target=DtoCServer)
threads.append(thd_ctrl)


if __name__ == '__main__':
    logging.basicConfig()
    
    for thd in threads:
        thd.start()
