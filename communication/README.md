1.  python grpc教程
   1. 链接：https://grpc.io/docs/languages/python/quickstart/
   2. 常用命令（更新proto生成）
   ```
   python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. ./data_transfer.proto
   ```
2. 多次启动节点后，可能会出现接收不到数据，需要强制杀死节点
   ```
   ps aux | grep gps
   kill -9 xxxx
   <!-- xxx为进程号 -->
   ```